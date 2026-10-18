// =============================================================================
// io/gimbal/gimbal.cpp —— 云台串口通信层实现（lt-2027 哨兵下位机协议 v3.0）
// -----------------------------------------------------------------------------
// 【一句话】把「串口字节流」翻译成「弧度制姿态 + 弹速」，把「弧度制绝对目标角」
//           翻译成「度制 28 字节 UsbCtrl 线帧」。
//
// 【与 v2.0 的差别】
//   1) 帧格式：43B/29B（0x5A A5 / 0x7F FE）→ 42B/28B（0xFF / 0x0D，带 crc8）。
//   2) 单位：弧度 → 度（线上），本层内做换算，对外仍保持弧度制。
//   3) 下行角度：相对增量 → 世界系【绝对角】（下位机自己算 big_yaw_target）。
//   4) 波特率：115200 → 921600（与下位机 usart.c 的 huart1 一致）。
//   5) 链路模型：下位机"收到一帧才回一帧"，故构造期先发空帧敲门 + 发送线程带空闲心跳。
//
// 【线程模型】
//   - read_thread：唯一读者，独占 serial_mutex_ 读串口；解析成功后加 mutex_ 更新
//     state_/mode_/统计量，并把 (q, t) 推入插值队列 queue_。
//   - tx_thread：唯一写者，独占 serial_mutex_ 写串口；只发最新一帧控制量。
//   - 业务线程：只调用 send()/state()/mode()/diagnostics()/q()，全部内部加锁。
//   - 锁顺序：任何路径都【不会】在持有 mutex_ 时再去拿 serial_mutex_（反之亦然），
//     因此不存在锁序死锁。
// =============================================================================

#include "gimbal.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "tools/file_logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace io
{
namespace
{
// 本文件专用日志器：logs/io/gimbal/gimbal.cpp/{run,warn,error}.log
// 为什么不用 tools::logger()：那是全局单文件（logs/<时间戳>.log），排查云台问题时
// 会被相机/追踪/规划等模块的输出淹没；这里按文件分流，三档且新覆盖旧。
std::shared_ptr<spdlog::logger> glog()
{
  static auto instance = tools::file_logger("io/gimbal/gimbal.cpp");
  return instance;
}

// 打开后打印每一帧的 hex（调试用；线上量很小，默认关闭）
constexpr bool kVerboseSerialHexLog = false;

// 无数据判定（取代旧的"4000 次空读就重连"，那会在 1s 内反复 close/open，
// 每次 open 后 100ms 的 flushInput() 还会把刚到的数据丢掉）
constexpr std::int64_t kSilentWarnMs = 2000;     // 连续无字节 → 输出一次排查清单
constexpr std::int64_t kSilentReportMs = 10000;  // 之后每 10s 补一行计数，便于知道静默了多久
constexpr std::int64_t kSilentReopenMs = 30000;  // 静默 30s 且一个字节都没收到 → 只强制重开一次
constexpr std::size_t kFirstBytesHexMax = 32;    // 首包取证最多打印的字节数

// 构造期"敲门"间隔：下位机 daemon 连续 50ms 收不到合法帧就判离线并重启 DMA，
// 所以敲门必须比 50ms 快得多（20ms = 3 帧/60ms，留 2.5 倍余量）。
constexpr int kKnockIntervalMs = 20;

// 弧度 ↔ 度（与下位机 DEGREE_2_RAD = 0.0174533 / RAD_2_DEGREE 等价）
constexpr double kRadToDeg = 57.295779513082320877;
constexpr double kDegToRad = 0.017453292519943295;

// 上行遥测合理性上限（线上单位是度，所以按度校验，用于挡掉错位/伪帧）
constexpr float kMaxYawDeg = 180.5f;        // yaw 世界角不会超过 ±180°
constexpr float kMaxPitchDeg = 90.5f;       // pitch 物理上不超过 ±90°
constexpr float kMaxYawVelDps = 2000.0f;    // 约 35 rad/s，超过一定是坏帧
constexpr float kMaxPitchVelDps = 2000.0f;
constexpr float kMaxBulletSpeed = 40.0f;    // 弹速枚举最大 30 m/s

// 解析流缓冲：约 24 帧遥测，够一次 read() 拿到多帧；满了保留尾部一帧继续同步
constexpr std::size_t kStreamBufferSize = 1024;

// ---------------------------------------------------------------------------
// 串口设备的"身份证"（sysfs 取证）
// ---------------------------------------------------------------------------
// 为什么需要：/dev/ttyUSB0 这种名字会随插拔顺序变（这次是 CH340，下次可能是
// 一块 ST-Link），排查"收不到数据"时必须能确认"我们打开的到底是哪个物理设备"。
// 只有 ttyUSB*/ttyACM* 这类 USB 串口有这些字段；pty、/dev/gimbal 软链接读不到。
struct SerialPortInfo
{
  std::string path;          // /dev/ttyUSB0
  bool exists = false;       // 设备节点此刻是否存在
  std::string vid_pid;       // 1a86:7523（厂商:产品）
  std::string driver;        // ch341-uart / cdc_acm
  std::string product;       // USB Serial / STM32 Virtual COM Port
  std::string manufacturer;  // wch.cn / STMicroelectronics
  std::string serial;        // 设备序列号（同型号多台时唯一区分）

  std::string str() const
  {
    std::string out = path;
    out += exists ? " [存在]" : " [不存在]";
    if (!vid_pid.empty()) out += " vid:pid=" + vid_pid;
    if (!driver.empty()) out += " driver=" + driver;
    if (!product.empty()) out += " product=\"" + product + "\"";
    if (!manufacturer.empty()) out += " manufacturer=\"" + manufacturer + "\"";
    if (!serial.empty()) out += " serial=" + serial;
    if (exists && vid_pid.empty()) out += " (非 ttyUSB/ttyACM 或 sysfs 不可读，无法核对身份)";
    return out;
  }
};

// 读取 sysfs 小文件的首行（不存在/读失败返回空串，绝不抛异常）
std::string read_sysfs_line(const std::filesystem::path & path)
{
  std::ifstream in(path);
  if (!in.is_open()) return {};
  std::string line;
  std::getline(in, line);
  while (!line.empty() && (line.back() == '\n' || line.back() == '\r' || line.back() == ' ')) {
    line.pop_back();
  }
  return line;
}

// 探测一个设备节点：节点是否存在 + USB 身份（能拿到多少算多少）
SerialPortInfo probe_serial_port(const std::string & path)
{
  namespace fs = std::filesystem;
  SerialPortInfo info;
  info.path = path;

  std::error_code ec;
  info.exists = fs::exists(path, ec);
  if (!info.exists) return info;

  try {
    // /sys/class/tty/ttyUSB0/device 指向 USB 接口目录（形如 1-1:1.0），
    // 接口的父目录才是"USB 设备"，idVendor/idProduct/product/serial 都在那里。
    const auto tty_dir = fs::path("/sys/class/tty") / fs::path(path).filename();
    auto device = tty_dir / "device";
    if (!fs::exists(device, ec)) return info;

    auto usb = device;
    for (int i = 0; i < 4 && !usb.empty(); ++i) {
      if (fs::exists(usb / "idVendor", ec)) break;
      if (!usb.has_parent_path() || usb.parent_path() == usb) { usb.clear(); break; }
      usb = usb.parent_path();
    }
    if (!usb.empty()) {
      const auto vid = read_sysfs_line(usb / "idVendor");
      const auto pid = read_sysfs_line(usb / "idProduct");
      if (!vid.empty() && !pid.empty()) {
        info.vid_pid = vid + ":" + pid;
        info.product = read_sysfs_line(usb / "product");
        info.manufacturer = read_sysfs_line(usb / "manufacturer");
        info.serial = read_sysfs_line(usb / "serial");
      }
    }

    // 驱动名（ch341-uart / cdc_acm / ftdi_sio ...）：device/driver 是符号链接
    const auto driver_link = device / "driver";
    if (fs::exists(driver_link, ec)) {
      info.driver = fs::read_symlink(driver_link, ec).filename().string();
    }
  } catch (...) {
    // sysfs 结构随内核版本有差异，取证失败不影响主流程
  }

  return info;
}

// 当前机器上"能当云台口用"的设备清单：/dev/gimbal 优先，其余按名字排序
std::vector<std::string> list_serial_candidates()
{
  namespace fs = std::filesystem;
  std::vector<std::string> ports;
  std::error_code ec;

  if (fs::exists("/dev/gimbal", ec)) ports.emplace_back("/dev/gimbal");

  std::vector<std::string> others;
  fs::directory_iterator it("/dev", ec);
  if (!ec) {
    for (const auto & entry : it) {
      std::error_code entry_ec;
      if (!entry.is_character_file(entry_ec)) continue;
      const auto name = entry.path().filename().string();
      if (name.rfind("ttyUSB", 0) == 0 || name.rfind("ttyACM", 0) == 0) {
        others.push_back(entry.path().string());
      }
    }
  }
  std::sort(others.begin(), others.end());
  ports.insert(ports.end(), others.begin(), others.end());
  return ports;
}

// 串口设备名探测：配置值存在就用配置值，否则按常见命名猜一个
std::string resolve_serial_port(const std::string & configured_port)
{
  namespace fs = std::filesystem;

  if (!configured_port.empty() && fs::exists(configured_port)) {
    return configured_port;
  }

  static const std::vector<std::string> preferred_ports = {
    "/dev/gimbal",
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
    "/dev/ttyACM0",
    "/dev/ttyACM1",
  };

  for (const auto & port : preferred_ports) {
    if (fs::exists(port)) {
      return port;
    }
  }

  std::vector<std::string> detected_ports;
  try {
    for (const auto & entry : fs::directory_iterator("/dev")) {
      if (!entry.is_character_file()) {
        continue;
      }
      const auto name = entry.path().filename().string();
      if (name.rfind("ttyUSB", 0) == 0 || name.rfind("ttyACM", 0) == 0) {
        detected_ports.push_back(entry.path().string());
      }
    }
  } catch (...) {
    return configured_port;
  }

  if (detected_ports.empty()) {
    return configured_port;
  }

  std::sort(detected_ports.begin(), detected_ports.end());
  return detected_ports.front();
}

// 一次性配置超时参数（构造与重连共用）
serial::Timeout make_timeout()
{
  serial::Timeout timeout = serial::Timeout::simpleTimeout(kSerialTimeoutMs);
  timeout.inter_byte_timeout = kSerialInterByteMs;
  return timeout;
}

// 把字节缓冲格式化成 "AA BB CC" 用于错误现场取证
// 墙上时钟 `YYYY-MM-DD HH:MM:SS.mmm`：快照文件里必须有绝对时间，
// 否则事后没法把日志和现场的"几点几分"对上。
std::string wall_clock_now()
{
  auto now = std::chrono::system_clock::now();
  auto secs = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
  std::tm tm_buf{};
  localtime_r(&secs, &tm_buf);
  char buf[64];
  std::snprintf(
    buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d.%03d", tm_buf.tm_year + 1900, tm_buf.tm_mon + 1,
    tm_buf.tm_mday, tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec, static_cast<int>(ms));
  return buf;
}

std::string hex_dump(const uint8_t * data, std::size_t size)
{
  std::string hex;
  hex.reserve(size * 3);
  for (std::size_t i = 0; i < size; ++i) {
    char b[4];
    std::snprintf(b, sizeof(b), "%02X ", static_cast<unsigned>(data[i]));
    hex += b;
  }
  return hex;
}
}  // namespace

uint8_t crc8(const uint8_t * data, std::size_t size)
{
  // init 0x00 / 多项式 0x31 / 高位优先 —— 与下位机 modules/algorithm/crc8.c
  // 的 sht75_crc_table 查表实现逐位等价（例：crc8({0x01}) == 0x31 == 49，表值一致）
  uint8_t crc = 0x00;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 0x80) {
        crc = static_cast<uint8_t>((crc << 1) ^ 0x31);
      } else {
        crc = static_cast<uint8_t>(crc << 1);
      }
    }
  }
  return crc;
}

Gimbal::Gimbal(const std::string & config_path)
{
  // ── 读配置：失败时把路径和"应该长什么样"都写进日志 ──────────────────────────
  std::string com_port;
  try {
    auto yaml = tools::load(config_path);
    com_port = tools::read<std::string>(yaml, "com_port");  // 配置里的串口设备
    // 可选键：波特率 / 是否校验 crc8（配置里没有时用默认值，不报错）
    baudrate_ = tools::read<int>(yaml, "baudrate", kDefaultBaudrate);
    crc8_check_ = tools::read<bool>(yaml, "crc8_check", true);
  } catch (const std::exception & e) {
    glog()->error(
      "[Gimbal] 读取配置失败: {} (path={})\n  配置文件必须至少包含 com_port，例如:\n"
      "    com_port: /dev/ttyUSB0\n    baudrate: 921600\n    crc8_check: true",
      e.what(), config_path);
    tools::snapshot("io/gimbal/gimbal.cpp", std::string("fatal=bad_config path=") + config_path);
    exit(1);
  }
  configured_port_ = com_port;

  glog()->info(
    "[Gimbal] 配置: path={} com_port={} baud={} crc8_check={}", config_path, com_port, baudrate_,
    crc8_check_ ? "true" : "false");

  // ── 端口枚举：把机器上所有"看起来能用"的串口连同 USB 身份写进日志 ──────────
  // 现场最常见的坑：/dev/ttyUSB0 不一定是转接板（插拔顺序会变），而下面的回退逻辑
  // 会静默选中一个别的设备 —— 结果是"串口打开了，但一个字节都收不到"。
  {
    const auto candidates = list_serial_candidates();
    std::string report;
    for (const auto & port : candidates) report += "\n    " + probe_serial_port(port).str();
    if (report.empty()) report = "\n    (无：没有 /dev/gimbal，也没有 /dev/ttyUSB* 与 /dev/ttyACM*)";
    glog()->info("[Gimbal] 本机串口候选 {} 个:{}", candidates.size(), report);
  }

  resolved_port_ = resolve_serial_port(com_port);
  if (resolved_port_ != com_port) {
    glog()->warn(
      "[Gimbal] 配置的 com_port '{}' 不存在，回退到 '{}'。若收不到数据，先确认这个端口是否真的是"
      "云台转接板。", com_port, resolved_port_);
  }
  glog()->info("[Gimbal] 选用端口: {}", probe_serial_port(resolved_port_).str());

  try {
    serial_.setPort(resolved_port_);
    serial_.setBaudrate(baudrate_);  // 921600 8N1（下位机 USART1）

    // 低延迟优先：42 字节 @921600 约 0.46 ms，50ms 总超时/5ms 字节间超时都很宽裕
    serial::Timeout timeout = make_timeout();  // setTimeout 形参是非 const 引用，必须是左值
    serial_.setTimeout(timeout);

    serial_.open();
    glog()->info(
      "[Gimbal] 串口已打开: {} @ {} 8N1 (crc8_check={})", resolved_port_, baudrate_,
      crc8_check_ ? "true" : "false");

    // 初始化时清空可能的积压数据
    try {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      if (serial_.isOpen()) {
        serial_.flushInput();
        serial_.flushOutput();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    } catch (...) {
    }
  } catch (const std::exception & e) {
    // 打开失败是最常见的现场故障，把端口、权限、候选表一次留全
    glog()->error(
      "[Gimbal] 打开串口失败: {}\n  端口={}\n  常见原因:\n"
      "    1) 设备没插 / 节点名变了（插拔顺序会改变 ttyUSB0/ttyUSB1）；\n"
      "    2) 权限不足：当前用户不在 dialout 组（sudo usermod -aG dialout $USER 后重新登录）；\n"
      "    3) 节点被别的进程占用（fuser /dev/ttyUSB0）；\n"
      "    4) 转接板驱动没装（CH340 需要 ch341 驱动，dmesg 里能看到）。",
      e.what(), resolved_port_);
    tools::snapshot(
      "io/gimbal/gimbal.cpp",
      std::string("fatal=open_serial_failed port=") + resolved_port_ + " err=" + e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);
  tx_thread_ = std::thread(&Gimbal::tx_thread, this);

  // 下位机是「请求-响应式」：只有收到一帧 UsbCtrl 才会回发一帧遥测。
  // 所以这里必须先主动发空帧（mode=0，不控制）去敲门，再限时等待首帧姿态；
  // 否则 queue_.pop() 会永久阻塞在"永远收不到第一帧"上（v2.0 的老问题）。
  // 敲门间隔必须远小于下位机 daemon 的 50ms 判离线阈值（reload_count=5 × osDelay(10)）。
  auto wait_start = std::chrono::steady_clock::now();
  bool got_first_frame = false;
  int knock_count = 0;
  while (!quit_) {
    if (!queue_.empty()) {
      got_first_frame = true;
      break;
    }
    auto elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - wait_start)
        .count();
    if (elapsed_ms >= kFirstFrameTimeoutMs) break;

    VisionToGimbal idle;  // mode = 0：不控制，只为触发下位机回帧
    enqueue_cmd(idle);
    ++knock_count;
    std::this_thread::sleep_for(std::chrono::milliseconds(kKnockIntervalMs));
  }

  if (got_first_frame) {
    glog()->info("[Gimbal] First q received（敲门 {} 次后收到首帧）。", knock_count);
  } else {
    // 到这里还没帧：把"我们发了什么、对方回了什么"摆出来，代替原来只喊一句"没收到"
    const auto rx_bytes = rx_bytes_.load();
    glog()->warn(
      "[Gimbal] {} ms 内没有收到遥测帧——下位机一帧都没回。\n"
      "  发送侧: tx_frames={} 失败={} 敲门={} 次（每 {} ms 一帧, baud={}）\n"
      "  接收侧: rx_bytes={}\n"
      "  {}",
      kFirstFrameTimeoutMs, tx_frames_.load(), tx_fail_.load(), knock_count, kKnockIntervalMs, baudrate_,
      rx_bytes, rx_bytes == 0
                  ? "rx_bytes==0 ⇒ 物理上没有任何字节到达 PC：\n"
                    "    查 TX/RX 是否交叉、GND 是否共地、转接板是否插在自瞄那个口\n"
                    "    （下位机 USART1 = PB7(RX)/PA9(TX)，不是 PA9/PA10），以及板子烧的是不是这套固件。"
                  : "rx_bytes>0 ⇒ 有字节到但解不出帧：\n"
                    "    看 run.log 里 \"疑似坏帧\" 的 hex dump，多半是波特率或协议版本不对。");
    tools::snapshot(
      "io/gimbal/gimbal.cpp",
      std::string("fatal=no_first_frame rx_bytes=") + std::to_string(rx_bytes) +
        " tx_frames=" + std::to_string(tx_frames_.load()) + " port=" + resolved_port_);
  }
}

/*析构函数，用于退出*/
Gimbal::~Gimbal()
{
  quit_ = true;
  tx_queue_.push(VisionToGimbal{});  // 唤醒发送线程（若仍阻塞在 pop 上）
  if (thread_.joinable()) thread_.join();
  if (tx_thread_.joinable()) tx_thread_.join();
  try {
    serial_.close();
  } catch (...) {
  }
}

/*获取云台模式——0: 空闲, 1: 自瞄, 2: 小符, 3: 大符*/
GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

/*获取云台状态——弧度制（线上度制已换算）*/
GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

/*获取诊断信息 - 用于监测串口实时性和可靠性*/
GimbalDiagnostics Gimbal::diagnostics() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  GimbalDiagnostics diag;

  diag.frame_count = frame_count_;
  diag.error_count = error_count_;
  diag.rx_frame_rate = rx_frame_rate_;
  diag.tx_latency_ms = tx_latency_ms_.load();
  diag.queue_depth = tx_backlog_.load();
  diag.serial_ok = serial_.isOpen();
  diag.rx_bytes = rx_bytes_.load();
  diag.tx_frames = tx_frames_.load();
  diag.tx_fail = tx_fail_.load();

  // 接收延迟 = 距上一次成功解析的时长（越大说明链路越差/下位机没在回帧）
  auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
  auto last_rx_ns = last_rx_ok_ns_.load();
  if (last_rx_ns > 0) {
    diag.rx_latency_ms = static_cast<double>(now_ns - last_rx_ns) / 1e6;
  }

  return diag;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";  // 默认
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

/*带超时的姿态查询：队列里凑不出"一前一后"两帧就返回 false，不阻塞*/
bool Gimbal::try_q(std::chrono::steady_clock::time_point t, Eigen::Quaterniond & out, int timeout_ms)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(std::max(timeout_ms, 0));

  while (true) {
    // 插值需要两帧；本类是队列唯一消费者，size()>=2 时 pop()/front() 不会阻塞
    if (queue_.size() < 2) {
      if (std::chrono::steady_clock::now() >= deadline) return false;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    const double t_ab = tools::delta_time(t_a, t_b);
    const double t_ac = tools::delta_time(t_a, t);
    out = q_a.slerp(t_ab > 0.0 ? t_ac / t_ab : 0.0, q_b).normalized();

    if (t < t_a) return true;   // t 在最早一帧之前：按最近两帧外推（与 q() 语义一致）
    if (t <= t_b) return true;  // t 落在 [t_a, t_b]：正常插值
    queue_.pop();               // t 还在后面：丢掉 t_b，前进一帧重试
  }
}

void Gimbal::send(io::VisionToGimbal cmd)
{
  // mode == 0 表示"不控制云台"：角度字段清零，避免下位机把过期目标当有效值用
  // （注意角度是绝对角，残留值会让云台回到旧目标）
  if (cmd.mode == 0) {
    cmd.yaw = 0.0f;
    cmd.yaw_vel = 0.0f;
    cmd.yaw_acc = 0.0f;
    cmd.pitch = 0.0f;
    cmd.pitch_vel = 0.0f;
    cmd.pitch_acc = 0.0f;
  }

  // 异步发送：控制线程只入队，不阻塞在串口写上
  enqueue_cmd(cmd);
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  VisionToGimbal cmd;
  cmd.mode = control ? (fire ? 2 : 1) : 0;
  cmd.yaw = yaw;  // rad，绝对角（不再与 state 相减：新下位机自己算目标 - 当前）
  cmd.yaw_vel = yaw_vel;
  cmd.yaw_acc = yaw_acc;
  cmd.pitch = pitch;
  cmd.pitch_vel = pitch_vel;
  cmd.pitch_acc = pitch_acc;

  send(cmd);  // 复用逻辑帧版本的清零与入队逻辑
}

void Gimbal::enqueue_cmd(const VisionToGimbal & cmd)
{
  tx_backlog_.fetch_add(1);
  tx_queue_.push(cmd);  // 队满时按 PopWhenFull=true 丢最旧，保证发的是最新控制量
}

UsbCtrl Gimbal::pack_ctrl(const VisionToGimbal & cmd) const
{
  UsbCtrl frame;
  frame.mode = cmd.mode;

  // 弧度 → 度；yaw 先归一化到 (-180, 180]，避免下位机收到 ±360 附近的角度跳变
  frame.yaw = static_cast<float>(tools::limit_rad(cmd.yaw) * kRadToDeg);
  frame.yaw_vel = static_cast<float>(cmd.yaw_vel * kRadToDeg);
  frame.yaw_acc = static_cast<float>(cmd.yaw_acc * kRadToDeg);
  frame.pitch = static_cast<float>(cmd.pitch * kRadToDeg);
  frame.pitch_vel = static_cast<float>(cmd.pitch_vel * kRadToDeg);
  frame.pitch_acc = static_cast<float>(cmd.pitch_acc * kRadToDeg);

  // crc8 覆盖前 26 字节（含 head，不含 crc8 自身与 tail），与下位机打包顺序一致
  frame.crc8 = crc8(reinterpret_cast<const uint8_t *>(&frame), kCtrlFrameSize - 2);

  return frame;
}

void Gimbal::write_ctrl(const UsbCtrl & frame)
{
  const uint8_t * raw = reinterpret_cast<const uint8_t *>(&frame);

  if (kVerboseSerialHexLog) {
    glog()->debug("[Gimbal] tx {}B: {}", kCtrlFrameSize, hex_dump(raw, kCtrlFrameSize));
  }

  auto tx_start = std::chrono::steady_clock::now();
  bool ok = true;
  bool open = false;
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    try {
      open = serial_.isOpen();
      if (!open) {
        ok = false;
      } else {
        serial_.write(raw, kCtrlFrameSize);
        // 记下最后一帧原始字节：接收侧用它做"回环自检"
        // （如果收到的字节和这一帧完全一样，说明 TX/RX 被短接，下位机根本没回）
        std::memcpy(last_tx_frame_.data(), raw, kCtrlFrameSize);
        has_tx_frame_ = true;
      }
    } catch (const std::exception & e) {
      ok = false;
      glog()->error("[Gimbal] 写串口失败: {} (port={})", e.what(), resolved_port_);
    }
  }

  auto tx_end = std::chrono::steady_clock::now();
  double tx_latency = std::chrono::duration<double, std::milli>(tx_end - tx_start).count();

  if (ok) {
    const auto total = tx_frames_.fetch_add(1) + 1;
    auto old = tx_latency_ms_.load();
    tx_latency_ms_.store(old > 0.0 ? 0.9 * old + 0.1 * tx_latency : tx_latency);

    if (total == 1) {
      // 首帧下行单独留证：现场可以用逻辑分析仪/示波器对照这串字节
      glog()->info(
        "[Gimbal] 首帧下行已写出 ({}B, mode={}): {}\n"
        "  帧结构自检: head={:#04x} tail={:#04x} crc8={:#04x}  —— 若下位机端口上量不到这串字节，"
        "问题在 PC→板 这一段（线序/端口/波特率）。",
        kCtrlFrameSize, static_cast<int>(frame.mode), hex_dump(raw, kCtrlFrameSize),
        static_cast<int>(raw[0]), static_cast<int>(raw[kCtrlFrameSize - 1]),
        static_cast<int>(raw[kCtrlFrameSize - 2]));
    }

    if (tx_latency > 5.0) {
      // 超过 5ms 说明有人在持 serial_mutex_ 干慢事（比如重连），发送线程被挡住了
      glog()->warn(
        "[Gimbal] 下行写串口耗时 {:.2f}ms（异常，正常应 <0.5ms）：多半是别的线程长时间占着 "
        "serial_mutex_（重连？）。", tx_latency);
    }
  } else {
    const auto fail_total = tx_fail_.fetch_add(1) + 1;
    // 重连期间串口会短暂关闭，这一句可能连打几十条：前 3 次 + 每 100 次各报一条
    if (!open && (fail_total <= 3 || fail_total % 100 == 0)) {
      glog()->warn(
        "[Gimbal] 下行未发出：串口未打开 (port={}, 累计失败 {})", resolved_port_, fail_total);
    }
  }
}

void Gimbal::tx_thread()
{
  glog()->info("[Gimbal] tx_thread started (心跳周期 {}ms).", kIdleHeartbeatMs);

  auto last_tx = std::chrono::steady_clock::now();
  auto last_report = last_tx;
  uint64_t frames_at_report = 0;
  uint64_t heartbeats = 0;

  while (!quit_) {
    VisionToGimbal cmd;
    bool has_cmd = false;

    // 本线程是唯一消费者，生产者只 push，所以 empty() 与 pop() 之间队列不会变空
    if (!tx_queue_.empty()) {
      tx_queue_.pop(cmd);
      // 背压时只发最新控制量，丢弃过期命令以降低控制滞后
      while (!tx_queue_.empty()) {
        tx_queue_.pop(cmd);
      }
      has_cmd = true;
    }

    if (has_cmd) {
      auto frame = pack_ctrl(cmd);
      write_ctrl(frame);
      last_tx = std::chrono::steady_clock::now();
      tx_backlog_.store(0);  // 队列已排空
    } else {
      auto silent_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now() - last_tx)
                         .count();
      if (silent_ms >= kIdleHeartbeatMs) {
        // 空闲心跳：上层超过 100ms 没调 send() 时补发 mode=0 空帧。
        // 两个作用：(1) 请求-响应链路靠它保活，否则收不到姿态；
        //           (2) 明确告诉下位机"不要控制云台"，不会继续执行过期目标。
        VisionToGimbal idle;
        idle.mode = 0;
        auto frame = pack_ctrl(idle);
        write_ctrl(frame);
        last_tx = std::chrono::steady_clock::now();
        ++heartbeats;
      }
    }

    // 每 5s 一条发送侧摘要（debug：只进 run.log，不刷控制台）
    // 排查"收不到首帧"时，这一行能证明"我们确实在发，问题在对面"
    auto report_now = std::chrono::steady_clock::now();
    if (report_now - last_report >= std::chrono::seconds(5)) {
      const double secs = std::chrono::duration<double>(report_now - last_report).count();
      const auto total = tx_frames_.load();
      glog()->debug(
        "[Gimbal] tx 摘要: frames={} (+{}/{:.1f}s = {:.1f}Hz) 失败={} 心跳={} 队列深度={} 写延迟={:.3f}ms",
        total, total - frames_at_report, secs, secs > 0 ? (total - frames_at_report) / secs : 0.0,
        tx_fail_.load(), heartbeats, tx_backlog_.load(), tx_latency_ms_.load());
      last_report = report_now;
      frames_at_report = total;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  glog()->info("[Gimbal] tx_thread stopped. 累计下行 {} 帧，失败 {} 次",
    tx_frames_.load(), tx_fail_.load());
}

bool Gimbal::handle_frame(const uint8_t * buffer)
{
  UsbAutoSendToNuc raw;
  std::memcpy(&raw, buffer, kTelemetryFrameSize);

  // 1) 合理性检查（线上是度，先按度挡掉错位/伪帧，再换算）
  //    packed 结构体的字段既不能绑 fmt 的引用形参、也不能取地址，先拷进局部变量
  const uint8_t mode = raw.mode;
  const float yaw_deg = raw.yaw;
  const float yaw_vel_dps = raw.yaw_vel;
  const float pitch_deg = raw.pitch;
  const float pitch_vel_dps = raw.pitch_vel;
  const float bullet_speed = raw.bullet_speed;
  const uint16_t bullet_count = raw.bullet_count;
  const float q_w = raw.q[0], q_x = raw.q[1], q_y = raw.q[2], q_z = raw.q[3];

  if (
    std::abs(yaw_deg) > kMaxYawDeg || std::abs(pitch_deg) > kMaxPitchDeg ||
    std::abs(yaw_vel_dps) > kMaxYawVelDps || std::abs(pitch_vel_dps) > kMaxPitchVelDps ||
    std::abs(bullet_speed) > kMaxBulletSpeed) {
    tools::logger()->warn(
      "[Gimbal] Data sanity check failed: yaw={:.2f}deg, pitch={:.2f}deg, "
      "yaw_vel={:.2f}deg/s, pitch_vel={:.2f}deg/s, bullet_speed={:.2f}",
      yaw_deg, pitch_deg, yaw_vel_dps, pitch_vel_dps, bullet_speed);
    return false;
  }

  // 2) 四元数模长检查 + 归一化
  //    下位机在 UsbVsioionSetAltiitude() 里已按"与上位机约定: xy交换"处理过轴，
  //    线上顺序就是 w,x,y,z，这里直接读，不要再交换回去。
  Eigen::Quaterniond q(q_w, q_x, q_y, q_z);
  if (!std::isfinite(q.norm()) || q.norm() < 0.5 || q.norm() > 1.5) {
    tools::logger()->warn("[Gimbal] Invalid quaternion norm: {:.3f}", q.norm());
    return false;
  }
  q.normalize();

  auto t = std::chrono::steady_clock::now();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    rx_data_ = raw;

    // 3) 度 → 弧度
    //    下位机上行 pitch 发的是 -muzzle_pitch_deg（TONGJI_PITCH_UPLINK_SIGN = -1），
    //    即"视觉约定"下的 pitch，所以这里直接换算，不再取反；
    //    下行同理：视觉发的 pitch 由下位机自己取反去驱动电机。
    state_.yaw = static_cast<float>(tools::limit_rad(yaw_deg * kDegToRad));
    state_.yaw_vel = static_cast<float>(yaw_vel_dps * kDegToRad);
    state_.pitch = static_cast<float>(pitch_deg * kDegToRad);
    state_.pitch_vel = static_cast<float>(pitch_vel_dps * kDegToRad);
    state_.bullet_speed = bullet_speed;
    state_.bullet_count = bullet_count;

    switch (mode) {
      case 0:
        mode_ = GimbalMode::IDLE;
        break;
      case 1:
        mode_ = GimbalMode::AUTO_AIM;
        break;
      case 2:
        mode_ = GimbalMode::SMALL_BUFF;
        break;
      case 3:
        mode_ = GimbalMode::BIG_BUFF;
        break;
      default:
        mode_ = GimbalMode::IDLE;
        tools::logger()->warn("[Gimbal] Invalid mode: {}", static_cast<int>(mode));
        break;
    }

    // 4) 统计：帧数 + 帧率（指数滑动平均，避免再开滑动窗口）
    frame_count_++;
    auto now_ns = t.time_since_epoch().count();
    if (last_frame_ns_ > 0 && now_ns > last_frame_ns_) {
      double dt = static_cast<double>(now_ns - last_frame_ns_) / 1e9;
      if (dt > 0.0) {
        double inst_rate = 1.0 / dt;
        rx_frame_rate_ = rx_frame_rate_ > 0.0 ? 0.9 * rx_frame_rate_ + 0.1 * inst_rate : inst_rate;
      }
    }
    last_frame_ns_ = now_ns;
  }

  // 记录成功接收时间戳（用于诊断延迟）
  last_rx_ok_ns_.store(t.time_since_epoch().count());

  queue_.push({q, t});

  if (kVerboseSerialHexLog) {
    glog()->debug("[Gimbal] rx {}B: {}", kTelemetryFrameSize, hex_dump(buffer, kTelemetryFrameSize));
  }

  // 首帧单独留一条 info 取证：这是"链路到底通没通"的唯一铁证
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (frame_count_ == 1) {
      glog()->info(
        "[Gimbal] 首帧遥测已收到 ({}B): mode={} yaw={:.2f}deg->{:.3f}rad pitch={:.2f}deg->{:.3f}rad "
        "bullet_speed={:.2f} count={} q=({:.3f},{:.3f},{:.3f},{:.3f})\n  dump={}\n"
        "  链路已通：端口、波特率、线序、下位机固件四者都对。",
        kTelemetryFrameSize, static_cast<int>(mode), yaw_deg, state_.yaw, pitch_deg, state_.pitch,
        bullet_speed, bullet_count, q.w(), q.x(), q.y(), q.z(), hex_dump(buffer, kTelemetryFrameSize));
    }
  }

  // 周期摘要降为 debug（只进 run.log）：控制台不该被 100Hz 的统计刷屏
  static int diag_counter = 0;
  if (++diag_counter >= 200) {
    diag_counter = 0;
    auto diag = diagnostics();
    glog()->debug(
      "[Gimbal] rx 摘要: {:.1f}Hz frames={} errors={} rx_bytes={} tx_frames={} 延迟={:.2f}ms serial_ok={}",
      diag.rx_frame_rate, diag.frame_count, diag.error_count, diag.rx_bytes, diag.tx_frames,
      diag.rx_latency_ms, diag.serial_ok ? "yes" : "no");
  }

  return true;
}

void Gimbal::read_thread()
{
  glog()->info("[Gimbal] read_thread started (port={}, baud={}).", resolved_port_, baudrate_);

  uint8_t stream_buf[kStreamBufferSize] = {0};
  std::size_t stream_len = 0;
  int consecutive_errors = 0;

  // 诊断状态
  bool first_bytes_logged = false;      // 是否已经打印过"收到的第一包字节"
  bool echo_warned = false;             // 回环（TX/RX 短接）告警只打一次
  bool silent_warned = false;           // 静默告警只打一次
  bool reopened_after_silence = false;  // 静默 30s 后是否已经"强制重开"过一次
  auto up = [] { return std::chrono::steady_clock::now().time_since_epoch().count(); };
  std::int64_t last_any_byte_ns = up();  // 最近一次真的收到字节（含垃圾字节）
  std::int64_t last_silent_report_ns = last_any_byte_ns;
  std::int64_t last_snapshot_ns = 0;

  while (!quit_) {
    const auto loop_now_ns = up();

    // ── 0) 每秒覆盖写一次状态快照 ───────────────────────────────────────────
    // 进程被 Ctrl-C / 卡死 / 崩溃后，先看 logs/io/gimbal/gimbal.cpp/state.log，
    // 它永远只有一行、永远是最后一秒的状态，比翻 run.log 快得多。
    if (loop_now_ns - last_snapshot_ns >= 1000000000LL) {
      last_snapshot_ns = loop_now_ns;
      int frames = 0, errors = 0;
      double rate = 0.0;
      std::int64_t last_frame_ns = 0;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        frames = frame_count_;
        errors = error_count_;
        rate = rx_frame_rate_;
        last_frame_ns = last_frame_ns_;
      }
      char snap[512];
      std::snprintf(
        snap, sizeof(snap),
        "time=%s port=%s baud=%d rx_bytes=%llu tx_frames=%llu tx_fail=%llu rx_stream_queued=%zu "
        "rx_frames=%d rx_errors=%d rx_rate=%.1fHz last_frame_ago_ms=%lld silent_ms=%lld\n",
        wall_clock_now().c_str(), resolved_port_.c_str(), baudrate_,
        static_cast<unsigned long long>(rx_bytes_.load()),
        static_cast<unsigned long long>(tx_frames_.load()),
        static_cast<unsigned long long>(tx_fail_.load()), queue_.size(), frames, errors, rate,
        static_cast<long long>(last_frame_ns > 0 ? (loop_now_ns - last_frame_ns) / 1000000LL : -1),
        static_cast<long long>((loop_now_ns - last_any_byte_ns) / 1000000LL));
      tools::snapshot("io/gimbal/gimbal.cpp", snap);
    }

    // ── 1) 解析错太多：重连（波特率/线序不对时的兜底）────────────────────────
    if (consecutive_errors > kReconnectErrorThreshold) {
      consecutive_errors = 0;
      glog()->warn(
        "[Gimbal] 连续 {} 次解析失败（收到字节但凑不出合法帧），尝试重连串口。"
        "这类现象通常是波特率不对或 TX/RX 接反。", kReconnectErrorThreshold);
      reconnect();
      continue;
    }

    // ── 2) 批量读串口流（一次读多字节，减少系统调用和锁竞争）──────────────────
    std::size_t nread = 0;
    std::size_t new_start = 0;   // 本批新字节在 stream_buf 里的起点
    bool serial_open = false;
    bool echo_detected = false;
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      try {
        serial_open = serial_.isOpen();
        if (serial_open) {
          const std::size_t avail = serial_.available();
          if (avail > 0) {
            if (stream_len == kStreamBufferSize) {
              // 缓冲满了说明长期没同步上，保留末尾一帧的窗口避免丢帧头
              std::memmove(
                stream_buf, stream_buf + kStreamBufferSize - kTelemetryFrameSize,
                kTelemetryFrameSize);
              stream_len = kTelemetryFrameSize;
            }
            const std::size_t room = kStreamBufferSize - stream_len;
            new_start = stream_len;
            nread = serial_.read(stream_buf + stream_len, std::min(avail, room));
            stream_len += nread;

            // ── 回环自检 ───────────────────────────────────────────────────
            // 收到的字节和我们刚写出去的那一帧逐字节相同 ⇒ 转接板的 TX 和 RX
            // 被短接/自环，或者我们把自己发的帧又读了回来。这种情况下下位机
            // 一个字节都没参与，光看"有数据"会误判链路已通。
            if (
              has_tx_frame_ && nread >= kCtrlFrameSize &&
              std::memcmp(stream_buf + new_start, last_tx_frame_.data(), kCtrlFrameSize) == 0) {
              echo_detected = true;
            }
          }
        }
      } catch (const std::exception & e) {
        glog()->warn("[Gimbal] 读串口异常: {} (port={})", e.what(), resolved_port_);
      }
    }

    if (!serial_open) {
      glog()->warn("[Gimbal] 串口未打开，尝试重连...");
      reconnect();
      continue;
    }

    // ── 3) 这一批没读到字节：做静默诊断，绝不无脑重连 ────────────────────────
    if (nread == 0) {
      const auto silent_ms = (loop_now_ns - last_any_byte_ns) / 1000000;

      if (!silent_warned && silent_ms >= kSilentWarnMs) {
        silent_warned = true;
        last_silent_report_ns = loop_now_ns;
        glog()->warn(
          "[Gimbal] 串口已打开，但连续 {} ms 收不到任何字节（已发 {} 帧, baud={}, port={}）。\n"
          "  ⇒ 下位机一个字节都没回，不是我们解析出错。按顺序查：\n"
          "  1) 下位机是否上电、是否烧的这套固件（本仓库源码：自瞄通道 = USART1，"
          "PB7=RX / PA9=TX，注意不是常见的 PA9/PA10 那组）；\n"
          "  2) PC-TX 是否接到板子的 USART1_RX(PB7)、GND 是否共地（UART 必须共地）；\n"
          "  3) 波特率：源码 huart1 是 921600 8N1（robot_cmd.c 注释里的 460800 是旧值）；\n"
          "  4) 端口是否选错：上电顺序变了会让 ttyUSB0/ttyUSB1 对调，看上面\"本机串口候选\"；\n"
          "  5) 把转接板自己的 TX-RX 短接做回环：若日志出现 rx_echo 说明转接板和本程序正常，"
          "问题在板子那一端；\n"
          "  6) 若 rx_bytes 在涨、errors 也在涨 ⇒ 收到了字节但解不出帧：波特率/线序/协议版本不对。",
          silent_ms, tx_frames_.load(), baudrate_, resolved_port_);
      } else if (silent_warned && loop_now_ns - last_silent_report_ns >= kSilentReportMs * 1000000LL) {
        last_silent_report_ns = loop_now_ns;
        const bool node_present = std::filesystem::exists(resolved_port_);
        glog()->info(
          "[Gimbal] 仍然无数据: 已静默 {:.1f}s, rx_bytes={}, 设备节点={}, 已发 {} 帧",
          silent_ms / 1000.0, rx_bytes_.load(), node_present ? "还在" : "消失", tx_frames_.load());

        if (!node_present) {
          glog()->warn("[Gimbal] 串口设备节点 {} 已消失（被拔掉？），重连...", resolved_port_);
          reconnect();
          continue;
        }
        if (silent_ms >= kSilentReopenMs && !reopened_after_silence) {
          // 一个字节都没收到过 + 静默 30s：可能是转接板/驱动卡死，重开一次句柄试试
          reopened_after_silence = true;
          glog()->warn(
            "[Gimbal] 静默超过 {}s 且从未收到任何字节，强制重开串口一次以排除驱动/转接板卡死。",
            kSilentReopenMs / 1000);
          reconnect();
          continue;
        }
      }

      std::this_thread::sleep_for(std::chrono::microseconds(200));
      continue;
    }

    // ── 4) 有字节到达 ───────────────────────────────────────────────────────
    last_any_byte_ns = loop_now_ns;
    silent_warned = false;
    rx_bytes_.fetch_add(nread);

    if (!first_bytes_logged) {
      first_bytes_logged = true;
      const auto head_len = std::min<std::size_t>(nread, kFirstBytesHexMax);
      glog()->info(
        "[Gimbal] 首次收到字节 {} 个: {}{}", nread, hex_dump(stream_buf + new_start, head_len),
        nread > head_len ? " ..." : "");
    }

    if (echo_detected && !echo_warned) {
      echo_warned = true;
      glog()->warn(
        "[Gimbal] rx_echo: 收到的字节与本端刚发出的下行帧完全相同 ⇒ TX/RX 被短接或自环，"
        "下位机并没有回帧。检查转接板是否自环、是否把转接板 TX 接到了自己的 RX。");
    }

    // ── 5) 在流缓冲里逐帧解析 ───────────────────────────────────────────────
    // 0xFF 是很常见的字节，所以"找到帧头"远不足以认帧：必须同时满足
    // 帧尾 == 0x0D 且 crc8 正确，否则只滑 1 字节继续找。
    while (stream_len > 0) {
      std::size_t head_pos = stream_len;
      for (std::size_t i = 0; i < stream_len; ++i) {
        if (stream_buf[i] == kFrameHead) {
          head_pos = i;
          break;
        }
      }

      if (head_pos == stream_len) {
        // 一个帧头都没有：全部丢弃
        stream_len = 0;
        break;
      }

      if (head_pos > 0) {
        std::memmove(stream_buf, stream_buf + head_pos, stream_len - head_pos);
        stream_len -= head_pos;
      }

      if (stream_len < kTelemetryFrameSize) break;  // 帧不完整，等下一批字节

      const uint8_t * frame = stream_buf;
      const bool tail_ok = frame[kTelemetryFrameSize - 1] == kFrameTail;
      const bool crc_ok = crc8(frame, kTelemetryFrameSize - 2) == frame[kTelemetryFrameSize - 2];

      if (tail_ok && (crc_ok || !crc8_check_)) {
        if (handle_frame(frame)) {
          if (consecutive_errors > 0) {
            glog()->info("[Gimbal] 重新同步成功（此前连续 {} 次失败）", consecutive_errors);
          }
          std::memmove(stream_buf, stream_buf + kTelemetryFrameSize, stream_len - kTelemetryFrameSize);
          stream_len -= kTelemetryFrameSize;
          consecutive_errors = 0;
          continue;
        }
      }

      // 伪帧头 / 校验失败 / 数据不合理：计一次错误，滑动 1 字节继续同步
      consecutive_errors++;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        error_count_++;
      }

      if (consecutive_errors <= 5) {
        glog()->warn(
          "[Gimbal] 疑似坏帧: head={:#04x}, tail={:#04x}(应为 {:#04x}), crc8={:#04x}->{:#04x}, "
          "dump={}",
          static_cast<int>(frame[0]), static_cast<int>(frame[kTelemetryFrameSize - 1]),
          static_cast<int>(kFrameTail), static_cast<int>(frame[kTelemetryFrameSize - 2]),
          static_cast<int>(crc8(frame, kTelemetryFrameSize - 2)),
          hex_dump(frame, kTelemetryFrameSize));
      } else if (consecutive_errors == 50) {
        // 到这一步已经确认"有字节、但凑不出帧" —— 这是波特率/线序问题的典型指纹
        glog()->warn(
          "[Gimbal] 已连续 50 次解析失败但确实收到了字节（rx_bytes={}）：高度怀疑波特率不匹配"
          "（配置 {}，下位机源码 921600）或 TX/RX 接反。", rx_bytes_.load(), baudrate_);
      }

      // 定期清空积压数据，帮助快速重新同步（降低频率避免阻塞）
      if (consecutive_errors % 50 == 0) {
        try {
          std::lock_guard<std::mutex> lock(serial_mutex_);
          serial_.flushInput();
          glog()->info("[Gimbal] 已清空接收缓冲以重新同步");
        } catch (const std::exception & e) {
          glog()->warn("[Gimbal] 清空接收缓冲失败: {}", e.what());
        }
      }

      std::memmove(stream_buf, stream_buf + 1, stream_len - 1);
      stream_len -= 1;
    }
  }

  glog()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  constexpr int max_retry_count = 15;

  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    glog()->warn("[Gimbal] 重连串口 {}/{} (port={})", i + 1, max_retry_count, resolved_port_);

    // 1) 关旧句柄：锁内只做系统调用，绝不在持锁时 sleep。
    //    旧实现持 serial_mutex_ 睡了 500ms，发送线程被一起冻住，
    //    日志里那条 "High TX latency: 551ms" 就是这么来的。
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      try {
        if (serial_.isOpen()) {
          serial_.flushInput();
          serial_.flushOutput();
          serial_.close();
        }
      } catch (const std::exception & e) {
        glog()->warn("[Gimbal] 关闭串口时异常: {}", e.what());
      }
    }

    // 2) 锁外等 500ms，给 USB 设备/转接板复位时间
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (quit_) break;

    // 3) 重新解析端口：拔插后节点名可能变（ttyUSB0 <-> ttyUSB1），不重新解析就永远打不开
    const auto new_port = resolve_serial_port(configured_port_);
    if (new_port != resolved_port_) {
      glog()->warn("[Gimbal] 端口节点已变化: {} -> {}", resolved_port_, new_port);
      resolved_port_ = new_port;
    }

    try {
      {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        serial_.setPort(resolved_port_);
        serial_.open();
        serial_.setBaudrate(baudrate_);
        serial::Timeout timeout = make_timeout();  // 重新配置超时（与初始化一致）
        serial_.setTimeout(timeout);
      }

      // 4) 打开后锁外等 100ms 让 USB 枚举稳定，再丢掉可能收到的半帧
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        serial_.flushInput();
        serial_.flushOutput();
      }

      queue_.clear();  // 重启前的旧姿态已经过期，别让上层拿去过期数据
      glog()->info("[Gimbal] 重连成功: {}", probe_serial_port(resolved_port_).str());
      return;
    } catch (const std::exception & e) {
      glog()->warn("[Gimbal] 重连失败: {} (port={})", e.what(), resolved_port_);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  glog()->error("[Gimbal] 重连 {} 次全部失败：串口不可用，数据链路已断。", max_retry_count);
}


}  // namespace io
