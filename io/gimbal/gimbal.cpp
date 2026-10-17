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
#include <filesystem>
#include <vector>


#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace io
{
namespace
{
// 打开后打印每一帧的 hex（调试用；线上量很小，默认关闭）
constexpr bool kVerboseSerialHexLog = false;

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
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");  // 配置里的串口设备
  // 可选键：波特率 / 是否校验 crc8（配置里没有时用默认值，不报错）
  baudrate_ = tools::read<int>(yaml, "baudrate", kDefaultBaudrate);
  crc8_check_ = tools::read<bool>(yaml, "crc8_check", true);

  auto resolved_port = resolve_serial_port(com_port);
  if (resolved_port != com_port) {
    tools::logger()->warn(
      "[Gimbal] Configured com_port '{}' not found, fallback to '{}'.", com_port, resolved_port);
  }

  try {
    serial_.setPort(resolved_port);
    serial_.setBaudrate(baudrate_);  // 921600 8N1（下位机 USART1）

    // 低延迟优先：42 字节 @921600 约 0.46 ms，50ms 总超时/5ms 字节间超时都很宽裕
    serial::Timeout timeout = make_timeout();  // setTimeout 形参是非 const 引用，必须是左值
    serial_.setTimeout(timeout);

    serial_.open();

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
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);
  tx_thread_ = std::thread(&Gimbal::tx_thread, this);

  // 下位机是「请求-响应式」：只有收到一帧 UsbCtrl 才会回发一帧遥测。
  // 所以这里必须先主动发空帧（mode=0，不控制）去敲门，再限时等待首帧姿态；
  // 否则 queue_.pop() 会永久阻塞在"永远收不到第一帧"上（v2.0 的老问题）。
  auto wait_start = std::chrono::steady_clock::now();
  bool got_first_frame = false;
  while (!quit_) {
    if (!queue_.empty()) {
      got_first_frame = true;
      break;
    }
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - wait_start)
                        .count();
    if (elapsed_ms >= kFirstFrameTimeoutMs) break;

    VisionToGimbal idle;  // mode = 0：不控制，只为触发下位机回帧
    enqueue_cmd(idle);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  if (got_first_frame) {
    tools::logger()->info("[Gimbal] First q received.");
  } else {
    tools::logger()->warn(
      "[Gimbal] No telemetry frame within {}ms (baud={}, crc8_check={}). "
      "检查：1) 下位机 USART1 波特率是否为 921600；2) 协议是否为 0xFF/0x0D 42 字节新版；"
      "3) 若下位机不计算 crc8，请在 yaml 里设 crc8_check: false。",
      kFirstFrameTimeoutMs, baudrate_, crc8_check_ ? "true" : "false");
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
  if (kVerboseSerialHexLog) {
    tools::logger()->debug(
      "[Gimbal] tx {} bytes: {}",
      kCtrlFrameSize, hex_dump(reinterpret_cast<const uint8_t *>(&frame), kCtrlFrameSize));
  }

  auto tx_start = std::chrono::steady_clock::now();
  bool ok = true;
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    try {
      if (!serial_.isOpen()) {
        ok = false;
      } else {
        serial_.write(reinterpret_cast<const uint8_t *>(&frame), kCtrlFrameSize);
      }
    } catch (const std::exception & e) {
      ok = false;
      tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
    }
  }

  auto tx_end = std::chrono::steady_clock::now();
  double tx_latency = std::chrono::duration<double, std::milli>(tx_end - tx_start).count();

  if (ok) {
    auto old = tx_latency_ms_.load();
    tx_latency_ms_.store(old > 0.0 ? 0.9 * old + 0.1 * tx_latency : tx_latency);
    if (tx_latency > 5.0) {
      tools::logger()->warn("[Gimbal] High TX latency: {:.2f}ms", tx_latency);
    }
  }
}

void Gimbal::tx_thread()
{
  tools::logger()->info("[Gimbal] tx_thread started.");

  auto last_tx = std::chrono::steady_clock::now();

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
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  tools::logger()->info("[Gimbal] tx_thread stopped.");
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
    tools::logger()->debug("[Gimbal] rx {} bytes: {}", kTelemetryFrameSize, hex_dump(buffer, kTelemetryFrameSize));
  }

  // 定期打印诊断信息（降低日志频率）
  static int diag_counter = 0;
  if (++diag_counter >= 200) {
    diag_counter = 0;
    auto diag = diagnostics();
    tools::logger()->info(
      "[Gimbal] Diag: {:.1f}Hz, frame={}, error={}, latency={:.2f}ms, serial_ok={}", diag.rx_frame_rate,
      diag.frame_count, diag.error_count, diag.rx_latency_ms, diag.serial_ok ? "yes" : "no");
  }

  return true;
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");

  uint8_t stream_buf[kStreamBufferSize] = {0};
  std::size_t stream_len = 0;
  int consecutive_errors = 0;
  int no_data_count = 0;

  while (!quit_) {
    if (consecutive_errors > kReconnectErrorThreshold) {
      consecutive_errors = 0;
      tools::logger()->warn(
        "[Gimbal] Too many consecutive errors (>{}), attempting to reconnect...",
        kReconnectErrorThreshold);
      reconnect();
      continue;
    }

    // 1) 批量读取串口流（减少锁竞争和系统调用次数）
    std::size_t nread = 0;
    bool serial_open = false;
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      try {
        serial_open = serial_.isOpen();
        if (serial_open) {
          std::size_t avail = serial_.available();
          if (avail > 0) {
            if (stream_len == kStreamBufferSize) {
              // 缓冲满了说明长期没同步上，保留末尾一帧的窗口避免丢帧头
              std::memmove(
                stream_buf, stream_buf + kStreamBufferSize - kTelemetryFrameSize,
                kTelemetryFrameSize);
              stream_len = kTelemetryFrameSize;
            }
            std::size_t room = kStreamBufferSize - stream_len;
            nread = serial_.read(stream_buf + stream_len, std::min(avail, room));
            stream_len += nread;
          }
        }
      } catch (const std::exception & e) {
        tools::logger()->warn("[Gimbal] Failed to read serial stream: {}", e.what());
      }
    }

    if (!serial_open) {
      tools::logger()->warn("[Gimbal] Serial not open, reconnecting...");
      reconnect();
      continue;
    }

    if (nread == 0) {
      no_data_count++;
      if (no_data_count > 4000) {
        no_data_count = 0;
        tools::logger()->warn("[Gimbal] No serial data for extended period, triggering reconnect");
        reconnect();
        continue;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(200));
      continue;
    }
    no_data_count = 0;

    // 2) 在流缓冲里逐帧解析：0xFF 是常见字节，所以"找到帧头"还不够，
    //    必须同时满足 帧尾 == 0x0D 且 crc8 正确才认帧；否则只滑 1 字节继续找。
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
        // 把帧头挪到缓冲区开头
        std::memmove(stream_buf, stream_buf + head_pos, stream_len - head_pos);
        stream_len -= head_pos;
      }

      if (stream_len < kTelemetryFrameSize) break;  // 帧不完整，等下一批字节

      const uint8_t * frame = stream_buf;
      bool tail_ok = frame[kTelemetryFrameSize - 1] == kFrameTail;
      bool crc_ok =
        crc8(frame, kTelemetryFrameSize - 2) == frame[kTelemetryFrameSize - 2];

      if (tail_ok && (crc_ok || !crc8_check_)) {
        if (handle_frame(frame)) {
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
        tools::logger()->warn(
          "[Gimbal] Bad frame candidate: head={:#04x}, tail={:#04x}(expect {:#04x}), "
          "crc8={:#04x}->{:#04x}, dump: {}",
          static_cast<int>(frame[0]), static_cast<int>(frame[kTelemetryFrameSize - 1]),
          static_cast<int>(kFrameTail), static_cast<int>(frame[kTelemetryFrameSize - 2]),
          static_cast<int>(crc8(frame, kTelemetryFrameSize - 2)),
          hex_dump(frame, kTelemetryFrameSize));
      }

      // 定期清空积压数据，帮助快速重新同步（降低频率避免阻塞）
      if (consecutive_errors % 50 == 0) {
        try {
          std::lock_guard<std::mutex> lock(serial_mutex_);
          serial_.flushInput();
          tools::logger()->info("[Gimbal] Flushed input buffer to resync");
        } catch (const std::exception & e) {
          tools::logger()->warn("[Gimbal] Flush failed: {}", e.what());
        }
      }

      std::memmove(stream_buf, stream_buf + 1, stream_len - 1);
      stream_len -= 1;
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 15;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      if (serial_.isOpen()) {
        serial_.flushInput();
        serial_.flushOutput();
        serial_.close();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } catch (...) {
    }

    try {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      serial_.open();
      serial_.setBaudrate(baudrate_);
      serial::Timeout timeout = make_timeout();  // 重新配置超时（与初始化一致）
      serial_.setTimeout(timeout);

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      serial_.flushInput();
      serial_.flushOutput();

      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }
}

}  // namespace io
