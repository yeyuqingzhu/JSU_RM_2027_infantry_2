#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{
/*==============================================================================
 * 云台串口协议 v3.0（适配 lt-2027 哨兵下位机，取代原 v2.0 私有协议）
 * ============================================================================
 * 【为什么换协议】本项目原 v2.0 是自定帧（HEAD 0x5A 0xA5 / TAIL 0x7F 0xFE，
 * 43B/29B，弧度制，下行发"相对增量"）。2027 赛季改用湖大电控组的哨兵下位机
 * （参考仓库 modules/master_machine/seasky_protocol.c + master_process.c），
 * 该下位机的自瞄通道是另一套线上格式，因此本文件整体重写为与之一一对应。
 *
 * 【线格式】两帧均以 0xFF 为帧头、0x0D 为帧尾，均带 crc8 校验字节，
 *           全部为小端（x86 / ARM 均为小端，直接 memcpy）。
 *
 *   云台 → 视觉（上位机），42 字节 UsbAutoSendToNuc（对应下位机
 *   USB_AUTO_SEND_TO_NUC_DATA_t / USB_SEND_TO_NUC_SIZE）：
 *     [0]    head            0xFF
 *     [1]    mode            0:空闲 1:自瞄 2:小符 3:大符
 *     [2]    float q[4]      w,x,y,z（下位机已按"与上位机约定: xy交换"做过轴交换）
 *     [18]   float yaw       世界系 yaw，单位【度】
 *     [22]   float yaw_vel   单位【度/秒】
 *     [26]   float pitch     单位【度】
 *     [30]   float pitch_vel 单位【度/秒】
 *     [34]   float bullet_speed  单位 m/s
 *     [38]   uint16 bullet_count 累计发弹数
 *     [40]   uint8 crc8      前 40 字节的 crc8
 *     [41]   tail            0x0D
 *
 *   视觉 → 云台，28 字节 UsbCtrl（对应下位机 USB_CTRL / USB_CTRL_SIZE）：
 *     [0]    head            0xFF
 *     [1]    mode            0:不控制 1:控制云台不开火 2:控制云台且开火
 *     [2]    float yaw       目标 yaw，单位【度】，【绝对角】
 *     [6]    float yaw_vel   单位【度/秒】
 *     [10]   float yaw_acc   单位【度/秒²】
 *     [14]   float pitch     目标 pitch，单位【度】，【绝对角】
 *     [18]   float pitch_vel 单位【度/秒】
 *     [22]   float pitch_acc 单位【度/秒²】
 *     [26]   uint8 crc8      前 26 字节的 crc8
 *     [27]   tail            0x0D
 *
 * 【单位约定】线上一切角度都是【度】（原 v2.0 是弧度）。本层对外（`send()` /
 * `state()`）仍保持 `tools/math_tools.hpp` 的弧度制约定，度↔弧度的换算只发生在
 * 本文件的打包/解包处，上层（planner / solver / buff_aimer）完全无感。
 *
 * 【绝对角约定】下行 yaw/pitch 是【世界系绝对角】，下位机直接
 * `big_yaw_target = yaw * DEGREE_2_RAD`、`pitch_target = GimbalLimitPitch(pitch)`
 * （原 v2.0 是"相对增量"，由本层内部用 `目标 - 当前` 得到）。因此本层不再做差值，
 * 上层 planner 给出的绝对角原样换算成度发出即可。
 *
 * 【请求-响应式链路】下位机只在 `DecodeUsb()` 成功解析到一帧 UsbCtrl 之后才回发一帧
 * 遥测（`UsbVisionSend()`）。即：**上位机不发帧，就永远收不到姿态**。所以：
 *   1) 构造函数启动线程后会先发空帧（mode=0）去"敲门"，再限时等待首帧姿态；
 *   2) `tx_thread()` 带空闲心跳：上层超过 100 ms 没调用 `send()`，就自动以 20 Hz 补发
 *      mode=0 的空帧——既保持链路/姿态回传，又保证下位机不会继续执行过期控制量。
 *
 * 【串口参数】921600 8N1（下位机 Src/usart.c 中 `huart1.Init.BaudRate = 921600`；
 * 参考仓库 robot_cmd.c 里的注释 460800 是错的）。可用 yaml 的 `baudrate` 覆盖。
 * 其余可选键：`crc8_check`（默认 true，关闭后只校验帧头帧尾）。
 *
 * 【crc8】与下位机 modules/algorithm/crc8.c 完全一致：init 0x00、多项式 0x31、
 * 高位优先（等价于 SHT75/Dallas 查表实现），用位运算实现，避免 256 字节表。
 * ==============================================================================
 */

// 帧定界与帧长（与下位机 seasky_protocol.h 中宏一一对应）
constexpr uint8_t kFrameHead = 0xFF;      // USB_PROTOCOL_CMD_ID
constexpr uint8_t kFrameTail = 0x0D;      // USB_FRAME_TAIL
constexpr std::size_t kCtrlFrameSize = 28;       // USB_CTRL_SIZE
constexpr std::size_t kTelemetryFrameSize = 42;  // USB_SEND_TO_NUC_SIZE
constexpr int kDefaultBaudrate = 921600;

// 默认（可被 yaml 覆盖）运行参数
constexpr int kSerialTimeoutMs = 50;         // 单次读总超时
constexpr int kSerialInterByteMs = 5;        // 字节间超时
constexpr int kReconnectErrorThreshold = 100;  // 连续解析错误超过此值则重连
constexpr int kIdleHeartbeatMs = 100;        // 上层静默多久后开始补发空帧
constexpr int kFirstFrameTimeoutMs = 2000;   // 构造期等待首帧姿态的上限

/**
 * @brief 计算 crc8（init 0x00 / poly 0x31 / 高位优先），与下位机 crc8.c 一致。
 * @param data 待校验数据首地址，可为空（size==0 时直接返回 0）。
 * @param size 字节数。
 * @return uint8_t 校验值。
 */
uint8_t crc8(const uint8_t * data, std::size_t size);

/**
 * @brief 视觉 → 云台的下行 28 字节线帧（对应下位机 USB_CTRL）。
 * @note 字段单位是【度】，不是弧度；本层由 `Gimbal::pack_ctrl()` 从弧度制控制量
 *       换算而来，上层不要直接构造它。
 * @note `packed` + 直接 memcpy 到串口；x86/ARM 都是小端，与下位机一致。
 */
struct __attribute__((packed)) UsbCtrl
{
  uint8_t head = kFrameHead;  // 0xFF 帧头
  uint8_t mode = 0;           // 0:不控制 1:控制云台不开火 2:控制云台且开火
  float yaw = 0.0f;           // 目标 yaw，度，绝对角
  float yaw_vel = 0.0f;       // deg/s，前馈
  float yaw_acc = 0.0f;       // deg/s²，前馈
  float pitch = 0.0f;         // 目标 pitch，度，绝对角
  float pitch_vel = 0.0f;     // deg/s，前馈
  float pitch_acc = 0.0f;     // deg/s²，前馈
  uint8_t crc8 = 0;           // 前 26 字节的 crc8
  uint8_t tail = kFrameTail;  // 0x0D 帧尾
};

static_assert(sizeof(UsbCtrl) == kCtrlFrameSize, "UsbCtrl 必须是 28 字节的线帧");

/**
 * @brief 云台 → 视觉的上行 42 字节线帧（对应下位机 USB_AUTO_SEND_TO_NUC_DATA_t）。
 * @note 字段单位是【度】；`Gimbal::read_thread()` 收到后换算成弧度写入 `GimbalState`。
 * @note q 的顺序是 w,x,y,z，但下位机在 `UsbVsioionSetAltiitude()` 里做过
 *       "与上位机约定: xy交换"（`q[1]=q_in[2]; q[2]=q_in[1];`），
 *       即线上顺序就是视觉端期望的顺序，本层【不要再交换回来】。
 */
struct __attribute__((packed)) UsbAutoSendToNuc
{
  uint8_t head = kFrameHead;   // 0xFF 帧头
  uint8_t mode = 0;            // 0:空闲 1:自瞄 2:小符 3:大符
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w,x,y,z
  float yaw = 0.0f;            // 世界系 yaw，度
  float yaw_vel = 0.0f;        // deg/s
  float pitch = 0.0f;          // 度
  float pitch_vel = 0.0f;      // deg/s
  float bullet_speed = 0.0f;   // m/s
  uint16_t bullet_count = 0;   // 累计发弹数
  uint8_t crc8 = 0;            // 前 40 字节的 crc8
  uint8_t tail = kFrameTail;   // 0x0D 帧尾
};

static_assert(sizeof(UsbAutoSendToNuc) == kTelemetryFrameSize, "UsbAutoSendToNuc 必须是 42 字节的线帧");

/**
 * @brief 视觉 → 云台的控制量（【逻辑帧】，不是线帧）：弧度制、绝对角。
 *
 * 这是上层唯一需要构造的东西（`planner` 的 `Plan` 直接喂给 `Gimbal::send()` 即可）。
 * 单位与 `tools/math_tools.hpp` 一致：yaw/pitch 为 rad，vel 为 rad/s，acc 为 rad/s²；
 * yaw/pitch 是【世界系绝对角】，与 `GimbalState::yaw/pitch` 同一零点。
 * 打包成 28 字节度制线帧的工作由 `Gimbal` 内部完成。
 */
struct VisionToGimbal
{
  uint8_t mode = 0;    // 0: 不控制 1: 控制云台但不开火 2: 控制云台且开火
  float yaw = 0.0f;    // rad，绝对角
  float yaw_vel = 0.0f;  // rad/s
  float yaw_acc = 0.0f;  // rad/s²
  float pitch = 0.0f;    // rad，绝对角
  float pitch_vel = 0.0f;  // rad/s
  float pitch_acc = 0.0f;  // rad/s²
};

enum class GimbalMode
{
  IDLE,        // 空闲
  AUTO_AIM,    // 自瞄
  SMALL_BUFF,  // 小符
  BIG_BUFF     // 大符
};

struct GimbalState
{
  float yaw;           // rad，世界系绝对角（由线上度制换算）
  float yaw_vel;       // rad/s
  float pitch;         // rad
  float pitch_vel;     // rad/s
  float bullet_speed;  // m/s
  uint16_t bullet_count;
};

// 诊断结构：用于监测串口实时性和可靠性
struct GimbalDiagnostics
{
  uint32_t frame_count = 0;  // 累计收到的有效帧数
  uint32_t error_count = 0;  // 累计解析/校验失败次数
  double rx_frame_rate = 0.0;  // 接收帧率 (Hz)，指数滑动平均
  double rx_latency_ms = 0.0;  // 距上一次成功接收的时长 (ms)
  double tx_latency_ms = 0.0;  // 写串口耗时 (ms)，指数滑动平均
  int queue_depth = 0;         // 待发送队列深度
  bool serial_ok = false;      // 串口是否打开
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path);

  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  GimbalDiagnostics diagnostics() const;  // 实时诊断信息
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  /**
   * @brief 发送控制量（弧度、绝对角）。
   * @param control false 表示无目标（mode=0，下位机不接管云台）。
   * @param fire true 时 mode=2（下位机允许开火），否则 mode=1。
   * @note 内部做 rad→deg 后打包成 28 字节 UsbCtrl；只入队，不阻塞在串口写上。
   */
  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);

  /**
   * @brief 发送控制量（逻辑帧版本，字段为弧度、绝对角）。
   * @note `mode == 0` 时 yaw/pitch 会被清零（下位机忽略这两个字段）。
   */
  void send(io::VisionToGimbal cmd);

private:
  serial::Serial serial_;

  std::thread thread_;
  std::thread tx_thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;
  mutable std::mutex serial_mutex_;

  UsbAutoSendToNuc rx_data_;  // 云台到视觉，视觉是 receive
  tools::ThreadSafeQueue<VisionToGimbal, true> tx_queue_{128};
  std::atomic<int> tx_backlog_{0};  // 队列深度（供 diagnostics 使用）

  GimbalMode mode_ = GimbalMode::IDLE;  // 默认空闲
  GimbalState state_{};                 // 云台状态（弧度制）
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>, true>
    queue_{5000};  // 姿态插值队列，增大至 5000 以应对高速连发

  std::atomic<std::int64_t> last_rx_ok_ns_{0};

  // 以下统计量由 mutex_ 保护（与 state_ 同锁，避免再加一把锁）
  uint32_t frame_count_ = 0;
  uint32_t error_count_ = 0;
  double rx_frame_rate_ = 0.0;
  std::int64_t last_frame_ns_ = 0;
  std::atomic<double> tx_latency_ms_{0.0};  // 写串口耗时，由写线程单独维护

  // 配置
  int baudrate_ = kDefaultBaudrate;
  bool crc8_check_ = true;

  void read_thread();  // 读取线程
  void tx_thread();    // 发送线程
  void reconnect();    // 重新连接

  // 入队一条控制量（维护 tx_backlog_ 深度计数）
  void enqueue_cmd(const VisionToGimbal & cmd);

  // 一次性写完 28 字节线帧（含加锁、异常处理、tx 延迟统计）
  void write_ctrl(const UsbCtrl & frame);

  // 逻辑帧（弧度、绝对角）→ 线帧（度、绝对角），含 crc8
  UsbCtrl pack_ctrl(const VisionToGimbal & cmd) const;

  // 校验并解析一帧 42 字节上行遥测；成功则更新 state_/mode_/queue_ 并返回 true
  bool handle_frame(const uint8_t * buffer);
};

}  // namespace io

#endif  // IO__GIMBAL_HPP
