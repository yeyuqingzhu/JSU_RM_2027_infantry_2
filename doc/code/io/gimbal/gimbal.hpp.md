# io/gimbal/gimbal.hpp

**概要**：云台串口通信协议 **v3.0**（2027 赛季 lt-2027 哨兵下位机）的对外接口声明——帧常量与 `crc8()`、两个 `packed` 线格式结构体（视觉→云台 28 字节 `UsbCtrl`、云台→视觉 42 字节 `UsbAutoSendToNuc`）、**逻辑帧** `VisionToGimbal`（无帧界，纯控制量）、模式/状态/诊断三个辅助结构，以及 `Gimbal` 类（读/发两条线程 + 姿态时间戳队列 + 互斥保护的状态快照）。**对外 API 一律弧度制、绝对角**；「度制」只存在于线格式结构体的字段里，换算点只有 `Gimbal::pack_ctrl()` 与 `Gimbal::handle_frame()`。

## 协议常量（头文件）

| 常量 | 值 | 含义 |
| --- | --- | --- |
| `kFrameHead` | `0xFF` | 两个方向的帧头 |
| `kFrameTail` | `0x0D` | 两个方向的帧尾（`USB_FRAME_TAIL`） |
| `kCtrlFrameSize` | 28 | `UsbCtrl` 字节数（`USB_CTRL_SIZE`） |
| `kTelemetryFrameSize` | 42 | `UsbAutoSendToNuc` 字节数（`USB_SEND_TO_NUC_SIZE`） |
| `kDefaultBaudrate` | 921600 | 下位机 `Src/usart.c` 里 `huart1` 的实际波特率 |
| `kSerialTimeoutMs` | 50 | `serial::Timeout` 总超时 |
| `kSerialInterByteMs` | 5 | 字节间超时（切帧用） |
| `kReconnectErrorThreshold` | 100 | 连续解析失败超过此值即重连 |
| `kIdleHeartbeatMs` | 100 | 上层静默超过此值后补发 `mode=0` 心跳（20 Hz） |
| `kFirstFrameTimeoutMs` | 2000 | 构造期等首帧的上限（超时只 warn） |

### `uint8_t crc8(const uint8_t * data, std::size_t size)`

- **作用**：与下位机 `modules/algorithm/crc8.c`（`sht75_crc_table` 查表法）逐位等价的 CRC-8：init `0x00`、poly `0x31`（反射）、高位优先、无 final xor。
- **参数**：`data` 起始地址；`size` 字节数（发送端传 26 / 40，即「整帧减去 crc8 与 tail」）。
- **返回**：校验值。
- **备注**：位实现而非 256 项查表，避免在头文件里塞常量表；`crc8({0x01}) == 0x31` 可作为自测点（`proto_test`（离线构建，见 `build/gimbal_verify/proto_test.cpp`，不入库）已用下位机表前 16 项核对）。

## 结构体

### `struct __attribute__((packed)) UsbCtrl`（28 字节，视觉 → 云台）

| 偏移 | 字段 | 类型 | 含义 |
| --- | --- | --- | --- |
| 0 | `head` | `uint8_t` | `0xFF` |
| 1 | `mode` | `uint8_t` | 0 不控制 / 1 控制云台不开火 / 2 控制云台且开火（**注意**：上行遥测的 mode 是另一套语义 0/1/2/3） |
| 2 | `yaw` | `float` | 目标 yaw **绝对角，度** |
| 6 | `yaw_vel` | `float` | 前馈角速度，**度/秒** |
| 10 | `yaw_acc` | `float` | 前馈角加速度，**度/秒²** |
| 14 | `pitch` | `float` | 目标 pitch **绝对角，度**（不取反） |
| 18 | `pitch_vel` | `float` | 度/秒 |
| 22 | `pitch_acc` | `float` | 度/秒² |
| 26 | `crc8` | `uint8_t` | 对前 26 字节的 CRC-8 |
| 27 | `tail` | `uint8_t` | `0x0D` |

- **备注**：`static_assert(sizeof(UsbCtrl) == kCtrlFrameSize)` 把长度钉死，字段一改就编译报错。小端序 `memcpy` 直传，与下位机 `usb_memory_from_buffer` 的字节序一致。

### `struct __attribute__((packed)) UsbAutoSendToNuc`（42 字节，云台 → 视觉）

| 偏移 | 字段 | 类型 | 含义 |
| --- | --- | --- | --- |
| 0 | `head` | `uint8_t` | `0xFF` |
| 1 | `mode` | `uint8_t` | 0 空闲 / 1 自瞄 / 2 小符 / 3 大符 |
| 2 | `q[4]` | `float` | IMU 四元数，**wxyz** 顺序（下位机已做 xy 交换，视觉侧按 w,x,y,z 直接读） |
| 18 | `yaw` | `float` | 世界系 yaw，**度** |
| 22 | `yaw_vel` | `float` | 度/秒 |
| 26 | `pitch` | `float` | muzzle pitch，**度**（下位机乘了 `TONGJI_PITCH_UPLINK_SIGN = -1`，已折算到视觉约定） |
| 30 | `pitch_vel` | `float` | 度/秒 |
| 34 | `bullet_speed` | `float` | 弹速，m/s |
| 38 | `bullet_count` | `uint16_t` | 累计发弹数 |
| 40 | `crc8` | `uint8_t` | 对前 40 字节的 CRC-8 |
| 41 | `tail` | `uint8_t` | `0x0D` |

- **备注**：同上 `static_assert`。`q` 的默认成员初始化 `{1, 0, 0, 0}` 让「敲门帧」的零初始化不会产生范数为 0 的四元数。

### `struct VisionToGimbal`（**逻辑帧**，非线格式）

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `mode` | `uint8_t` | 0 不控制 / 1 控制不开火 / 2 控制且开火 |
| `yaw` / `pitch` | `float` | 目标**绝对角（rad）** |
| `yaw_vel` / `yaw_acc` | `float` | 偏航前馈（rad/s、rad/s²） |
| `pitch_vel` / `pitch_acc` | `float` | 俯仰前馈（rad/s、rad/s²） |

- **备注**：**没有 head/tail**——它是本层的内部/便捷递交类型，由 `send()` 接收后在 `pack_ctrl()` 里打包 + 换算成度制。保留这个类型名让 `tests/fire_test.cpp` 这类已有调用方无需改动。

## 枚举与辅助结构

| 类型 | 成员 / 字段 |
| --- | --- |
| `enum class GimbalMode` | `IDLE`、`AUTO_AIM`、`SMALL_BUFF`、`BIG_BUFF`（由上行帧 `mode` 0–3 映射，非法值回落 `IDLE` 并告警） |
| `struct GimbalState` | `yaw`、`yaw_vel`、`pitch`、`pitch_vel`、`bullet_speed`、`bullet_count`（**弧度制**） |
| `struct GimbalDiagnostics` | `frame_count`、`error_count`、`rx_frame_rate`、`rx_latency_ms`、`tx_latency_ms`、`queue_depth`、`serial_ok`（v3.0 起**七项全部填充**） |

## 类 `io::Gimbal`

- **职责**：一个 `Gimbal` 对象 = 一条串口 + 一条读线程 + 一条发线程。读线程「流式找 `0xFF` → 取 42B → 校验尾 + crc8 → 更新快照 → 压入 (四元数, 时间戳)」；发线程「取最新控制量 → 打包 28B → 写串口」，并在上层静默时补发心跳。
- **线程模型**：
  - `mutex_` 保护 `mode_` / `state_` 与统计量（`frame_count_`、`error_count_`、`rx_frame_rate_`、`last_frame_ns_`）；
  - `serial_mutex_` 串行化所有串口读写；
  - `queue_`（容量 5000，满时丢最旧）保存 `(Eigen::Quaterniond, time_point)` 供 `q(t)` 插值；
  - `tx_queue_`（容量 128，满时丢最旧）+ `tx_backlog_`（待发条数计数）实现「保新丢旧」的异步发送。
- **典型用法**：
  ```cpp
  io::Gimbal gimbal(config_path);          // 打开串口 + 敲门等首帧（最多 2 s，超时只 warn）
  if (gimbal.mode() == io::GimbalMode::AUTO_AIM) {
    auto gs = gimbal.state();              // 弧度制快照
    Eigen::Quaterniond q = gimbal.q(t);    // 按图像时间戳插值
    gimbal.send(true, fire, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc);  // 弧度、绝对角
  }
  ```
- **备注**：非拷贝、不可重入；`q(t)` 会**消费**队列，必须按时间戳单调递增调用；不调用 `send()` 时发线程仍以 20 Hz 发 `mode=0` 心跳（v2.0 没有心跳，下位机若不支持会有脱控风险——v3.0 的板子是「收到才回」，没有心跳视觉端就收不到姿态）。

### 成员变量

| 成员 | 类型 | 含义 |
| --- | --- | --- |
| `serial_` | `serial::Serial` | 串口对象（vendored libserial） |
| `thread_` / `tx_thread_` | `std::thread` | 读线程 / 发线程 |
| `quit_` | `std::atomic<bool>` | 退出标志 |
| `mutex_` / `serial_mutex_` | `mutable std::mutex` | 状态锁 / 串口锁 |
| `rx_data_` | `UsbAutoSendToNuc` | 最近一次成功解析的原始遥测帧 |
| `tx_queue_` | `ThreadSafeQueue<VisionToGimbal, true>`（128） | 待发逻辑帧，满时丢最旧 |
| `tx_backlog_` | `std::atomic<int>` | 待发条数（`queue_depth` 诊断来源） |
| `mode_` / `state_` | `GimbalMode` / `GimbalState` | 受 `mutex_` 保护的快照 |
| `queue_` | `ThreadSafeQueue<tuple<Quaterniond, time_point>, true>`（5000） | 姿态时间序列 |
| `last_rx_ok_ns_` | `std::atomic<int64_t>` | 最近成功解析时刻（ns），算 `rx_latency_ms` |
| `frame_count_` / `error_count_` / `rx_frame_rate_` / `last_frame_ns_` | 统计量 | 受 `mutex_` 保护；`error_count_` 在伪帧头滑动时会被放大 |
| `tx_latency_ms_` | `std::atomic<double>` | 最近一次写串口耗时（EMA） |
| `baudrate_` / `crc8_check_` | `int` / `bool` | 配置项：`yaml` 的 `baudrate`（默认 921600）、`crc8_check`（默认 true） |

### 私有函数

| 声明 | 说明 |
| --- | --- |
| `bool handle_frame(const uint8_t * frame)` | 解析一帧 42B 遥测：校验合理性与四元数范数 → 更新快照/统计 → 压姿态队列。返回是否接受 |
| `UsbCtrl pack_ctrl(const VisionToGimbal & cmd) const` | 逻辑帧 → 28B 线帧：弧度转度、填 crc8 与帧界 |
| `void enqueue_cmd(const VisionToGimbal & cmd)` | 入 `tx_queue_` 并 `tx_backlog_++` |
| `void write_ctrl(const UsbCtrl & frame)` | 持锁写串口 + 发送延迟统计 |
| `void read_thread()` / `void tx_thread()` / `void reconnect()` | 读线程主循环 / 发线程主循环（含心跳）/ 最多 15 次的重连流程 |

> v2.0 里的 `bool read(uint8_t*, size_t)`（有定义无调用）与 `bool findFrameHead()`（无定义无调用）已在 v3.0 中删除，帧头查找内联在 `read_thread` 的滑动窗口里。
