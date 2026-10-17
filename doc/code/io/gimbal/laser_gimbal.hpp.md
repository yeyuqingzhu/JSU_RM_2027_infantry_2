# io/gimbal/laser_gimbal.hpp

**概要**：Lracking **激光云台**串口协议的接口声明——与 [gimbal.hpp](gimbal.hpp.md) 帧布局相同（HEAD `0x5A 0xA5` / TAIL `0x7F 0xFE`，上行 43 字节、下行 29 字节），但**线上单位是「度」**、且下行 yaw/pitch 是**绝对角**（不是增量）。API 层统一换算成弧度，以与工程其余部分一致。

## 结构体

### `struct __attribute__((packed)) LaserGimbalToVision`（43 字节，STM32 → NUC）

| 字段 | 类型 | 单位 / 含义 |
| --- | --- | --- |
| `head[2]` / `tail[2]` | `uint8_t` | `0x5A 0xA5` / `0x7F 0xFE` |
| `mode` | `uint8_t` | 云台模式（原样透传给上层，本文件不做枚举映射） |
| `q[4]` | `float` | 四元数 wxyz |
| `yaw` / `yaw_vel` | `float` | **度**、**度/秒** |
| `pitch` / `pitch_vel` | `float` | **度**、**度/秒** |
| `bullet_speed` | `float` | 弹速 m/s |
| `bullet_count` | `uint16_t` | 累计发弹数 |

- **备注**：`static_assert(sizeof(LaserGimbalToVision) == 43)` —— 与 `gimbal.hpp` 的 `<= 64` 不同，这里是**精确断言**，字段改动会在编译期报错。

### `struct __attribute__((packed)) LaserVisionToGimbal`（29 字节，NUC → STM32）

| 字段 | 类型 | 单位 / 含义 |
| --- | --- | --- |
| `head[2]` / `tail[2]` | `uint8_t` | `0x5A 0xA5` / `0x7F 0xFE` |
| `mode` | `uint8_t` | 由调用方给定，原样下发 |
| `yaw` / `pitch` | `float` | **度，绝对角**（不是增量） |
| `yaw_vel` / `pitch_vel` | `float` | **度/秒**，前馈 |
| `yaw_acc` / `pitch_acc` | `float` | **度/秒²**，前馈 |

- **备注**：`static_assert(... == 29)`。与 `VisionToGimbal` 的字段顺序完全一致，区别只在单位与「绝对 vs 增量」。

### `struct LaserGimbalState`

| 字段 | 类型 | 单位 |
| --- | --- | --- |
| `mode` | `uint8_t` | 原样 |
| `q[4]` | `float` | 四元数 wxyz |
| `yaw` / `yaw_vel` | `float` | **rad**、**rad/s**（已由度换算） |
| `pitch` / `pitch_vel` | `float` | **rad**、**rad/s** |
| `bullet_speed` / `bullet_count` | `float` / `uint16_t` | m/s、发弹数 |

## 类 `io::LaserGimbal`

- **职责**：一条串口 + 一条读线程；读线程解析最新一帧并覆盖内部 `state_`，上层用 `read()` 取快照、用 `send()` 下发绝对角指令。**没有发送队列、没有发送线程、没有姿态时间序列**。
- **与 `io::Gimbal` 的差异**：

  | 维度 | `io::Gimbal` | `io::LaserGimbal` |
  | --- | --- | --- |
  | 构造参数 | YAML 配置路径（含 `com_port`） | 端口字符串 + 波特率（默认 115200） |
  | 线上角度单位 | 弧度 | **度** |
  | 下行 yaw/pitch | **相对增量**（内部换算） | **绝对角** |
  | 发送方式 | 异步队列 + 发线程 | 同步直写串口 |
  | 姿态时间对齐 | 有 `queue_` + `q(t)` 插值 | 无 |
  | 自动重连 / 诊断 | 有（>100 错误重连、`diagnostics()`） | 无 |
  | 使用者 | `standard_mpc`、`auto_aim_debug_mpc`、`auto_buff_debug_mpc` 等 | 仅 `tests/laser_gimbal_test.cpp` |

- **典型用法**：
  ```cpp
  io::LaserGimbal laser("/dev/ttyUSB0", 115200);
  auto s = laser.read();                                  // yaw/pitch 已是 rad
  laser.send(1, yaw_rad, 0.f, 0.f, pitch_rad, 0.f, 0.f);  // 绝对角，内部转度
  ```

### 成员变量

| 成员 | 类型 | 含义 |
| --- | --- | --- |
| `serial_` | `serial::Serial` | 串口对象 |
| `read_thread_` | `std::thread` | 读线程 |
| `quit_` | `std::atomic<bool>` | 退出标志 |
| `mutex_` | `mutable std::mutex` | 保护 `state_` |
| `state_` | `LaserGimbalState` | 最近一帧（弧度） |
| `has_state_` | `std::atomic<bool>` | 是否已收到过帧；**当前无任何读取者（死成员）** |

### 私有函数

| 声明 | 状态 |
| --- | --- |
| `void read_loop()` | 读线程主循环 |
| `bool find_and_sync_frame(uint8_t *, size_t &, uint8_t *, size_t)` | **只有声明，无定义也无调用（死声明）**，同步逻辑内联在 `read_loop` 中 |
