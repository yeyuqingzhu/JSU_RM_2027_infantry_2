# `io/gimbal/` 逐文件函数说明

- **对应源码目录**：`io/gimbal/`
- **职责**：云台串口通信层——把「串口字节流」翻译成「姿态 + 弹速」，把「目标角 + 前馈」翻译成「下位机可执行的帧」。上层（`tasks/auto_aim/*`、`tasks/auto_buff/*`、`src/*`）只依赖本层的**弧度制、绝对角** API，不关心线格式。
- **两种云台**：
  - `Gimbal`（工程主力）：**协议 v3.0**，适配 2027 赛季 lt-2027 哨兵下位机（湖南大学 RoboMaster 电控组 `seasky_protocol.c` / `master_process.c`）。线上 0xFF/0x0D + crc8 + **度制绝对角**，921600 8N1；本层内部做度↔弧度换算，对外仍是弧度制。
  - `LaserGimbal`（Lracking 激光云台）：另一套自定帧，度制绝对角，仅 `laser_gimbal_test` 使用，未受本次改造影响。

## 文件清单

| 源文件 | 说明文档 | 备注 |
| --- | --- | --- |
| `gimbal.hpp` | [gimbal.hpp.md](gimbal.hpp.md) | 帧常量 + `crc8()` + 两个线帧结构体（`UsbCtrl` 28 B / `UsbAutoSendToNuc` 42 B）+ 逻辑帧 `VisionToGimbal` + `GimbalMode/State/Diagnostics` + `Gimbal` 类声明 |
| `gimbal.cpp` | [gimbal.cpp.md](gimbal.cpp.md) | `crc8` 位实现、端口身份探测（sysfs VID:PID/驱动）、构造期敲门（20 ms）与首帧取证、读线程带 crc 的滑窗解析 + 静默三档 + 回环自检 + `state.log` 快照、发线程（含 100 ms 空闲心跳）、`q(t)`/`try_q()` 插值、不持锁重连 |
| `laser_gimbal.hpp` | [laser_gimbal.hpp.md](laser_gimbal.hpp.md) | 激光云台线格式（度制、绝对角）+ `LaserGimbal` 类声明 |
| `laser_gimbal.cpp` | [laser_gimbal.cpp.md](laser_gimbal.cpp.md) | 激光云台读写实现（同步直写、无重连/无日志） |

## 协议速查（`Gimbal`，v3.0）

| 方向 | 结构体 | 长度 | 关键字段 |
| --- | --- | --- | --- |
| 云台 → 视觉 | `UsbAutoSendToNuc` | 42 B | `mode`(0 空闲/1 自瞄/2 小符/3 大符)、`q[4]`(wxyz)、`yaw`/`yaw_vel`、`pitch`/`pitch_vel`、`bullet_speed`、`bullet_count`（**度、度/秒**） |
| 视觉 → 云台 | `UsbCtrl` | 28 B | `mode`(0 不控制/1 控制不开火/2 控制且开火)、`yaw`/`pitch`＝**绝对角**（度）、`*_vel`/`*_acc` 前馈（度/秒、度/秒²） |
| 帧界 | — | — | HEAD `0xFF`，TAIL `0x0D`，crc8（init `0x00`/poly `0x31`/高位优先）；**921600** 8N1 |

**字节偏移（与下位机一一对应）**

- `UsbCtrl`：`0` head、`1` mode、`2` yaw、`6` yaw_vel、`10` yaw_acc、`14` pitch、`18` pitch_vel、`22` pitch_acc、`26` crc8（覆盖前 26 B）、`27` tail。
- `UsbAutoSendToNuc`：`0` head、`1` mode、`2` `q[4]`、`18` yaw、`22` yaw_vel、`26` pitch、`30` pitch_vel、`34` bullet_speed、`38` bullet_count、`40` crc8（覆盖前 40 B）、`41` tail。

**三条容易踩的语义（与 v2.0 不同）**

1. **单位是度**：v2.0 线上是弧度；v3.0 线上全是度（与下位机 `DEGREE_2_RAD`/`RAD_2_DEGREE` 换算处对齐）。换算只在 `Gimbal::pack_ctrl()`（rad→deg）与 `Gimbal::handle_frame()`（deg→rad）发生。
2. **下行是绝对角**：v2.0 发「目标 − 当前」的相对增量；v3.0 下位机自己算 `big_yaw_target = yaw * DEGREE_2_RAD`，所以 `Gimbal::send()` **不再减 `state()`**，planner 给的绝对角原样换算发出。
3. **请求-响应式链路**：下位机只有成功解析到一帧 `UsbCtrl` 才回发一帧遥测。因此本层①构造期先发空帧「敲门」（间隔 50 ms，上限 2 s）；②`tx_thread()` 在上层静默 > 100 ms 后以 10 Hz（`kIdleHeartbeatMs = 100`）补发 `mode=0` 空帧（链路保活 + 不执行过期目标）。**去掉心跳会导致视觉端收不到姿态。**
   - **注意节奏**：下位机自瞄 daemon 是 100 Hz、`reload_count = 5`（`master_process.c:275` + `robot_task.h` 的 `osDelay(10)`），即**50 ms 收不到 `UsbCtrl` 就判视觉离线**并 `USARTServiceInit()` 重启 DMA 接收。本层构造期敲门间隔正好压在 50 ms 上、空闲心跳 100 ms 是阈值 2 倍，因此空闲时下位机侧会周期性刷 vision offline（链路仍可用，握手阶段略抖）。若要根治把 `kIdleHeartbeatMs` 与敲门间隔都降到 ≤ 30 ms（尚未改，见 [troubleshooting C-5](../../../troubleshooting.md)）。

**pitch 符号**：下位机上行发的是 `TONGJI_PITCH_UPLINK_SIGN * muzzle_pitch_deg`（该宏 = −1），即已是「视觉约定」下的 pitch；本层直接度→弧度，**不再取反**。下行同理由下位机自己取反去驱动电机。换底板时先核对这一条。

> `LaserGimbal` 的帧界仍是 `0x5A 0xA5` / `0x7F 0xFE`（43 B / 29 B），与 `Gimbal` 现在的 v3.0 帧**互不兼容**，两者不要混接。

## 使用关系

- **诊断**：本目录的日志写到 `logs/io/gimbal/gimbal.cpp/{run,warn,error,state}.log`（`tools::file_logger`，覆盖式轮转）；排查「收不到帧」先看 `state.log` 的 `rx_bytes`，判据与完整排查顺序见 [troubleshooting C-6](../../../troubleshooting.md)。
- `io::Gimbal`（串口）使用者：`src/standard_mpc.cpp`、`src/auto_aim_debug_mpc.cpp`、`src/auto_buff_debug_mpc.cpp`、`tasks/auto_buff/buff_aimer.hpp`、`calibration/capture.cpp`、`tests/gimbal_test.cpp`、`tests/fire_test.cpp`、`tests/planner_test.cpp`。
- `io::LaserGimbal` 使用者：仅 `tests/laser_gimbal_test.cpp`。
- 同一个工程里另有一条**旧的 CAN 通道** `io::CBoard`（`src/standard.cpp`、`src/mt_auto_aim_debug.cpp`、`src/auto_buff_debug.cpp`、`src/sentry*.cpp`、`tests/{cboard_test,gimbal_response_test,handeye_test,camera_thread_test}.cpp`）。两条路径并存，选哪条取决于下位机形态（见 [`AGENT.md` §7](../../../../AGENT.md)）——注意 `tests/gimbal_response_test.cpp` 走的是 CBoard，**不是**本目录的串口实现。
- **验证本目录代码只能用串口侧程序**：`tests/gimbal_test.cpp`（基础控制/开火）、`tests/fire_test.cpp`、`tests/planner_test.cpp`、`src/*_mpc.cpp`。用 `gimbal_response_test` / `cboard_test` / `sentry*` 验证串口适配，无论本目录怎么改都「连不上」——它们连的是 `can0`，yaml 里也没有 `com_port`（详见 [troubleshooting C-5](../../../troubleshooting.md)）。

## 已知问题 / 注意事项（截至 2026-10-18）

v2.0 时代的 8 条问题中，随本次重写已解决：注释与超时参数不一致（现在注释与 `kSerialTimeoutMs/kSerialInterByteMs` 唯一来源一致）、长度断言不对称（改为 `static_assert(sizeof == 28/42)`）、死声明 `findFrameHead()`/死代码 `Gimbal::read()`（已删除）、角度上限 3.15f（改为 ±180.5° 度制校验）、诊断字段未实现（7 个字段全部填充）、无空闲心跳（已加 10 Hz `mode=0` 心跳（`kIdleHeartbeatMs` = 100 ms））、构造无超时阻塞（改为「敲门 + 2 s 上限」）。

仍然保留 / 新出现的注意点：

1. **双向 crc8 不对称（已按固件源码核实）**：下位机**接收**侧 `get_usb_protocol_info()` 只校验帧头 `0xFF` 与 `rx_buf[27] == 0x0D`，**不校验 crc8**；但**发送**侧 `get_usb_protocol_send_data()` 会 `tx_buf[index] = crc_8(tx_buf, index)`（覆盖前 40 B）真填 crc8。固件 `CRC_START_8 = 0x00` + sht75 查表与本层 `crc8()` 的位实现等价（`crc8({0x01}) == 0x31`）。**结论：正常固件不会因 crc8 校验失败**，`crc8_check: true`（默认）可以一直开着；只有换成不填 crc8 的第三方固件时才需要设 `false`。
2. **构造期等首帧有 2 s 上限**：超时只打 warn 不退出（避免像旧实现那样永久卡死），但此时 `q(t)` 仍会阻塞等姿态——「启动后日志正常、但画面不出结果」时先看这条 warn。
3. **串口打开失败仍 `exit(1)`**（沿用旧行为）：上层无法捕获，属于硬失败。
4. **`error_count` 口径**：伪帧头逐字节滑动时每滑一次记一次错误，因此链路很差时该值会被放大；`rx_frame_rate` 是指数滑动平均（不是窗口统计）。
5. **`queue_depth` 语义**：由 `tx_backlog_` 计数、发线程排空后立即清零，表示「上次发送以来积压的待发条数」，不是精确队列长度（`ThreadSafeQueue` 无 `size()`）。
6. **`laser_gimbal.cpp` 缺 `<cstring>`**：`memcpy`/`memmove` 依赖间接包含（同 [troubleshooting B-1](../../../troubleshooting.md)），本次未改。
7. **配置键**：`com_port` 必填；`baudrate`（默认 921600）与 `crc8_check`（默认 true）可选，缺省即用默认值。

## 约定

新增/删除本目录源文件时：在本目录建立与源文件同名的 `<文件名>.md`，同步更新上表、[`../../../directory_tree.md`](../../../directory_tree.md)，内容模板与维护规则见 [`../../README.md`](../../README.md)。
