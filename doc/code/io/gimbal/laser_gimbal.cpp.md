# io/gimbal/laser_gimbal.cpp

**概要**：`io::LaserGimbal` 的实现——打开串口（默认 115200 bps，100 ms 总超时）、启动读线程做流式帧解析（找 `0x5A 0xA5` → 取 43 字节 → 校验 `0x7F 0xFE` → 度制合理性检查 → 转弧度存入 `state_`），以及同步直写的 `send()`（弧度转度后一次性 `write`）。

## 匿名命名空间常量

| 名字 | 值 | 用途 |
| --- | --- | --- |
| `kDegToRad` / `kRadToDeg` | `π/180` / `180/π` | 度弧度换算 |
| `kPacketSize` | 43 | 上行帧长 |
| `kStreamBufSize` | 512 | 流缓冲大小 |
| `kMaxAngleDeg` | 360.0f | 角度合理性上限（度） |
| `kMaxVelDeg` | 720.0f | 角速度合理性上限（度/秒） |

## 函数

### `LaserGimbal(const std::string & port, int baudrate = 115200)`

- **作用**：`setPort` → `setBaudrate` → `simpleTimeout(100)` → `open()` → `flushInput()` + `flushOutput()` → 启动读线程。
- **参数**：`port` 串口设备路径（如 `/dev/ttyUSB0`，**不做回退探测**）；`baudrate` 波特率，默认 115200。
- **备注**：**不捕获异常**——串口不存在或无权限时，`serial_.open()` 抛出的 `std::exception` 会直接冒到调用方（与 `io::Gimbal` 的 `exit(1)` 策略相反）。这里也没有「等首帧」的阻塞，构造完成即返回。

### `~LaserGimbal()`

- **作用**：置 `quit_ = true`，`join` 读线程，串口若仍打开则关闭。

### `LaserGimbalState read()`

- **作用**：加 `mutex_` 拷贝并返回 `state_`。
- **返回**：`LaserGimbalState`（角度已换算为 rad）。
- **备注**：**不判断是否收到过帧**——尚未收到任何合法帧时返回的是全 0 结构（`has_state_` 没有被 `read()` 检查，也没有对外接口），调用方无法区分「真的 0」与「还没数据」。

### `void send(uint8_t mode, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel, float pitch_acc)`

- **作用**：把 6 个弧度值乘 `kRadToDeg` 转成度填入 `LaserVisionToGimbal`（`yaw`/`pitch` 是**绝对角**），同步 `serial_.write()` 一次发完。
- **参数**：`mode` 原样下发；`yaw`/`pitch` 目标绝对角（rad）；`*_vel`、`*_acc` 前馈（rad/s、rad/s²）。
- **备注**：写异常被空 `catch` **静默吞掉**（只丢帧、不记录、不重试）；无发送队列，因此调用线程直接被串口写阻塞。

### `void read_loop()`

- **作用**：读线程主循环：
  1. `available()` 批量读入 512 字节流缓冲（满时保留末尾 64 字节）；
  2. 读到 0 字节则以 500 µs 让出 CPU；
  3. 当缓冲 ≥ 43 字节时循环：滑动查找 `0x5A 0xA5`（找不到只保留最后 1 字节）；不足一帧则把帧头挪到开头等待；取 43 字节；
  4. 校验末尾 `0x7F 0xFE`，失败直接丢弃（**不计数、不打日志**）；
  5. `memcpy` 成 `LaserGimbalToVision`，做合理性检查 `|yaw|,|pitch| ≤ 360°`、`|vel| ≤ 720°/s`，越界丢弃（同样静默）；
  6. 转弧度填入 `LaserGimbalState`，加锁覆盖 `state_`，并置 `has_state_ = true`。
- **备注 / 已知问题**：
  - 全程**没有错误计数与日志**，串口协议不匹配时表现为「`read()` 永远返回 0，且无任何输出」，比 `io::Gimbal` 更难排障；
  - 本文件使用 `std::memmove` / `std::memcpy` 但**未 `#include <cstring>`**，依赖其它头文件间接引入（同类隐藏依赖问题见 [`doc/troubleshooting.md` B-1](../../../troubleshooting.md)）；
  - `has_state_` 写入后无人读取（死成员）；
  - 私有声明 `find_and_sync_frame` 无定义、无调用（死声明），同步逻辑已内联在本函数里。
