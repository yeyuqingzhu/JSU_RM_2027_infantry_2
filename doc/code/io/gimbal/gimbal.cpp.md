# io/gimbal/gimbal.cpp

**概要**：`io::Gimbal` 的实现（协议 v3.0）——与下位机 `crc8.c` 等价的 CRC-8、串口端口探测与打开（默认 921600 bps、50 ms 总超时 + 5 ms 字节间超时）、构造期的「敲门 + 等首帧（2 s 上限）」、读线程带「帧尾 + crc8」双重校验的滑动窗口解析（伪帧头只滑 1 字节）、发线程的「只发最新 + 空闲 20 Hz 心跳」、按时间戳的四元数插值、以及连续错误超过 100 次时的自动重连。

## 匿名命名空间辅助

| 名字 | 作用 |
| --- | --- |
| `constexpr bool kVerboseSerialHexLog = false` | 串口收发十六进制日志总开关（默认关闭） |
| `constexpr double kRadToDeg = 57.295779513082320877` / `kDegToRad = 0.017453292519943295` | 度↔弧度换算（线格式是度、对外 API 是弧度） |
| `constexpr float kMaxYawDeg = 180.5f` / `kMaxPitchDeg = 90.5f` / `kMaxYawVelDps = 2000.0f` / `kMaxPitchVelDps = 2000.0f` / `kMaxBulletSpeed = 40.0f` | 度制下的合理性门限（yaw 留 0.5° 余量给「±π 折算到 ±180」的边界值） |
| `constexpr size_t kStreamBufferSize = 1024` | 读线程流缓冲大小 |
| `std::string resolve_serial_port(const std::string & configured_port)` | 端口回退策略：配置路径存在则直接用；否则依次试 `/dev/gimbal`、`/dev/ttyUSB0`、`/dev/ttyUSB1`、`/dev/ttyACM0`、`/dev/ttyACM1`；再否则扫 `/dev` 取排序后第一个 `ttyUSB*`/`ttyACM*`；全失败返回配置值原样 |
| `serial::Timeout make_timeout()` | 生成 `simpleTimeout(50)` 且 `inter_byte_timeout = 5` 的 `Timeout` 左值（`serial::Serial::setTimeout` 形参是非 const 引用，必须传左值） |
| `std::string hex_dump(const uint8_t *, size_t)` | 伪帧头/坏帧日志用的十六进制转储 |

## 函数

### `uint8_t crc8(const uint8_t * data, std::size_t size)`

- **作用**：位实现 CRC-8（init `0x00`、poly `0x31` 反射、高位优先），与下位机 `sht75_crc_table` 查表逐字节等价。
- **返回**：校验字节。
- **备注**：`crc8({0x01}) == 0x31 == 49`，与下位机表项 `sht75_crc_table[1] = 49` 对得上；`proto_test`（离线构建，见 `build/gimbal_verify/proto_test.cpp`，不入库）用表的前 16 项做过核对。

### `Gimbal(const std::string & config_path)`

- **作用**：读 YAML（`com_port` 必填；`baudrate` 默认 921600；`crc8_check` 默认 true）→ 解析实际端口（回退时 warn）→ 配置串口并打开 → flush 并等 100 ms 清积压 → 启动读线程与发线程 → **敲门循环**：每 50 ms 入队一个 `mode=0` 的空逻辑帧，最多 `kFirstFrameTimeoutMs = 2000` ms，收到第一帧姿态即打印 `[Gimbal] First q received.`。
- **参数**：`config_path` YAML 路径（缺 `com_port` 时 `tools::read` 记日志并 `exit(1)`）。
- **返回**：无。
- **备注 / 边界条件**：
  - **为什么必须敲门**：下位机是请求-响应式（`master_process.c` 的 `DecodeUsb()` 解析成功才 `UsbVisionSend()`），不发帧就永远收不到遥测；v2.0 的构造函数在这里是「无超时阻塞」，换板后会永久卡死。
  - 2 s 内没收到首帧只打 warn 并继续运行（提示查波特率/协议版本/`crc8_check`），不再永久阻塞；
  - 串口打开失败仍走 **`exit(1)`**（沿用旧行为，上层无法捕获）。

### `~Gimbal()`

- **作用**：置 `quit_ = true`，压一个空帧唤醒阻塞在 `pop()` 上的发线程，`join` 两线程，最后 `close()`。
- **备注**：发线程现在用「空则 sleep 5 ms」的轮询，压帧仍是无害的冗余唤醒。

### `GimbalMode mode() const` / `GimbalState state() const`

- **作用**：加 `mutex_` 拷贝快照。`state()` 里的 yaw/pitch/vel 已是**弧度制**（`handle_frame` 里换算）。
- **备注**：返回副本；高频每帧调用会与读线程争锁，但临界区很短。

### `GimbalDiagnostics diagnostics() const`

- **作用**：v3.0 起**七项全部填充**：`frame_count`/`error_count`/`rx_frame_rate`/`tx_latency_ms`/`queue_depth`（= `tx_backlog_`）/`serial_ok`/`rx_latency_ms`（当前 − 最近成功解析时刻）。
- **备注**：`error_count` 在伪帧头逐字节滑动时会放大；`queue_depth` 语义是「上次发送以来积压条数」，发线程排空后立即归零（`ThreadSafeQueue` 没有 `size()`）。

### `std::string str(GimbalMode mode) const`

- **作用**：`IDLE` / `AUTO_AIM` / `SMALL_BUFF` / `BIG_BUFF`，未知返回 `"INVALID"`。

### `Eigen::Quaterniond q(std::chrono::steady_clock::time_point t)`

- **作用**：在姿态时间序列上取 `t` 时刻的插值四元数：`pop` 队首 `a`、`front` 取 `b`、按 `(t−t_a)/(t_b−t_a)` 做 `slerp`；`t < t_a` 时外插返回；否则继续推进直到 `t_a < t <= t_b`。
- **备注 / 边界条件**：**会消费队列**（必须按 `t` 单调递增调用）；`t_ab == 0` 会除零；队列空时 `pop()/front()` 阻塞——读线程已死时主线程会永久卡在 `q(t)` 里。

### `void send(io::VisionToGimbal cmd)`

- **作用**：`cmd.mode == 0` 时把六个角度量清零（不控制就不该带目标角），其余原样 `enqueue_cmd`。**不再像 v2.0 那样减 `state()` 折成增量**——下位机自己要绝对角。
- **备注**：单位仍是弧度；度制换算推迟到发线程的 `pack_ctrl()`。

### `void send(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel, float pitch_acc)`

- **作用**：便捷重载，填 `mode = control ? (fire ? 2 : 1) : 0` 后转调上一个重载。
- **备注**：算法层（`Planner` / `Aimer`）唯一出口；弧度、绝对角。

### `UsbCtrl pack_ctrl(const VisionToGimbal & cmd) const`

- **作用**：逻辑帧 → 28B 线帧。`yaw = limit_rad(cmd.yaw) × kRadToDeg`、`pitch = cmd.pitch × kRadToDeg`（**不取反**，符号由下位机处理）、三个前馈量同样 × `kRadToDeg`；填 `head = 0xFF`、`tail = 0x0D`、`crc8 = crc8(&frame, 26)`。

### `void enqueue_cmd(const VisionToGimbal & cmd)` / `void write_ctrl(const UsbCtrl & frame)`

- `enqueue_cmd`：`tx_queue_.push(cmd)` 且 `tx_backlog_++`。
- `write_ctrl`：持 `serial_mutex_` 判 `isOpen()` 后 `serial_.write(reinterpret_cast<const uint8_t*>(&frame), 28)`；异常记 warn；发送耗时进 EMA（权重 0.9/0.1），> 5 ms 打 warn。
- **备注**：`UsbCtrl` 是 `packed` 结构体，**不能**取字段地址或绑 fmt 引用形参，只能整体按字节写下 / 打印前先拷到局部变量。

### `void tx_thread()`

- **作用**：5 ms 轮询循环：
  1. `tx_queue_` 非空 → `pop` 出一条，再 `while (!empty()) pop();` 只留**最新** → `pack_ctrl` + `write_ctrl` → `tx_backlog_.store(0)`；
  2. 队列空且距上次发送 ≥ `kIdleHeartbeatMs = 100` ms → 发一帧 `mode = 0` 的**空闲心跳**（`enqueue_cmd(VisionToGimbal{})` 等价物直接打包发送）。
- **备注**：心跳有三个作用——链路保活（否则下位机不回遥测、视觉端 `q(t)` 断流）、下位机超时保护不脱控、绝不执行过期目标。这是 v3.0 相对 v2.0 最重要的行为变化。

### `void read_thread()`

- **作用**：读线程主循环：
  1. 串口未打开 → `reconnect()`；
  2. 持锁 `available()` 批量读入 1024B 流缓冲；连续 4000 次读 0 字节 → 视为链路死掉并 `reconnect()`；
  3. 在缓冲里找第一个 `0xFF`；找到就挪到开头，不足 42B 等下一批；
  4. 凑满 42B → 校验 `frame[41] == 0x0D` **且** `crc8(frame, 40) == frame[40]`（`crc8_check_ == false` 时只校验帧界）；
  5. 通过 → `handle_frame()`，然后吃掉这 42B 继续解析；
  6. 失败 → `consecutive_errors++`、`error_count_++`；前 5 次打 `[Gimbal] Bad frame candidate: head=... tail=... crc8=... dump: ...`；每 50 次 `flushInput()`；然后**只向前滑 1 字节**继续找头（伪帧头容错，不会因为一个坏字节丢掉后面的真帧）；
  7. `consecutive_errors > 100` → `reconnect()`。
- **备注**：v2.0 是「校验失败就 flushInput 重来」，波特率/协议不匹配时会陷入「找到假头 → 失败 → flush」的循环；现在滑 1 字节，最坏情况是遍历缓冲，但一定能自愈。

### `bool handle_frame(const uint8_t * frame)`

- **作用**：把 42B 拷进 `UsbAutoSendToNuc`，再把每个 `packed` 字段拷进局部变量（避免 packed 字段绑定引用形参），按**度**做合理性检查（`|yaw| ≤ 180.5`、`|pitch| ≤ 90.5`、`|vel| ≤ 2000`、`bullet_speed ∈ [0, 40]`），四元数范数须在 `[0.5, 1.5]` 内且 normalize 后 `wxyz` 展开；通过则持 `mutex_` 写 `rx_data_`、`state_`（度→弧度）、`mode_`（0→IDLE/1→AUTO_AIM/2→SMALL_BUFF/3→BIG_BUFF/default→IDLE+warn）、`frame_count_++`、帧率 EMA、`last_frame_ns_`；随后 `last_rx_ok_ns_.store(...)`、`queue_.push({q, t})`、每 200 帧打印一次 Diag。
- **返回**：是否接受该帧。
- **备注**：pitch 不做取反（下位机上行已乘 `TONGJI_PITCH_UPLINK_SIGN = -1`）；`bullet_count` 由下位机拨盘上升沿计数，直接用。

### `void reconnect()`

- **作用**：最多 15 次：close + 睡 500 ms，open + 重设波特率与超时（**左值** `Timeout`）+ 睡 100 ms + flush + `queue_.clear()`；成功后 break。
- **备注**：15 次全失败后返回，读线程仍在循环并在下次连续错误超阈值时再进；`queue_.clear()` 会丢历史姿态，紧接着主线程 `q(t)` 可能阻塞等新帧。
