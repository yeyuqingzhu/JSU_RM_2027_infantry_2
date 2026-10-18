# io/gimbal/gimbal.cpp

**概要**：`io::Gimbal` 的实现（协议 v3.0）——与下位机 `crc8.c` 等价的 CRC-8、串口端口探测与打开（默认 921600 bps、50 ms 总超时 + 5 ms 字节间超时）、构造期的「敲门 + 等首帧（2 s 上限）」、读线程带「帧尾 + crc8」双重校验的滑动窗口解析（伪帧头只滑 1 字节）、发线程的「只发最新 + 空闲 10 Hz 心跳」、按时间戳的四元数插值、以及连续错误超过 100 次时的自动重连。
**日志**：全文件用 `tools::file_logger("io/gimbal/gimbal.cpp")`，日志落在 `logs/io/gimbal/gimbal.cpp/{run,warn,error,state}.log`（三档轮转覆盖 + 每秒覆盖写的状态快照），而不是全局 `tools::logger()` 的单文件。目的是"串口收不到数据"时日志不被其他模块淹没，且事后能一眼看出打开的端口、发了多少帧、对方回了多少字节。

## 匿名命名空间辅助

| 名字 | 作用 |
| --- | --- |
| `std::shared_ptr<spdlog::logger> glog()` | 本文件专用日志器（`logs/io/gimbal/gimbal.cpp/` 下的 run/warn/error/state 四文件），内部 `static` 惰性创建 |
| `constexpr bool kVerboseSerialHexLog = false` | 串口收发逐帧十六进制日志总开关（默认关闭；首帧仍会无条件留证） |
| `constexpr std::int64_t kSilentWarnMs = 2000` / `kSilentReportMs = 10000` / `kSilentReopenMs = 30000` | 静默三档：2 s 打一次排查清单；之后每 10 s 补一行计数；30 s 且一字节未收到则强制重开一次句柄 |
| `constexpr std::size_t kFirstBytesHexMax = 32` | 首次收到字节时最多转储多少字节 |
| `constexpr int kKnockIntervalMs = 20` | 构造期敲门间隔（下位机 daemon 50 ms 判离线，20 ms 留 2.5 倍余量） |
| `struct SerialPortInfo` + `read_sysfs_line()` + `probe_serial_port()` | 从 `/sys/class/tty/<name>/device` 上溯到 USB 设备目录，取 `idVendor:idProduct`、`product`、`manufacturer`、`serial` 与 `device/driver` 符号链接名，拼成一行可读身份（`str()`）；伪终端等拿不到时留空并注明 |
| `std::vector<std::string> list_serial_candidates()` | 列出 `/dev/gimbal`（若存在）+ `/dev` 下所有 `ttyUSB*`/`ttyACM*`（排序），用于把"本机串口候选"整表写进日志 |
| `std::string wall_clock_now()` | `YYYY-MM-DD HH:MM:SS.mmm` 墙上时钟，给 `state.log` 快照用 |
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

- **作用**：
  1. 读 YAML（`com_port` 必填；`baudrate` 默认 921600；`crc8_check` 默认 true），失败时记 `error` + 快照并 `exit(1)`；记下 `configured_port_` 并打印一行配置摘要；
  2. **枚举本机串口候选**（`list_serial_candidates()` + 每个端口的 `SerialPortInfo` 身份）写进 `run.log`；
  3. `resolve_serial_port()` 解析实际端口（回退则 warn），再打印"选用端口 + 身份"；
  4. 打开串口、flush、等 100 ms，启动读/发线程；
  5. **敲门循环**：每 `kKnockIntervalMs = 20` ms 入队一个 `mode=0` 的空逻辑帧，最多 `kFirstFrameTimeoutMs = 2000` ms；
  6. 收到首帧 → `[Gimbal] First q received（敲门 N 次后收到首帧）`；没收到 → 打一条含「发送侧 tx_frames/失败/敲门次数 + 接收侧 rx_bytes」的 warn，并按 `rx_bytes` 是否为 0 给出两条互斥的排查路径，同时写 `state.log`。
- **参数**：`config_path` YAML 路径。
- **返回**：无。
- **备注 / 边界条件**：
  - **为什么必须敲门**：下位机是请求-响应式（`master_process.c` 的 `DecodeUsb()` 解析成功才 `UsbVisionSend()`），不发帧就永远收不到遥测；v2.0 的构造函数在这里是「无超时阻塞」，换板后会永久卡死。
  - `rx_bytes == 0` 与 `rx_bytes > 0` 是两条完全不同的故障链：前者是物理层（线序/端口/波特率/固件），后者是协议层（波特率不对/协议版本不同），日志会把结论直接写出来，不必再看代码猜。
  - 串口打开失败、配置读取失败仍走 **`exit(1)`**（沿用旧行为，上层无法捕获）。

### `~Gimbal()`

- **作用**：置 `quit_ = true`，压一个空帧唤醒阻塞在 `pop()` 上的发线程，`join` 两线程，最后 `close()`。
- **备注**：发线程现在用「空则 sleep 5 ms」的轮询，压帧仍是无害的冗余唤醒。

### `GimbalMode mode() const` / `GimbalState state() const`

- **作用**：加 `mutex_` 拷贝快照。`state()` 里的 yaw/pitch/vel 已是**弧度制**（`handle_frame` 里换算）。
- **备注**：返回副本；高频每帧调用会与读线程争锁，但临界区很短。

### `GimbalDiagnostics diagnostics() const`

- **作用**：**十项全部填充**：`frame_count`/`error_count`/`rx_frame_rate`/`tx_latency_ms`/`queue_depth`（= `tx_backlog_`）/`serial_ok`/`rx_latency_ms`（当前 − 最近成功解析时刻），以及 2026-10-18 新增的 `rx_bytes`（累计收到的原始字节，含垃圾字节）/`tx_frames`（成功写出的下行帧数）/`tx_fail`（写失败次数）。
- **备注**：`error_count` 在伪帧头逐字节滑动时会放大；`queue_depth` 语义是「上次发送以来积压条数」，发线程排空后立即归零。判"链路有没有通"看 `rx_bytes`（==0 就是物理层没通），判"我们在不在发"看 `tx_frames`。

### `std::string str(GimbalMode mode) const`

- **作用**：`IDLE` / `AUTO_AIM` / `SMALL_BUFF` / `BIG_BUFF`，未知返回 `"INVALID"`。

### `Eigen::Quaterniond q(std::chrono::steady_clock::time_point t)`

- **作用**：在姿态时间序列上取 `t` 时刻的插值四元数：`pop` 队首 `a`、`front` 取 `b`、按 `(t−t_a)/(t_b−t_a)` 做 `slerp`；`t < t_a` 时外插返回；否则继续推进直到 `t_a < t <= t_b`。
- **备注 / 边界条件**：**会消费队列**（必须按 `t` 单调递增调用）；`t_ab == 0` 会除零；队列空时 `pop()/front()` 阻塞——读线程已死时主线程会永久卡在 `q(t)` 里。

### `bool try_q(std::chrono::steady_clock::time_point t, Eigen::Quaterniond & out, int timeout_ms)`

- **作用**：`q(t)` 的**非阻塞**版本：队列里凑不出"一前一后"两帧就等（每 1 ms 轮询一次 `queue_.size()`），超时返回 `false`；凑齐后按与 `q()` 相同的规则插值/外插，把结果写进 `out`。`timeout_ms <= 0` 表示只探一次。
- **参数**：`t` 查询时刻；`out` 输出姿态；`timeout_ms` 最长等待毫秒数。
- **返回**：`true` 表示 `out` 有效。
- **备注 / 边界条件**：**不会永久阻塞**——这是它与 `q(t)` 的本质区别。测试/诊断程序应一律用它：`q(t)` 在无首帧时会把调用线程连同"后面的 `send()`"一起冻死，而下位机只在你发帧后才回帧，于是链路永久沉默（这正是"运行 gimbal_test 一直收不到首帧且毫无输出"的成因之一）。会消费队列（与 `q` 一致，必须按 `t` 单调调用）。

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
- `write_ctrl`：持 `serial_mutex_` 判 `isOpen()` 后 `serial_.write(reinterpret_cast<const uint8_t*>(&frame), 28)`，成功后把这一帧原始字节存进 `last_tx_frame_`（供接收侧做回环自检）；异常记 `error`；`tx_frames_`/`tx_fail_` 计数；**首帧**单独打一条 info（28B hex + `head/tail/crc8` 自检，方便用逻辑分析仪对照）；发送耗时进 EMA（权重 0.9/0.1），> 5 ms 打 warn 并提示"多半是别的线程长时间占着 `serial_mutex_`"；串口未打开时只在第 1/2/3 次与每 100 次各打一条（重连期间会连续失败，避免刷屏）。
- **备注**：`UsbCtrl` 是 `packed` 结构体，**不能**取字段地址或绑 fmt 引用形参，只能整体按字节写下 / 打印前先拷到局部变量。

### `void tx_thread()`

- **作用**：5 ms 轮询循环：
  1. `tx_queue_` 非空 → `pop` 出一条，再 `while (!empty()) pop();` 只留**最新** → `pack_ctrl` + `write_ctrl` → `tx_backlog_.store(0)`；
  2. 队列空且距上次发送 ≥ `kIdleHeartbeatMs = 100` ms → 发一帧 `mode = 0` 的**空闲心跳**（`enqueue_cmd(VisionToGimbal{})` 等价物直接打包发送）。
- **备注**：心跳有三个作用——链路保活（否则下位机不回遥测、视觉端 `q(t)` 断流）、下位机超时保护不脱控、绝不执行过期目标。这是 v3.0 相对 v2.0 最重要的行为变化。
- **日志**：每 5 s 一条 `debug` 发送侧摘要（frames/失败/心跳/队列深度/写延迟，只进 `run.log`，不刷控制台）；退出时打累计帧数与失败数。
- **注意**：`kIdleHeartbeatMs` 同时是"控制指令有效期"。**不要**为了迁就下位机 50 ms 判离线阈值把它降到 30 ms——那会让发送频率低于 33 Hz 的上层（测试程序、低频规划器）的指令被心跳盖掉（`loopback_test` 的 C 段会直接 FAIL：`after send` 读到 mode=IDLE、角度全 0）。敲门间隔已改为 20 ms，握手已经足够快。

### `void read_thread()`

- **作用**：读线程主循环：
  0. 每 1 s 用 `tools::snapshot()` **覆盖写**一行状态快照到 `state.log`（时间戳/端口/波特率/`rx_bytes`/`tx_frames`/`tx_fail`/姿态队列深度/帧数/错误数/帧率/距上一帧毫秒/静默毫秒）。进程被 Ctrl-C、崩溃或卡死之后，**先看这个文件**；
  1. `consecutive_errors > 100` → `reconnect()`（说明"有字节但凑不出帧"）；
  2. 持锁 `available()` 批量读入 1024B 流缓冲；本批字节起点记为 `new_start`；
     - 顺带做**回环自检**：`has_tx_frame_` 且新字节前 28 B 与 `last_tx_frame_` 逐字节相同 → 置 `echo_detected`（"TX/RX 被短接/自环"）；
  3. 串口未打开 → `reconnect()`；
  4. 本批 0 字节 → **静默三档**（取代旧的"连续 4000 次空读就重连"）：
     - 静默 ≥ 2 s（只报一次）：输出六条排查清单（固件/共地/在哪个 UART/波特率/端口选择/回环法），并把 `rx_bytes`+`tx_frames` 一起打出来；
     - 之后每 10 s 一条 `info` 计数；若此时设备节点已消失 → `reconnect()`；
     - 静默 ≥ 30 s 且**从未收到任何字节** → 强制 `reconnect()` 一次（排除驱动/转接板卡死），只做一次；
     - 否则 sleep 200 µs 继续——**不再因为"没数据"就疯转重连**（旧实现每秒重连一次，而重连会 `flushInput()`，反而可能把刚到的帧丢掉）；
  5. 有字节：记 `last_any_byte_ns`、清静默告警、`rx_bytes_ += nread`；**首次**收到字节打一条 info（最多 32 B hex）；`echo_detected` 时打一次 `rx_echo` warn；
  6. 在缓冲里找第一个 `0xFF`；找到就挪到开头，不足 42B 等下一批；
  7. 凑满 42B → 校验 `frame[41] == 0x0D` **且** `crc8(frame, 40) == frame[40]`（`crc8_check_ == false` 时只校验帧界）；
  8. 通过 → `handle_frame()`（若此前有连续失败，补一条"重新同步成功"info），吃掉这 42B 继续解析；
  9. 失败 → `consecutive_errors++`、`error_count_++`；前 5 次打 `[Gimbal] 疑似坏帧: head=... tail=... crc8=... dump: ...`；第 50 次专门打一条"有字节但连续 50 次解不出帧 → 高度怀疑波特率不匹配或 TX/RX 接反"；每 50 次 `flushInput()`；然后**只向前滑 1 字节**继续找头（伪帧头容错，不会因为一个坏字节丢掉后面的真帧）。
- **备注**：v2.0 是「校验失败就 flushInput 重来」，波特率/协议不匹配时会陷入「找到假头 → 失败 → flush」的循环；现在滑 1 字节，最坏情况是遍历缓冲，但一定能自愈。
- **备注（诊断设计）**：`rx_bytes`/`first_bytes_logged`/`echo`/静默三档这四件事，覆盖了现场所有"收不到"的形态——0 字节（物理层）、有字节但解不出（波特率/协议）、收到的是自己的回声（自环）、设备被拔（节点消失）。

### `bool handle_frame(const uint8_t * frame)`

- **作用**：把 42B 拷进 `UsbAutoSendToNuc`，再把每个 `packed` 字段拷进局部变量（避免 packed 字段绑定引用形参），按**度**做合理性检查（`|yaw| ≤ 180.5`、`|pitch| ≤ 90.5`、`|vel| ≤ 2000`、`bullet_speed ∈ [0, 40]`），四元数范数须在 `[0.5, 1.5]` 内且 normalize 后 `wxyz` 展开；通过则持 `mutex_` 写 `rx_data_`、`state_`（度→弧度）、`mode_`（0→IDLE/1→AUTO_AIM/2→SMALL_BUFF/3→BIG_BUFF/default→IDLE+warn）、`frame_count_++`、帧率 EMA、`last_frame_ns_`；随后 `last_rx_ok_ns_.store(...)`、`queue_.push({q, t})`；**`frame_count_ == 1` 时单独打一条 info 首帧取证**（42B hex + mode/角度/弹速/弹丸数/四元数，并注明"链路已通：端口、波特率、线序、固件四者都对"）；每 200 帧打一条 `debug` 摘要（帧率/帧数/错误数/`rx_bytes`/`tx_frames`/延迟，只进 `run.log`，不刷控制台）。
- **返回**：是否接受该帧。
- **备注**：pitch 不做取反（下位机上行已乘 `TONGJI_PITCH_UPLINK_SIGN = -1`）；`bullet_count` 由下位机拨盘上升沿计数，直接用。

### `void reconnect()`

- **作用**：最多 15 次重试，每次：
  1. **锁内**只做 close（含 flush），随即释放锁；
  2. **锁外**睡 500 ms（给 USB 设备复位）；
  3. 重新 `resolve_serial_port(configured_port_)`——拔插后 `ttyUSB0`/`ttyUSB1` 可能对调，不重新解析就永远打不开旧节点，变化时打 warn；
  4. 锁内 `setPort` + `open` + `setBaudrate` + `setTimeout`（**左值** `Timeout`）；
  5. 锁外睡 100 ms，再锁内 `flushInput`/`flushOutput`；`queue_.clear()` 后打 `重连成功: <端口身份>` 并 `return`。
  - 15 次全失败 → 打一条 `error`（"串口不可用，数据链路已断"）后返回。
- **备注**：**绝不在持 `serial_mutex_` 时 sleep**——旧实现在锁内睡 500 ms，把发线程一起冻住，日志里那条 `High TX latency: 551.74ms` 就是这么来的。`queue_.clear()` 会丢历史姿态，紧接着上层 `q(t)` 可能阻塞等新帧（诊断程序请用 `try_q`）。
