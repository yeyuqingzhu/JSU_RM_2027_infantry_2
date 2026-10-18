# tools/file_logger.hpp

**概要**：「按源文件分文件夹」的覆盖式日志器接口——给每个源文件一个独立日志目录 `logs/<源文件路径>/`，目录里按严重程度分 `run.log`（全收）、`warn.log`（warn 及以上）、`error.log`（error 及以上）三档，每档写满后**新文件覆盖旧备份**（`max_files = 1`），另提供 `snapshot()` 每次整份覆盖写 `state.log`。与 `tools::logger()` 并存、互不影响。

- **来源**：本项目新增（非 sp_vision_25）——2026-10-18 为排查「云台串口收不到首帧」而写
- **依赖**：spdlog（`spdlog/spdlog.h`）、C++17 `<memory>`、`<string>`；实现见 [file_logger.cpp.md](file_logger.cpp.md)，链接期需 libspdlog + libfmt（`SPDLOG_FMT_EXTERNAL=1`）
- **迁移说明**：无上游对应物。设计动机是 `tools::logger()` 的两个缺陷：①所有模块混写一个 `logs/<时间戳>.log`，出事时要在一份几万行的文件里翻；②文件名带时间戳且不轮转，跑几十次就堆几十个文件、磁盘只增不减
- **线程安全**：`file_logger()` / `snapshot()` / `set_file_logger_root()` 内部用同一把互斥锁；返回的 spdlog logger 用的是 `_mt` sink，可跨线程直接写。**`set_file_logger_root()` 仅应在任何 `file_logger()` 之前调用**

## 目录结构

对源文件标识 `"io/gimbal/gimbal.cpp"`，在日志根目录（默认进程工作目录下的 `logs/`）下生成：

| 文件 | 内容 | 级别 | 上限 |
| --- | --- | --- | --- |
| `run.log` | 运行日志：调试与运行轨迹，**三档日志里唯一收 debug 的** | trace 及以上 | 512 KB（+ 1 份备份） |
| `warn.log` | 警告日志：warn / error / critical | warn 及以上 | 128 KB（+ 1 份备份） |
| `error.log` | 报错日志：error / critical | err 及以上 | 64 KB（+ 1 份备份） |
| `state.log` | 状态快照：`snapshot()` 每次整份 truncate 重写，**只有最后一份** | — | 由调用者控制 |

- **三档是「包含」关系而非互斥分流**：一条 `error` 同时出现在三个文件里，这样单拿 `error.log` 就能看全部报错，不必跨文件拼。
- 每个文件的实际磁盘占用上界是 `2 × 上限`（当前文件 + 1 份旧备份），**单源文件整套日志永久有界（约 1.4 MB）**，不需要人工清理。

## 函数

### `std::shared_ptr<spdlog::logger> file_logger(const std::string & source)`

- **作用**：取得「某个源文件专用」的三档覆盖式日志器；同一 `source` 重复调用返回**同一个实例**。
- **参数**：`source` 源文件标识，通常直接写相对项目根的路径（如 `"io/gimbal/gimbal.cpp"`），它会成为 `logs/` 下的文件夹名。
- **返回**：非空的 `shared_ptr<spdlog::logger>`，级别 `trace`、`flush_on(info)`、带一个彩色控制台 sink（info 及以上，带日志器名）。
- **备注 / 边界条件**：
  - 名字清洗规则（`sanitize()`，实现在 `.cpp`）：按 `/` 切段、丢掉空段与 `.`/`..`、每段只保留 `[A-Za-z0-9._-]`，其余字符（中文/空格/冒号）替换成 `_`；清洗后为空则用 `unknown`。因此**传绝对路径也不会写到日志根目录外面去**；
  - 首次调用时创建 `logs/<source>/` 目录与三个文件；目录建不出来直接抛 `spdlog::spdlog_ex`（不吞异常，避免「以为在记日志其实没记」）；
  - 实例常驻（静态表持有强引用），不存在「析构早了拿到野 logger」；
  - 推荐用法：在源文件里 `static auto logger = tools::file_logger("io/gimbal/gimbal.cpp");` 或包一层 `glog()` 函数，避免每次调用都进锁。

### `void snapshot(const std::string & source, const std::string & text)`

- **作用**：覆盖式写一份「最后一刻状态快照」到 `logs/<source>/state.log`（`std::ios::trunc` 整份重写）。
- **参数**：`source` 同 `file_logger()`；`text` 快照正文（多行用 `'\n'` 连接；末尾无换行时自动补一个）。
- **返回**：无。
- **备注**：存在的意义是「进程卡死 / 被 Ctrl-C / 段错误之后，`run.log` 可能还没来得及 flush，而 `state.log` 每秒被重写一次」——打开它就是崩溃前一秒的关键计数（收了多少字节、发了几帧、端口是否还开着）。单次写入是同长同步写，**调用者负责限频（建议 ≥ 500 ms 一次）**；目录不存在会自动建；任何异常（含无写权限）都被吞掉，因为它本身是尽力而为的诊断手段，不能反过来影响主流程。

### `void set_file_logger_root(const std::string & root)`

- **作用**：修改日志根目录（默认 `"logs"`，相对进程当前工作目录）。
- **参数**：`root` 新根目录；空字符串表示恢复默认。
- **返回**：无。
- **备注**：不会自动创建目录（创建由后续 `file_logger()` 建 sink 时完成）；**只应在创建任何 `file_logger` 之前调用**，已创建的日志器不受影响（sink 已绑定路径）。提供它是为了让离线验证/单测把日志写到临时目录，避免污染 `logs/`。
