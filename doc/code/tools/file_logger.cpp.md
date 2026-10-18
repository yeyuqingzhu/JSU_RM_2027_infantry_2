# tools/file_logger.cpp

**概要**：实现 [file_logger.hpp](file_logger.hpp.md)——静态表 `source → logger` 缓存实例、每个源文件装配「run/warn/error 三个 `rotating_file_sink_mt` + 一个彩色控制台 sink」、`snapshot()` 用 `ofstream(trunc)` 覆盖写 `state.log`。

- **来源**：本项目新增（非 sp_vision_25）
- **依赖**：spdlog（`sinks/rotating_file_sink.h`、`sinks/stdout_color_sinks.h`）、C++17 `<filesystem>`（建目录、路径归一）、`<fstream>`（快照）、`<map>`/`<mutex>`/`<string>`/`<vector>`
- **线程安全**：查表/建表/`snapshot()`/`set_file_logger_root()` 共用一把 `g_mutex`；sink 均为 `_mt` 版本，日志写入本身线程安全

## 匿名命名空间（常量与内部状态）

| 名字 | 值 / 类型 | 作用 |
| --- | --- | --- |
| `constexpr size_t kRunMaxBytes` | `512 * 1024` | `run.log` 单文件上限（覆盖十几次典型排查的时长） |
| `constexpr size_t kWarnMaxBytes` | `128 * 1024` | `warn.log` 上限 |
| `constexpr size_t kErrorMaxBytes` | `64 * 1024` | `error.log` 上限 |
| `constexpr size_t kMaxBackupFiles` | `1` | 只留 1 份备份：**新的覆盖旧的**（单档占用上界 `2 × max_size`） |
| `constexpr const char * kFilePattern` | `"[%Y-%m-%d %H:%M:%S.%e] [%l] %v"` | 文件排版（文件名已是源文件路径，不再重复写日志器名） |
| `constexpr const char * kConsolePattern` | `"[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%n] %v"` | 控制台排版（带颜色与日志器名） |
| `constexpr const char * kDefaultRoot` | `"logs"` | 默认日志根目录 |
| `std::mutex g_mutex` | — | 保护下面两个全局量的锁 |
| `std::string g_root` | 初值 `"logs"` | 当前日志根目录（可由 `set_file_logger_root()` 改） |
| `std::map<std::string, std::shared_ptr<spdlog::logger>> g_loggers` | — | 清洗后的 `source` → 日志器，强引用常驻 |

### `std::string sanitize(const std::string & source)`

- **作用**：把任意「源文件标识」清洗成安全的相对目录名。
- **参数**：`source` 原始标识（可含 `/`、绝对路径、中文等）。
- **返回**：清洗后的相对路径；清洗后为空时返回 `"unknown"`。
- **备注**：按 `/` 切段 → 丢弃空段与 `"."`/`".."` 段（**禁止向上跳，杜绝 `../` 逃出日志根目录**）→ 每段只保留 `[A-Za-z0-9._-]`，其余字符替换为 `_`。绝对路径开头的空段被丢弃，因此 `/etc/passwd` 只会变成 `logs/etc/passwd/`。

### `std::string logger_name(const std::string & sanitized)`

- **作用**：取清洗后路径的最后一段作为日志器名字（控制台里显示 `gimbal.cpp` 比显示整条路径清爽）。
- **参数**：`sanitized`（`sanitize()` 的输出）。
- **返回**：最后一段；无 `/` 时原样返回。
- **备注**：只影响控制台显示的 `[%n]`，不影响文件路径。

## 函数

### `std::shared_ptr<spdlog::logger> file_logger(const std::string & source)`

- **作用**：见头文件同名函数；这里是实现——持锁 → 用 `lexically_normal()` 归一化根目录 → `sanitize()` 得 key → 查表命中则直接返回 → 否则建三个 `rotating_file_sink_mt`（各设级别 `trace`/`warn`/`err` 与 `kFilePattern`）、一个 `stdout_color_sink_mt`（级别 `info`、`kConsolePattern`）→ 装配成 logger（级别 `trace`、`flush_on(info)`）→ 存表并返回。
- **参数**：`source` 源文件标识。
- **返回**：非空 `shared_ptr<spdlog::logger>`。
- **备注 / 边界条件**：
  - **为什么 `flush_on(info)`**：排查现场时进程常被 Ctrl-C 或直接崩，等 `warn` 才 flush 会丢掉关键上下文；本层日志已在 `gimbal.cpp` 侧限频，不构成性能负担；
  - **为什么文件 sink 级别是「递增包含」而不是互斥分流**：让 `error.log` 单独拿出来就是完整报错集，不必跨三个文件拼；
  - 目录不存在时由 `rotating_file_sink_mt` 自动递归创建；无写权限或路径非法会抛 `spdlog::spdlog_ex`（不吞异常）；
  - 已实测（`build/gimbal_verify/rotate_test.cpp`，不入库）：写 4 万行后 `run.log` 466 KB + `run.1.log` 512 KB、`warn.log` 22 KB、`error.log` 2 KB，**尺寸被牢牢压在上限内**。

### `void snapshot(const std::string & source, const std::string & text)`

- **作用**：持锁 → 归一化根目录 → `create_directories(logs/<source>/)` → `std::ofstream(dir/"state.log", std::ios::trunc)` 整份重写；正文末尾无换行时补一个。
- **参数**：`source` 同 `file_logger()`；`text` 快照正文。
- **返回**：无（`void`）。
- **备注**：整个函数体包在 `try/catch(...)` 里、且 `!out.is_open()` 时静默返回——快照是「尽力而为」的诊断手段，写不进去（只读盘、权限不足）绝不能反过来把主流程搞崩。调用频率由调用方控制（`gimbal.cpp` 里是 1 s 一次）。

### `void set_file_logger_root(const std::string & root)`

- **作用**：持锁修改 `g_root`；传空串恢复 `kDefaultRoot`。
- **参数**：`root` 新根目录。
- **返回**：无。
- **备注**：不建目录、不影响已创建的日志器（sink 在创建时已绑定绝对/相对路径）。
