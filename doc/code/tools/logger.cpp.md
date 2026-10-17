# tools/logger.cpp

**概要**：实现 `tools::logger()`——首次调用时创建 `logs/<时间戳>.log` 文件 sink 与彩色控制台 sink（级别均 debug，info 及以上立即 flush）。

- **来源**：sp_vision_25 的 `tools/logger.cpp`
- **依赖**：spdlog 1.15.3（`sinks/basic_file_sink.h`、`sinks/stdout_color_sinks.h`）、fmt（`<fmt/chrono.h>` 提供 time_point 格式化）、C++17 `<chrono>`、`<string>`
- **迁移说明**：代码与源文件一致，直接使用 `fmt::format`（曾短暂改为 `spdlog::fmt_lib` 门面，后为保持与源逐字一致而回退）。注意本机 spdlog 为 `SPDLOG_FMT_EXTERNAL=1` 构建版，**任何** spdlog 格式化在链接期都需要 libfmt，故 [`tools/CMakeLists.txt`](../../../tools/CMakeLists.txt) 中的 `fmt::fmt` 是必需依赖，不可移除
- **线程安全**：spdlog logger 本身线程安全；全局指针 `logger_` 的“检查并创建”不是原子的，多线程首次并发调用可能重复创建，建议入口处先调用一次

## 全局对象

### `std::shared_ptr<spdlog::logger> logger_ = nullptr;`

- **作用**：全局日志器实例，`nullptr` 表示尚未初始化。
- **备注**：源工程未加 `static`，迁移保持原样；仅在本文件内被 `logger()` / `set_logger()` 使用。

## 函数

### `void set_logger()`

- **作用**：构造文件 sink 与控制台 sink，装配成全局 `logger_`（实现细节，未在头文件中声明，外部不可见）。
- **参数**：无。
- **返回**：无（写入全局 `logger_`）。
- **备注**：实现要点——文件名 `logs/<当前时间 %Y-%m-%d_%H-%M-%S>.log`（fmt 对 time_point 的 `%S` 带小数秒，实际形如 `2026-10-11_09-26-33.397172716.log`，几乎不会重名）；文件 sink 用 `_mt` 多线程安全版本并传 `truncate = true`；两个 sink 与 logger 级别都是 debug，`flush_on(info)`。边界——`logs/` 缺失会自动创建，无法创建或无写权限时抛 `spdlog::spdlog_ex`；重复调用会丢弃旧 logger（旧 `shared_ptr` 释放，文件句柄关闭）。

### `std::shared_ptr<spdlog::logger> logger()`

- **作用**：返回全局日志器，首次调用时惰性初始化。
- **参数**：无。
- **返回**：`logger_` 的 `shared_ptr`（除非构造抛异常，否则不为 `nullptr`）。
- **备注**：实现要点——`if (!logger_) set_logger(); return logger_;`，无锁；多线程首次并发调用可能重复创建（源工程行为，保持一致）。
