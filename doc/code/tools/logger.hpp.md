# tools/logger.hpp

**概要**：声明全局日志器（基于 spdlog）的唯一对外接口 `tools::logger()`，上层模块用它输出日志，日志同时写入 `logs/` 下的文件与彩色控制台。

- **来源**：sp_vision_25 的 `tools/logger.hpp`
- **依赖**：spdlog（本机 1.15.3，系统包以 `SPDLOG_FMT_EXTERNAL=1` 构建，链接 libfmt）
- **线程安全**：spdlog logger 本身线程安全（多线程写同一条日志由内部互斥保护）；但 `logger()` 的首次惰性创建不是原子的，多线程首次并发调用存在竞态，建议在程序入口先调用一次
- **格式约定**：日志格式化语法为 spdlog/fmt 的 `{}` 占位符（不是 printf 的 `%d`）
- **调用时机**：可在任意时刻调用；返回的 `shared_ptr` 常驻，不会因局部变量析构而失效。首次调用会在当前工作目录写 `logs/<时间戳>.log`，spdlog 1.15 会自动创建缺失的 `logs/` 目录

## 函数

### `std::shared_ptr<spdlog::logger> logger()`

- **作用**：返回全局日志器（单例，惰性初始化）。
- **参数**：无。
- **返回**：`std::shared_ptr<spdlog::logger>`；首次调用时创建“文件 + 彩色控制台”双 sink 的日志器（级别 debug，`flush_on(info)`）。
- **备注**：`logs/` 缺失时会被自动创建；目录无法创建或无写权限时底层 `basic_file_sink` 构造失败并抛 `spdlog::spdlog_ex`，本函数不捕获、也不会返回 `nullptr`。用法：`tools::logger()->info("{}", value);`
