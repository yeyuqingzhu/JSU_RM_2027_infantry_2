# tools/exiter.cpp

**概要**：实现 `tools/exiter.hpp`——定义进程级退出标志与“是否已安装信号处理器”标志，并在 `Exiter` 构造时用 `std::signal` 注册 SIGINT 处理器。

- **来源**：sp_vision_25 的 `tools/exiter.cpp`
- **依赖**：`<csignal>`（`std::signal`、`SIGINT`）、`<stdexcept>`（`std::runtime_error`）
- **线程安全**：`exit_` 是一个普通 `bool`，由信号处理器写、业务线程读。C++ 标准不保证跨线程/信号的可见性（严格做法是 `volatile sig_atomic_t` 或 `std::atomic<bool>`），源工程使用 `bool`，迁移保持原样以免改变行为
- **调用时机**：`Exiter` 必须在程序启动早期构造；信号处理器只做“置位”，不调用任何非 async-signal-safe 函数
- **覆盖范围**：只处理 SIGINT（Ctrl+C），不处理 SIGTERM/SIGQUIT，也不在退出时做清理

## 全局对象

### `bool exit_ = false;`

- **作用**：全局退出标志，`false` = 继续运行，`true` = 已收到 SIGINT。
- **备注**：写侧在信号处理器中（异步信号上下文），读侧在业务线程中。

### `bool exiter_inited_ = false;`

- **作用**：是否已经安装过 SIGINT 处理器，用于阻止创建第二个 `Exiter`（防止重复注册）。

## 函数

### `Exiter::Exiter()`

- **作用**：检查唯一性并注册 SIGINT 处理器，处理器被调用时把 `exit_` 置为 `true`。
- **参数**：无。
- **返回**：无。
- **备注**：实现要点——`if (exiter_inited_) throw std::runtime_error(...)` 先做唯一性检查；`std::signal(SIGINT, [](int){ exit_ = true; })` 使用无捕获 lambda（可隐式转换为函数指针），信号编号参数被忽略。该注册会**覆盖默认行为**（默认 Ctrl+C 立即终止进程），因此 Ctrl+C 后进程不会退出，而是由业务线程轮询 `exit_` 后自行收尾。边界——第二次构造抛异常且不改变任何状态。

### `bool Exiter::exit() const`

- **作用**：返回全局退出标志的当前值。
- **参数**：无。
- **返回**：`true` 表示已收到 SIGINT。
- **备注**：实现要点——`return exit_;`，单条 `bool` 读取，无锁、可在任意线程任意频率调用。
