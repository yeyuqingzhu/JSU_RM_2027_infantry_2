# tools/exiter.hpp

**概要**：声明 `Exiter` 类——封装“收到 SIGINT 后置位”的进程级退出标志，主线程构造它安装信号处理器，其它线程通过 `exit()` 查询以实现优雅退出。

- **来源**：sp_vision_25 的 `tools/exiter.hpp`
- **依赖**：无（实现仅用 `<csignal>`、`<stdexcept>`）
- **线程安全**：`exit()` 只读一个 `bool`，多线程并发读安全；写侧发生在信号处理器中（见 `exiter.cpp`）
- **调用时机**：必须在任何线程轮询 `exit()` 之前构造，且生命周期覆盖整个程序；全局只能有一个实例

## 类

### `class Exiter`

- **职责**：进程级退出标志的持有者——构造时安装 SIGINT 信号处理器（把内部静态标志置为 true），通过 `exit()` 把该标志暴露给业务线程。
- **成员变量**：类本身没有成员变量，退出标志与“已初始化”标志以文件级静态变量形式定义在 `exiter.cpp` 中。
- **用法**：在 `main` 中构造一个实例作为“信号处理器已安装”的凭证，工作线程循环条件写 `while (!exiter.exit())`。

## 函数

### `Exiter::Exiter()`

- **作用**：构造实例并安装 SIGINT 处理器（捕获 Ctrl+C）。
- **参数**：无。
- **返回**：无。
- **备注**：进程内第二次构造会抛 `std::runtime_error("Multiple Exiter instances!")`，因此该对象必须全局唯一。

### `bool Exiter::exit() const`

- **作用**：查询是否已经收到过 SIGINT。
- **参数**：无。
- **返回**：`true` 表示已收到 Ctrl+C（应尽快退出主循环）；`false` 表示继续运行。
- **备注**：`const` 成员函数，只读取标志，可在任意线程反复调用；未构造 `Exiter` 时调用返回 `false`（静态标志初值为 false）。
