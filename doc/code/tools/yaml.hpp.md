# tools/yaml.hpp

**概要**：把 YAML 配置文件的加载与取值收敛为三个 `inline` 函数——`load` 读取整个文件、`read` 按 key 取值（可带默认值）；配置缺失或解析失败时打印错误并直接 `exit(1)`，让配置错误在启动阶段暴露。

- **来源**：sp_vision_25 的 `tools/yaml.hpp`
- **依赖**：yaml-cpp（本机 0.8.0）、`tools/logger.hpp`（spdlog）；另显式包含 `<cstdlib>`（`std::exit` 的声明头）
- **线程安全**：函数无共享可写状态；`YAML::Node` 为值语义，多线程各自读取不同实例安全（同一 Node 并发只读也安全）。但错误路径会终止进程，不要在实时/业务线程里对“预期缺 key”的配置做探测
- **单位**：本文件只负责取值，不做单位换算
- **调用时机**：仅在程序初始化阶段读取配置；无缓存，运行期不要反复 `LoadFile`
- **行为约定**：key 不存在（或节点不是映射）→ 打印 error 并 `exit(1)`；key 存在但类型不匹配 → yaml-cpp 抛 `YAML::TypedBadConversion`，本文件不捕获

## 函数

### `inline YAML::Node load(const std::string & path)`

- **作用**：读取并解析指定路径的 YAML 文件，返回根节点。
- **参数**：`path` 配置文件路径（相对路径相对于进程当前工作目录），如 `"configs/standard.yaml"`。
- **返回**：`YAML::Node` 根节点；失败时不会返回（进程已 `exit(1)`）。
- **备注**：实现要点——`YAML::LoadFile(path)`；`YAML::BadFile`（文件不存在/无权限）与 `YAML::ParserException`（语法错误）两个 catch 分支都先 `logger()->error(...)` 再 `exit(1)`；其它 yaml-cpp 异常会向上抛出。迁移补充——函数末尾增加一行**不可达**的 `return YAML::Node{}`，用于在开启 `-Wreturn-type`/`-Werror` 时消除“非 void 函数走到末尾”的告警，语义与行为不变（实测 GCC 15 不告警，属防御性修补）。

### `template <typename T> inline T read(const YAML::Node & yaml, const std::string & key)`

- **作用**：读取映射节点 `yaml` 中 `key` 对应的值并转换为类型 `T`，缺失即终止进程。
- **参数**：`yaml` 已加载的映射节点；`key` 字段名。
- **返回**：类型为 `T` 的值（由 `YAML::Node::as<T>()` 转换）。
- **备注**：实现要点——用 `if (yaml[key])` 判断节点有效性（yaml-cpp 0.8.0 中 const 版 `operator[]` 对缺失 key 返回 Undefined 节点、`operator bool` 为 false，不会抛异常）；否则打印 error 并 `exit(1)`。边界——类型不匹配抛 `YAML::TypedBadConversion`。迁移补充——末尾增加不可达的 `return T{}`（要求 `T` 可默认构造），仅为消除 `-Wreturn-type` 告警。

### `template <typename T> inline T read(const YAML::Node & yaml, const std::string & key, const T & default_value)`

- **作用**：读取 `key` 对应的值；缺失时返回调用方给的默认值（**不**退出进程）。
- **参数**：`yaml` 映射节点；`key` 字段名；`default_value` 缺省值，类型必须与 `T` 一致。
- **返回**：key 存在时返回 `yaml[key].as<T>()`，否则返回 `default_value`（按值拷贝）。
- **备注**：key 缺失是正常流程；类型不匹配仍会抛 `YAML::TypedBadConversion`。常用于带默认值的 PID/阈值等可调参数。
