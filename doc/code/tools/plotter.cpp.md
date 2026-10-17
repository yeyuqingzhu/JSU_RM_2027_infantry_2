# tools/plotter.cpp

**概要**：`tools/plotter.hpp` 中 `Plotter` 类的实现——创建 UDP socket、填写目标地址、把 JSON 序列化后单播发送、析构时关闭 socket；全部逻辑基于 POSIX socket。

> 来源：sp_vision_25 `tools/plotter.cpp`（逐文件迁移，行为与协议一致）。
> 依赖：`<sys/socket.h>`（`socket` / `sendto`）、`<arpa/inet.h>`（`htons` / `inet_addr`）、`<unistd.h>`（`close`）、nlohmann/json（经头文件间接引入）。
> 注意事项：
> - **端口占用**：只创建发送用 UDP socket，不 `bind` 本地端口，不会因端口被占用而失败；目的端口由构造参数决定（默认 9870，须与上位机一致）。
> - **连接失败/失败行为**：`socket()` 失败（`socket_ = -1`）、`inet_addr` 解析失败（地址变成 255.255.255.255）、`sendto` 失败（接收端未启动）**都不会报错、不会抛异常**，只丢失这一帧调试数据。
> - **线程安全**：`plot()` 用 `mutex_` 串行化发送；构造/析构本身不加锁，需使用者保证对象生命周期内不被并发析构。
> - **异常面**：`json.dump()` 在数据含 NaN/Inf 时可能抛异常并向上传播，本实现不做捕获。

## 函数

### `Plotter::Plotter(std::string host, uint16_t port)`

- **作用**：构造绘图器——创建 UDP socket 并填写目标地址结构。
- **参数**：`host` 目标 IPv4 字符串；`port` 目标端口（主机字节序）。
- **返回**：无。
- **备注**（实现要点与关键步骤）：
  1. `socket_ = ::socket(AF_INET, SOCK_DGRAM, 0)` 创建 IPv4 UDP socket；失败返回 -1，后续发送静默失败；
  2. `destination_.sin_family = AF_INET` 指定 IPv4 地址族；
  3. `destination_.sin_port = ::htons(port)` 把端口转成网络字节序；
  4. `destination_.sin_addr.s_addr = ::inet_addr(host.c_str())` 解析点分十进制地址；`host` 非法时得到 `INADDR_NONE`（255.255.255.255）；
  5. 未初始化 `sockaddr_in` 的 `sin_zero` 填充字节（源工程遗留，见报告）。

### `Plotter::~Plotter()`

- **作用**：析构——释放 socket 描述符。
- **参数**：无。
- **返回**：无。
- **备注**（实现要点与关键步骤）：单行 `::close(socket_)`；`socket_` 非法（-1）时仅失败返回，无副作用；不发送任何结束标记，上位机靠超时判断数据流停止。

### `void Plotter::plot(const nlohmann::json & json)`

- **作用**：把一个 JSON 对象序列化后作为单个 UDP 数据报发出。
- **参数**：`json` 待发送的 JSON 对象。
- **返回**：无（返回值被有意忽略）。
- **备注**（实现要点与关键步骤）：
  1. `std::lock_guard<std::mutex> lock(mutex_)` 加锁，把同一实例上的发送串行化（UDP 单次 `sendto` 本身原子，此处保证调用顺序与序列化文本的独占）；
  2. `auto data = json.dump()` 序列化为紧凑 JSON 文本（UTF-8、无缩进、无换行）——即与上位机约定的报文内容；
  3. `::sendto(socket_, data.c_str(), data.length(), 0, reinterpret_cast<sockaddr *>(&destination_), sizeof(destination_))` 作为单个数据报发出；
  4. `sendto` 返回值（实际字节数或 -1）被忽略：调试通道不做重传与错误上报，接收端离线时丢包即可；
  5. `data.c_str()` 的 `length()` 与 `dump()` 的输出长度一致（JSON 文本中无内嵌 `\0`），故等价于整串发送。
