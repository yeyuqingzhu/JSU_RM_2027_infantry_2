// =============================================================================
// 文件：tools/plotter.cpp
// -----------------------------------------------------------------------------
// 【本文件作用】
//   tools/plotter.hpp 中 Plotter 类的实现：创建 UDP socket、填写目标地址、
//   把 JSON 序列化后单播发送、析构时关闭 socket。全部逻辑依赖 POSIX socket。
//
// 【来源】
//   sp_vision_25 工程的 tools/plotter.cpp（逐文件迁移，行为与协议保持一致）。
//
// 【依赖】
//   - POSIX socket：<sys/socket.h>（socket/sendto）、<arpa/inet.h>（htons/inet_addr）、
//     <unistd.h>（close）
//   - nlohmann/json（经头文件间接引入）
//
// 【注意事项】
//   1. 端口占用：只创建发送用 UDP socket，不 bind 本地端口，因此不会因为端口被
//      占用而失败；报文目的端口由构造参数决定（默认 9870，需与上位机一致）。
//   2. 失败时的行为：socket() 失败（socket_ = -1）、inet_addr 解析失败
//      （地址变成 255.255.255.255）、sendto 失败（接收端未启动等）都不会报错、
//      不会抛异常，只是这一帧调试数据静默丢失 —— 调试通道不阻断主流程。
//   3. 线程安全：plot() 用 mutex_ 串行化发送；构造/析构本身不加锁，要求使用者
//      保证对象生命周期内不被并发析构。
//   4. JSON 序列化可能抛异常（例如容器内出现 NaN/Inf 且未设置 dump 的错误处理
//      策略时为默认行为）；异常会向上传播给调用方，本实现不做捕获。
// =============================================================================

#include "plotter.hpp"

#include <arpa/inet.h>   // htons, inet_addr
#include <sys/socket.h>  // socket, sendto
#include <unistd.h>      // close

namespace tools
{
Plotter::Plotter(std::string host, uint16_t port)
{
  // 创建 IPv4 的 UDP socket（SOCK_DGRAM）。失败时返回 -1，后续发送会静默失败。
  socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);

  // 填写目标地址：地址族固定为 IPv4，端口转网络字节序，地址由点分十进制字符串解析。
  // host 非法时 inet_addr 返回 INADDR_NONE（255.255.255.255），报文将发往广播地址。
  destination_.sin_family = AF_INET;
  destination_.sin_port = ::htons(port);
  destination_.sin_addr.s_addr = ::inet_addr(host.c_str());
}

// 析构：释放 socket 描述符。socket_ 非法（-1）时 close 仅失败返回，无副作用。
Plotter::~Plotter() { ::close(socket_); }

void Plotter::plot(const nlohmann::json & json)
{
  // 加锁：把同一实例上的发送串行化，避免多线程同时复用 socket_ 造成竞争
  //（UDP 单个 sendto 本身是原子的，但这里保证调用顺序与 JSON dump 的独占性）。
  std::lock_guard<std::mutex> lock(mutex_);

  // 序列化为紧凑 JSON 文本（UTF-8、无缩进、无换行）—— 这就是与上位机的报文内容。
  auto data = json.dump();

  // 作为单个 UDP 数据报发出。返回值（实际发出的字节数，失败为 -1）被有意忽略：
  // 调试通道不做重传与错误上报，接收端不在线时丢包即可。
  ::sendto(
    socket_, data.c_str(), data.length(), 0, reinterpret_cast<sockaddr *>(&destination_),
    sizeof(destination_));
}

}  // namespace tools
