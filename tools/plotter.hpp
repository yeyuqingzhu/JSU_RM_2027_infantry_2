#ifndef TOOLS__PLOTTER_HPP
#define TOOLS__PLOTTER_HPP

// =============================================================================
// 文件：tools/plotter.hpp
// -----------------------------------------------------------------------------
// 【本文件作用】
//   调试数据“绘图器”的发送端接口。把上层组装好的 nlohmann::json 对象序列化成
//   紧凑 JSON 文本，通过 UDP 单播发给本机运行的配套上位机绘图程序，用于实时观察
//   算法内部量（观测器状态量、残差、卡方检验、云台响应等）的曲线变化。
//
// 【来源】
//   sp_vision_25 工程的 tools/plotter.hpp（逐文件迁移，接口与 socket 协议一致）。
//
// 【依赖】
//   - POSIX socket：<netinet/in.h>（sockaddr_in），实现文件另用 <sys/socket.h>、
//     <arpa/inet.h>、<unistd.h>
//   - nlohmann/json（3.11.3，头文件 /usr/include/nlohmann/json.hpp）
//   - C++ 标准库：<mutex>、<string>
//
// 【注意事项】
//   1. 端口占用：本类只做 UDP 发送，不 bind、不 listen，因此不会占用本地端口，
//      也不存在“端口被占用”而构造失败的情况；只要目标地址可达即可发送。
//   2. 连接失败：UDP 无连接，接收端不存在时 sendto 不报错，数据包被静默丢弃；
//      构造时 socket() 失败或 host 非法同样不抛异常（见实现说明）。
//   3. 线程安全：plot() 内部用 mutex_ 串行化，同一个实例可被多个线程安全调用；
//      不同实例之间不共享锁，各自独立。
//   4. 失败时的行为：发送失败不会抛出异常、不会终止程序、不会阻塞调用方，
//      仅仅丢失这一帧调试数据 —— 调试功能不应影响主流程。
//   5. 协议固定：目标端口默认 9870，报文为 json.dump() 产生的紧凑 JSON（UTF-8、
//      无换行），一次 plot() 对应一个 UDP 数据报。移植时不得改动，否则上位机不认。
// =============================================================================

#include <netinet/in.h>  // sockaddr_in

#include <mutex>
#include <nlohmann/json.hpp>
#include <string>

namespace tools
{
/**
 * 类职责：调试数据绘图器（UDP 发送端）。
 *
 *   构造时创建 UDP socket 并填写目标地址；每次 plot() 把传入的 json 序列化为
 *   紧凑 JSON 文本，作为单个 UDP 数据报发往 host:port（默认 127.0.0.1:9870）。
 *   接收方是配套的上位机绘图程序，负责把收到的 JSON 按键名解析并画成实时曲线。
 *
 *   生命周期：一个实例持有一个 socket，析构时关闭；实例不可拷贝（拷贝会导致
 *   两个对象持有同一描述符并在析构时二次 close，见实现文件的缺陷说明）。
 */
class Plotter
{
public:
  /**
   * 作用：构造绘图器，创建 UDP socket 并填写目标地址。
   * 参数：
   *   - host：目标主机地址（点分十进制 IPv4 字符串），默认 "127.0.0.1"（本机）。
   *   - port：目标端口号，主机字节序的 uint16_t，默认 9870；内部用 htons 转网络序。
   * 返回：无。
   * 边界条件：host 不是合法 IPv4 字符串时 inet_addr 返回 INADDR_NONE
   *           （255.255.255.255），报文会被当成广播地址发出；socket() 失败时
   *           socket_ 为 -1，后续发送静默失败。两种情况都不抛异常。
   */
  Plotter(std::string host = "127.0.0.1", uint16_t port = 9870);

  /**
   * 作用：析构，关闭构造时创建的 UDP socket。不发送任何结束标记。
   * 参数：无。
   * 返回：无。
   * 边界条件：socket_ 为 -1 时 close(-1) 只是失败返回，无副作用。
   */
  ~Plotter();

  /**
   * 作用：把 JSON 对象序列化后作为单个 UDP 数据报发送出去。
   * 参数：
   *   - json：待发送的数据。必须能被 nlohmann::json::dump() 序列化（含 NaN/Inf
   *           等非有限浮点数时会抛异常，见备注）。
   * 返回：无（sendto 的返回值被忽略，发送失败不报错）。
   * 备注：内部加 mutex_ 串行化，多线程可安全调用；报文不加换行符，也没有任何
   *       应答确认；sendto 本身是非阻塞的（不设置阻塞等待），通常微秒级返回。
   */
  void plot(const nlohmann::json & json);

private:
  int socket_;               ///< UDP socket 文件描述符：构造时创建，析构时 close。
  sockaddr_in destination_;  ///< 目标地址结构（IPv4 族 + 端口 + 地址），构造后不变。
  std::mutex mutex_;         ///< 串行化 plot() 的发送，保证多线程调用安全。
};

}  // namespace tools

#endif  // TOOLS__PLOTTER_HPP
