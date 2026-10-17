// =============================================================================
// tools/exiter.hpp —— Ctrl+C 退出标志（供各线程轮询的退出开关）
// -----------------------------------------------------------------------------
// 【作用】封装“收到 SIGINT 后置位”的退出标志：主线程构造 Exiter 安装信号处理器，
//         其它线程通过 exit() 查询，从而在收到 Ctrl+C 时优雅地结束主循环。
// 【来源】sp_vision_25 的 tools/exiter.hpp。
// 【依赖】无（实现仅用 <csignal>、<stdexcept>）。
// 【注意事项】
//   1) 线程安全：exit() 只读一个 bool，构造只做一次，读侧多线程并发安全；
//      写侧发生在信号处理器中（见 exiter.cpp 的说明）。
//   2) 单位/时机：无单位；必须在任何线程轮询 exit() 之前构造 Exiter 并保持其生命周期，
//      否则析构后信号处理器仍指向已释放状态（本类未提供恢复措施）。
//   3) 全局只能有一个 Exiter 实例，重复构造会抛 std::runtime_error。
//   4) 接口签名与源工程一致，不得改动。
// =============================================================================

#ifndef TOOLS__EXITER_HPP
#define TOOLS__EXITER_HPP

namespace tools
{
// Exiter —— 进程级退出标志的持有者。
// 职责：构造时安装 SIGINT 信号处理器，把内部静态标志置为 true；
//       通过 exit() 把该标志暴露给业务线程用于退出主循环。
// 用法：在 main 中构造一个实例（作为“信号处理器已安装”的凭证），
//       在各工作线程循环条件里写 while (!exiter.exit())。
class Exiter
{
public:
  // 作用：构造实例并安装 SIGINT 处理器（捕获 Ctrl+C）。
  // 参数：无。
  // 返回：无。
  // 边界：进程内第二次构造会抛 std::runtime_error("Multiple Exiter instances!")，
  //       因此该对象必须是全局唯一且生命周期覆盖整个程序的。
  Exiter();

  // 作用：查询是否已经收到过 SIGINT。
  // 参数：无。
  // 返回：true 表示进程已收到 Ctrl+C（应尽快退出主循环），false 表示继续运行。
  // 边界：未构造 Exiter 时调用返回 false（标志为静态全局，初值 false）。
  // 备注：const 成员函数，只读取标志，可在任意线程反复调用。
  bool exit() const;
};

}  // namespace tools

#endif  // TOOLS__EXITER_HPP