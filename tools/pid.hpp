#ifndef TOOLS__PID_HPP
#define TOOLS__PID_HPP

// =============================================================================
// 文件：tools/pid.hpp
// -----------------------------------------------------------------------------
// 【本文件作用】
//   PID 控制器通用件。位置式 PID：输出 = P 项 + I 项 + D 项，I 项带独立限幅
//   （积分抗饱和），总输出再做一次限幅；D 项作用在**反馈量**上（对反馈微分，
//   de = 上次反馈 − 本次反馈），而不是作用在误差 e = set − fdb 上，因此设定值
//   突变时不会产生微分冲击（derivative kick）。
//   通过构造参数 angular 可切换到角度模式：误差与反馈增量都用 tools::limit_rad
//   归一到 (-π, π]，避免 yaw / pitch 在 ±π 处跳变导致输出翻转。
//
// 【来源】
//   sp_vision_25 工程的 tools/pid.hpp（逐文件迁移）。
//   类名、成员名、函数签名、默认参数、成员初值与源工程完全一致，原有中文注释
//   原样保留，本次迁移只增加中文注释。
//
// 【依赖】
//   仅 C++ 标准库（.cpp 中经 math_tools.hpp 使用 tools::limit_rad 与 std::max/min）。
//
// 【注意事项】
//   1. 单位与量纲：dt 为 s；set / fdb 与返回值同量纲（角度模式为 rad，其余由调用方
//      决定，如 m、m/s）；kp / ki / kd 的量纲随之确定（ki 含 1/s，kd 含 s）。
//   2. dt_ = 0 会导致 D 项除零（得到 inf / NaN），构造函数不做校验，调用方必须传入
//      真实的控制周期（即 calc 的调用间隔）。
//   3. 首拍效应：last_fdb_ 初值为 0，若第一次调用时反馈量本身不为 0，则第一拍的
//      de = 0 − fdb 是一次"虚假"跳变，D 项会有一个冲击。需要平滑启动时应先空跑
//      一次 calc 或用实际反馈初始化 last_fdb_（该类未提供该接口）。
//   4. I 项累加是"无条件"的（不在输出饱和时停止积分）：抗饱和只靠 iout 的限幅，
//      即 clamp 积分器（conditional-free clamping），不是 back-calculation。
//   5. 本类持有状态（iout / last_fdb_），非线程安全；同一实例只能由控制线程调用，
//      且调用间隔应恒等于 dt_。
// =============================================================================

namespace tools
{
/**
 * 位置式 PID 控制器
 *
 * 职责：
 *   保存控制周期与三组增益，在每个控制周期调用一次 calc(set, fdb) 得到控制量。
 *   内部维护 I 项累加值与上次反馈值；三项分输出（pout / iout / dout）对外可见，
 *   便于上位机调试与整定。
 *
 * 典型用法：
 *   tools::PID pid(0.005, 8.0, 0.0, 0.02, 10.0, 3.0, true);  // 角度环
 *   float u = pid.calc(yaw_set, yaw_fdb);                     // 每 5 ms 调用一次
 */
class PID
{
public:
  /**
   * 作用：构造函数，保存控制周期、三组增益与两个限幅值；所有参数都是 const 成员，
   *       构造后不可修改（更改增益需重新构造对象）。
   * 参数：
   *   - dt：控制周期，单位 s，必须 > 0（D 项要除以它）。
   *   - kp / ki / kd：P / I / D 项系数，量纲取决于 set / fdb 的物理量。
   *   - max_out：总输出限幅（对称限幅，输出被限制在 [−max_out, +max_out]）。
   *   - max_iout：I 项单独限幅（对称），用于防止积分饱和。
   *   - angular：是否角度模式，默认 false；为 true 时误差与反馈增量按 (-π, π] 归一。
   * 返回：无（构造函数）。
   * 边界条件：不做参数校验；dt = 0 会在 calc 中除零，max_iout = 0 会使 I 项恒为 0，
   *          max_out = 0 会使输出恒为 0。
   */
  // dt: 控制周期, 单位: s
  // kp: P项系数
  // ki: I项系数
  // kd: D项系数
  // max_out: PID最大输出值
  // max_iout I项最大输出值
  PID(float dt, float kp, float ki, float kd, float max_out, float max_iout, bool angular = false);

  float pout = 0.0f;  // P项输出, 用于调试
  float iout = 0.0f;  // I项输出, 用于调试
  float dout = 0.0f;  // D项输出, 用于调试

  /**
   * 作用：执行一次 PID 计算：e = set − fdb（角度模式先归一），
   *       de = last_fdb − fdb（角度模式先归一），
   *       pout = kp·e，iout = clamp(iout + ki·dt·e, ±max_iout)，dout = kd·de/dt，
   *       返回值 = clamp(pout + iout + dout, ±max_out)。
   * 参数：
   *   - set：目标值（设定值），量纲与 fdb 相同；角度模式下单位 rad。
   *   - fdb：反馈值（当前测量值），量纲与 set 相同；角度模式下单位 rad。
   * 返回：限幅后的控制输出，范围 [−max_out, +max_out]（max_out > 0 时）。
   * 边界条件：
   *   - dt_ = 0 → dout 为 ±inf / NaN，返回值不可用；
   *   - 角度模式下 set − fdb 会被归一，超出 ±π 的误差不会产生反向输出；
   *   - 首次调用时 last_fdb_ 为 0，可能产生一次 D 项冲击（见文件头注意事项 3）；
   *   - 本函数会修改 pout / iout / dout 与 last_fdb_，非线程安全。
   */
  // 计算PID输出值
  // set: 目标值
  // fdb: 反馈值(feedback)
  float calc(float set, float fdb);

private:
  /** 控制周期，单位 s；同时是 calc 的期望调用间隔，D 项除以它得到微分。 */
  const float dt_;

  // kp_：P 项系数，输出与当前误差成正比；
  // ki_：I 项系数，输出与误差对时间的积分成正比（量纲含 1/s）；
  // kd_：D 项系数，输出与反馈量的变化率成正比（量纲含 s）。
  const float kp_, ki_, kd_;

  // max_out_：总输出限幅（对称），单位与返回的控制量相同；
  // max_iout_：I 项限幅（对称），单位与 I 项输出相同，用于防止积分饱和。
  const float max_out_, max_iout_;

  /** 是否角度模式：true 时用 tools::limit_rad 把误差与反馈增量归一到 (-π, π]。 */
  const bool angular_;

  float last_fdb_ = 0.0f;  // 上次反馈值
};

}  // namespace tools

#endif  // TOOLS__PID_HPP
