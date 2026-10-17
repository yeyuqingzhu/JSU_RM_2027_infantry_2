// =============================================================================
// 文件：tools/pid.cpp
// -----------------------------------------------------------------------------
// 【本文件作用】
//   tools/pid.hpp 中 PID 控制器的实现：构造函数保存参数、calc() 完成一次
//   位置式 PID 计算（P + I + D，I 项限幅、总输出限幅），以及文件内的对称限幅
//   辅助函数 clip()。
//
// 【来源】
//   sp_vision_25 工程的 tools/pid.cpp（逐文件迁移）。
//   计算顺序、公式、限幅方式与源工程完全一致，本次迁移只增加中文注释。
//
// 【依赖】
//   - tools/pid.hpp（同目录，"..." 相对包含，保持源工程写法）
//   - tools/math_tools.hpp（源工程同目录头文件，提供 tools::limit_rad；
//     角度模式必须链接该实现，见迁移报告中的依赖说明）
//   - C++ 标准库：std::max / std::min（经 Eigen 与 math_tools.hpp 间接引入，
//     源文件未显式包含 <algorithm>，见迁移报告中的"源文件缺陷"）
//
// 【注意事项】
//   1. 单位：dt_ 为 s，角度模式下 set / fdb 为 rad；其余量纲由调用方决定。
//   2. 数值稳定性：dt_ = 0 时 dout = de/dt 会得到 ±inf / NaN，构造函数不校验。
//   3. 调用时机：本文件带状态（iout、last_fdb_），必须在同一个控制线程里以恒定
//      周期 dt_ 调用 calc()，且不可多线程并发调用。
//   4. clip() 是文件级辅助函数：源工程把它定义在**全局命名空间**且未加 static，
//      存在与其他翻译单元重名冲突（ODR）的隐患，迁移时保持原样，详见迁移报告。
// =============================================================================

#include "pid.hpp"

#include "math_tools.hpp"

/**
 * 作用：对称限幅辅助函数，把 value 限制到 [min, max] 区间内。
 * 参数：value 待限幅的值；min 下界；max 上界（要求 min ≤ max）。
 * 返回：限幅后的值：value < min 时返回 min，value > max 时返回 max，否则原样返回。
 * 边界条件：min > max 时返回 min（std::min(max, value) 先取小，再 std::max(min, …)）；
 *          value 为 NaN 时比较全部为假，结果返回 min。
 * 备注：源工程将本函数定义在全局命名空间且未加 static —— 既是"函数未声明"类告警的
 *       来源，也可能与其他翻译单元的同名符号冲突；迁移时按"行为完全一致"要求保留
 *       原样，仅在报告中说明。参数名 min / max 与 std::min / std::max 同名，但函数
 *       体内使用的是带 std:: 限定的版本，不受影响。
 */
float clip(float value, float min, float max) { return std::max(min, std::min(max, value)); }

namespace tools
{
/**
 * 作用：构造函数，用初始化列表按声明顺序保存控制周期、三组增益、两个限幅值与
 *       角度模式标志；内部状态 iout / last_fdb_ 由成员初值给出（0）。
 * 参数：dt 控制周期 (s)；kp / ki / kd P/I/D 系数；max_out 总输出限幅；
 *       max_iout I 项限幅；angular 是否角度模式。
 * 返回：无。
 * 边界条件：不做参数校验（dt = 0、限幅为 0 等非法取值都会被原样接受，
 *          在 calc() 中表现为除零或恒零输出）。
 * 备注：所有参数成员都是 const，构造后不可修改；需要改增益只能重建对象。
 */
PID::PID(float dt, float kp, float ki, float kd, float max_out, float max_iout, bool angular)
: dt_(dt), kp_(kp), ki_(ki), kd_(kd), max_out_(max_out), max_iout_(max_iout), angular_(angular)
{
}

/**
 * 作用：执行一次位置式 PID 计算。步骤：
 *         1) 误差 e = set − fdb；角度模式下先用 tools::limit_rad 归一到 (-π, π]；
 *         2) 反馈增量 de = last_fdb_ − fdb（角度模式同样归一），即"对反馈微分"，
 *            可避免设定值突变引起的微分冲击；
 *         3) 记录本次反馈，供下一拍使用；
 *         4) pout = kp·e；iout = clip(iout + ki·dt·e, ±max_iout)（积分累加 + 抗饱和）；
 *            dout = kd·de/dt；
 *         5) 返回 clip(pout + iout + dout, ±max_out)。
 * 参数：set 目标值；fdb 反馈值（量纲与 set 相同，角度模式下单位为 rad）。
 * 返回：限幅后的控制输出，范围 [−max_out, +max_out]（max_out > 0 时）。
 * 边界条件：
 *   - dt_ = 0 → dout 为 ±inf / NaN（本实现不保护）；
 *   - 首次调用 last_fdb_ = 0，若 fdb 初值非 0 则 D 项有一次冲击；
 *   - 返回值恒在 ±max_out 内，即使内部某项已经饱和。
 * 备注：三项分输出写在成员 pout / iout / dout 上，供外部调试读取；
 *       本函数修改对象状态（iout、last_fdb_、pout、dout），非线程安全。
 */
float PID::calc(float set, float fdb)
{
  // 误差项：角度模式先把误差折算到 (-π, π]，避免跨越 ±π 时输出反向。
  float e = angular_ ? limit_rad(set - fdb) : (set - fdb);
  // 微分项：对"反馈量"求差分（负号体现在 last_fdb_ − fdb），而非对误差求差分，
  // 因此改变 set 不会在 D 项上产生尖峰；角度模式同样做环绕归一。
  float de = angular_ ? limit_rad(last_fdb_ - fdb) : (last_fdb_ - fdb);
  last_fdb_ = fdb;

  this->pout = e * kp_;
  // 积分项：先累加再限幅（clamp 型抗饱和），积分值不会无限增长。
  this->iout = clip(this->iout + e * dt_ * ki_, -max_iout_, max_iout_);
  this->dout = de / dt_ * kd_;

  // 总输出限幅：PI D 三项之和再夹到 ±max_out。
  return clip(this->pout + this->iout + this->dout, -max_out_, max_out_);
}

}  // namespace tools
