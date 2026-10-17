// =============================================================================
// 文件：tools/trajectory.cpp
// -----------------------------------------------------------------------------
// 【本文件作用】
//   tools/trajectory.hpp 中 Trajectory 构造函数的实现：用真空抛体模型把
//   (v0, d, h) 化为关于 tan(pitch) 的一元二次方程，求根后取"飞行时间更短"的那条
//   弹道，输出俯仰角 pitch（rad，抬头为正）与飞行时间 fly_time（s）。
//
// 【来源】
//   sp_vision_25 工程的 tools/trajectory.cpp（逐文件迁移）。
//   重力加速度取值 9.7833、二次方程系数写法、双解取舍规则（比 t1 / t2 取小）与
//   源工程完全一致，本次迁移只增加中文注释。
//
// 【依赖】
//   仅 C++ 标准库 <cmath>（std::sqrt / std::atan / std::cos）；无第三方依赖。
//
// 【注意事项】
//   1. 单位：v0 (m/s)、d (m)、h (m)、pitch (rad)、fly_time (s)；h 上正下负。
//   2. 数值稳定性：判别式 delta = b² − 4ac 是两个大数相减，极端参数（超远距离、
//      极小初速）下会出现有效位损失；d = 0 时 a = 0，后面的除法会得到 inf / NaN，
//      本实现不做保护（源工程行为，见迁移报告中的"源文件缺陷"）。
//   3. 无解处理：delta < 0 时置 unsolvable = true 并**直接 return**，不计算角度与
//      时间，成员保持默认值 0（迁移时补上了默认成员初始化，见迁移报告）。
//   4. 调用时机：构造函数即完成解算，纯计算、无副作用、可高频调用；
//      v0 必须使用与实弹一致的初速标定值。
// =============================================================================

#include "trajectory.hpp"

#include <cmath>

namespace tools
{
// 源工程使用的重力加速度取值：9.7833 m/s²（非标准值 9.80665，为按实测弹道
// 标定出的等效重力加速度），单位 m/s²。迁移时原样保留，勿改为标准值。
constexpr double g = 9.7833;

/**
 * 作用：弹道解算构造函数。推导过程（真空抛体模型）：
 *         水平方向 d = v0·cosθ·t，竖直方向 h = v0·sinθ·t − g·t²/2，
 *         消去 t 并令 T = tanθ，得到关于 T 的一元二次方程
 *             a·T² + b·T + c = 0，
 *         其中 a = g·d² / (2·v0²)、b = −d、c = a + h；
 *         判别式 delta = b² − 4·a·c，两根 T = (−b ± √delta) / (2a)，
 *         对应高低两条弹道，飞行时间 t = d / (v0·cosθ)；
 *         取 t 较小的那一支（低伸平射弹道），写入 pitch 与 fly_time。
 * 参数：v0 子弹初速度大小 (m/s)；d 目标水平距离 (m)；h 目标相对枪口的高度差 (m)。
 * 返回：无（构造函数）；结果写入 unsolvable / pitch / fly_time。
 * 边界条件：
 *   - delta < 0：距离超出射程或初速过小 → unsolvable = true，立即返回，
 *     pitch / fly_time 保持默认值 0（调用方必须先判断 unsolvable）；
 *   - d = 0：a = 0，后续 (−b ± √delta) / (2a) 出现 0/0 → NaN，本实现不保护；
 *   - v0 = 0：a 为 inf / NaN，结果不可用，本实现不保护；
 *   - cosθ 不会为 0（|θ| < π/2，θ = atan(有限实数)），故 t = d/(v0·cosθ) 不除零。
 * 备注：pitch 可以为负（目标低于枪口时压低）；两条弹道中还用"时间更短"作为取舍
 *       依据，与源工程一致。
 */
Trajectory::Trajectory(const double v0, const double d, const double h)
{
  // 二次项系数 a = g·d²/(2·v0²)，一次项系数 b = −d，常数项 c = a + h。
  auto a = g * d * d / (2 * v0 * v0);
  auto b = -d;
  auto c = a + h;
  // 判别式：< 0 表示该初速打不到（d, h）这个点，直接标记无解。
  auto delta = b * b - 4 * a * c;

  if (delta < 0) {
    unsolvable = true;
    return;
  }

  unsolvable = false;
  // 两根（对应高抛与平射两条弹道）：tanθ = (−b ± √delta) / (2a)。
  auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
  auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
  auto pitch_1 = std::atan(tan_pitch_1);
  auto pitch_2 = std::atan(tan_pitch_2);
  // 由水平方程反解飞行时间；cos 为正，故 t > 0。
  auto t_1 = d / (v0 * std::cos(pitch_1));
  auto t_2 = d / (v0 * std::cos(pitch_2));

  // 取飞行时间更短的一支（低伸平射弹道）：暴露时间短、受重力与风偏影响小。
  pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
  fly_time = (t_1 < t_2) ? t_1 : t_2;
}

}  // namespace tools
