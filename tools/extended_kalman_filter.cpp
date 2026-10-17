// =============================================================================
// 文件：tools/extended_kalman_filter.cpp
// -----------------------------------------------------------------------------
// 【本文件作用】
//   EKF 通用件 tools/extended_kalman_filter.hpp 的实现：
//   带初值构造函数的初始化、predict 的两个重载、update 的两个重载（含 Joseph
//   形式的协方差更新与卡方检验 NIS / NEES 统计）。
//
// 【来源】
//   sp_vision_25 工程的 tools/extended_kalman_filter.cpp（逐文件迁移）。
//   所有数学公式、迭代/分支逻辑、常量取值（含卡方阈值 0.711）与源工程完全一致，
//   原有中文注释全部原样保留，本次迁移只增加中文注释。
//
// 【依赖】
//   - tools/extended_kalman_filter.hpp（同目录，"..." 相对包含，保持源工程写法）
//   - Eigen3（经头文件间接引入）：矩阵乘法、转置、inverse()
//   - C++ 标准库：<numeric>（std::accumulate 统计滑动窗口失败次数）
//
// 【注意事项】
//   1. 数值稳定性：协方差更新用 Joseph 形式 (I−KH)P(I−KH)ᵀ + KRKᵀ，比简化的
//      (I−KH)P 更能保持对称性与半正定性；但求逆仍是通用 inverse()，S 或 P 病态
//      时会得到 NaN / Inf，本实现不做兜底。
//   2. 算法细节（勿改）：K 与状态更新都使用**先验** x；而写入调试数据的 residual
//      与 NIS 使用**更新后**的 x 重新计算，NEES 使用后验 P，均与源工程一致。
//   3. 单位：本文件不引入任何单位，单位完全由调用方赋予 x / z（自瞄中为弧度与米）。
//   4. 调用时机：predict / update 会就地修改成员，非线程安全；每次量测到达时
//      先 predict 再 update，两者必须在同一线程内按序调用。
//   5. 边界：量测维 m < 4 时末尾 residual[0..3] 越界；滑动窗口为空时失败率计算
//      会除零（正常情况下窗口初值含一个 0，且每次 update 都会 push，故不会为空，
//      但 recent_nis_failures 是公有成员，外部 clear() 后即可能触发）。
// =============================================================================

#include "extended_kalman_filter.hpp"

#include <numeric>

namespace tools
{
/**
 * 作用：带初值的构造函数。保存状态初值 x0 与协方差初值 P0，按状态维生成单位阵 I，
 *       保存状态加法 x_add，并预置调试字典 data 的所有键（初值 0）。
 * 参数：
 *   - x0：状态初值，维度 n。
 *   - P0：状态协方差初值，n×n。
 *   - x_add：状态加法函数对象，默认欧氏加法。
 * 返回：无。
 * 边界条件：不做维度检查；x0 为空向量时 I 为 0×0 矩阵，后续矩阵运算会断言失败。
 *           data 中所有键都显式赋 0.0，保证首次 update 之前读取 data 也能得到确定值。
 * 备注：data["nis_fail"] / data["nees_fail"] 在 update 中超阈值时被置 1，且不会复位，
 *       是"粘滞"的失败标志；data["recent_nis_failures"] 保存的是窗口内失败率（0~1）。
 */
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
  data["residual_yaw"] = 0.0;
  data["residual_pitch"] = 0.0;
  data["residual_distance"] = 0.0;
  data["residual_angle"] = 0.0;
  data["nis"] = 0.0;
  data["nees"] = 0.0;
  data["nis_fail"] = 0.0;
  data["nees_fail"] = 0.0;
  data["recent_nis_failures"] = 0.0;
}

/**
 * 作用：线性预测重载。把线性状态转移写成函数对象后转发给三参数重载，
 *       即 x = F·x，P = F·P·Fᵀ + Q。
 * 参数：F 状态转移矩阵 n×n；Q 过程噪声协方差 n×n。
 * 返回：预测后的状态向量 x。
 * 边界条件：F / Q 维度不匹配时 Eigen 断言失败；本重载不做数值兜底。
 * 备注：lambda 以引用捕获 F，仅在本次调用内使用，F 的生命周期无需延长。
 */
Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

/**
 * 作用：非线性预测重载。x = f(x)，P = F·P·Fᵀ + Q（F 为 f 的雅可比，由调用方给出）。
 * 参数：F 状态转移雅可比 n×n；Q 过程噪声协方差 n×n；
 *       f 非线性状态转移函数（输入旧状态、返回新状态）。
 * 返回：预测后的状态向量 x。
 * 边界条件：f 返回维度必须为 n；f 抛异常时 P 已更新而 x 未更新，对象不一致。
 * 备注：实现顺序刻意是"先更新 P、再更新 x"，两者都以更新前的 x 为线性化点（与源工程一致）。
 */
Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  P = F * P * F.transpose() + Q;
  x = f(x);
  return x;
}

/**
 * 作用：线性量测更新重载。把线性观测方程写成函数对象后转发给五参数重载。
 * 参数：z 量测向量 m 维；H 观测矩阵 m×n；R 量测噪声协方差 m×m；
 *       z_subtract 量测减法（默认欧氏减法）。
 * 返回：更新后的状态向量 x。
 * 边界条件：同五参数重载（S 奇异时结果 NaN / Inf；m < 4 时调试数据越界）。
 * 备注：lambda 以引用捕获 H，仅在本次调用内使用。
 */
Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
}

/**
 * 作用：非线性量测更新重载（核心实现）。关键步骤如下：
 *         1) 记录先验状态 x_prior，供 NEES 使用；
 *         2) 计算卡尔曼增益 K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹（P 为先验协方差）；
 *         3) 用 Joseph 形式更新后验协方差 P（数值上保持对称半正定）；
 *         4) 更新状态 x = x_add(x, K·z_subtract(z, h(x)))（h(x) 取先验 x）；
 *         5) 用**后验** x 重算残差，并做卡方检验（NIS / NEES）与滑动窗口失败率统计；
 *         6) 刷新调试字典 data 并返回 x。
 * 参数：
 *   - z：量测向量，维度 m（源工程 m = 4；要求 m ≥ 4）。
 *   - H：观测雅可比矩阵，m×n。
 *   - R：量测噪声协方差矩阵，m×m，应正定。
 *   - h：观测函数，输入状态、输出预测量测，返回维度必须为 m。
 *   - z_subtract：量测减法函数对象，默认欧氏减法；角度类量测应传入环绕减法。
 * 返回：更新后的状态向量 x（即成员 x）。
 * 边界条件：
 *   - H·P·Hᵀ + R 奇异或病态 → K 为 NaN / Inf，x 与 P 被污染，本函数不做检测；
 *   - m < 4 时下方 residual[0..3] 越界（未定义行为）；
 *   - recent_nis_failures 为空（被外部 clear）时失败率计算除零，得到 ±Inf / NaN。
 * 备注：residual / NIS 使用后验 x、NEES 使用后验 P，这是源工程的刻意实现，勿改。
 */
Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  // 1) 保存先验状态：NEES 需要 (x_后验 − x_先验)。
  Eigen::VectorXd x_prior = x;
  // 2) 卡尔曼增益 K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹。此处 P 仍是先验协方差，
  //    S = H·P·Hᵀ + R 应正定；inverse() 不做正定性检查，病态时直接产出 NaN / Inf。
  Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

  // Stable Compution of the Posterior Covariance
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  // 3) Joseph 形式更新后验协方差：数值稳定性优于 (I−K·H)·P，
  //    在 K 接近奇异或 R 很小时仍能保持 P 的对称性与半正定性。
  P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();

  // 4) 状态更新：x = x_add(x, K·(z − h(x)))。注意 h(x) 取的是"更新前"的 x（先验），
  //    与 K 的线性化点一致；x_add 负责非线性流形（如角度）上的累加。
  x = x_add(x, K * z_subtract(z, h(x)));

  /// 卡方检验
  // 5) 残差用"更新后"的 x 重算（后验残差），非标准 KF 的新息——源工程行为，保持原样。
  Eigen::VectorXd residual = z_subtract(z, h(x));
  // 新增检验
  // S 也用更新后的 P 重算（不是算 K 时用的先验 P），NIS = rᵀ·S⁻¹·r。
  Eigen::MatrixXd S = H * P * H.transpose() + R;
  double nis = residual.transpose() * S.inverse() * residual;
  // NEES = (x − x_prior)ᵀ·P⁻¹·(x − x_prior)，P 为后验协方差。
  double nees = (x - x_prior).transpose() * P.inverse() * (x - x_prior);

  // 卡方检验阈值（自由度=4，取置信水平95%）
  // 说明：源工程中两个阈值都取 0.711，与"自由度 4、95% 置信"的常见查表值（约 9.488）
  // 并不一致，属源工程既有取值，迁移时原样保留，不擅自修正。
  constexpr double nis_threshold = 0.711;
  constexpr double nees_threshold = 0.711;

  // 失败计数：注意 data["nis_fail"] / data["nees_fail"] 是"粘滞"标志，一旦置 1 不再复位；
  // 私有成员 nis_count_ / nees_count_ / total_count_ 只累加，目前没有对外读取接口。
  if (nis > nis_threshold) nis_count_++, data["nis_fail"] = 1;
  if (nees > nees_threshold) nees_count_++, data["nees_fail"] = 1;
  total_count_++;
  // 记录最近一次 NIS，供外部免查 map 直接读取。
  last_nis = nis;

  // 6) NIS 失败滑动窗口：本次结果入队，超过 window_size 则弹出最旧的。
  recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);

  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();
  }

  // 窗口内失败率（0~1，量纲一）。窗口为空时这里会除零，正常情况下窗口初值含一个 0
  // 且每次 update 都会 push，因此不会为空；但 recent_nis_failures 为公有成员，
  // 外部若 clear() 过就可能触发。
  int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
  double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

  // 7) 刷新调试数据。residual 的 4 个分量依次对应 yaw / pitch / distance / angle，
  //    这里隐含要求量测维 m ≥ 4，否则越界。
  data["residual_yaw"] = residual[0];
  data["residual_pitch"] = residual[1];
  data["residual_distance"] = residual[2];
  data["residual_angle"] = residual[3];
  data["nis"] = nis;
  data["nees"] = nees;
  data["recent_nis_failures"] = recent_rate;

  return x;
}

}  // namespace tools
