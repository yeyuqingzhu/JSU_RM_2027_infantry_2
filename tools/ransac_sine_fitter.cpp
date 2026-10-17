// =============================================================================
// 文件：tools/ransac_sine_fitter.cpp
// -----------------------------------------------------------------------------
// 【本文件作用】
//   tools/ransac_sine_fitter.hpp 中 RansacSineFitter 的实现：构造（播种随机引擎）、
//   数据缓冲管理（add_data 的时间断档清空）、RANSAC 主循环（fit）、固定 ω 的线性
//   最小二乘子问题（fit_partial_model）、内点统计（evaluate_inliers）。
//
// 【来源】
//   sp_vision_25 工程的 tools/ransac_sine_fitter.cpp（逐文件迁移）。
//   迭代次数、随机采样范围、SVD 求解方式（bdcSvd + ComputeThinU|ComputeThinV）、
//   硬编码阈值（5 与 150）以及原有注释均与源工程完全一致，本次迁移只增加中文注释。
//
// 【依赖】
//   - tools/ransac_sine_fitter.hpp（同目录，"..." 相对包含，保持源工程写法）
//   - Eigen3：MatrixXd / VectorXd / Vector3d、JacobiSVD::solve
//   - C++ 标准库：<algorithm>（std::shuffle）、<cmath>（std::sqrt / std::atan2 /
//     std::abs / std::sin / std::cos）、<random>（std::uniform_real_distribution）、
//     <numeric>（std::iota，由 Eigen 间接引入，见迁移报告的"源文件缺陷"）
//
// 【注意事项】
//   1. 结果的随机性：gen_ 由 std::random_device 播种，每次进程运行结果都可能不同，
//      调试时不要指望两份日志能逐位复现。
//   2. 时间复杂度：fit() 每轮迭代都要遍历全量数据统计内点，复杂度约为
//      max_iterations × 数据条数；数据条数被限制在 150 上下，故整体开销有界。
//   3. 单位：t 与 v 的单位由调用方决定；threshold_ 与 v 同单位；ω 为 rad/s（t 为秒时）。
//   4. 本文件不抛异常（除 Eigen 内部可能的断言），也不做任何参数合法性检查。
// =============================================================================

#include "ransac_sine_fitter.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <random>

namespace tools
{

/**
 * 作用：构造函数。按声明顺序初始化 4 个配置参数，并把随机引擎 gen_ 用
 *       std::random_device 的随机种子播种（因此结果不可复现）。
 * 参数：max_iterations RANSAC 最大迭代次数；threshold 内点阈值（单位同 v）；
 *       min_omega / max_omega 角频率采样上下界（rad/s）。
 * 返回：无。
 * 边界条件：不做参数校验。max_iterations ≤ 0 时 fit() 空转；min_omega > max_omega
 *          时 std::uniform_real_distribution 行为未定义（通常返回边界值）。
 * 备注：std::random_device 在部分平台是确定性实现，但绝大多数 Linux 平台使用
 *       硬件熵源，故这里视为非确定性播种。
 */
RansacSineFitter::RansacSineFitter(
  int max_iterations, double threshold, double min_omega, double max_omega)
: max_iterations_(max_iterations),
  threshold_(threshold),
  min_omega_(min_omega),
  max_omega_(max_omega),
  gen_(std::random_device{}())
{
}

/**
 * 作用：追加一个观测点 (t, v)；若距上一个点的时间间隔超过 5（即 t − 队尾 t > 5），
 *       先清空历史数据再追加，避免把两段不连续的信号混在一起拟合。
 * 参数：t 时间戳（单位由调用方决定，通常为秒）；v 观测值。
 * 返回：无。
 * 边界条件：缓冲区为空时不比较时间，直接追加；"5" 是硬编码阈值，单位与 t 一致；
 *          不检查 t 的单调性（时间回退不会触发清空，但会让拟合失真）。
 * 备注：清空会丢弃此前所有样本，包括已经拟合出的 best_result_ 所依赖的数据
 *       （best_result_ 本身不会被清空，仍保留历史最优值）。
 */
void RansacSineFitter::add_data(double t, double v)
{
  if (fit_data_.size() > 0 && (t - fit_data_.back().first > 5)) fit_data_.clear();
  fit_data_.emplace_back(std::make_pair(t, v));
}

/**
 * 作用：RANSAC 主循环。每轮迭代：洗牌索引 → 取前 3 个点作为样本 → 在
 *       [min_omega_, max_omega_] 内均匀随机取一个 ω → 解固定 ω 的线性最小二乘得到
 *       (A1, A2, C) → 换算成 (A, φ) → 用全量数据统计内点 → 若内点数超过历史最优
 *       则整体更新 best_result_。循环结束后，若数据超过 150 条则丢弃最旧的一条。
 * 参数：无。
 * 返回：无；结果写入公有成员 best_result_。
 * 边界条件：
 *   - 数据少于 3 条时立即返回（best_result_ 不变）；
 *   - max_iterations_ ≤ 0 时循环体一次都不执行，best_result_ 不变；
 *   - 若本轮所有假设都没能超过历史最优内点数，best_result_ 保持原值（只增不减）；
 *   - ω 是全域随机采样而非由样本估计，因此覆盖不到真实频率时会解出无意义的参数，
 *     依赖调用方通过 min/max_omega_ 给出合理的搜索范围。
 * 备注：内点统计用循环外层的 fit_data_（全量数据），不是本轮样本，这是 RANSAC 的
 *       标准做法：用随机最小样本集生成假设、用全量数据投票。
 */
void RansacSineFitter::fit()
{
  // 少于 3 个点时连一组"3 点假设"都凑不出来（叠加随机 ω 后方程组仍欠定），直接放弃。
  if (fit_data_.size() < 3) return;

  // ω 在给定区间内均匀随机采样：频率维度靠随机搜索，而非从三个点解析求解。
  std::uniform_real_distribution<double> omega_dist(min_omega_, max_omega_);
  // 索引数组与数据一一对应，每轮洗牌后取前 3 个即为随机 3 点样本（无放回）。
  std::vector<size_t> indices(fit_data_.size());
  std::iota(indices.begin(), indices.end(), 0);

  for (int iter = 0; iter < max_iterations_; ++iter) {
    std::shuffle(indices.begin(), indices.end(), gen_);

    std::vector<std::pair<double, double>> sample;
    for (int i = 0; i < 3; ++i) {
      sample.push_back(fit_data_[indices[i]]);
    }

    double omega = omega_dist(gen_);
    Eigen::Vector3d params;
    // 固定 ω 后模型关于 (A1, A2, C) 是线性的，用 SVD 解最小二乘；
    // 异常（返回 false）时跳过本组假设。
    if (!fit_partial_model(sample, omega, params)) continue;

    double A1 = params(0);
    double A2 = params(1);
    double C = params(2);

    // A1·sin(ωt) + A2·cos(ωt) = A·sin(ωt + φ)，
    // 故 A = sqrt(A1² + A2²)（非负振幅），φ = atan2(A2, A1) ∈ (-π, π]。
    double A = std::sqrt(A1 * A1 + A2 * A2);
    double phi = std::atan2(A2, A1);

    int inlier_count = evaluate_inliers(A, omega, phi, C);

    // 只接受严格更优的假设：内点数相同时保留先出现的那组（结果与源工程一致）。
    if (inlier_count > best_result_.inliers) {
      best_result_.A = A;
      best_result_.omega = omega;
      best_result_.phi = phi;
      best_result_.C = C;
      best_result_.inliers = inlier_count;
    }
  }

  // 每调用一次 fit() 最多丢弃最旧的一个点：数据量长期稳定在 150 上下，
  // 既保证拟合反映最近的信号，又限制每轮的遍历开销。注意这里不是清空。
  if (fit_data_.size() > 150) fit_data_.pop_front();
}

/**
 * 作用：固定 ω 的线性最小二乘子问题。把模型 v = A1·sin(ωt) + A2·cos(ωt) + C 写成
 *       设计矩阵 X（每行 [sin(ωt), cos(ωt), 1]）与观测向量 Y，用 bdcSvd 求最小二乘解。
 * 参数：sample 样本点集（每项为 (t, v)）；omega 已固定的角频率（rad/s）；
 *       params 输出 (A1, A2, C)，仅当返回 true 时有效。
 * 返回：true 求解成功；false 表示求解过程中抛出 C++ 异常（如分配失败），调用方跳过。
 * 边界条件：sample 为空时 X 为 0×3、Y 为 0 维，solve 返回零向量且不抛异常；
 *          ω = 0 时 X 的第 0 列全 0（秩亏），bdcSvd 给最小范数解，同样不抛异常，
 *          因此正常路径下总是返回 true。
 * 备注：用 SVD（Eigen::ComputeThinU | Eigen::ComputeThinV）而非 QR/正规方程，
 *       是为了在样本共线、ω 取值退化时仍能得到数值上可用的最小范数解。
 */
bool RansacSineFitter::fit_partial_model(
  const std::vector<std::pair<double, double>> & sample, double omega, Eigen::Vector3d & params)
{
  Eigen::MatrixXd X(sample.size(), 3);
  Eigen::VectorXd Y(sample.size());

  // 逐行填充设计矩阵：X·[A1, A2, C]ᵀ 即为模型预测值。
  for (size_t i = 0; i < sample.size(); ++i) {
    double t = sample[i].first;
    double y = sample[i].second;
    X(i, 0) = std::sin(omega * t);
    X(i, 1) = std::cos(omega * t);
    X(i, 2) = 1.0;
    Y(i) = y;
  }

  // try/catch 只是兜底：Eigen 在维度/数值异常时通常走断言而非抛异常，
  // 这里主要防标准库异常（如内存不足）；返回 false 由调用方 continue 跳过本组假设。
  try {
    params = X.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);
    return true;
  } catch (...) {
    return false;
  }
}

/**
 * 作用：用当前全部缓存数据评价一组模型参数，统计内点个数（RANSAC 的评分函数）。
 * 参数：A 振幅；omega 角频率（rad/s）；phi 初相（rad）；C 直流偏置。
 * 返回：内点个数，即满足 |v − (A·sin(ωt + φ) + C)| < threshold_ 的样本条数。
 * 边界条件：缓冲区为空时返回 0；threshold_ ≤ 0 时几乎无内点；
 *          公式与 sine_function 相同，但源工程未复用该内联函数，此处保持一致。
 * 备注：本函数被 fit() 每轮调用一次，遍历全量数据，是 fit() 的主要耗时来源。
 */
int RansacSineFitter::evaluate_inliers(double A, double omega, double phi, double C)
{
  int count = 0;
  for (const auto & p : fit_data_) {
    double t = p.first;
    double y = p.second;
    double pred = A * std::sin(omega * t + phi) + C;
    if (std::abs(y - pred) < threshold_) {
      ++count;
    }
  }
  return count;
}

}  // namespace tools
