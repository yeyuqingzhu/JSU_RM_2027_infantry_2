// =============================================================================
// 文件：tools/ransac_sine_fitter.hpp
// -----------------------------------------------------------------------------
// 【本文件作用】
//   RANSAC + 正弦模型 v(t) = A·sin(ω·t + φ) + C 的鲁棒拟合器 RansacSineFitter
//   的对外接口声明。用于对"近似正弦"的时序观测量做抗外点拟合（例如转动目标/
//   能量机关类周期性运动的幅值、角频率、初相、直流偏置估计）。
//   拟合思路：每轮随机取 3 个点 + 在 [min_omega, max_omega] 内随机取一个 ω，
//   固定 ω 后模型对 (A1, A2, C) 是线性的，用 SVD 解线性最小二乘；再用全部数据
//   统计内点数，取内点最多的那组参数作为结果。
//
// 【来源】
//   sp_vision_25 工程的 tools/ransac_sine_fitter.hpp（逐文件迁移）。
//   类名、成员名、函数签名、默认成员初值、算法逻辑均与源工程完全一致，本次迁移
//   只增加中文注释。注意源文件用的是 #pragma once 而非 #ifndef 保护宏，原样保留。
//
// 【依赖】
//   - Eigen3（Eigen/Dense）：Eigen::Vector3d / Eigen::MatrixXd / Eigen::VectorXd
//     与 bdcSvd 最小二乘求解；
//   - C++ 标准库：<deque>（fit_data_ 数据缓冲）、<random>（mt19937 引擎）、
//     <vector>（随机样本）、<iostream>（源文件中未实际使用，保留原样）、
//     <algorithm>/<numeric>（std::shuffle / std::iota，在 .cpp 中）。
//
// 【注意事项】
//   1. 结果不可复现：随机引擎 gen_ 由 std::random_device 播种，同一份数据多次
//      fit() 可能得到不同结果（除非外部固定种子——本类未提供该接口）。
//   2. 单位：t 与 v 的单位由调用方决定；threshold_ 与 v 同单位；当 t 以秒为单位
//      时 ω 的单位是 rad/s。构造函数里的 5（add_data 的时间断档阈值）与 150
//      （数据长度上限）是硬编码魔数，单位分别与 t 一致、为"条"。
//   3. 使用方式：反复 add_data(t, v) 喂数据，再调 fit()，结果从公有成员
//      best_result_ 读取。best_result_ 只增不减（不会因新一轮更差而被清零），
//      因此调用方必须自己判断 inliers 是否足够。
//   4. 线程安全：无任何同步，单个实例不可多线程并发使用。
//   5. 数值：ω = 0 时设计矩阵第 0 列全为 0（秩亏），SVD 会给出最小范数解而不会抛
//      异常，此时解出的 φ / A 无物理意义；min_omega_ 与 max_omega_ 应取正区间。
// =============================================================================

#pragma once

#include <Eigen/Dense>
#include <deque>
#include <iostream>
#include <random>
#include <vector>

namespace tools
{

/**
 * RANSAC 正弦拟合器
 *
 * 职责：
 *   缓存一小段时序数据 (t, v)，用 RANSAC 从中鲁棒地拟合 v ≈ A·sin(ω·t + φ) + C，
 *   并对外暴露最优结果 best_result_。内部负责数据缓冲的自动裁剪（时间断档清空、
 *   长度超限丢弃最旧点），不负责数据的采集与结果的有效性判定。
 *
 * 典型用法：
 *   tools::RansacSineFitter fitter(200, 0.05, 0.5, 4.0);
 *   for (auto & [t, v] : measurements) fitter.add_data(t, v);
 *   fitter.fit();
 *   if (fitter.best_result_.inliers > 阈值) { 使用 fitter.best_result_ ... }
 */
class RansacSineFitter
{
public:
  /** 一次拟合的结果：正弦模型的四个参数 + 内点数（内点数越多代表该假设越可信）。 */
  struct Result
  {
    /** 振幅，单位与观测量 v 相同，恒为非负（由 A = sqrt(A1² + A2²) 得到）。 */
    double A = 0.0;
    /** 角频率，单位 rad/s（当 t 以秒计时），由 RANSAC 在 [min_omega, max_omega] 内随机采样。 */
    double omega = 0.0;
    /** 初相，单位 rad，取值 (-π, π]（由 atan2(A2, A1) 得到）。 */
    double phi = 0.0;
    /** 直流偏置（正弦中心线），单位与 v 相同。 */
    double C = 0.0;
    /** 内点个数：满足 |v − 模型预测| < threshold_ 的样本条数，作为 RANSAC 评分。 */
    int inliers = 0;
  };

  /** 当前已知的最优拟合结果；调用 fit() 后被刷新，只会变好不会变差。 */
  Result best_result_;

  /**
   * 作用：构造函数，保存 RANSAC 与正弦模型的全部配置参数，并用 std::random_device
   *       播种内部随机引擎（因此结果不可复现）。
   * 参数：
   *   - max_iterations：RANSAC 最大迭代次数（次），每次迭代试一组"3 点 + 1 个 ω"的假设。
   *   - threshold：内点判定阈值，单位与观测量 v 相同（|v − 预测值| < threshold 视为内点）。
   *   - min_omega：角频率随机采样的下界，单位 rad/s（当 t 以秒计时）。
   *   - max_omega：角频率随机采样的上界，单位 rad/s，必须 ≥ min_omega。
   * 返回：无（构造函数）。
   * 边界条件：构造时不做任何参数校验；max_iterations ≤ 0 时 fit() 不做任何迭代；
   *          min_omega > max_omega 时 std::uniform_real_distribution 的行为未定义。
   */
  RansacSineFitter(int max_iterations, double threshold, double min_omega, double max_omega);

  /**
   * 作用：向内部缓冲区追加一个观测点 (t, v)。
   * 参数：
   *   - t：时间戳，单位由调用方决定（一般以秒计，需单调递增）。
   *   - v：该时刻的观测值，单位由调用方决定。
   * 返回：无。
   * 边界条件：若缓冲区非空且 (t − 队尾时刻) > 5，则认为中间发生了时间断档
   *          （换目标、丢帧、暂停等），先清空全部历史再放入本点。这里的 5 是
   *          硬编码阈值（单位与 t 一致，即通常为 5 s），不可配置。
   *          函数不检查 t 的单调性：乱序（回退的）时间戳不会触发清空，但会让
   *          拟合失真。
   */
  void add_data(double t, double v);

  /**
   * 作用：对当前缓冲区内的全部数据执行一轮 RANSAC 正弦拟合，并在结果优于
   *       best_result_ 时更新它。
   * 参数：无（使用构造参数与 add_data 累积的数据）。
   * 返回：无；结果从公有成员 best_result_ 读取。
   * 边界条件：
   *   - 数据少于 3 条时直接返回，best_result_ 保持原值；
   *   - 每次调用结束若数据超过 150 条，会丢弃最旧的一条，因此长期调用时计算量有界；
   *   - 本轮若无更优结果，best_result_ 不会被清零，仍是历史最优。
   */
  void fit();

  /**
   * 作用：正弦模型求值 v(t) = A·sin(ω·t + φ) + C（内联，供调用方复用同一公式）。
   * 参数：t 自变量（时间，单位同 add_data 的 t）；A 振幅；omega 角频率；phi 初相；C 直流偏置。
   * 返回：该时刻的模型预测值，单位与观测量 v 相同。
   * 边界条件：无；参数非法（如 ω 为 Inf）时按 IEEE 浮点规则返回 NaN / Inf。
   */
  double sine_function(double t, double A, double omega, double phi, double C)
  {
    return A * std::sin(omega * t + phi) + C;
  }

private:
  /** RANSAC 最大迭代次数（次）；每次迭代尝试一组"随机 3 点 + 随机 ω"的假设。 */
  int max_iterations_;

  /** 内点判定阈值，单位与观测量 v 相同；越小越严格、内点数越少。 */
  double threshold_;

  /** 角频率随机采样下界，单位 rad/s（当 t 以秒计时）。 */
  double min_omega_;

  /** 角频率随机采样上界，单位 rad/s（当 t 以秒计时）。 */
  double max_omega_;

  /** Mersenne-Twister 随机数引擎，由 std::random_device 播种；用于洗牌取点与采样 ω。 */
  std::mt19937 gen_;

  /** 观测数据缓冲：元素为 (t, v)；由 add_data 追加，时间断档时清空，超长时丢弃最旧点。 */
  std::deque<std::pair<double, double>> fit_data_;

  /**
   * 作用：固定 ω，对给定样本解线性最小二乘，求 (A1, A2, C)，使
   *       v ≈ A1·sin(ω·t) + A2·cos(ω·t) + C。
   * 参数：
   *   - sample：样本点集（一般恰好 3 个点，任意 n ≥ 1 亦可），元素为 (t, v)。
   *   - omega：已固定的角频率，单位 rad/s。
   *   - params：输出参数，维度 3，依次为 (A1, A2, C)；仅当返回 true 时有效。
   * 返回：true 表示求解完成（params 已写入）；false 表示求解过程抛出 C++ 异常
   *       （如内存分配失败），调用方应跳过本组假设。
   * 边界条件：sample 为空时构造出 0×3 的设计矩阵，求解返回空/零向量，不抛异常；
   *           ω = 0 时设计矩阵秩亏，SVD 给出最小范数解而不报错。
   */
  bool fit_partial_model(
    const std::vector<std::pair<double, double>> & sample, double omega, Eigen::Vector3d & params);

  /**
   * 作用：用当前全部缓存数据评价一组模型参数的质量，统计内点个数。
   * 参数：A 振幅；omega 角频率；phi 初相；C 直流偏置（单位同 Result 中同名成员）。
   * 返回：内点个数（int）：满足 |v − (A·sin(ω·t + φ) + C)| < threshold_ 的样本条数。
   * 边界条件：缓冲区为空时返回 0；数据越多本函数越耗时（每次假设都要遍历全量数据）。
   */
  int evaluate_inliers(double A, double omega, double phi, double C);
};

}  // namespace tools
