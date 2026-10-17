#ifndef TOOLS__EXTENDED_KALMAN_FILTER_HPP
#define TOOLS__EXTENDED_KALMAN_FILTER_HPP

// =============================================================================
// 文件：tools/extended_kalman_filter.hpp
// -----------------------------------------------------------------------------
// 【本文件作用】
//   扩展卡尔曼滤波（Extended Kalman Filter, EKF）通用件 ExtendedKalmanFilter
//   的对外接口声明。类内不假设任何具体模型：状态转移矩阵 F、过程噪声 Q、观测
//   矩阵 H、观测噪声 R，以及非线性函数 f / h、非线性流形上的加减法 x_add /
//   z_subtract 全部由调用方逐次传入。因此同一份实现既能当纯线性卡尔曼滤波用
//   （predict(F,Q) / update(z,H,R)），也能通过传入 f / h 与自定义加减法支持
//   角度等非线性量（例如自瞄中的 yaw / pitch 环绕）。
//   除滤波本体外，类内还内建了卡方检验（NIS / NEES）统计，用于现场诊断滤波
//   是否发散、量测是否异常（自瞄中表现为残差异常、跟踪跳变）。
//
// 【来源】
//   sp_vision_25 工程的 tools/extended_kalman_filter.hpp（逐文件迁移）。
//   类名、成员名、函数签名与重载结构、默认参数、算法逻辑均与源工程完全一致，
//   本次迁移只增加中文注释。
//
// 【依赖】
//   - Eigen3（Eigen/Dense）：Eigen::VectorXd / Eigen::MatrixXd 及其矩阵运算；
//   - C++ 标准库：<deque>（NIS 失败滑动窗口）、<functional>（承载调用方传入的
//     f / h / x_add / z_subtract）、<map>（data 调试字典）。
//     注：成员中用到 std::string，源文件未显式包含 <string>，靠 <map> 间接引入
//     （见迁移报告中的"源文件缺陷"）。
//
// 【注意事项】
//   1. 维度约定：状态维 n = x.rows()，量测维 m = z.rows()。要求 F 为 n×n、
//      Q 为 n×n、H 为 m×n、R 为 m×m。维度不匹配时 Eigen 会在运行时断言失败
//      （Debug 下直接 abort），本类不做任何维度检查，调用方必须自行保证。
//   2. 求逆风险（数值稳定性）：update 中的 (H P H^T + R).inverse() 与 P.inverse()
//      都是通用求逆。R 奇异 / P 病态 / 数值下溢时会得到 NaN 或 Inf，本类不做
//      兜底，调用方需保证 R 正定、P 数值健康（例如定期对称化、避免过程噪声过小）。
//   3. 索引风险：update 末尾用 residual[0..3] 写调试数据，隐含要求量测维 m ≥ 4；
//      m < 4 时是越界访问（Release 下 Eigen 不检查，属未定义行为）。
//   4. 单位：本类完全不关心单位，单位由调用方赋予 x / z。源工程自瞄中依次为
//      yaw（弧度）、pitch（弧度）、distance（米）、angle（弧度）；卡方检验量是
//      白化后的量纲一统计量，与单位无关。
//   5. 调用时机：predict 与 update 都会就地修改成员 x / P / data，**非线程安全**，
//      同一实例不可多线程并发调用；不同实例互不影响。
//   6. 默认构造函数 = default：这样构造出的对象 x / P 为空矩阵、x_add 为空
//      std::function，此时调用 predict / update 会抛 std::bad_function_call。
//      该构造函数仅供容器或成员声明使用，真正参与滤波必须用带初值的构造函数。
// =============================================================================

#include <Eigen/Dense>
#include <deque>
#include <functional>
#include <map>

namespace tools
{
/**
 * 扩展卡尔曼滤波器（EKF）
 *
 * 职责：
 *   以「状态向量 x + 状态协方差 P」为核心，提供 predict（时间更新）与 update
 *   （量测更新）两个步骤，并顺带统计卡方检验指标用于诊断。类本身不持有任何
 *   具体模型，F / Q / H / R / f / h 全部由调用方传入，属纯通用件。
 *   状态与协方差直接以公有成员暴露，调用方读取不需要 getter。
 *
 * 典型调用序列（源工程自瞄中的用法）：
 *   tools::ExtendedKalmanFilter ekf(x0, P0);
 *   ekf.predict(F, Q);            // 线性预测；非线性时用三参数重载传入 f
 *   ekf.update(z, H, R);          // 线性观测；非线性时用带 h 的重载
 *   double nis = ekf.data["nis"]; // 读卡方检验结果做异常判定
 */
class ExtendedKalmanFilter
{
public:
  /** 后验状态向量，维度 n。每次 predict / update 后被就地更新；初值由构造函数给出。 */
  Eigen::VectorXd x;

  /**
   * 后验状态协方差矩阵，维度 n×n。初始化后应为对称半正定阵；update 采用 Joseph
   * 形式更新以尽量保持对称性与半正定性（详见 .cpp 中的实现说明）。
   */
  Eigen::MatrixXd P;

  /**
   * 默认构造函数（= default）：x / P 为空矩阵，x_add 为空函数对象，data 为空表。
   * 直接调用 predict / update 会抛 std::bad_function_call，仅供声明占位使用。
   */
  ExtendedKalmanFilter() = default;

  /**
   * 作用：带初值的构造函数，建立滤波器并预置调试字典 data 的各个键。
   * 参数：
   *   - x0：状态初值，维度 n；需给出物理上合理的初值（单位由调用方定义）。
   *   - P0：状态协方差初值，n×n 对称半正定阵；对角线为各状态量的方差初值。
   *   - x_add：状态加法函数对象，用于非线性流形（如角度）上的状态累加，
   *            默认实现为欧氏加法 a + b；自定义版本可处理角度环绕。
   * 返回：无（构造函数）。
   * 边界条件：不做任何维度校验；x0.rows() 必须等于 P0 的行列数，否则后续矩阵
   *          运算会在 Eigen 断言处失败。x0 为空向量时内部单位阵 I 退化为 0×0。
   */
  ExtendedKalmanFilter(
    const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a + b; });

  /**
   * 作用：线性状态预测（时间更新）：x = F·x，P = F·P·Fᵀ + Q。
   * 参数：
   *   - F：状态转移矩阵，n×n。
   *   - Q：过程噪声协方差矩阵，n×n（半正定）；描述模型在一周期内的不确定度。
   * 返回：预测后的状态向量 x（返回的就是成员 x 本身，非副本）。
   * 边界条件：F / Q 维度必须为 n×n，否则 Eigen 断言失败；本重载不做数值兜底。
   * 备注：内部转发给三参数重载，f 取线性形式 F·x，等效于标准 KF 的预测步。
   */
  Eigen::VectorXd predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q);

  /**
   * 作用：非线性状态预测：x = f(x)，P = F·P·Fᵀ + Q。
   *       其中 F 是 f 在上一时刻状态处的雅可比矩阵（由调用方线性化后传入）。
   * 参数：
   *   - F：状态转移雅可比矩阵，n×n，必须与 f 在当前线性化点一致，否则滤波发散。
   *   - Q：过程噪声协方差矩阵，n×n。
   *   - f：非线性状态转移函数，输入旧状态、输出新状态，返回维度必须仍为 n。
   * 返回：预测后的状态向量 x（即成员 x）。
   * 边界条件：f 返回值维度与 x 不一致时后续运算断言失败；f 抛出异常时协方差已被
   *          更新而状态未更新，对象处于不一致状态（调用方需自行保证 f 不抛异常）。
   * 备注：实现顺序为「先更新 P，再更新 x」，两者使用的都是更新前的 x 作为线性化
   *       点（与源工程一致，务必保持）。
   */
  Eigen::VectorXd predict(
    const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f);

  /**
   * 作用：线性量测更新（含卡方检验）：
   *         K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹，P 用 Joseph 形式更新，x = x + K·(z − H·x)。
   * 参数：
   *   - z：量测向量，维度 m（源工程中 m = 4，且要求 m ≥ 4，见文件头注意事项 3）。
   *   - H：观测矩阵，m×n。
   *   - R：量测噪声协方差矩阵，m×m（正定）。
   *   - z_subtract：量测减法函数对象，默认欧氏减法 a − b；角度类量测可传入环绕减法。
   * 返回：更新后的状态向量 x（即成员 x）。
   * 边界条件：S = H·P·Hᵀ + R 奇异时 K 变 NaN / Inf；R 非正定时不报错但结果不可信。
   * 备注：内部转发给带 h 的五参数重载，h 取线性形式 H·x，等效于标准 KF 的更新步。
   */
  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });

  /**
   * 作用：非线性量测更新（含卡方检验）：h(x) 为观测方程，H 为其在当前状态处的雅可比。
   * 参数：
   *   - z：量测向量，维度 m（要求 m ≥ 4）。
   *   - H：观测雅可比矩阵，m×n，由调用方在状态处线性化得到。
   *   - R：量测噪声协方差矩阵，m×m（正定）。
   *   - h：观测函数，输入状态、输出预测量测，返回维度必须为 m。
   *   - z_subtract：量测减法函数对象，默认欧氏减法；角度量测应传入环绕减法。
   * 返回：更新后的状态向量 x（即成员 x）。
   * 边界条件：同线性重载；h 返回值维度必须为 m，否则断言失败。
   * 备注（源工程行为细节，迁移时原样保留，勿"顺手修正"）：
   *   1. 增益 K 用先验 x 与先验 P 计算；状态更新项中的 h(x) 同样取更新前的 x；
   *   2. 写入 data 的 residual 与 NIS 是在 x 更新完成之后、用**后验** x 重新计算的
   *      残差，并非标准卡尔曼滤波中的新息 z − h(x_先验)，这是源工程的刻意实现；
   *   3. NEES 用 (x − x_先验) 与**后验** P 计算；
   *   4. data["nis_fail"] / data["nees_fail"] 是"粘滞"失败标志：一旦置 1 不会自动复位。
   */
  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });

  // 调试字典（键 → 值），由构造函数预置、每次 update 刷新。键的含义：
  //   residual_yaw / residual_pitch / residual_distance / residual_angle：
  //     后验残差的 4 个分量（弧度、弧度、米、弧度）；
  //   nis / nees：本次更新的卡方检验统计量（量纲一）；
  //   nis_fail / nees_fail：粘滞失败标志，超阈值时置 1 且不复位；
  //   recent_nis_failures：最近 window_size 次中 NIS 超阈值的比例（0~1）。
  // 公有成员，外部可读；但写入只发生在 update 内部，调用方不要自行改键值。
  std::map<std::string, double> data;  //卡方检验数据

  /** 最近 window_size 次 NIS 是否超阈值的滑动窗口，元素 1 = 失败、0 = 通过。 */
  std::deque<int> recent_nis_failures{0};

  /** NIS 滑动窗口长度（次），默认 100；窗口满后每来一次量测弹出最旧的一个。 */
  size_t window_size = 100;

  /**
   * 最近一次 update 计算出的 NIS 值，供外部快速读取（与 data["nis"] 相同）。
   * 注：源文件此成员未初始化（首次 update 前读取其值属未定义行为），迁移时补上默认
   *     初值 0.0；这不影响任何已定义行为，详见迁移报告。
   */
  double last_nis = 0.0;

private:
  /** 单位阵 n×n，构造时按 x0.rows() 生成，update 中用于 Joseph 形式的协方差更新。 */
  Eigen::MatrixXd I;

  /** 状态加法函数对象，predict / update 中用于在非线性流形上累加状态修正量。 */
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add;

  /** NEES 超阈值累计次数。当前只累加、无对外读取接口，预留给统计/调试。 */
  int nees_count_ = 0;

  /** NIS 超阈值累计次数。当前只累加、无对外读取接口，预留给统计/调试。 */
  int nis_count_ = 0;

  /** update 调用总次数。当前只累加、无对外读取接口，预留给统计/调试。 */
  int total_count_ = 0;
};

}  // namespace tools

#endif  // TOOLS__EXTENDED_KALMAN_FILTER_HPP
