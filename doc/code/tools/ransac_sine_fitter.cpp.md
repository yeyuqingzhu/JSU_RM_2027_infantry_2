# tools/ransac_sine_fitter.cpp

**概要**：`tools/ransac_sine_fitter.hpp` 的实现——构造与随机引擎播种、数据缓冲管理（时间断档清空）、RANSAC 主循环、固定 ω 的线性最小二乘子问题（SVD）、内点统计。

## 函数

### `RansacSineFitter::RansacSineFitter(int max_iterations, double threshold, double min_omega, double max_omega)`

- **作用**：实现要点——初始化列表按声明顺序依次赋值 `max_iterations_`、`threshold_`、`min_omega_`、`max_omega_`，并用 `std::random_device{}()` 作为种子构造 `gen_`（`std::mt19937`）。
- **参数**：`max_iterations` 迭代次数；`threshold` 内点阈值（单位同 v）；`min_omega` / `max_omega` 角频率采样上下界（rad/s）。
- **返回**：无。
- **备注**：函数体为空，无参数校验。因种子来自随机设备，同一份数据两次运行的结果不保证相同（源工程行为，调试时不要指望逐位复现）。

### `void RansacSineFitter::add_data(double t, double v)`

- **作用**：实现要点——单行判断 + 追加：`if (fit_data_.size() > 0 && (t - fit_data_.back().first > 5)) fit_data_.clear();` 然后 `fit_data_.emplace_back(std::make_pair(t, v));`。
- **参数**：`t` 时间戳（通常为秒）；`v` 观测值。
- **返回**：无。
- **备注**：`5` 是硬编码的时间断档阈值（单位同 t），超过则认为信号不连续（丢帧、换目标、暂停），先清空历史；缓冲区为空时不做时间比较。清空数据**不会**清空 `best_result_`，历史最优仍保留。不检查 t 单调性。

### `void RansacSineFitter::fit()`

- **作用**：实现要点（关键步骤）：
  1. 数据少于 3 条直接 `return`（凑不出「3 点 + 随机 ω」的假设）；
  2. 构造 `std::uniform_real_distribution<double> omega_dist(min_omega_, max_omega_)`，以及 0..N−1 的索引数组 `indices`（`std::iota`）；
  3. 迭代 `max_iterations_` 次：`std::shuffle(indices, gen_)` → 取 `indices[0..2]` 对应的 3 个点组成 `sample` → `omega = omega_dist(gen_)` 随机采样角频率（**频率靠随机搜索，不是从 3 点解析求得**）→ `fit_partial_model(sample, omega, params)`，返回 false 时 `continue`；
  4. 由 `params` 换算 `A = sqrt(A1² + A2²)`、`phi = atan2(A2, A1)`，`C = params(2)`；
  5. `evaluate_inliers(A, omega, phi, C)` 统计内点，**仅当内点数严格大于**当前最优时整体覆盖 `best_result_`（打平则保留先出现的那组）；
  6. 循环结束后：`if (fit_data_.size() > 150) fit_data_.pop_front();` —— 每次调用只丢弃最旧的一条，使数据量长期稳定在 150 上下，限制每轮的遍历开销。
- **参数**：无。
- **返回**：无（结果写 `best_result_`）。
- **备注**：`max_iterations_ ≤ 0` 时循环空转，`best_result_` 不变；`omega = 0` 时设计矩阵第 0 列全 0（秩亏），SVD 给出最小范数解，A / φ 无物理意义。复杂度约为 `max_iterations × 数据条数`。

### `bool RansacSineFitter::fit_partial_model(const std::vector<std::pair<double,double>> & sample, double omega, Eigen::Vector3d & params)`

- **作用**：实现要点——构造设计矩阵 `X`（n×3，每行为 `[sin(ωt), cos(ωt), 1]`）与观测向量 `Y`（n 维，每行为 v），然后 `params = X.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);` 得到最小二乘解 (A1, A2, C)。
- **参数**：`sample` 样本点集（一般 3 个点，n ≥ 1 亦可）；`omega` 已固定的角频率（rad/s）；`params` 输出维度 3。
- **返回**：`true` 求解完成；`false` 求解过程抛出了 C++ 异常（`catch (...)` 兜底），此时 `params` 内容未定义，调用方须跳过本组假设。
- **备注**：Eigen 在维度/数值异常时通常走断言而非抛异常，因此这里的 `try/catch` 主要防标准库异常（如内存不足），正常路径几乎总是返回 true；用 SVD 而非 QR/正规方程是为了在样本退化、ω 取值退化时仍得到数值可用的最小范数解。

### `int RansacSineFitter::evaluate_inliers(double A, double omega, double phi, double C)`

- **作用**：实现要点——遍历 `fit_data_`，对每个点计算 `pred = A * sin(omega * t + phi) + C`，若 `std::abs(y - pred) < threshold_` 则计数 +1，最后返回计数。
- **参数**：`A` 振幅；`omega` 角频率（rad/s）；`phi` 初相（rad）；`C` 直流偏置。
- **返回**：内点个数（int）。
- **备注**：缓冲区为空时返回 0；阈值是严格小于（`<`），等于 `threshold_` 的点不算内点；公式与 `sine_function` 完全相同但未复用该内联函数（源工程写法，迁移时保持一致）。
