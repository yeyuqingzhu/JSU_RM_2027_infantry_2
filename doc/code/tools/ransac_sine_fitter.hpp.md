# tools/ransac_sine_fitter.hpp

**概要**：RANSAC + 正弦模型 `v(t) = A·sin(ω·t + φ) + C` 的鲁棒拟合器对外接口声明——随机取 3 点并随机采样 ω，固定 ω 后用 SVD 解线性最小二乘，再用全量数据统计内点，取内点最多的参数。

## 类

### `tools::RansacSineFitter`

- **职责**：缓存一小段时序数据 (t, v)，用 RANSAC 鲁棒拟合正弦参数，并对外暴露最优结果 `best_result_`；内部负责数据缓冲的自动裁剪（时间断档清空、长度超限丢弃最旧点），不负责数据采集与结果有效性判定。
- **典型用法**：
  ```cpp
  tools::RansacSineFitter fitter(200, 0.05, 0.5, 4.0);
  for (auto & [t, v] : measurements) fitter.add_data(t, v);
  fitter.fit();
  if (fitter.best_result_.inliers > 阈值) { /* 使用 fitter.best_result_ */ }
  ```
- **备注**：非线程安全；随机引擎由 `std::random_device` 播种，**结果不可复现**。

### 嵌套结构 `RansacSineFitter::Result`

| 成员 | 类型 | 含义 |
| --- | --- | --- |
| `A` | `double` | 振幅，单位同观测量 v，恒非负（`sqrt(A1² + A2²)`） |
| `omega` | `double` | 角频率，rad/s（t 以秒计时），由 RANSAC 在 [min_omega, max_omega] 内随机采样 |
| `phi` | `double` | 初相，rad，范围 (-π, π]（`atan2(A2, A1)`） |
| `C` | `double` | 直流偏置（正弦中心线），单位同 v |
| `inliers` | `int` | 内点个数：`\|v − 模型预测\| < threshold_` 的样本数，作为 RANSAC 评分 |

### 成员变量

| 成员 | 类型 | 含义 |
| --- | --- | --- |
| `best_result_`（public） | `Result` | 当前已知最优拟合结果，调用 `fit()` 后刷新，只增不减 |
| `max_iterations_` | `int` | RANSAC 最大迭代次数，每轮试一组「随机 3 点 + 随机 ω」假设 |
| `threshold_` | `double` | 内点判定阈值，单位同 v |
| `min_omega_` / `max_omega_` | `double` | 角频率随机采样下界 / 上界，rad/s |
| `gen_` | `std::mt19937` | 随机数引擎，`std::random_device` 播种；用于洗牌与采样 ω |
| `fit_data_` | `std::deque<std::pair<double,double>>` | 观测数据缓冲 (t, v)；断档清空、超长丢最旧 |

## 函数

### `RansacSineFitter(int max_iterations, double threshold, double min_omega, double max_omega)`

- **作用**：保存全部配置参数，并用 `std::random_device` 播种随机引擎。
- **参数**：`max_iterations` 最大迭代次数（次）；`threshold` 内点阈值（单位同 v）；`min_omega` / `max_omega` 角频率采样上下界（rad/s，要求 min ≤ max）。
- **返回**：无（构造函数）。
- **备注**：不做参数校验；`max_iterations ≤ 0` 时 `fit()` 空转；`min_omega > max_omega` 时 `std::uniform_real_distribution` 行为未定义。因播种来源为随机设备，同一份数据多次拟合可能得到不同结果。

### `void add_data(double t, double v)`

- **作用**：向内部缓冲区追加一个观测点 (t, v)。
- **参数**：`t` 时间戳（单位由调用方定，通常为秒，应单调递增）；`v` 观测值。
- **返回**：无。
- **备注**：缓冲区非空且 `t − 队尾时刻 > 5` 时先清空全部历史（5 是硬编码阈值，单位与 t 一致，即通常 5 s），用于隔离掉帧/换目标造成的两段不连续信号；不检查 t 的单调性，时间回退不会清空但会让拟合失真。

### `void fit()`

- **作用**：对缓冲区全部数据执行一轮 RANSAC 拟合，结果优于 `best_result_` 时更新它。
- **参数**：无。
- **返回**：无（结果从 `best_result_` 读取）。
- **备注**：数据 < 3 条时立即返回（不清空结果）；每轮迭代 =「洗牌取 3 点 → 随机采 ω → 解线性最小二乘 → 全量数据统计内点 → 与历史最优比较（仅严格更优才更新）」；每次调用结束若数据超过 150 条会丢弃最旧的一条，保证计算量有界；本轮无更优结果时 `best_result_` 保持历史最优，**不会被清零**，故调用方必须自行判断 inliers 是否足够。

### `double sine_function(double t, double A, double omega, double phi, double C)`

- **作用**：正弦模型求值 `v(t) = A·sin(ω·t + φ) + C`（内联，供调用方复用同一公式）。
- **参数**：`t` 自变量（单位同 `add_data` 的 t）；`A` 振幅；`omega` 角频率；`phi` 初相；`C` 直流偏置。
- **返回**：该时刻的模型预测值，单位同 v。
- **备注**：无边界检查；参数非法（如 ω 为 Inf）时按 IEEE 浮点规则返回 NaN / Inf。

### `bool fit_partial_model(const std::vector<std::pair<double,double>> & sample, double omega, Eigen::Vector3d & params)`（private）

- **作用**：固定 ω，对给定样本解线性最小二乘 `v ≈ A1·sin(ωt) + A2·cos(ωt) + C`，输出 (A1, A2, C)。
- **参数**：`sample` 样本点集（一般恰好 3 个点）；`omega` 已固定的角频率；`params` 输出，维度 3。
- **返回**：`true` 求解完成（params 有效）；`false` 表示求解过程抛出 C++ 异常（如内存分配失败），调用方应跳过本组假设。
- **备注**：用 `bdcSvd(ComputeThinU | ComputeThinV).solve` 而非正规方程；ω = 0 时设计矩阵秩亏，SVD 给最小范数解而不报错。

### `int evaluate_inliers(double A, double omega, double phi, double C)`（private）

- **作用**：用当前全部缓存数据评价一组模型参数，统计内点个数（RANSAC 评分函数）。
- **参数**：`A` 振幅；`omega` 角频率；`phi` 初相；`C` 直流偏置。
- **返回**：内点个数：满足 `|v − (A·sin(ωt + φ) + C)| < threshold_` 的样本条数。
- **备注**：缓冲区为空时返回 0；每次假设都要遍历全量数据，是 `fit()` 的主要耗时来源。
