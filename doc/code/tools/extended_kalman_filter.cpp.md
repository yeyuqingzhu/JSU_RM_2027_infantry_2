# tools/extended_kalman_filter.cpp

**概要**：EKF 通用件 `tools/extended_kalman_filter.hpp` 的实现——构造函数初始化与调试字典预置、predict 的两个重载、update 的两个重载（Joseph 形式协方差更新 + NIS/NEES 卡方检验 + 滑动窗口失败率统计）。

## 函数

### `ExtendedKalmanFilter::ExtendedKalmanFilter(const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0, std::function<...> x_add)`

- **作用**：实现要点——初始化列表按成员声明顺序依次初始化 `x(x0)`、`P(P0)`、`I(Identity(x0.rows(), x0.rows()))`、`x_add(x_add)`；函数体内把 `data` 的 9 个键全部显式赋 0.0。
- **参数**：`x0` 状态初值（n 维）；`P0` 协方差初值（n×n）；`x_add` 状态加法函数对象。
- **返回**：无。
- **备注**：`I` 的维度取自 `x0.rows()`，因此 x0 为空向量时 I 为 0×0；构造时不做任何维度一致性检查。data 预置键的意义：保证首次 update 之前读取 `data["nis"]` 等也能拿到确定值（否则 `std::map::operator[]` 会静默插入 0.0，但语义上不再是「已初始化」）。

### `Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)`

- **作用**：实现要点——仅做转发：用 `[&]` 捕获 F 构造线性状态转移 lambda `f(x) = F * x`，交给三参数重载执行，本函数自身不含数学运算。
- **参数**：`F` 状态转移矩阵 n×n；`Q` 过程噪声 n×n。
- **返回**：预测后的状态 x。
- **备注**：lambda 以引用捕获 F，只在本次调用内使用，F 的生命周期无需延长。

### `Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q, std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)`

- **作用**：实现要点（关键步骤，顺序不可交换）：
  1. `P = F * P * F.transpose() + Q;` —— 先用**预测前**的 F 与 P 更新协方差；
  2. `x = f(x);` —— 再做非线性状态推进；
  3. `return x;`
- **参数**：`F` 状态转移雅可比 n×n；`Q` 过程噪声 n×n；`f` 状态转移函数。
- **返回**：预测后的状态 x。
- **备注**：协方差用的是调用方给出的线性化矩阵 F，本函数不对 f 做自动微分；f 返回维度必须为 n。

### `Eigen::VectorXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R, std::function<...> z_subtract)`

- **作用**：实现要点——仅做转发：用 `[&]` 捕获 H 构造线性观测 lambda `h(x) = H * x`，连同 z_subtract 一起交给五参数重载。
- **参数**：`z` 量测向量 m 维；`H` 观测矩阵 m×n；`R` 量测噪声 m×m；`z_subtract` 量测减法。
- **返回**：更新后的状态 x。
- **备注**：所有卡方检验与调试数据刷新都发生在被转发的五参数重载里。

### `Eigen::VectorXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R, std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h, std::function<...> z_subtract)`

- **作用**：实现要点（关键步骤，逐条对应源码注释）：
  1. `Eigen::VectorXd x_prior = x;` —— 保存先验状态，供 NEES 使用；
  2. `K = P * H.transpose() * (H * P * H.transpose() + R).inverse();` —— 卡尔曼增益，此时 P 仍为先验协方差；通用 `inverse()` 不做正定性检查；
  3. `P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();` —— **Joseph 形式**更新后验协方差（源码注释引用了 rlabbe 的 Kalman-and-Bayesian-Filters-in-Python），数值上比 `(I−KH)P` 更能保持对称性与半正定性；
  4. `x = x_add(x, K * z_subtract(z, h(x)));` —— 状态更新，其中 `h(x)` 取的是**更新前**的 x（先验），与第 2 步的线性化点一致；
  5. 卡方检验：`residual = z_subtract(z, h(x))` 用**后验** x 重算残差；`S = H * P * H.transpose() + R` 也用更新后的 P 重算；`nis = residualᵀ·S⁻¹·residual`；`nees = (x − x_prior)ᵀ·P⁻¹·(x − x_prior)`；
  6. 阈值判定与计数：`nis > 0.711` / `nees > 0.711` 时分别自增 `nis_count_` / `nees_count_` 并把 `data["nis_fail"] / data["nees_fail"]` 置 1（粘滞，不复位）；`total_count_++`；`last_nis = nis`；
  7. 滑动窗口：`recent_nis_failures.push_back(nis > 阈值 ? 1 : 0)`，超过 `window_size` 则 `pop_front()`；用 `std::accumulate` 求窗口内失败数并除以窗口长度得到失败率；
  8. 写调试数据：`data["residual_yaw/pitch/distance/angle"] = residual[0..3]`，`data["nis"]`、`data["nees"]`、`data["recent_nis_failures"] = recent_rate`，返回 x。
- **参数**：`z` 量测向量 m 维（要求 m ≥ 4）；`H` 观测雅可比 m×n；`R` 量测噪声协方差 m×m（正定）；`h` 观测函数；`z_subtract` 量测减法。
- **返回**：更新后的状态向量 x（即成员 x）。
- **备注 / 边界条件**：
  - `H·P·Hᵀ + R` 或 P 奇异、病态时 `inverse()` 会产出 NaN / Inf 并污染 x、P，本函数不做兜底；
  - `residual[0..3]` 越界访问的前提是 m < 4（Release 下无检查，属未定义行为）；
  - `recent_nis_failures` 为空（被外部 clear）时失败率计算除零；
  - 残差/NIS 使用后验 x、NEES 使用后验 P，属源工程刻意实现，迁移时保持原样；
  - 源码中三个私有计数器只累加、从未被读取，属"预留统计量"，迁移时保留（编译器不会告警）。
