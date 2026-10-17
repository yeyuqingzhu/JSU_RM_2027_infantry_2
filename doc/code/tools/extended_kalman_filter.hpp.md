# tools/extended_kalman_filter.hpp

**概要**：扩展卡尔曼滤波（EKF）通用件的对外接口声明——类内不绑定任何具体模型，状态转移 F、过程噪声 Q、观测矩阵 H、观测噪声 R 以及非线性函数 f / h、非线性流形上的加减法 x_add / z_subtract 全部由调用方传入，并内建 NIS / NEES 卡方检验统计用于诊断。

## 类

### `tools::ExtendedKalmanFilter`

- **职责**：维护「状态向量 x + 状态协方差 P」，提供 predict（时间更新）与 update（量测更新）两个步骤，并统计卡方检验指标。状态与协方差直接以公有成员暴露，读取无需 getter。
- **典型调用序列**：`ExtendedKalmanFilter ekf(x0, P0); ekf.predict(F, Q); ekf.update(z, H, R);` 之后读 `ekf.x` / `ekf.data["nis"]`。
- **备注**：predict / update 会就地修改成员，**非线程安全**；不同实例彼此独立。

### 成员变量

| 成员 | 类型 | 含义 |
| --- | --- | --- |
| `x` | `Eigen::VectorXd` | 后验状态向量，维度 n；每次 predict / update 后被就地更新 |
| `P` | `Eigen::MatrixXd` | 后验状态协方差，n×n；update 用 Joseph 形式更新以保持对称半正定 |
| `data` | `std::map<std::string, double>` | 调试字典：残差 4 分量、nis、nees、nis_fail、nees_fail、recent_nis_failures |
| `recent_nis_failures` | `std::deque<int>` | 最近 window_size 次 NIS 是否超阈值的滑动窗口（元素 1 = 失败、0 = 通过），初值含一个 0 |
| `window_size` | `size_t` | 滑动窗口长度，默认 100 |
| `last_nis` | `double` | 最近一次 update 算出的 NIS（与 `data["nis"]` 相同） |
| `I`（private） | `Eigen::MatrixXd` | 单位阵 n×n，按 x0.rows() 生成，用于 Joseph 形式更新 |
| `x_add`（private） | `std::function<VectorXd(VectorXd, VectorXd)>` | 状态加法，非线性流形（如角度）上的累加 |
| `nees_count_` / `nis_count_` / `total_count_`（private） | `int` | 超阈值累计次数与 update 总次数，目前只累加、无对外读取接口 |

### 维度与数值约定

- 状态维 n = x.rows()，量测维 m = z.rows()；要求 F 为 n×n、Q 为 n×n、H 为 m×n、R 为 m×m，**本类不做维度检查**，不匹配时 Eigen 断言失败（Debug 下 abort）。
- 单位由调用方赋予：源工程自瞄中 4 维量测依次为 yaw(rad)、pitch(rad)、distance(m)、angle(rad)；卡方统计量是量纲一的。
- `update` 末尾按 `residual[0..3]` 写调试数据，隐含要求 **m ≥ 4**，否则越界（Release 下 Eigen 不检查）。

## 函数

### `ExtendedKalmanFilter()`

- **作用**：默认构造函数（`= default`），x / P 为空矩阵、x_add 为空函数对象、data 为空表。
- **参数**：无。
- **返回**：对象。
- **备注**：仅供容器或成员声明占位；此时调用 predict / update 会抛 `std::bad_function_call`。真正参与滤波必须用带初值的构造函数。

### `ExtendedKalmanFilter(const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0, std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add = 欧氏加法)`

- **作用**：带初值的构造函数，保存状态/协方差初值，按状态维生成单位阵 I，并预置 data 的全部键（初值 0）。
- **参数**：
  - `x0`：状态初值，维度 n（单位由调用方定义）。
  - `P0`：状态协方差初值，n×n 对称半正定阵。
  - `x_add`：状态加法函数对象，默认欧氏加法 `a + b`；角度类状态可传入环绕加法。
- **返回**：无（构造函数）。
- **备注**：不做维度校验（x0.rows() 必须等于 P0 的行列数）；x0 为空向量时 I 退化为 0×0。`data["nis_fail"] / data["nees_fail"]` 在 update 中超阈值后置 1 且**不复位**（粘滞标志）。

### `Eigen::VectorXd predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)`

- **作用**：线性状态预测：`x = F·x`，`P = F·P·Fᵀ + Q`。
- **参数**：`F` 状态转移矩阵 n×n；`Q` 过程噪声协方差 n×n（半正定）。
- **返回**：预测后的状态向量 x（即成员 x，非副本）。
- **备注**：内部转发给三参数重载，f 取线性形式 `F·x`；等效于标准卡尔曼滤波的预测步。不做数值兜底。

### `Eigen::VectorXd predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q, std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)`

- **作用**：非线性状态预测：`x = f(x)`，`P = F·P·Fᵀ + Q`，其中 F 是 f 在上一状态处的雅可比（由调用方线性化后传入）。
- **参数**：`F` 状态转移雅可比 n×n；`Q` 过程噪声协方差 n×n；`f` 非线性状态转移函数（输入旧状态、输出新状态，维度必须仍为 n）。
- **返回**：预测后的状态向量 x。
- **备注**：实现顺序是「先更新 P、再更新 x」，两者都以更新前的 x 为线性化点（与源工程一致，勿改）。f 抛异常时 P 已更新而 x 未更新，对象会处于不一致状态，调用方需保证 f 不抛异常。

### `Eigen::VectorXd update(const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R, std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract = 欧氏减法)`

- **作用**：线性量测更新：`K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹`，P 用 Joseph 形式更新，`x = x + K·(z − H·x)`，并做卡方检验统计。
- **参数**：`z` 量测向量 m 维（要求 m ≥ 4）；`H` 观测矩阵 m×n；`R` 量测噪声协方差 m×m（应正定）；`z_subtract` 量测减法，默认 `a − b`。
- **返回**：更新后的状态向量 x（即成员 x）。
- **备注**：内部转发给带 h 的五参数重载，h 取线性形式 `H·x`。`H·P·Hᵀ + R` 奇异时 K 为 NaN / Inf，本重载不检测。

### `Eigen::VectorXd update(const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R, std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h, std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract = 欧氏减法)`

- **作用**：非线性量测更新（核心实现）：h(x) 为观测方程，H 为其雅可比；计算增益、Joseph 形式更新协方差、更新状态，再做 NIS / NEES 卡方检验与滑动窗口失败率统计，最后刷新 data。
- **参数**：`z` 量测向量 m 维（m ≥ 4）；`H` 观测雅可比 m×n；`R` 量测噪声协方差 m×m；`h` 观测函数（输入状态、输出预测量测，维度必须为 m）；`z_subtract` 量测减法，默认欧氏减法，角度量测应传环绕减法。
- **返回**：更新后的状态向量 x（即成员 x）。
- **备注（源工程行为细节，务请保持）**：
  1. 增益 K 用**先验** x 与**先验** P 计算；状态更新项中的 h(x) 同样取更新前的 x，与线性化点一致；
  2. 写入 `data` 的 residual 与 NIS 是在 x 更新完成后、用**后验** x 重新计算的残差，并不是标准 KF 的新息 `z − h(x_先验)`；
  3. NEES 用 `(x − x_先验)` 与**后验** P 计算；
  4. 两个阈值常量都取 0.711（注释写的是自由度 4、95% 置信），与常见查表值约 9.488 不一致，属源工程既有取值，迁移时原样保留；
  5. `data["nis_fail"] / data["nees_fail"]` 为粘滞标志；`recent_nis_failures` 被外部 clear 后失败率计算会除零。
