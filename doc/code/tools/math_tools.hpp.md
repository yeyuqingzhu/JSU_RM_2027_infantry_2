# tools/math_tools.hpp

**概要**：声明一组与平台无关的纯数学工具函数——角度归一化、四元数/旋转矩阵与欧拉角互转、直角坐标与球坐标（yaw-pitch-distance）互转及其雅可比矩阵、时间差、向量夹角、上下限裁剪与平方。

- **来源**：sp_vision_25 的 `tools/math_tools.hpp`
- **依赖**：Eigen3（`<Eigen/Geometry>`）、C++17 `<chrono>`；实现文件额外使用 OpenCV4 的 `CV_PI` 宏
- **线程安全**：全部为无状态纯函数，不读写共享数据，可并发调用
- **单位约定**：角度一律为弧度（rad）；长度/距离单位由调用方自行统一，本文件不做换算

## 函数

### `double limit_rad(double angle)`

- **作用**：把任意弧度角归一化到左开右闭区间 (-pi, pi]。
- **参数**：`angle` —— 待归一化的角度，单位 rad，可为任意**有限**实数。
- **返回**：与 `angle` 等价的角度，单位 rad，范围 (-pi, pi]。
- **备注**：恰好为 -pi 的输入被映射为 +pi；NaN 原样返回；±inf 会死循环，调用方须保证输入有限。

### `Eigen::Vector3d eulers(Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic = false)`

- **作用**：按指定旋转轴顺序，把四元数转换为三轴欧拉角（内部按模长归一化，输入无需单位化）。
- **参数**：`q` 旋转四元数（`coeffs()` 顺序 x, y, z, w）；`axis0/axis1/axis2` 旋转轴编号 0=x、1=y、2=z，按“先 axis0、再 axis1、最后 axis2”的施加顺序给出（zyx 传 `2,1,0`）；`extrinsic` false=内旋/随体轴（默认），true=外旋/固定轴。
- **返回**：`Eigen::Vector3d`，三个分量依次为绕 axis0、axis1、axis2 的转角，单位 rad，已逐个归一化到 (-pi, pi]。
- **备注**：零四元数使内部 `n2 = 0`，结果退化为 NaN；万向锁（中间角与 0 或 pi 的距离 < 1e-7）时解不唯一，函数会把一个自由度置 0，取法与其它库不保证一致。参考实现：<https://github.com/evbernardes/quaternion_to_euler>。

### `Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic = false)`

- **作用**：按指定旋转轴顺序，把 3x3 旋转矩阵转换为三轴欧拉角（内部先转四元数）。
- **参数**：`R` 3x3 旋转矩阵（非正交矩阵会被隐式近似为最近似旋转）；其余参数同四元数重载。
- **返回**：`Eigen::Vector3d`，三个分量依次为绕 axis0、axis1、axis2 的转角，单位 rad，范围 (-pi, pi]。
- **备注**：边界条件与四元数重载相同；`R` 为零矩阵时结果为 NaN。

### `Eigen::Matrix3d rotation_matrix(const Eigen::Vector3d & ypr)`

- **作用**：欧拉角（yaw-pitch-roll）转旋转矩阵，实现 `R = Rz(yaw) * Ry(pitch) * Rx(roll)`（源注释：zyx，先绕 z 轴，再绕 y 轴，最后绕 x 轴）。
- **参数**：`ypr` —— `ypr[0]=yaw`（绕 z）、`ypr[1]=pitch`（绕 y）、`ypr[2]=roll`（绕 x），单位 rad，不要求落在 (-pi, pi] 内。
- **返回**：`Eigen::Matrix3d`，正交旋转矩阵（未做正交化）。
- **备注**：无奇异点，任意有限输入都返回有限值。

### `Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz)`

- **作用**：直角坐标 (x, y, z) 转球坐标 (yaw, pitch, distance)。
- **参数**：`xyz` —— 直角坐标，单位任意（与输出的 distance 同量纲）。
- **返回**：`(yaw, pitch, distance)`：`yaw = atan2(y, x)` ∈ (-pi, pi]（rad）；`pitch = atan2(z, sqrt(x²+y²))` ∈ (-pi/2, pi/2)（rad）；`distance = |xyz|` ≥ 0。
- **备注**：零向量返回 `(0, 0, 0)`；`x = y = 0` 时 yaw 退化为 0。

### `Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d & xyz)`

- **作用**：给出 `xyz2ypd` 的解析雅可比矩阵 `J = d(yaw, pitch, distance) / d(x, y, z)`。
- **参数**：`xyz` —— 直角坐标，单位任意（三行量纲分别为 rad/长度、rad/长度、无量纲）。
- **返回**：3x3 `Eigen::MatrixXd`，行序 yaw、pitch、distance，列序 x、y、z。
- **备注**：`x² + y² = 0`（落在 z 轴上）或 `xyz` 为零向量时出现 inf/NaN，调用方必须避开该奇异点。

### `Eigen::Vector3d ypd2xyz(const Eigen::Vector3d & ypd)`

- **作用**：球坐标 (yaw, pitch, distance) 转直角坐标 (x, y, z)，与 `xyz2ypd` 互逆。
- **参数**：`ypd` —— yaw、pitch 单位 rad；distance 单位与输出一致（传入负值相当于整体取反，函数不校验）。
- **返回**：`Eigen::Vector3d(x, y, z)`，`x = d·cos(pitch)·cos(yaw)`、`y = d·cos(pitch)·sin(yaw)`、`z = d·sin(pitch)`。
- **备注**：无奇异点，任意有限输入都返回有限值。

### `Eigen::MatrixXd ypd2xyz_jacobian(const Eigen::Vector3d & ypd)`

- **作用**：给出 `ypd2xyz` 的解析雅可比矩阵（函数名沿用源工程，实际是 `d(x, y, z) / d(yaw, pitch, distance)`）。
- **参数**：`ypd` —— yaw、pitch 单位 rad；distance 单位与输出一致。
- **返回**：3x3 `Eigen::MatrixXd`，行序 x、y、z，列序 yaw、pitch、distance（前两列量纲为长度，第三列无量纲）。
- **备注**：无奇异点；distance 为 0 时矩阵全零。

### `double delta_time(const std::chrono::steady_clock::time_point & a, const std::chrono::steady_clock::time_point & b)`

- **作用**：计算时间差 `a - b`，单位秒。
- **参数**：`a`、`b` —— `std::chrono::steady_clock::time_point`（单调时钟，不受系统时间调整影响）。
- **返回**：`a - b`，单位 s，可为负（a 早于 b 时）。
- **备注**：时钟分辨率有限，同一时刻两次采样可能得到 0；判零应使用 epsilon。禁止与 `system_clock` 的时间点混用。

### `double get_abs_angle(const Eigen::Vector2d & vec1, const Eigen::Vector2d & vec2)`

- **作用**：计算两个二维向量的夹角（无方向，取绝对值）。源注释：向量夹角总是返回 0 ~ pi，来自 SJTU。
- **参数**：`vec1`、`vec2` —— 任意二维向量，量纲无关（只出现在比值中）。
- **返回**：夹角，单位 rad，范围 [0, pi]。
- **备注**：任一向量为零向量（`norm() == 0`）时直接返回 0；浮点误差可能使 cos 略超出 [-1, 1] 导致 `acos` 返回 NaN（源工程行为，保持不变）。

### `template <typename T> T square(T const & a)`

- **作用**：通用平方函数模板。
- **参数**：`a` —— 任意支持 `operator*` 且可拷贝的类型（数值、Eigen 矩阵等）。
- **返回**：`a * a`，类型与入参相同。
- **备注**：header-only，可内联；整数类型可能溢出。

### `double limit_min_max(double input, double min, double max)`

- **作用**：把输入饱和裁剪到 `[min, max]` 区间。
- **参数**：`input` 待裁剪值；`min`、`max` 下界与上界（不检查 `min <= max`）。
- **返回**：`input > max` 返回 `max`；`input < min` 返回 `min`；否则原样返回。
- **备注**：NaN 输入原样返回 NaN；`min > max` 时区间语义失效。
