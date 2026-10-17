# tools/math_tools.cpp

**概要**：实现 `tools/math_tools.hpp` 中声明的全部数学工具——角度归一化、姿态与坐标变换及其解析雅可比矩阵、时间差、向量夹角与上下限裁剪。

- **来源**：sp_vision_25 的 `tools/math_tools.cpp`
- **依赖**：`<cmath>`；`<opencv2/core.hpp>`（**仅**取 `CV_PI` 宏，不使用任何 OpenCV 运行时代码）；Eigen3（经 `math_tools.hpp` 间接引入）
- **线程安全**：无静态/全局可写状态，天然线程安全
- **实现要点与关键步骤**：本文件不抛异常、除注明处外不做输入校验，非法输入的结果为 NaN/inf；全部函数无状态、可内联，适合在实时线程调用

## 函数

### `double limit_rad(double angle)`

- **作用**：以 2·pi 为步长反复加减，把任意角度折算到 (-pi, pi]。
- **参数**：`angle`，单位 rad。
- **返回**：等价角度，单位 rad，范围 (-pi, pi]。
- **备注**：实现要点——用 `while` 而非 `fmod`，避免负数取模的符号问题；判断条件写作 `angle <= -CV_PI`，因此 -pi 被映射为 +pi。边界——NaN 原样返回，±inf 死循环。

### `Eigen::Vector3d eulers(Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic)`

- **作用**：四元数 → 三轴欧拉角，支持内旋/外旋两种约定。
- **参数**：`q` 四元数（coeffs 顺序 x, y, z, w）；`axis0/axis1/axis2` 轴编号与施加顺序；`extrinsic` 是否外旋。
- **返回**：`Vector3d`，依次为绕 axis0、axis1、axis2 的转角，单位 rad，逐个归一化到 (-pi, pi]。
- **备注**：实现要点——采用 quaternion_to_euler 的“半和/半差”算法，按 proper（首末轴相同）与 tait-bryan（三轴互异）分别构造 `(a, b, c, d)`，`sign` 区分左右手系；最后由 `limit_rad` 逐个归一化，非 proper 情形再做 `pitch -= pi/2` 与符号修正，内外旋交换首末分量。边界——零四元数导致 `n2 = 0` 得到 NaN；万向锁（阈值 1e-7）时非唯一解被置 0。

### `Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic)`

- **作用**：旋转矩阵 → 欧拉角。
- **参数**：`R` 旋转矩阵；其余同四元数重载。
- **返回**：与四元数重载一致的欧拉角，单位 rad，范围 (-pi, pi]。
- **备注**：实现要点——`Eigen::Quaterniond q(R)` 构造时会按最近似旋转归一化，故不要求 `R` 严格正交；随后直接复用四元数重载，无重复算法。

### `Eigen::Matrix3d rotation_matrix(const Eigen::Vector3d & ypr)`

- **作用**：欧拉角 → 旋转矩阵（`Rz(yaw)·Ry(pitch)·Rx(roll)`）。
- **参数**：`ypr[0]=yaw`、`ypr[1]=pitch`、`ypr[2]=roll`，单位 rad。
- **返回**：`Matrix3d` 旋转矩阵。
- **备注**：实现要点——9 个元素逐项展开（源码用 `// clang-format off` 保持矩阵排版以便人工核对），三角函数各算一次复用；无奇异点。

### `Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz)`

- **作用**：直角坐标 → 球坐标。
- **参数**：`xyz` 直角坐标，单位任意。
- **返回**：`(yaw, pitch, distance)`，角度单位 rad，distance 非负。
- **备注**：实现要点——`atan2` 两次、`sqrt` 一次，全 double 精度；零向量返回 `(0,0,0)`。

### `Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d & xyz)`

- **作用**：给出 `xyz2ypd` 的解析雅可比。
- **参数**：`xyz` 直角坐标。
- **返回**：3x3 `MatrixXd`，行序 yaw、pitch、distance，列序 x、y、z。
- **备注**：实现要点——pitch 的两个偏导写作“先除 (x²+y²) 再整体除 (x²+y²)^1.5”的等价形式，减少重复求 r；`d yaw/d z` 显式为 0。边界——`x²+y² = 0` 时出现 inf/NaN。

### `Eigen::Vector3d ypd2xyz(const Eigen::Vector3d & ypd)`

- **作用**：球坐标 → 直角坐标，`xyz2ypd` 的逆变换。
- **参数**：`ypd` —— yaw、pitch（rad），distance（单位与输出一致）。
- **返回**：`Vector3d(x, y, z)`。
- **备注**：无奇异点；distance 为负时结果相当于整体取反（不校验）。

### `Eigen::MatrixXd ypd2xyz_jacobian(const Eigen::Vector3d & ypd)`

- **作用**：给出 `ypd2xyz` 的解析雅可比。
- **参数**：`ypd` 同 `ypd2xyz`。
- **返回**：3x3 `MatrixXd`，行序 x、y、z，列序 yaw、pitch、distance。
- **备注**：实现要点——先算 `sin/cos` 再组合，减少重复计算；无奇异点。

### `double delta_time(const std::chrono::steady_clock::time_point & a, const std::chrono::steady_clock::time_point & b)`

- **作用**：求 `a - b`，转为秒。
- **参数**：`a`、`b` steady_clock 时间点。
- **返回**：`a - b`，单位 s，可为负。
- **备注**：实现要点——用 `std::chrono::duration<double>` 承接差值再取 `count()`，避免手写除法损失精度。

### `double get_abs_angle(const Eigen::Vector2d & vec1, const Eigen::Vector2d & vec2)`

- **作用**：二维向量夹角（0 ~ pi）。
- **参数**：`vec1`、`vec2` 任意二维向量。
- **返回**：夹角，单位 rad，范围 [0, pi]。
- **备注**：实现要点——先判零向量（任一 `norm() == 0` 直接返回 0）再算 `acos(dot/(|v1||v2|))`；`norm()` 被调用两次未做缓存（与源工程一致）。边界——浮点误差可能让 `acos` 返回 NaN。

### `double limit_min_max(double input, double min, double max)`

- **作用**：饱和裁剪到 `[min, max]`。
- **参数**：`input`、`min`、`max`。
- **返回**：裁剪后的值。
- **备注**：实现要点——先与 `max` 比较再与 `min` 比较，两个分支都返回边界值；NaN 输入原样返回。

## 迁移修补（与源文件的行为无关的改动）

1. `eulers` 中循环变量 `i` 改名为 `idx`：源文件中它与外层 `auto i = axis0` 同名，在 `-Wshadow` 下告警；作用域与语义不变。
