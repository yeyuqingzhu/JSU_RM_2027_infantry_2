// =============================================================================
// tools/math_tools.cpp —— tools/math_tools.hpp 中声明的数学工具的实现
// -----------------------------------------------------------------------------
// 【作用】实现角度归一化、姿态/坐标变换及其雅可比矩阵等纯数学函数。
// 【来源】sp_vision_25 的 tools/math_tools.cpp。
// 【依赖】<cmath>（三角函数、sqrt、pow、acos）；<opencv2/core.hpp>（仅取 CV_PI 宏，
//         不使用任何 OpenCV 运行时代码）；Eigen3（经 math_tools.hpp 间接引入）。
// 【注意事项】
//   1) 线程安全：全部函数无静态/全局可写状态，天然线程安全。
//   2) 单位：角度为弧度，距离单位与输入一致（不做换算）。
//   3) 除零/奇异点：雅可比矩阵在坐标原点、z 轴等奇异点会出现 inf 或 NaN，
//      调用方必须保证输入不在奇异点上（详见函数内注释）。
//   4) 本文件不抛异常，除注明处外也不做输入合法性校验，输入非法时结果为 NaN 或 inf。
// =============================================================================

#include "math_tools.hpp"

#include <cmath>
#include <opencv2/core.hpp>  // CV_PI

namespace tools
{
// ----------------------------------------------------------------------------
// double limit_rad(double angle) —— 角度归一化到 (-pi, pi]
// ----------------------------------------------------------------------------
// 作用：以 2*pi 为步长反复加减，把任意角度折算到 (-pi, pi]。
// 参数：angle，单位 rad。
// 返回：等价角度，单位 rad，范围 (-pi, pi]。
// 边界：-pi 被映射为 +pi；NaN 原样返回；±inf 会死循环。
// 实现要点：用 while 而非 fmod，避免 fmod 对负数的取模符号问题；
//           判断条件写作 angle <= -PI，因此区间左端开、右端闭。
// ----------------------------------------------------------------------------
double limit_rad(double angle)
{
  while (angle > CV_PI) angle -= 2 * CV_PI;
  while (angle <= -CV_PI) angle += 2 * CV_PI;
  return angle;
}

// ----------------------------------------------------------------------------
// Eigen::Vector3d eulers(Eigen::Quaterniond, int, int, int, bool) —— 四元数转欧拉角
// ----------------------------------------------------------------------------
// 作用：按给定轴序把四元数转成三轴欧拉角，支持内旋(intrinsic)/外旋(extrinsic)。
// 参数：q 四元数（coeffs 顺序 x,y,z,w，内部按模长归一化，无需预先单位化）；
//       axis0/axis1/axis2 轴编号 0=x,1=y,2=z，表示施加顺序；extrinsic 是否外旋。
// 返回：Vector3d，依次为绕 axis0、axis1、axis2 的转角，单位 rad，逐个归一化到 (-pi, pi]。
// 边界：零四元数导致 n2 = 0 → NaN；万向锁（|eulers[1]| < 1e-7 或 |eulers[1]-pi| < 1e-7）
//       时解不唯一，按约定把某个自由度置 0。
// 实现要点：采用 quaternion_to_euler 的“半和/半差”算法；proper（首末轴相同）与
//           tait-bryan（三轴互异）两种情形分别构造 (a,b,c,d)，其中 sign 区分左右手系；
//           最后对非 proper 情形做 pitch -= pi/2 与符号修正，内外旋再交换首末分量。
// ----------------------------------------------------------------------------
Eigen::Vector3d eulers(Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic)
{
  if (!extrinsic) std::swap(axis0, axis2);

  auto i = axis0, j = axis1, k = axis2;
  auto is_proper = (i == k);
  if (is_proper) k = 3 - i - j;
  auto sign = (i - j) * (j - k) * (k - i) / 2;

  double a, b, c, d;
  Eigen::Vector4d xyzw = q.coeffs();
  if (is_proper) {
    a = xyzw[3];
    b = xyzw[i];
    c = xyzw[j];
    d = xyzw[k] * sign;
  } else {
    a = xyzw[3] - xyzw[j];
    b = xyzw[i] + xyzw[k] * sign;
    c = xyzw[j] + xyzw[3];
    d = xyzw[k] * sign - xyzw[i];
  }

  Eigen::Vector3d eulers;
  auto n2 = a * a + b * b + c * c + d * d;
  eulers[1] = std::acos(2 * (a * a + b * b) / n2 - 1);

  auto half_sum = std::atan2(b, a);
  auto half_diff = std::atan2(-d, c);

  auto eps = 1e-7;
  auto safe1 = std::abs(eulers[1]) >= eps;
  auto safe2 = std::abs(eulers[1] - CV_PI) >= eps;
  auto safe = safe1 && safe2;
  if (safe) {
    eulers[0] = half_sum + half_diff;
    eulers[2] = half_sum - half_diff;
  } else {
    if (!extrinsic) {
      eulers[0] = 0;
      if (!safe1) eulers[2] = 2 * half_sum;
      if (!safe2) eulers[2] = -2 * half_diff;
    } else {
      eulers[2] = 0;
      if (!safe1) eulers[0] = 2 * half_sum;
      if (!safe2) eulers[0] = 2 * half_diff;
    }
  }

  // 迁移修补：循环变量原为 i，与外层 auto i = axis0 同名（-Wshadow 告警），
  // 重命名为 idx；作用域与语义完全不变。
  for (int idx = 0; idx < 3; idx++) eulers[idx] = limit_rad(eulers[idx]);

  if (!is_proper) {
    eulers[2] *= sign;
    eulers[1] -= CV_PI / 2;
  }

  if (!extrinsic) std::swap(eulers[0], eulers[2]);

  return eulers;
}

// ----------------------------------------------------------------------------
// Eigen::Vector3d eulers(Eigen::Matrix3d R, ...) —— 旋转矩阵转欧拉角
// ----------------------------------------------------------------------------
// 作用：把 3x3 旋转矩阵转成欧拉角，直接复用四元数重载。
// 参数：R 旋转矩阵；其余参数含义同四元数重载。
// 返回：与四元数重载一致的欧拉角，单位 rad，范围 (-pi, pi]。
// 边界：R 非正交时被隐式近似为最近似旋转；R 为零矩阵时退化为 NaN。
// 实现要点：Eigen::Quaterniond q(R) 构造时按“最近似旋转”做归一化，因此不需要
//           调用方保证 R 严格正交。
// ----------------------------------------------------------------------------
Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic)
{
  Eigen::Quaterniond q(R);
  return eulers(q, axis0, axis1, axis2, extrinsic);
}

// ----------------------------------------------------------------------------
// Eigen::Matrix3d rotation_matrix(const Eigen::Vector3d & ypr) —— 欧拉角转旋转矩阵
// ----------------------------------------------------------------------------
// 作用：按 R = Rz(yaw) * Ry(pitch) * Rx(roll) 生成旋转矩阵。
// 参数：ypr[0]=yaw（绕 z），ypr[1]=pitch（绕 y），ypr[2]=roll（绕 x），单位 rad。
// 返回：Matrix3d 旋转矩阵（未做正交化）。
// 边界：无奇异点，任意有限输入均返回有限值。
// 实现要点：展开后的 9 个元素逐项写出（clang-format off 保持矩阵排版便于人工核对），
//           三角函数各自只计算一次以复用。
// ----------------------------------------------------------------------------
Eigen::Matrix3d rotation_matrix(const Eigen::Vector3d & ypr)
{
  double roll = ypr[2];
  double pitch = ypr[1];
  double yaw = ypr[0];
  double cos_yaw = cos(yaw);
  double sin_yaw = sin(yaw);
  double cos_pitch = cos(pitch);
  double sin_pitch = sin(pitch);
  double cos_roll = cos(roll);
  double sin_roll = sin(roll);
  // clang-format off
    Eigen::Matrix3d R{
      {cos_yaw * cos_pitch, cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll, cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll},
      {sin_yaw * cos_pitch, sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll, sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll},
      {         -sin_pitch,                                cos_pitch * sin_roll,                                cos_pitch * cos_roll}
    };
  // clang-format on
  return R;
}

// ----------------------------------------------------------------------------
// Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz) —— 直角坐标转球坐标
// ----------------------------------------------------------------------------
// 作用：直角坐标 → (yaw, pitch, distance)。
// 参数：xyz 直角坐标，单位任意（与 distance 同量纲）。
// 返回：yaw = atan2(y, x) ∈ (-pi, pi]；pitch = atan2(z, sqrt(x²+y²)) ∈ (-pi/2, pi/2)；
//       distance = sqrt(x²+y²+z²) >= 0；角度单位 rad。
// 边界：零向量返回 (0, 0, 0)；x=y=0 时 yaw 退化为 0。
// 实现要点：全部使用 double 精度，sqrt 与 atan2 各调用一次，无中间状态。
// ----------------------------------------------------------------------------
Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz)
{
  auto x = xyz[0], y = xyz[1], z = xyz[2];
  auto yaw = std::atan2(y, x);
  auto pitch = std::atan2(z, std::sqrt(x * x + y * y));
  auto distance = std::sqrt(x * x + y * y + z * z);
  return {yaw, pitch, distance};
}

// ----------------------------------------------------------------------------
// Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d & xyz) —— xyz2ypd 的雅可比
// ----------------------------------------------------------------------------
// 作用：给出 xyz2ypd 的解析雅可比 J = d(yaw, pitch, distance)/d(x, y, z)。
// 参数：xyz 直角坐标（单位与输入一致；三行量纲分别为 rad/长度、rad/长度、无量纲）。
// 返回：3x3 MatrixXd，行序为 yaw、pitch、distance，列序为 x、y、z。
// 边界：x²+y² = 0（z 轴上）时分母为 0，矩阵出现 inf/NaN；xyz 为零向量同理。
// 实现要点：pitch 的两个偏导写成“先除以 (x²+y²) 再整体除 (x²+y²)^1.5”的等价形式，
//           避免多次重复求 r；d yaw/d z 与 d yaw 无关项显式为 0。
// ----------------------------------------------------------------------------
Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d & xyz)
{
  auto x = xyz[0], y = xyz[1], z = xyz[2];

  auto dyaw_dx = -y / (x * x + y * y);
  auto dyaw_dy = x / (x * x + y * y);
  auto dyaw_dz = 0.0;

  auto dpitch_dx = -(x * z) / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 1.5));
  auto dpitch_dy = -(y * z) / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 1.5));
  auto dpitch_dz = 1 / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 0.5));

  auto ddistance_dx = x / std::pow((x * x + y * y + z * z), 0.5);
  auto ddistance_dy = y / std::pow((x * x + y * y + z * z), 0.5);
  auto ddistance_dz = z / std::pow((x * x + y * y + z * z), 0.5);

  // clang-format off
  Eigen::MatrixXd J{
    {dyaw_dx, dyaw_dy, dyaw_dz},
    {dpitch_dx, dpitch_dy, dpitch_dz},
    {ddistance_dx, ddistance_dy, ddistance_dz}
  };
  // clang-format on

  return J;
}

// ----------------------------------------------------------------------------
// Eigen::Vector3d ypd2xyz(const Eigen::Vector3d & ypd) —— 球坐标转直角坐标
// ----------------------------------------------------------------------------
// 作用：球坐标 (yaw, pitch, distance) → 直角坐标 (x, y, z)，是 xyz2ypd 的逆变换。
// 参数：ypd[0]=yaw、ypd[1]=pitch（单位 rad），ypd[2]=distance（单位与输出一致）。
// 返回：Vector3d(x, y, z)，满足 x = d·cos(pitch)·cos(yaw)、y = d·cos(pitch)·sin(yaw)、
//       z = d·sin(pitch)。
// 边界：无奇异点；distance 为负时结果相当于整体取反（不做校验）。
// ----------------------------------------------------------------------------
Eigen::Vector3d ypd2xyz(const Eigen::Vector3d & ypd)
{
  auto yaw = ypd[0], pitch = ypd[1], distance = ypd[2];
  auto x = distance * std::cos(pitch) * std::cos(yaw);
  auto y = distance * std::cos(pitch) * std::sin(yaw);
  auto z = distance * std::sin(pitch);
  return {x, y, z};
}

// ----------------------------------------------------------------------------
// Eigen::MatrixXd ypd2xyz_jacobian(const Eigen::Vector3d & ypd) —— ypd2xyz 的雅可比
// ----------------------------------------------------------------------------
// 作用：给出 ypd2xyz 的解析雅可比 J = d(x, y, z)/d(yaw, pitch, distance)。
// 参数：ypd 同 ypd2xyz。
// 返回：3x3 MatrixXd，行序为 x、y、z，列序为 yaw、pitch、distance
//       （前两列量纲为长度，第三列无量纲）。
// 边界：无奇异点（distance = 0 时矩阵全零）。
// 实现要点：三角函数先各自算一次再复用，减少重复计算。
// ----------------------------------------------------------------------------
Eigen::MatrixXd ypd2xyz_jacobian(const Eigen::Vector3d & ypd)
{
  auto yaw = ypd[0], pitch = ypd[1], distance = ypd[2];
  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);
  double cos_pitch = std::cos(pitch);
  double sin_pitch = std::sin(pitch);

  auto dx_dyaw = distance * cos_pitch * -sin_yaw;
  auto dy_dyaw = distance * cos_pitch * cos_yaw;
  auto dz_dyaw = 0.0;

  auto dx_dpitch = distance * -sin_pitch * cos_yaw;
  auto dy_dpitch = distance * -sin_pitch * sin_yaw;
  auto dz_dpitch = distance * cos_pitch;

  auto dx_ddistance = cos_pitch * cos_yaw;
  auto dy_ddistance = cos_pitch * sin_yaw;
  auto dz_ddistance = sin_pitch;

  // clang-format off
  Eigen::MatrixXd J{
    {dx_dyaw, dx_dpitch, dx_ddistance},
    {dy_dyaw, dy_dpitch, dy_ddistance},
    {dz_dyaw, dz_dpitch, dz_ddistance}
  };
  // clang-format on

  return J;
}

// ----------------------------------------------------------------------------
// double delta_time(a, b) —— 时间差（秒）
// ----------------------------------------------------------------------------
// 作用：求 a - b，转为秒为单位的 double。
// 参数：a、b 为 steady_clock 时间点（单调时钟）。
// 返回：a - b，单位 s，可为负。
// 边界：分辨率不足时可能为 0；禁止与 system_clock 的时间点混用。
// 实现要点：用 std::chrono::duration<double> 承接差值再取 count()，避免手写除法的精度损失。
// ----------------------------------------------------------------------------
double delta_time(
  const std::chrono::steady_clock::time_point & a, const std::chrono::steady_clock::time_point & b)
{
  std::chrono::duration<double> c = a - b;
  return c.count();
}

// ----------------------------------------------------------------------------
// double get_abs_angle(vec1, vec2) —— 二维向量夹角
// ----------------------------------------------------------------------------
// 作用：返回两个二维向量之间的夹角（0 ~ pi，无方向）。
// 参数：vec1、vec2 任意二维向量（量纲无关，因为只出现在比值里）。
// 返回：夹角，单位 rad，范围 [0, pi]。
// 边界：任一向量为零向量时直接返回 0，不进入除法；
//       浮点误差可能使 cos 略超出 [-1,1] 而让 acos 返回 NaN（源工程行为，保持不改）。
// 实现要点：cos = dot/(|v1||v2|)，norm() 被调用两次（未做缓存，与源工程一致）。
// ----------------------------------------------------------------------------
double get_abs_angle(const Eigen::Vector2d & vec1, const Eigen::Vector2d & vec2)
{
  if (vec1.norm() == 0. || vec2.norm() == 0.) {
    return 0.;
  }
  return std::acos(vec1.dot(vec2) / (vec1.norm() * vec2.norm()));
}

// ----------------------------------------------------------------------------
// double limit_min_max(input, min, max) —— 上下限裁剪
// ----------------------------------------------------------------------------
// 作用：把 input 饱和到 [min, max]。
// 参数：input 待裁剪值；min/max 下界与上界（不检查 min <= max）。
// 返回：裁剪后的值。
// 边界：NaN 输入原样返回（两个比较都为 false）；min > max 时区间语义失效。
// ----------------------------------------------------------------------------
double limit_min_max(double input, double min, double max)
{
  if (input > max)
    return max;
  else if (input < min)
    return min;
  return input;
}
}  // namespace tools