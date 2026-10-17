// =============================================================================
// tools/math_tools.hpp —— 通用数学工具（角度归一化 / 姿态与坐标变换 / 时间差）
// -----------------------------------------------------------------------------
// 【作用】为上层算法提供与平台无关的纯数学函数：角度归一化；四元数、旋转矩阵
//         与欧拉角互转；直角坐标与球坐标（yaw-pitch-distance）互转及其雅可比
//         矩阵；时间差；向量夹角；上下限裁剪；平方。
// 【来源】sp_vision_25 的 tools/math_tools.hpp（同济大学 SuperPower 战队 25 赛季
//         视觉框架），迁移时函数签名与语义保持完全一致。
// 【依赖】Eigen3（<Eigen/Geometry>，实际提供 Core/Geometry 的向量与矩阵类型）、
//         C++17 标准库 <chrono>；对应的实现文件额外使用 OpenCV4 的 CV_PI 宏。
// 【注意事项】
//   1) 线程安全：本文件全部为无状态纯函数，不读写任何共享可写数据，
//      多线程并发调用安全。
//   2) 单位约定：角度一律为弧度（rad）；长度/距离单位由调用方自行决定
//      （m 或 mm 均可），本文件不做任何单位换算，混用不同量纲由调用方负责。
//   3) 调用时机：无状态、无缓存、不做滤波，可在任意时机调用。其中
//      xyz2ypd / ypd2xyz 及其雅可比矩阵常用于扩展卡尔曼滤波的量测模型，
//      需注意原点、天顶等奇异点处的数值行为（见各函数说明）。
//   4) 头文件保护宏、命名空间 tools 与全部函数签名与源工程一致，不得改动。
// =============================================================================

#ifndef TOOLS__MATH_TOOLS_HPP
#define TOOLS__MATH_TOOLS_HPP

#include <Eigen/Geometry>
#include <chrono>

namespace tools
{
// 将弧度值限制在(-pi, pi]
// 作用：把任意弧度角归一化到左开右闭区间 (-pi, pi]。
// 参数：angle —— 待归一化的角度，单位 rad，可以是任意有限的实数（含 |angle| >> 2*pi）。
// 返回：与 angle 等价的角度，单位 rad，取值范围 (-pi, pi]。
// 边界：恰好等于 -pi 的输入会被映射为 +pi；输入为 NaN 时两个 while 条件都不成立，
//       原样返回 NaN；输入为 ±inf 时 while 循环不会终止，调用方必须保证输入有限。
double limit_rad(double angle);

// 四元数转欧拉角
// x = 0, y = 1, z = 2
// e.g. 先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转：axis0=2, axis1=1, axis2=0
// 参考：https://github.com/evbernardes/quaternion_to_euler
// 作用：按指定旋转轴顺序，把四元数转换为三轴欧拉角（内部按模长归一化，输入无需单位化）。
// 参数：q —— 旋转四元数；Eigen 的 coeffs() 存储顺序为 (x, y, z, w)，w 为实部。
//       axis0/axis1/axis2 —— 三个旋转轴编号，0=x、1=y、2=z，按“先绕 axis0 转、
//                            再绕 axis1 转、最后绕 axis2 转”的顺序给出（如 zyx 传 2,1,0）。
//       extrinsic —— false（默认）表示内旋/随体轴（intrinsic）；true 表示外旋/固定轴（extrinsic）。
// 返回：Eigen::Vector3d，三个分量依次是绕 axis0、axis1、axis2 的转角，单位 rad，
//       已逐个归一化到 (-pi, pi]。
// 边界：零四元数（模长为 0）会使内部 n2 = 0，结果退化为 NaN，调用方须保证 q 非零；
//       万向锁（中间角接近 0 或 pi，判据 eps = 1e-7）时欧拉角不唯一，函数会把
//       其中一个自由度置 0 并按约定给出剩余角度，不保证与其它库的取法一致。
Eigen::Vector3d eulers(
  Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic = false);

// 旋转矩阵转欧拉角
// x = 0, y = 1, z = 2
// e.g. 先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转：axis0=2, axis1=1, axis2=0
// 作用：按指定旋转轴顺序，把 3x3 旋转矩阵转换为三轴欧拉角。
// 参数：R —— 3x3 旋转矩阵（应先转为四元数，因此非正交矩阵会被隐式“投影”为最近似的旋转）。
//       axis0/axis1/axis2、extrinsic —— 含义同上面的四元数重载。
// 返回：Eigen::Vector3d，三个分量依次是绕 axis0、axis1、axis2 的转角，单位 rad，范围 (-pi, pi]。
// 边界：与四元数重载相同；另外 R 为零矩阵时四元数模长为 0，结果为 NaN。
Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic = false);

// 欧拉角转旋转矩阵
// zyx:先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转
// 作用：欧拉角（yaw-pitch-roll）转 3x3 旋转矩阵，实现 R = Rz(yaw) * Ry(pitch) * Rx(roll)。
// 参数：ypr —— 长度 3 的向量，按 ypr[0]=yaw（绕 z）、ypr[1]=pitch（绕 y）、ypr[2]=roll（绕 x）
//              的约定存放三轴转角，单位 rad，不要求落在 (-pi, pi] 内（三角函数本身周期）。
// 返回：Eigen::Matrix3d，正交旋转矩阵（数值上近似正交，未做正交化）。
// 边界：无奇异点；任意实数输入都返回有限值。
Eigen::Matrix3d rotation_matrix(const Eigen::Vector3d & ypr);

// 直角坐标系转球坐标系
// ypd为yaw、pitch、distance的缩写
// 作用：把直角坐标 (x, y, z) 转成球坐标 (yaw, pitch, distance)。
// 参数：xyz —— 直角坐标向量，单位任意（与 distance 输出同量纲），通常为 m 或 mm。
// 返回：Eigen::Vector3d(yaw, pitch, distance)：
//       yaw = atan2(y, x)，范围 (-pi, pi]，单位 rad；
//       pitch = atan2(z, sqrt(x²+y²))，范围 (-pi/2, pi/2)，单位 rad；
//       distance = |xyz|，非负，单位与输入一致。
// 边界：xyz 为零向量时 yaw = pitch = 0、distance = 0（atan2(0,0)=0，不报错）；
//       x=y=0 且 z≠0 时 yaw = 0（天顶/天底方向 yaw 退化，无实际意义）。
Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz);

// 直角坐标系转球坐标系转换函数对xyz的雅可比矩阵
// 作用：给出 xyz2ypd 的解析雅可比矩阵 J = d(yaw, pitch, distance) / d(x, y, z)。
// 参数：xyz —— 直角坐标向量，单位任意（与输入量纲一致，雅可比各行量纲不同：rad/长度、
//              rad/长度、无量纲）。
// 返回：3x3 动态矩阵 Eigen::MatrixXd：
//       第 0 行 d yaw / d(x,y,z)，第 1 行 d pitch / d(x,y,z)，第 2 行 d distance / d(x,y,z)。
// 边界：x = y = 0（正上/正下方，即球坐标奇异点）时分母 x²+y² = 0，矩阵元素为 ±inf 或 NaN；
//       xyz 为零向量时同理。调用方必须保证输入不落在 z 轴上。
Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d & xyz);

// 球坐标系转直角坐标系
// 作用：把球坐标 (yaw, pitch, distance) 转回直角坐标 (x, y, z)。
// 参数：ypd —— yaw、pitch 单位 rad，distance 单位与输出一致（非负；若传入负值，
//              结果相当于把坐标取反，函数本身不做校验）。
// 返回：Eigen::Vector3d(x, y, z)，与 xyz2ypd 互逆。
// 边界：无奇异点，任意有限输入都返回有限值。
Eigen::Vector3d ypd2xyz(const Eigen::Vector3d & ypd);

// 球坐标系转直角坐标系转换函数对xyz的雅可比矩阵
//   注意：函数名沿用源工程（实际是 ypd2xyz 对 (yaw, pitch, distance) 的雅可比，
//   即 d(x, y, z) / d(yaw, pitch, distance)）。
// 作用：给出 ypd2xyz 的解析雅可比矩阵。
// 参数：ypd —— yaw、pitch 单位 rad；distance 单位与输出一致。
// 返回：3x3 动态矩阵 Eigen::MatrixXd：
//       第 0 行 d x / d(yaw, pitch, distance)，第 1 行 d y / ...，第 2 行 d z / ...。
//       第 0、1 列量纲为 长度，第 2 列无量纲。
// 边界：无奇异点；任意有限输入都返回有限值。
Eigen::MatrixXd ypd2xyz_jacobian(const Eigen::Vector3d & ypd);

// 计算时间差a - b，单位：s
// 作用：求两个 steady_clock 时间点之差，得到秒为单位的 double。
// 参数：a、b —— std::chrono::steady_clock::time_point（单调时钟，不受系统时间调整影响）。
// 返回：a - b，单位 s，可为负（当 a 早于 b 时）。
// 边界：steady_clock 计时精度通常为 ns 级但分辨率有限，同一时刻两次采样可能为 0；
//       返回值不建议直接用于判零，应使用 epsilon 比较。
double delta_time(
  const std::chrono::steady_clock::time_point & a, const std::chrono::steady_clock::time_point & b);

// 向量夹角 总是返回 0 ~ pi 来自SJTU
// 作用：计算两个二维向量的夹角（无方向，取绝对值）。
// 参数：vec1、vec2 —— 任意二维向量，量纲任意（只在比值中出现，故与单位无关）。
// 返回：夹角，单位 rad，范围 [0, pi]。
// 边界：任一向量为零向量（norm() == 0）时直接返回 0（不做未定义的角度计算）；
//       由于浮点误差，cos 值可能略微超出 [-1, 1] 导致 acos 返回 NaN——
//       输入为真实浮点数据时存在此风险，调用方需自行保证数据质量。
double get_abs_angle(const Eigen::Vector2d & vec1, const Eigen::Vector2d & vec2);

// 返回输入值的平方
// 作用：通用平方函数模板。
// 参数：a —— 任意支持 operator* 且可拷贝的类型（数值、Eigen 矩阵等）。
// 返回：a * a，类型与入参相同（注意整数类型可能溢出）。
// 边界：无；模板为 header-only，编译器可内联。
template <typename T>
T square(T const & a)
{
  return a * a;
};

// 作用：把输入裁剪到 [min, max] 区间内（饱和限幅）。
// 参数：input —— 待裁剪的值；min、max —— 下界与上界（不做 min <= max 的检查）。
// 返回：input 落在 [min, max] 时原样返回；大于 max 返回 max；小于 min 返回 min。
// 边界：若调用方传入 min > max，则当 input > max 时返回 max、input < min 时返回 min，
//       即结果不再唯一依赖输入；min == max 时恒返回该值。NaN 输入原样返回 NaN。
double limit_min_max(double input, double min, double max);
}  // namespace tools

#endif  // TOOLS__MATH_TOOLS_HPP