#ifndef TOOLS__IMG_TOOLS_HPP
#define TOOLS__IMG_TOOLS_HPP

// =============================================================================
// 文件：tools/img_tools.hpp
// -----------------------------------------------------------------------------
// 【本文件作用】
//   图像绘制工具集（对外接口声明）。提供在 cv::Mat 上就地绘制调试图元的轻量
//   函数：画单个点、画一组点（折线/轮廓）、写文字。典型用途是把自瞄算法识别出
//   的装甲板角点、预测点、跟踪状态等叠加显示在相机画面上，便于现场调试与回放。
//
// 【来源】
//   sp_vision_25 工程的 tools/img_tools.hpp（逐文件迁移，接口与绘制行为一致）。
//
// 【依赖】
//   - OpenCV4：cv::Mat / cv::Point / cv::Point2f / cv::Scalar / cv::circle /
//     cv::drawContours / cv::putText（头文件目录 /usr/local/include/opencv4）
//   - C++ 标准库：<string>、<vector>
//
// 【注意事项】
//   1. 所有绘制函数都是就地修改传入的图像 img，返回类型均为 void，不产生新图；
//   2. img 未做空图检查：img 为空（img.empty()）时 OpenCV 的绘制函数是空操作，
//      不会崩溃，但也不会画出任何内容；
//   3. 坐标允许越界：OpenCV 会把图元自动裁剪到图像范围内，越界部分静默丢弃，
//      不会抛异常；
//   4. 颜色一律遵循 OpenCV 约定使用 BGR 顺序（默认 {0, 0, 255} 即红色），
//      不是 RGB；
//   5. 本文件不提供线程安全保证：多线程同时对同一张 img 绘制会产生数据竞争，
//      调用方需自行加锁，或保证同一张图只由单个线程写入。
// =============================================================================

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace tools
{
/**
 * 作用：在图像上绘制一个实心圆点，用于标记角点、中心点等关键位置。
 * 参数：
 *   - img：待绘制的图像，就地修改；传入空图时为空操作。
 *   - point：圆心坐标，单位像素，原点在图像左上角，x 向右、y 向下；允许越界。
 *   - color：BGR 颜色，默认 {0, 0, 255}（红色）。
 *   - radius：圆半径，单位像素，默认 3；内部以 thickness = -1 填充为实心圆。
 * 返回：无。
 * 边界条件：point 越界时只画出落在图像内的部分；radius <= 0 时不可见；
 *           img 为空时不做任何绘制。
 */
void draw_point(
  cv::Mat & img, const cv::Point & point, const cv::Scalar & color = {0, 0, 255}, int radius = 3);

/**
 * 作用：把一组整型点当作一条轮廓/折线绘制在图像上（画装甲板四点、轨迹等）。
 * 参数：
 *   - img：待绘制的图像，就地修改。
 *   - points：点的集合，单位像素；空集合时为空操作。
 *   - color：BGR 颜色，默认 {0, 0, 255}（红色）。
 *   - thickness：线条宽度，单位像素，默认 2。
 * 返回：无。
 * 边界条件：points 为空时 cv::drawContours 为空操作；点越界时自动裁剪；
 *           实现上借用 cv::drawContours，点列被当作“闭合轮廓”处理，
 *           即最后一个点会与第一个点相连（画出来是闭合多边形）。
 */
void draw_points(
  cv::Mat & img, const std::vector<cv::Point> & points, const cv::Scalar & color = {0, 0, 255},
  int thickness = 2);

/**
 * 作用：draw_points 的浮点坐标重载，内部先转成整型点再复用整型版本。
 * 参数：
 *   - img：待绘制的图像，就地修改。
 *   - points：浮点点集，单位像素。转换由 cv::Point2f 到 cv::Point 的隐式转换
 *             完成，内部走 saturate_cast<int>：四舍五入（cvRound，向偶数舍入），
 *             并非截断取整。
 *   - color：BGR 颜色，默认 {0, 0, 255}（红色）。
 *   - thickness：线条宽度，单位像素，默认 2。
 * 返回：无。
 * 边界条件：与整型版本一致；浮点值超出 int 可表示范围时会被饱和到
 *           INT_MIN/INT_MAX（saturate_cast 行为），该点通常落到图外。
 */
void draw_points(
  cv::Mat & img, const std::vector<cv::Point2f> & points, const cv::Scalar & color = {0, 0, 255},
  int thickness = 2);

/**
 * 作用：在图像上写一行文字，用于显示调试状态与数值。
 * 参数：
 *   - img：待绘制的图像，就地修改。
 *   - text：要绘制的字符串。字体为 Hershey 系列，只含 ASCII/拉丁字形，
 *           写中文会显示为乱码或问号。
 *   - point：文字左下角的基准点坐标，单位像素（注意是左下角，不是左上角）。
 *   - color：BGR 颜色，默认 {0, 255, 255}（黄色）。
 *   - font_scale：字体缩放系数，默认 1.0，无量纲；越大字越大。
 *   - thickness：笔画线宽，单位像素，默认 2。
 * 返回：无。
 * 边界条件：文字超出右/下边界时被裁剪；point 为负值时文字可能整体不可见。
 */
void draw_text(
  cv::Mat & img, const std::string & text, const cv::Point & point,
  const cv::Scalar & color = {0, 255, 255}, double font_scale = 1.0, int thickness = 2);

}  // namespace tools

#endif  // TOOLS__IMG_TOOLS_HPP
