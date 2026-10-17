// =============================================================================
// 文件：tools/img_tools.cpp
// -----------------------------------------------------------------------------
// 【本文件作用】
//   tools/img_tools.hpp 中图像绘制接口的实现。全部实现都是对 OpenCV 绘图 API
//   的一层薄封装，不引入额外状态，也没有全局变量。
//
// 【来源】
//   sp_vision_25 工程的 tools/img_tools.cpp（逐文件迁移，行为保持一致）。
//
// 【依赖】
//   仅依赖同目录的 img_tools.hpp（间接依赖 OpenCV4 与 C++ 标准库）。
//
// 【注意事项】
//   1. 所有函数就地修改传入的图像，不做空图/越界检查，依赖 OpenCV 自身的裁剪
//      行为；空图为空操作，不会崩溃。
//   2. 无任何内部同步：同一张图被多线程同时绘制会有数据竞争，调用方负责加锁。
//   3. 函数不抛异常（OpenCV 在参数非法时一般也只是打印错误或静默裁剪）；
//      真正的失败模式只有“画不出来”，不会中断上层流程。
// =============================================================================

#include "img_tools.hpp"

namespace tools
{
void draw_point(cv::Mat & img, const cv::Point & point, const cv::Scalar & color, int radius)
{
  // thickness = -1 表示填充实心圆；radius 为像素半径。
  // 圆心越界时 OpenCV 会自动裁剪，只画出落在图像内的部分。
  cv::circle(img, point, radius, color, -1);
}

void draw_points(
  cv::Mat & img, const std::vector<cv::Point> & points, const cv::Scalar & color, int thickness)
{
  // 把点列包成“单条轮廓”的容器，再交给 drawContours 绘制。
  // 注意：drawContours 会把轮廓视为闭合曲线，因此末点会与首点相连。
  std::vector<std::vector<cv::Point>> contours = {points};
  cv::drawContours(img, contours, -1, color, thickness);
}

void draw_points(
  cv::Mat & img, const std::vector<cv::Point2f> & points, const cv::Scalar & color, int thickness)
{
  // 浮点点列先转换为整型点列：转换由 cv::Point2f -> cv::Point 的隐式转换完成，
  // 内部为 saturate_cast<int>（四舍五入，向偶数舍入），不是简单截断。
  std::vector<cv::Point> int_points(points.begin(), points.end());
  // 复用整型重载，保证两种重载的绘制行为（含闭合轮廓语义）完全一致。
  draw_points(img, int_points, color, thickness);
}

void draw_text(
  cv::Mat & img, const std::string & text, const cv::Point & point, const cv::Scalar & color,
  double font_scale, int thickness)
{
  // 使用 Hershey 单线字体；point 是文字左下角基准点，不支持中文等非拉丁字形。
  cv::putText(img, text, point, cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
}

}  // namespace tools
