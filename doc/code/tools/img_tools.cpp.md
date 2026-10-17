# tools/img_tools.cpp

**概要**：`tools/img_tools.hpp` 中四个图像绘制接口的实现——对 OpenCV 绘图 API（`cv::circle` / `cv::drawContours` / `cv::putText`）的薄封装，无状态、无全局变量、不引入新依赖。

> 来源：sp_vision_25 `tools/img_tools.cpp`（逐文件迁移，行为保持一致）。
> 依赖：同目录 `img_tools.hpp`（间接依赖 OpenCV4 与标准库）。
> 注意事项：所有函数**就地修改**传入图像；不做空图/越界检查，依赖 OpenCV 自动裁剪（空图为空操作、不崩溃）；无内部同步，同一张图并发绘制需调用方加锁；函数不抛异常，失败模式只有“画不出来”，不会中断上层流程。

## 函数

### `void draw_point(cv::Mat & img, const cv::Point & point, const cv::Scalar & color, int radius)`

- **作用**：在图像上画实心圆点。
- **参数**：`img` 待绘制图像（就地修改）；`point` 圆心像素坐标；`color` BGR 颜色；`radius` 像素半径。
- **返回**：无。
- **备注**（实现要点与关键步骤）：
  1. 直接调用 `cv::circle(img, point, radius, color, -1)`；
  2. `thickness = -1` 是 OpenCV 约定的“填充实心”标记；
  3. 圆心越界由 OpenCV 内部裁剪，只画可见部分。

### `void draw_points(cv::Mat & img, const std::vector<cv::Point> & points, const cv::Scalar & color, int thickness)`

- **作用**：把整型点列当作一条轮廓绘制。
- **参数**：`img` 待绘制图像；`points` 像素点集；`color` BGR 颜色；`thickness` 线宽。
- **返回**：无。
- **备注**（实现要点与关键步骤）：
  1. 把点列包成“单条轮廓”的容器 `std::vector<std::vector<cv::Point>> contours = {points}`；
  2. 调用 `cv::drawContours(img, contours, -1, color, thickness)`，`contourIdx = -1` 表示绘制全部轮廓（此处仅一条）；
  3. **关键语义**：`drawContours` 把轮廓视为闭合曲线，末点会与首点相连，最终画出的是闭合多边形；
  4. `points` 为空时 `contours` 为含一条空轮廓的容器，`drawContours` 不画任何内容，不抛异常。

### `void draw_points(cv::Mat & img, const std::vector<cv::Point2f> & points, const cv::Scalar & color, int thickness)`

- **作用**：`draw_points` 的浮点重载。
- **参数**：`img` 待绘制图像；`points` 浮点像素点集；`color` BGR 颜色；`thickness` 线宽。
- **返回**：无。
- **备注**（实现要点与关键步骤）：
  1. 用迭代器区间构造 `std::vector<cv::Point> int_points(points.begin(), points.end())` 完成坐标类型转换；
  2. 该转换由 `cv::Point2f → cv::Point` 的隐式转换运算符完成，内部为 `saturate_cast<int>`：**四舍五入（向偶数舍入）**，不是截断；超范围数值饱和到 `INT_MIN/INT_MAX`；
  3. 随后调用整型重载 `draw_points(img, int_points, color, thickness)`，保证两个重载的绘制行为（含闭合轮廓语义）完全一致，避免两处实现漂移。

### `void draw_text(cv::Mat & img, const std::string & text, const cv::Point & point, const cv::Scalar & color, double font_scale, int thickness)`

- **作用**：在图像上写一行文字。
- **参数**：`img` 待绘制图像；`text` 文本；`point` 文字**左下角**基准点像素坐标；`color` BGR 颜色；`font_scale` 缩放系数；`thickness` 线宽。
- **返回**：无。
- **备注**（实现要点与关键步骤）：
  1. 调用 `cv::putText(img, text, point, cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)`；
  2. 字体固定为 Hershey 单线字体 `FONT_HERSHEY_SIMPLEX`，只含拉丁字形，中文会显示为乱码；
  3. `point` 是文字左下角（`putText` 约定），不是左上角；文字超出右下边界会被裁剪。
