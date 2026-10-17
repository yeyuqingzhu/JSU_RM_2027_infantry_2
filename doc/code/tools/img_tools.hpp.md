# tools/img_tools.hpp

**概要**：图像绘制工具集的对外接口声明，提供在 `cv::Mat` 上就地画点、画点列（闭合轮廓）与写文字四个函数，供自瞄/打符等算法做画面可视化调试。

> 来源：sp_vision_25 `tools/img_tools.hpp`（逐文件迁移，接口与绘制行为保持一致）。
> 依赖：OpenCV4（`/usr/local/include/opencv4`）、C++ 标准库 `<string>`、`<vector>`。
> 注意事项：所有函数都**就地修改**传入图像且不返回新图；不做空图与越界检查（依赖 OpenCV 自动裁剪，空图为空操作）；颜色为 BGR 顺序；**非线程安全**，同一张图并发绘制需调用方加锁。

## 函数

### `void draw_point(cv::Mat & img, const cv::Point & point, const cv::Scalar & color = {0, 0, 255}, int radius = 3)`

- **作用**：在图像上画一个实心圆点，用于标记角点、中心点等关键位置。
- **参数**：
  - `img`：待绘制图像，就地修改。
  - `point`：圆心坐标，单位像素，原点在图像左上角；允许越界。
  - `color`：BGR 颜色，默认 `{0, 0, 255}`（红色）。
  - `radius`：圆半径，单位像素，默认 3；内部以 `thickness = -1` 画实心圆。
- **返回**：无。
- **备注**：`point` 越界时只画出图像内部分；`radius <= 0` 时不可见；`img` 为空时为空操作，不抛异常。

### `void draw_points(cv::Mat & img, const std::vector<cv::Point> & points, const cv::Scalar & color = {0, 0, 255}, int thickness = 2)`

- **作用**：把一组整型点当作一条轮廓/折线画在图像上（装甲板四点、轨迹等）。
- **参数**：
  - `img`：待绘制图像，就地修改。
  - `points`：点集，单位像素；空集合时为空操作。
  - `color`：BGR 颜色，默认 `{0, 0, 255}`（红色）。
  - `thickness`：线宽，单位像素，默认 2。
- **返回**：无。
- **备注**：实现上借用 `cv::drawContours`，因此点列按**闭合轮廓**处理——最后一个点会与第一个点相连；点的顺序即连线顺序。

### `void draw_points(cv::Mat & img, const std::vector<cv::Point2f> & points, const cv::Scalar & color = {0, 0, 255}, int thickness = 2)`

- **作用**：`draw_points` 的浮点坐标重载，内部转成整型点集后复用整型版本。
- **参数**：
  - `img`：待绘制图像，就地修改。
  - `points`：浮点点集，单位像素；空集合时为空操作。
  - `color`：BGR 颜色，默认 `{0, 0, 255}`（红色）。
  - `thickness`：线宽，单位像素，默认 2。
- **返回**：无。
- **备注**：`cv::Point2f → cv::Point` 的隐式转换走 `saturate_cast<int>`，即**四舍五入（向偶数舍入）**而非截断；数值超出 `int` 范围时饱和到 `INT_MIN/INT_MAX`（该点通常落到图外）。与整型重载构成重载对，传 `{...}` 花括号列表调用时可能产生二义性，需显式指定 `std::vector<cv::Point>` 或 `std::vector<cv::Point2f>`。

### `void draw_text(cv::Mat & img, const std::string & text, const cv::Point & point, const cv::Scalar & color = {0, 255, 255}, double font_scale = 1.0, int thickness = 2)`

- **作用**：在图像上写一行文字，用于显示调试状态与数值。
- **参数**：
  - `img`：待绘制图像，就地修改。
  - `text`：要绘制的字符串；Hershey 字体不含中文字形，写中文会显示为乱码/问号。
  - `point`：文字**左下角**基准点坐标，单位像素，无默认值（必填）。
  - `color`：BGR 颜色，默认 `{0, 255, 255}`（黄色）。
  - `font_scale`：字体缩放系数，默认 1.0，无量纲。
  - `thickness`：笔画线宽，单位像素，默认 2。
- **返回**：无。
- **备注**：文字超出右/下边界时被裁剪；`point` 为负时文字可能整体不可见；不支持的字符不会抛异常。
