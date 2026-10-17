# tools/recorder.cpp

**概要**：`tools/recorder.hpp` 中 `Recorder` 的实现——构造时准备输出路径、首帧惰性初始化并启动保存线程，`record()` 按 fps 节流投递帧，`save_to_file()` 在后台线程把图像写入 AVI、把时间戳与四元数写入文本。

> **迁移说明**：由 sp_vision_25 的 `tools/recorder.cpp` **原样迁移**（逻辑、签名、include 均未改动），仅补充中文注释。

## 函数

### `Recorder::Recorder(double fps)`

- **作用**：构造录像器——生成输出路径、创建 `records/` 目录、初始化时间基准。
- **参数**：`fps` 录像帧率（默认值在头文件声明为 30），用于节流与 `cv::VideoWriter` 帧率。
- **返回**：无。
- **备注**：初始化列表 `init_(false)`, `fps_(fps)`, `queue_(1)`, `stop_thread_(false)`；`start_time_` 与 `last_time_` 均置为 `steady_clock::now()`。文件名用 `fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now())` 生成，文本与视频共用同一时间戳前缀（`.txt` / `.avi`）。只建目录、不开文件。

### `Recorder::~Recorder()`

- **作用**：停止保存线程并关闭输出文件。
- **参数**：无。
- **返回**：无。
- **备注**：`stop_thread_ = true` → `queue_.push({cv::Mat::zeros(0,0,0), {0,0,0,0}, now})`（毒丸，避免保存线程永久阻塞在 `pop()`）→ `join` → 若 `!init_` 直接返回，否则 `text_writer_.close()` 与 `video_writer_.release()`。原注释保留：`// 退出时给队列中额外推入一个空帧，避免pop一直等待`、`// 等待视频保存线程结束`。

### `void Recorder::save_to_file()`

- **作用**：保存线程主循环——取帧、写视频、写文本。
- **参数**：无。
- **返回**：无。
- **备注**：`while (!stop_thread_)` 循环；`queue_.pop(frame)` 阻塞取帧；`frame.img.empty()` 时打 debug 日志（`"Recorder received empty img. Skip this frame."`）并 `continue`；视频用 `video_writer_.write(frame.img)`；文本用 `frame.q.coeffs()` 得到 xyzw 后按 `xyzw[3] xyzw[0] xyzw[1] xyzw[2]`（即 **wxyz**）输出，时间用 `tools::delta_time(frame.timestamp, start_time_)` 换算为相对起始时间。

### `void Recorder::record(const cv::Mat & img, const Eigen::Quaterniond & q, const std::chrono::steady_clock::time_point & timestamp)`

- **作用**：提交一帧待记录数据（fps 节流 + 非阻塞入队）。
- **参数**：`img` 帧图像（空则忽略）；`q` 云台姿态四元数；`timestamp` 帧采集时刻。
- **返回**：无。
- **备注**：`img.empty()` 直接返回；`!init_` 时先 `init(img)`；随后 `since_last < 1.0 / fps_` 则返回（降采样丢弃）；通过后更新 `last_time_ = timestamp` 并 `queue_.push({img, q, timestamp})`。非线程安全，不应被多线程并发调用。

### `void Recorder::init(const cv::Mat & img)`

- **作用**：惰性初始化——打开文本文件、创建视频写手、启动保存线程。
- **参数**：`img` 首帧图像，提供视频分辨率。
- **返回**：无。
- **备注**：`text_writer_.open(text_path_)`；四字符码 `cv::VideoWriter::fourcc('M','J','P','G')`；`cv::VideoWriter(video_path_, fourcc, fps_, img.size())`；`saving_thread_ = std::thread(&Recorder::save_to_file, this)`（原注释：`// 启动保存线程`）；最后 `init_ = true`。

## 依赖

- 标准库：`<filesystem>` `<string>`（`<chrono>` 经头文件引入）
- 第三方：fmt（`<fmt/chrono.h>`，用于格式化文本行与文件名；spdlog 已依赖 fmt）、Eigen3、OpenCV4
- 项目内：`"recorder.hpp"`、`"math_tools.hpp"`（`tools::delta_time`）、`"tools/logger.hpp"`
