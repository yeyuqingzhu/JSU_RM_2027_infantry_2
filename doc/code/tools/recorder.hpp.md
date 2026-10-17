# tools/recorder.hpp

**概要**：声明异步录像器 `tools::Recorder`——把图像帧与云台姿态四元数分别落盘为 MJPG 视频和带相对时间戳的文本，写盘由后台线程完成，主线程只向容量为 1 的线程安全队列投递数据。

> **迁移说明**：由 sp_vision_25 的 `tools/recorder.hpp` **原样迁移**，接口、成员、逻辑均未改动，仅补充中文注释，并补上原文件缺失的 `#include <atomic>`（`std::atomic<bool> stop_thread_` 原本依赖间接包含）。

## 类

### `class Recorder`

- **职责**：记录一段视觉数据用于事后复盘。视频为 `.avi`（MJPG，帧率由构造参数 `fps` 指定）；文本每行 `相对起始时间 w x y z`（四元数按 **wxyz** 顺序输出）。输出目录固定为工作目录下的 `records/`，文件名取构造时的系统时间 `%Y-%m-%d_%H-%M-%S`。
- **线程安全语义**：仅 `queue_`（自带互斥）与 `stop_thread_`（原子）可跨线程访问；`record()` / `init()` 需由**单一线程**调用（`last_time_`、`init_` 等普通成员无保护）；保存线程只访问 `queue_`、`video_writer_`、`text_writer_`、`start_time_`、`stop_thread_`。队列容量为 1 且默认满队列策略为"丢弃新帧"，故磁盘慢时丢帧而不会阻塞主线程。

## 函数

### `Recorder(double fps = 30)`

- **作用**：构造录像器，生成输出路径并确保 `records/` 目录存在。
- **参数**：`fps` 录像帧率（默认 30），既用于采样节流（最小间隔 `1/fps`），也作为 `cv::VideoWriter` 的帧率。
- **返回**：无。
- **备注**：构造时只建目录、不开文件；真正的文件打开与保存线程创建延迟到首帧（`init()`）时进行。初始化列表：`init_(false)`、`fps_(fps)`、`queue_(1)`、`stop_thread_(false)`；`start_time_` 与 `last_time_` 均置为当前时刻。

### `~Recorder()`

- **作用**：通知保存线程退出、等待其 `join`，并关闭文本流与视频写手。
- **参数**：无。
- **返回**：无。
- **备注**：顺序为 `stop_thread_ = true` → 向队列推入一个空帧（毒丸，唤醒阻塞在 `pop()` 的保存线程）→ `join`（若 `saving_thread_.joinable()`）→ 若 `init_ == false` 直接返回，否则 `text_writer_.close()`、`video_writer_.release()`。因停止标志先于毒丸置位，队列中未写盘的剩余帧会被丢弃（原实现行为）。

### `void record(const cv::Mat & img, const Eigen::Quaterniond & q, const std::chrono::steady_clock::time_point & timestamp)`

- **作用**：提交一帧待记录数据（按 `fps_` 节流，非阻塞）。
- **参数**：`img` 当前帧图像，为空则直接忽略；`q` 该帧云台姿态四元数；`timestamp` 该帧采集时刻（`steady_clock`），用于节流判断与文本相对时间。
- **返回**：无。
- **备注**：首帧触发 `init(img)` 打开文件并启动保存线程；随后当 `delta_time(timestamp, last_time_) < 1.0 / fps_` 时直接返回（丢弃），否则更新 `last_time_` 并入队 `{img, q, timestamp}`。非线程安全，仅应由视觉主线程调用。

### `void init(const cv::Mat & img)`（私有）

- **作用**：首次记录时的惰性初始化——打开文本文件、创建 `cv::VideoWriter`、启动保存线程。
- **参数**：`img` 首帧图像，用于确定视频分辨率 `img.size()`。
- **返回**：无。
- **备注**：编码固定为 MJPG（`fourcc('M','J','P','G')`）；保存线程入口 `&Recorder::save_to_file`；最后置 `init_ = true`。只应在 `init_ == false` 时调用。

### `void save_to_file()`（私有）

- **作用**：保存线程主循环——从 `queue_` 取帧，写视频并写文本。
- **参数**：无。
- **返回**：无。
- **备注**：`stop_thread_` 为 `true` 时退出；空图像帧仅打 debug 日志后跳过；文本行格式 `"{相对时间} {w} {x} {y} {z}\n"`（由 Eigen 的 xyzw 排列重排为 wxyz）。只在 `saving_thread_` 中运行，独占写 `video_writer_` / `text_writer_`。

## 成员变量

| 成员 | 含义 |
| --- | --- |
| `struct FrameData { cv::Mat img; Eigen::Quaterniond q; std::chrono::steady_clock::time_point timestamp; }` | 队列中传递的单帧数据；空图像作为唤醒退出的毒丸 |
| `bool init_` | 是否已完成初始化（文件已打开、保存线程已启动） |
| `std::atomic<bool> stop_thread_` | 保存线程停止标志 |
| `double fps_` | 录像帧率，用于节流与 VideoWriter |
| `std::string text_path_` | 文本输出路径 `records/<时间>.txt` |
| `std::string video_path_` | 视频输出路径 `records/<时间>.avi` |
| `std::ofstream text_writer_` | 文本文件流，保存线程独占写入 |
| `cv::VideoWriter video_writer_` | 视频写手，保存线程独占写入 |
| `std::chrono::steady_clock::time_point start_time_` | 记录起始时刻，用于计算相对时间 |
| `std::chrono::steady_clock::time_point last_time_` | 上一帧入队时刻，用于 fps 节流 |
| `tools::ThreadSafeQueue<FrameData> queue_` | 帧数据队列（容量 1），生产者与保存线程的通道 |
| `std::thread saving_thread_` | 负责保存帧数据的线程（原注释：`// 负责保存帧数据的线程`） |

## 依赖

- 标准库：`<atomic>` `<chrono>` `<fstream>` `<string>` `<thread>`
- 第三方：Eigen3（`Eigen/Geometry`）、OpenCV4（`opencv2/opencv.hpp`）
- 项目内：`"tools/thread_safe_queue.hpp"`
