/**
 * @file recorder.cpp
 * @brief Recorder 的实现：异步把帧图像写成 AVI、把时间戳与四元数写成文本。
 *
 * 【作用】
 *   实现 tools/recorder.hpp 中 Recorder 的构造 / 析构、record() 采集入口、
 *   init() 初始化与 save_to_file() 保存线程主循环。
 *
 * 【来源】
 *   sp_vision_25 项目 tools/recorder.cpp（原样迁移，逻辑未作任何修改）。
 *
 * 【依赖】
 *   - 标准库：<filesystem> <string> <chrono>；
 *   - 第三方：fmt（<fmt/chrono.h>，用于格式化时间与浮点；spdlog 已依赖 fmt），
 *             Eigen3、OpenCV4（经 recorder.hpp 引入）；
 *   - 项目内："recorder.hpp"、"math_tools.hpp"（tools::delta_time）、"tools/logger.hpp"。
 *
 * 【线程模型与注意事项】
 *   - 构造 / record() / init() 运行在调用方线程；save_to_file() 运行在 saving_thread_。
 *   - 两个线程之间只通过 tools::ThreadSafeQueue<FrameData> queue_ 传递数据。
 *   - 析构在调用方线程执行：先置停止标志并推入空帧唤醒保存线程，再 join，最后关闭文件。
 *   - queue_ 容量为 1 且采用默认满队列策略（丢弃新帧 + 空回调），磁盘慢时表现为丢帧而非阻塞。
 */
#include "recorder.hpp"

#include <fmt/chrono.h>

#include <filesystem>
#include <string>

#include "math_tools.hpp"
#include "tools/logger.hpp"

namespace tools
{
/**
 * @brief 构造录像器：生成输出路径（records/<系统时间>.txt|.avi）并创建 records 目录。
 * @param fps 录像帧率，默认 30。
 * @note 初始 init_ = false、stop_thread_ = false；start_time_ 与 last_time_ 均置为当前时刻；
 *       队列容量固定为 1（超过 1 帧未消费即丢弃新帧）。
 * @note 初始化列表顺序与 recorder.hpp 中的成员声明顺序保持一致（init_ → stop_thread_ →
 *       fps_ → queue_），以消除 -Wreorder 告警；各成员互不依赖，初始化顺序不影响行为。
 * @note 只创建目录，不打开文件；文件名取自系统时间，格式 %Y-%m-%d_%H-%M-%S。
 */
Recorder::Recorder(double fps) : init_(false), stop_thread_(false), fps_(fps), queue_(1)
{
  start_time_ = std::chrono::steady_clock::now();
  last_time_ = start_time_;

  auto folder_path = "records";
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  text_path_ = fmt::format("{}/{}.txt", folder_path, file_name);
  video_path_ = fmt::format("{}/{}.avi", folder_path, file_name);

  std::filesystem::create_directory(folder_path);
}

/**
 * @brief 析构录像器：停线程、等 join、关文件。
 * @note 顺序为：置 stop_thread_ = true → 推入空帧（毒丸，唤醒阻塞在 pop 的保存线程）
 *       → join 保存线程 → 若已初始化则关闭文本流与视频写手。
 *       若从未记录过任何帧（init_ == false），文件从未打开，直接返回。
 * @warning 由于 stop_thread_ 在推入空帧前就已被置为 true，保存线程的 while 循环
 *          会立刻退出，队列中尚未写盘的剩余帧会被丢弃（这是原实现行为，迁移时保持一致）。
 */
Recorder::~Recorder()
{
  stop_thread_ = true;
  // 退出时给队列中额外推入一个空帧，避免pop一直等待
  queue_.push({cv::Mat::zeros(0, 0, 0), {0, 0, 0, 0}, std::chrono::steady_clock::now()});
  if (saving_thread_.joinable()) saving_thread_.join();  // 等待视频保存线程结束

  if (!init_) return;
  text_writer_.close();
  video_writer_.release();
}

/**
 * @brief 保存线程主循环：从队列取帧 → 写视频 → 写文本（时间戳 + wxyz 四元数）。
 * @note stop_thread_ 为 true 时退出循环；空图像帧仅打印 debug 日志后跳过。
 *       文本每行格式为 "相对起始时间 w x y z\n"（四元数按 wxyz 顺序，
 *       即由 Eigen 的 xyzw 排列重排后输出），时间单位与 delta_time 一致（秒）。
 * @note 本函数独占写 video_writer_、text_writer_，只在 saving_thread_ 中运行。
 */
void Recorder::save_to_file()
{
  while (!stop_thread_) {
    FrameData frame;
    queue_.pop(frame);  // 从队列中取出帧数据
    if (frame.img.empty()) {
      tools::logger()->debug("Recorder received empty img. Skip this frame.");
      continue;
    }
    // 写入视频文件
    video_writer_.write(frame.img);

    // 写入文本文件（输出顺序为wxyz）
    Eigen::Vector4d xyzw = frame.q.coeffs();
    auto since_begin = tools::delta_time(frame.timestamp, start_time_);
    text_writer_ << fmt::format(
      "{} {} {} {} {}\n", since_begin, xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  }
}

/**
 * @brief 提交一帧待记录数据（fps 节流 + 非阻塞入队）。
 * @param img       当前帧图像；img.empty() 时直接返回，不记录。
 * @param q         该帧对应的云台姿态四元数。
 * @param timestamp 该帧采集时刻（steady_clock）。
 * @return 无。
 * @note 若尚未初始化则先调用 init(img)（打开文件、启动保存线程）；
 *       随后判断与上一记录帧的时间差，小于 1/fps_ 秒则直接返回（丢弃该帧，实现降采样）；
 *       通过节流检查后更新 last_time_ 并入队。
 */
void Recorder::record(
  const cv::Mat & img, const Eigen::Quaterniond & q,
  const std::chrono::steady_clock::time_point & timestamp)
{
  if (img.empty()) return;
  if (!init_) init(img);

  auto since_last = tools::delta_time(timestamp, last_time_);
  if (since_last < 1.0 / fps_) return;

  last_time_ = timestamp;
  queue_.push({img, q, timestamp});
}

/**
 * @brief 首次记录时的惰性初始化：打开文本文件、创建视频写手、启动保存线程。
 * @param img 首帧图像，用于提供视频分辨率 img.size()。
 * @return 无。
 * @note 视频编码固定为 MJPG（fourcc 'MJPG'），帧率使用构造传入的 fps_；
 *       保存线程入口为 &Recorder::save_to_file；最后置 init_ = true。
 * @note 本函数非线程安全，只应由 record() 在单线程上下文中首次触发。
 */
void Recorder::init(const cv::Mat & img)
{
  text_writer_.open(text_path_);
  auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  video_writer_ = cv::VideoWriter(video_path_, fourcc, fps_, img.size());
  saving_thread_ = std::thread(&Recorder::save_to_file, this);  // 启动保存线程
  init_ = true;
}

}  // namespace tools
