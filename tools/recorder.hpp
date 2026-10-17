/**
 * @file recorder.hpp
 * @brief 录像 / 记录器：把图像帧与云台姿态异步落盘为视频 + 文本（时间戳与四元数）。
 *
 * 【作用】
 *   Recorder 记录一段"视觉数据"用于事后复盘：
 *   - 视频：把每一帧 cv::Mat 写成 .avi（MJPG 编码，帧率由构造参数 fps 指定）；
 *   - 文本：每行一条 "相对起始时间 w x y z"（四元数按 wxyz 顺序输出，供离线对齐姿态）。
 *   为保证主线程（视觉流水线）不被磁盘 IO 拖慢，写入工作由独立的 saving_thread_ 完成，
 *   主线程只通过线程安全队列 queue_ 投递帧数据。
 *
 * 【来源】
 *   sp_vision_25 项目 tools/recorder.hpp（原样迁移）。
 *
 * 【依赖】
 *   - 标准库：<atomic> <chrono> <fstream> <string> <thread>（<atomic> 为迁移时补充，见下）；
 *   - 第三方：Eigen3（Eigen::Quaterniond）、OpenCV4（cv::Mat / cv::VideoWriter）；
 *   - 项目内：tools/thread_safe_queue.hpp（异步帧队列）。
 *
 * 【线程模型与注意事项】
 *   - 生产者：record() / init() 由调用方线程（视觉主线程）调用，二者彼此不互斥，
 *     要求【同一时刻只有单线程调用 Recorder 的公开接口】，否则 last_time_ / init_
 *     等普通成员会被并发读写而数据竞争；
 *   - 消费者：saving_thread_ 内部循环执行 save_to_file()，只访问 queue_、
 *     video_writer_、text_writer_、start_time_ 与 stop_thread_，与生产者不共享可写状态；
 *   - queue_ 容量为 1：磁盘写入跟不上时按 ThreadSafeQueue 的默认策略（PopWhenFull == false、
 *     默认空回调）丢弃新帧，因此录像可能出现丢帧但不会阻塞主线程；
 *   - 析构时会向队列推入一个空帧以唤醒可能阻塞在 pop() 的保存线程。
 *
 *   【迁移补充说明】原文件使用了 std::atomic<bool> 却未包含 <atomic>，依赖其它头文件的
 *   间接包含。本次迁移补充 #include <atomic>（仅补头文件、不改任何行为与接口），
 *   以保证在不同标准库实现下都能稳定编译。
 */
#ifndef TOOLS__RECORDER_HPP
#define TOOLS__RECORDER_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tools/thread_safe_queue.hpp"
namespace tools
{
/**
 * @brief 异步录像器：视频落盘 + 四元数文本记录。
 *
 * 线程安全语义：仅 queue_ 与 stop_thread_ 可跨线程安全访问（队列自带互斥、标志为原子）；
 * record()/init() 与保存线程之间通过队列传递数据。除析构外的公开接口
 * 需由单一线程调用（详见文件头"线程模型与注意事项"）。
 */
class Recorder
{
public:
  /**
   * @brief 构造录像器：生成以当前系统时间命名的输出路径，并确保 records 目录存在。
   * @param fps 录像帧率（默认 30），同时用于采样节流（最小采样间隔 1/fps）与视频写入帧率。
   * @note 构造时只准备路径、不打开文件；真正的文件打开与保存线程创建延迟到首帧到达
   *       （init() 被调用）时进行，以避免产生空录像文件。
   */
  Recorder(double fps = 30);
  /**
   * @brief 析构录像器：通知保存线程退出、等待其 join，并关闭文本与视频文件。
   * @note 会先置 stop_thread_ = true 并推入一个空帧，唤醒可能阻塞在 queue_.pop() 的保存线程；
   *       若从未记录过任何帧（init_ == false），则直接返回，不做关闭动作。
   */
  ~Recorder();
  /**
   * @brief 提交一帧待记录数据（按 fps 节流，非阻塞）。
   * @param img       当前帧图像；为空则本次直接忽略。
   * @param q         该帧对应的云台姿态四元数。
   * @param timestamp 该帧的采集时刻（steady_clock），用于节流判断与文本中的相对时间。
   * @return 无。
   * @note 首帧会触发 init() 打开文件并启动保存线程；此后只有当距上一帧的时间
   *       超过 1/fps_ 秒时才会真正入队，否则直接返回（丢弃）。
   */
  void record(
    const cv::Mat & img, const Eigen::Quaterniond & q,
    const std::chrono::steady_clock::time_point & timestamp);

private:
  /**
   * @brief 队列中传递的单帧数据结构：图像 + 姿态 + 时间戳。
   */
  struct FrameData
  {
    cv::Mat img;                                       ///< 帧图像（空图像表示"毒丸"，用于唤醒退出）。
    Eigen::Quaterniond q;                              ///< 该帧对应的云台姿态四元数。
    std::chrono::steady_clock::time_point timestamp;   ///< 该帧的采集时刻。
  };
  bool init_;                                    ///< 是否已完成初始化（文件已打开、保存线程已启动）。
  std::atomic<bool> stop_thread_;                ///< 保存线程停止标志，true 表示请求退出。
  double fps_;                                   ///< 录像帧率，用于节流与 VideoWriter 帧率。
  std::string text_path_;                        ///< 文本文件输出路径（records/<时间>.txt）。
  std::string video_path_;                       ///< 视频文件输出路径（records/<时间>.avi）。
  std::ofstream text_writer_;                    ///< 文本文件流，由保存线程独占写入。
  cv::VideoWriter video_writer_;                 ///< 视频写入器，由保存线程独占写入。
  std::chrono::steady_clock::time_point start_time_;   ///< 记录起始时刻，用于计算文本中的相对时间。
  std::chrono::steady_clock::time_point last_time_;    ///< 上一帧入队时刻，用于 fps 节流判断。
  tools::ThreadSafeQueue<FrameData> queue_;      ///< 帧数据队列（容量 1），生产者与保存线程之间的通道。
  std::thread saving_thread_;  // 负责保存帧数据的线程
  /**
   * @brief 首次记录时初始化：打开文本文件、创建 VideoWriter、启动保存线程。
   * @param img 首帧图像，用于确定视频分辨率（img.size()）。
   * @note 只应在未初始化（init_ == false）时调用；由 record() 内部调用，非线程安全。
   */
  void init(const cv::Mat & img);
  /**
   * @brief 保存线程主循环：不断从 queue_ 取帧并写入视频与文本，直到 stop_thread_ 为 true。
   * @note 空图像帧（img.empty()）不写入文件，仅打印 debug 日志后跳过。
   */
  void save_to_file();
};

}  // namespace tools

#endif  // TOOLS__RECORDER_HPP
