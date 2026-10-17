/**
 * @file thread_pool.hpp
 * @brief 线程池 ThreadPool（纯标准库）+ 保序队列模板 OrderedQueue。
 *
 * 【作用】
 *   提供本项目并发基础设施的两件套：
 *   1. ThreadPool     —— 固定线程数的通用任务池，enqueue() 投递可调用对象，工作线程并发执行；
 *   2. OrderedQueue<> —— 按自增 id 严格保序的缓冲队列（模板），用于"多线程推理结果乱序返回、
 *                        消费端仍按帧号顺序取用"的场景。
 *
 * 【来源】
 *   sp_vision_25 项目 tools/thread_pool.hpp。
 *
 * 【依赖】
 *   仅标准库：<condition_variable> <functional> <mutex> <queue> <stdexcept> <thread>
 *             <unordered_map> <vector>；
 *   另包含项目内 "tools/logger.hpp"（OrderedQueue 的告警 / 销毁日志使用）。
 *   其中 <stdexcept>（std::runtime_error）与 <unordered_map> 为迁移时显式补充：
 *   原文件依赖其它头文件的间接包含，补上后本头文件可独立编译。
 *
 * 【线程模型与注意事项】
 *   - ThreadPool：tasks 由 queue_mutex 保护，condition 用于唤醒空闲工作线程；
 *     析构会置 stop 并 join 全部工作线程；每个任务从队列取出后【在锁外】执行，
 *     因此任务函数内部即使再调用 enqueue() 也不会自死锁。
 *   - OrderedQueue<FrameT>：enqueue()/dequeue()/try_dequeue() 由 mutex_ 保护；
 *     dequeue() 为阻塞接口（队列为空时等待），try_dequeue() 为非阻塞接口；
 *     get_size() 为【无锁】的估计值，不可作为同步依据（详见其注释）。
 *     FrameT 需可拷贝（缓冲与出队均按值拷贝）。
 *
 * 【与原文件的差异（分层原则）】
 *   原文件（sp_vision_25/tools/thread_pool.hpp）顶部还定义了：
 *     - struct Frame            （含 cv::Mat / Eigen::Quaterniond / std::list<auto_aim::Armor>）
 *     - inline std::vector<auto_aim::YOLO> create_yolo11s(...)
 *     - inline std::vector<auto_aim::YOLO> create_yolov8s(...)
 *   它们依赖 "tasks/auto_aim/yolo.hpp"，属于 auto_aim 层的内容，违反本项目
 *   "tools 是最底层、不得依赖上层模块"的分层原则。故本文件：
 *     - 删除 struct Frame、create_yolo11s、create_yolov8s；
 *     - 将 OrderedQueue 改为模板 template <typename FrameT> class OrderedQueue，
 *       队列与缓冲的元素类型由使用方（auto_aim 层）以 Frame 实例化。
 *   上述被删除的内容（Frame 及各 create_yolo* 工厂函数）按分层原则移交 auto_aim 层，
 *   本项目中【尚未实现】，后续应在 auto_aim 层补齐。
 *   回溯路径：sp_vision_25 仓库 tools/thread_pool.hpp（第 16-43 行，原 struct Frame
 *   与 create_yolo11s / create_yolov8s 所在位置）。
 */
#ifndef TOOLS__THREAD_POOL_HPP
#define TOOLS__THREAD_POOL_HPP

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <vector>

#include "tools/logger.hpp"

namespace tools
{
/**
 * @brief 按 id 严格保序的缓冲队列（模板）。
 *
 * 设计意图：生产者可能以任意顺序产出元素（例如多线程推理完成后按各自耗时先后返回），
 * 本类以"期望 id = current_id_（从 1 开始自增）"为基准把它们重新排成严格递增序列：
 *   - 收到的元素 id 正好等于 current_id_：直接入 main_queue_，并把 current_id_ +1 后
 *     继续从 buffer_ 中依次捞出后续连续的 id（连续追赶）；
 *   - 收到的元素 id 大于 current_id_：先暂存到 buffer_[id]，等待缺失的前序 id 到齐；
 *   - 收到的元素 id 小于 current_id_：视为重复 / 过期元素，记一条 warn("small id") 后丢弃。
 *
 * 线程安全语义：
 *   - enqueue() / dequeue() / try_dequeue() 由 mutex_ 互斥，可跨线程调用；
 *   - dequeue() 为【阻塞】接口：main_queue_ 为空时挂起线程，直到有元素入队并 notify_one；
 *   - try_dequeue() 为【非阻塞】接口：队列为空时立即返回 false；
 *   - get_size() 为【非线程安全】的规模估计，详见其注释。
 *
 * @tparam FrameT 元素类型，需可拷贝构造 / 拷贝赋值（内部按值存入 std::queue 与 std::unordered_map）。
 *                在 sp_vision_25 中原为 tools::Frame（现已移交 auto_aim 层，见文件头说明）。
 */
template <typename FrameT>
class OrderedQueue
{
public:
  /**
   * @brief 构造队列。
   * @note 期望的起始 id 为 1，即第一帧的 frame.id 必须为 1，否则会被记入 buffer_ 等待。
   */
  OrderedQueue() : current_id_(1) {}
  /**
   * @brief 析构队列：清空主队列与缓冲，并把 current_id_ 归零后打印日志。
   * @note 清空动作在锁内完成，日志在锁外打印（避免持锁做 IO）。
   *       析构后对象不可再使用；清空是"主动丢弃剩余元素"，不做任何等待。
   */
  ~OrderedQueue()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);

      main_queue_ = std::queue<FrameT>();
      buffer_.clear();
      current_id_ = 0;
    }
    tools::logger()->info("OrderedQueue destroyed, queue and buffer cleared.");
  }

  /**
   * @brief 入队一个元素，并按 id 顺序尽可能多地把连续元素搬入主队列。
   * @param item 待入队的元素（拷贝入队，其 id 决定存放位置）。
   * @return 无。
   * @note 非阻塞。若 item.id < current_id_，记为过期元素（打 warn("small id")）并丢弃；
   *       若 item.id == current_id_，直接入主队列并连续追赶 buffer_ 中后续连续的 id；
   *       否则暂存到 buffer_[item.id]。
   *       主队列非空时会 notify_one() 唤醒一个正在 dequeue() 等待的线程。
   */
  void enqueue(const FrameT & item)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (item.id < current_id_) {
      tools::logger()->warn("small id");
      return;
    }

    if (item.id == current_id_) {
      main_queue_.push(item);
      current_id_++;

      auto it = buffer_.find(current_id_);
      while (it != buffer_.end()) {
        main_queue_.push(it->second);
        buffer_.erase(it);
        current_id_++;
        it = buffer_.find(current_id_);
      }

      if (main_queue_.size() >= 1) {
        cond_var_.notify_one();
      }
    } else {
      buffer_[item.id] = item;
    }
  }

  /**
   * @brief 阻塞式出队：取出主队列头部元素（按 id 顺序，严格有序）。
   * @return 队首元素（按值拷贝返回；队首元素先从 main_queue_ 中拷贝再弹出）。
   * @note 主队列为空时阻塞等待，直到 enqueue() 成功入队并唤醒。
   *       注意：等待只保证"主队列非空"，缺失的前序 id 若一直不到达，被缓冲的元素不会被取出。
   */
  FrameT dequeue()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    cond_var_.wait(lock, [this]() { return !main_queue_.empty(); });

    FrameT item = main_queue_.front();
    main_queue_.pop();
    return item;
  }

  /**
   * @brief 非阻塞式出队：主队列非空则取出队首元素，否则立即返回 false。
   * @param item [out] 用于接收队首元素的引用，仅在返回 true 时被写入。
   * @return true 表示成功取出一个元素；false 表示当前主队列为空。
   */
  // 不会阻塞队列
  bool try_dequeue(FrameT & item)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (main_queue_.empty()) {
      return false;
    }
    item = main_queue_.front();
    main_queue_.pop();
    return true;
  }

  /**
   * @brief 返回当前积压元素总数（主队列 + 乱序缓冲）的估计值。
   * @return main_queue_.size() + buffer_.size()。
   * @note 【本函数不加锁，非线程安全】：与其它线程的 enqueue()/dequeue() 并发调用属于
   *       数据竞争（未定义行为），且返回值只是瞬时快照，可能立即过期。
   *       它只适合在"确定无并发写"的场合（如单线程观测、调试）拿来做粗略估计，
   *       不得用作同步 / 流控依据。此行为按原实现保持。
   */
  size_t get_size() { return main_queue_.size() + buffer_.size(); }

private:
  std::queue<FrameT> main_queue_;              ///< 已按 id 排好序、等待被消费的主队列。
  std::unordered_map<int, FrameT> buffer_;     ///< 乱序到达（id 超前）元素的暂存区，键为元素 id。
  int current_id_;                             ///< 下一个期望的 id；构造时为 1，析构时置 0。
  std::mutex mutex_;                           ///< 保护 main_queue_ / buffer_ / current_id_。
  std::condition_variable cond_var_;           ///< "主队列非空"条件变量，供 dequeue() 等待。
};

/**
 * @brief 固定线程数的通用线程池。
 *
 * 线程安全语义：enqueue() 可被多个线程并发调用；任务队列由 queue_mutex 保护，
 * 每个任务在【释放锁之后】执行，因此任务内部再向本线程池投递任务是安全的（不会自死锁）。
 * 析构时置 stop、清空未执行的任务队列并 join 所有工作线程；析构后 enqueue() 会抛异常。
 */
class ThreadPool
{
public:
  /**
   * @brief 构造线程池并立即创建 num_threads 个工作线程。
   * @param num_threads 工作线程数量。
   * @note 工作线程循环：等待"stop 或队列非空" → 取出一个任务 → 在锁外执行；
   *       stop 为 true 且任务队列为空时线程退出。
   */
  ThreadPool(size_t num_threads) : stop(false)
  {
    for (size_t i = 0; i < num_threads; ++i) {
      workers.emplace_back([this] {
        while (true) {
          std::function<void()> task;
          {
            std::unique_lock<std::mutex> lock(queue_mutex);
            condition.wait(lock, [this] { return stop || !tasks.empty(); });
            if (stop && tasks.empty()) {
              return;
            }
            task = std::move(tasks.front());
            tasks.pop();
          }
          task();
        }
      });
    }
  }

  /**
   * @brief 析构线程池：置停止标志、丢弃未执行任务、唤醒并 join 所有工作线程。
   * @note 未执行完的任务会被直接丢弃（不会等待任务执行完毕，只等待线程退出）。
   *       正在执行中的任务会先执行完（join 语义）。析构在对象生命周期结束时自动调用。
   */
  ~ThreadPool()
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      stop = true;
      tasks = std::queue<std::function<void()>>();
    }
    condition.notify_all();
    for (std::thread & worker : workers) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }

  /**
   * @brief 向线程池投递一个任务（非阻塞）。
   * @param f 任意可调用对象（lambda / 函数对象 / 函数指针等），被完美转发后存入任务队列。
   * @return 无。
   * @throw std::runtime_error 当线程池已停止（stop == true，即正在析构）时抛出。
   * @note 入队后 notify_one() 唤醒一个空闲工作线程。
   */
  // 添加任务到任务队列
  template <class F>
  void enqueue(F && f)
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      if (stop) {
        throw std::runtime_error("enqueue on stopped ThreadPool");
      }
      tasks.emplace(std::forward<F>(f));
    }
    condition.notify_one();
  }

private:
  std::vector<std::thread> workers;         // 工作线程
  std::queue<std::function<void()>> tasks;  // 任务队列
  std::mutex queue_mutex;                   // 任务队列互斥锁
  std::condition_variable condition;        // 条件变量，用于等待任务
  bool stop;                                // 是否停止线程池
};
}  // namespace tools

#endif  // TOOLS__THREAD_POOL_HPP
