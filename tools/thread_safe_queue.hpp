/**
 * @file thread_safe_queue.hpp
 * @brief 线程安全的有界队列模板：满时可选择"丢弃最旧元素"或"调用满回调"。
 *
 * 【作用】
 *   提供一个可被多个生产者 / 消费者线程同时访问的队列：
 *   - push() 入队。当队列长度已达到上限 max_size_ 时：
 *       * PopWhenFull == true ：先弹出队首（丢弃最旧元素），再入队；
 *       * PopWhenFull == false：调用 full_handler_ 回调后，直接丢弃本次待入队的元素。
 *   - pop() / front() / pop(T &) 在队列为空时会阻塞等待，直到有新元素入队或被 clear() 唤醒；
 *   - back() / empty() / size() / clear() 为非阻塞的辅助接口。
 *
 * 【来源】
 *   sp_vision_25 项目 tools/thread_safe_queue.hpp（原样迁移）。
 *   该文件为纯标准库模板，不依赖任何项目内模块，因此无需解耦、行为完全一致。
 *
 * 【依赖】
 *   仅标准库：<condition_variable> <functional> <iostream> <mutex> <queue>。
 *   不依赖项目内任何其它模块，属于 tools 层最底层的通用件，可被任意上层复用。
 *
 * 【线程模型与注意事项】
 *   - 除构造 / 析构外，所有公有成员函数内部均以 mutex_ 加锁，可跨线程安全调用。
 *   - 队列是"有界但不阻塞生产者"的设计：max_size_ 只影响 push() 的行为，
 *     生产者永远不会因为消费者太慢而被卡住，代价是主动丢帧。
 *   - full_handler_ 是在【持有互斥锁 mutex_ 的状态下】被调用的，
 *     回调内禁止再访问本队列（否则自死锁），也应尽量保持轻量、不阻塞。
 *   - 全类只有 not_empty_condition_ 一个条件变量：队列为空时等待，入队 / 清空时唤醒。
 *   - 模板参数 T 需要可拷贝赋值（pop(T &) 与 back() 依赖拷贝赋值）；
 *     若 T 只能移动，请使用 T pop() 重载（它使用 std::move）。
 *   - 本类不提供"元素已被消费"的反馈，push() 成功返回不代表元素一定能被弹出
 *     （可能因队满被丢弃）。
 */
#ifndef TOOLS__THREAD_SAFE_QUEUE_HPP
#define TOOLS__THREAD_SAFE_QUEUE_HPP

#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>

namespace tools
{
/**
 * @brief 线程安全的有界队列（模板）。
 *
 * 线程安全语义：所有公有成员函数通过 mutex_ 互斥，可在多线程环境下并发调用；
 * 其中 pop() / pop(T &) / front() 为【阻塞】接口，队列为空时挂起当前线程，
 * 直到有元素入队（push）或被 clear() 唤醒；其余接口为非阻塞。
 *
 * @tparam T            队列元素类型，需可拷贝（pop(T &) / back() 使用拷贝赋值）。
 * @tparam PopWhenFull  队列满时的策略：
 *                      true  = 丢弃队首（最旧）元素后入队，即"保新丢旧"；
 *                      false = 调用 full_handler_ 后丢弃本次新元素，即"保旧丢新"。
 */
template <typename T, bool PopWhenFull = false>
class ThreadSafeQueue
{
public:
  /**
   * @brief 构造队列。
   * @param max_size     队列容量上限，仅用于判断 push() 时是否已满。
   * @param full_handler 队列已满且 PopWhenFull == false 时调用的回调；
   *                     默认为空 lambda（即静默丢包）。回调在持锁状态下执行。
   */
  ThreadSafeQueue(
    size_t max_size, std::function<void(void)> full_handler = [] {})
  : max_size_(max_size), full_handler_(full_handler)
  {
  }

  /**
   * @brief 入队一个元素（非阻塞，队满时按策略丢弃）。
   * @param value 待入队的元素（拷贝入队）。
   * @return 无。
   * @note 队列已满时：PopWhenFull == true 则弹出队首再入队；
   *       PopWhenFull == false 则调用 full_handler_ 后直接返回（本次元素被丢弃）。
   *       入队成功后唤醒所有正在 pop()/front() 等待的线程。
   */
  void push(const T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.size() >= max_size_) {
      if (PopWhenFull) {
        queue_.pop();
      } else {
        full_handler_();
        return;
      }
    }

    queue_.push(value);
    not_empty_condition_.notify_all();
  }

  /**
   * @brief 阻塞式出队：把队首元素拷贝给 value 后弹出。
   * @param value [out] 用于接收队首元素的引用，需支持拷贝赋值。
   * @return 无。
   * @note 队列为空时阻塞等待，直到有元素入队或 clear() 被调用。
   *       正常情况下 wait 返回后队列必非空；保留的空队列分支仅作兜底日志。
   */
  void pop(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    if (queue_.empty()) {
      std::cerr << "Error: Attempt to pop from an empty queue." << std::endl;
      return;
    }

    value = queue_.front();
    queue_.pop();
  }

  /**
   * @brief 阻塞式出队（返回值版本）。
   * @return 队首元素（以移动方式取出）。
   * @note 队列为空时阻塞等待，语义同 pop(T &)；适用于只能移动的元素类型。
   */
  T pop()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    T value = std::move(queue_.front());
    queue_.pop();
    return std::move(value);
  }

  /**
   * @brief 阻塞式读取队首元素（不出队）。
   * @return 队首元素的拷贝。
   * @note 队列为空时阻塞等待；本接口不会移除元素，多消费者时同一元素可能被重复读取。
   */
  T front()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    return queue_.front();
  }

  /**
   * @brief 读取队尾元素（非阻塞，不出队）。
   * @param value [out] 用于接收队尾元素的引用。
   * @return 无。
   * @note 队列为空时不等待，仅打印错误信息到 std::cerr 后返回（value 不被修改）。
   */
  void back(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.empty()) {
      std::cerr << "Error: Attempt to access the back of an empty queue." << std::endl;
      return;
    }

    value = queue_.back();
  }

  /**
   * @brief 判断队列是否为空（非阻塞）。
   * @return true 表示队列当前为空。
   * @note 返回结果在其他线程并发入队 / 出队时立刻可能失效，仅适合做参考判断。
   */
  bool empty()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  /**
   * @brief 返回当前元素个数（非阻塞）。
   * @return 队列当前长度。
   * @note 与 empty() 一样，返回值在其他线程并发操作后立刻可能失效，只适合做
   *       诊断统计或"限时等待前先探一下深度"这类用途（例如 Gimbal::try_q()）。
   */
  size_t size()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.size();
  }

  /**
   * @brief 清空队列中所有元素（非阻塞）。
   * @return 无。
   * @note 清空后会唤醒所有等待线程；被唤醒的线程会因谓词不满足而继续等待，
   *       因此不会有人从空队列中取到"幽灵元素"。
   */
  void clear()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!queue_.empty()) {
      queue_.pop();
    }
    not_empty_condition_.notify_all();  // 如果其他线程正在等待队列不为空，这样可以唤醒它们
  }

private:
  std::queue<T> queue_;                            ///< 底层元素队列（由 mutex_ 保护）。
  size_t max_size_;                                ///< 容量上限，push() 时用于判断是否已满。
  mutable std::mutex mutex_;                       ///< 保护 queue_ 的互斥锁。
  std::condition_variable not_empty_condition_;    ///< "队列非空"条件变量，供 pop()/front() 等待。
  std::function<void(void)> full_handler_;         ///< 队满且 PopWhenFull == false 时的回调（持锁调用）。
};

}  // namespace tools

#endif  // TOOLS__THREAD_SAFE_QUEUE_HPP
