# tools/thread_pool.hpp

**概要**：提供并发基础设施两件套——固定线程数的通用任务池 `tools::ThreadPool`，以及按自增 id 严格保序的缓冲队列模板 `tools::OrderedQueue<FrameT>`（用于多线程推理结果乱序返回、消费端仍按序取用）。

> **迁移说明**：由 sp_vision_25 的 `tools/thread_pool.hpp` 迁移。`ThreadPool` 完整保留（仅补中文注释）；`OrderedQueue` 由非模板类改写为 **模板类**；原文件中的 `struct Frame`、`create_yolo11s()`、`create_yolov8s()` 已删除并**移交 auto_aim 层**（详见文末「与 sp_vision_25 的差异」）。

## 类

### `template <typename FrameT> class OrderedQueue`

- **职责**：把乱序到达、带自增 `id` 的元素重排为严格递增序列。以 `current_id_`（从 1 开始）为期望 id：`id == current_id_` 直接入主队列并连续从缓冲追赶后续连续 id；`id > current_id_` 暂存 `buffer_`；`id < current_id_` 视为过期元素，打 `warn("small id")` 日志后丢弃。
- **线程安全语义**：`enqueue()` / `dequeue()` / `try_dequeue()` 由 `mutex_` 互斥，可跨线程调用；`dequeue()` **阻塞**（主队列空时等待 `cond_var_`），`try_dequeue()` **非阻塞**；`get_size()` **无锁、非线程安全**。
- **模板参数**：`FrameT` 元素类型，需可拷贝构造/拷贝赋值（内部按值存入 `std::queue` 与 `std::unordered_map`）；需具备公有成员 `int id`。在 sp_vision_25 中原为 `tools::Frame`。

## 函数（`OrderedQueue`）

### `OrderedQueue()`

- **作用**：构造队列，`current_id_` 初始化为 1。
- **参数**：无。
- **返回**：无。
- **备注**：调用方必须从 `id == 1` 开始投递元素，否则元素会一直滞留在 `buffer_` 中等待前序 id。

### `~OrderedQueue()`

- **作用**：清空主队列与缓冲、把 `current_id_` 置 0，并打印销毁日志。
- **参数**：无。
- **返回**：无。
- **备注**：清空在锁内完成，日志（`"OrderedQueue destroyed, queue and buffer cleared."`）在锁外打印。剩余元素被直接丢弃，不做等待。

### `void enqueue(const FrameT & item)`

- **作用**：入队一个元素，并尽可能把缓冲中连续的元素一并搬入主队列。
- **参数**：`item` 待入队元素，其 `id` 决定存放位置。
- **返回**：无。
- **备注**：非阻塞。`id < current_id_` → `tools::logger()->warn("small id")` 后丢弃；`id == current_id_` → 入主队列、`current_id_++`，随后循环从 `buffer_` 追赶连续 id；否则 `buffer_[item.id] = item`。主队列非空时 `notify_one()` 唤醒一个等待中的 `dequeue()`。

### `FrameT dequeue()`

- **作用**：阻塞式出队，取出主队列头部元素。
- **参数**：无。
- **返回**：队首元素（拷贝返回，随后从主队列弹出）。
- **备注**：主队列为空时在 `cond_var_` 上阻塞等待。等待只保证主队列非空；若缺失的前序 id 始终不到达，被缓冲的元素不会被取出（无超时机制）。

### `bool try_dequeue(FrameT & item)`

- **作用**：非阻塞式出队，主队列非空则取出队首，否则立即失败。
- **参数**：`item` 输出引用，仅在返回 `true` 时被写入。
- **返回**：`true` 成功取出；`false` 主队列为空。
- **备注**：原注释保留：`// 不会阻塞队列`。

### `size_t get_size()`

- **作用**：返回当前积压元素总数（主队列 + 乱序缓冲）的估计值。
- **参数**：无。
- **返回**：`main_queue_.size() + buffer_.size()`。
- **备注**：**原实现无锁，非线程安全**（与并发写构成数据竞争，返回值可能立即过期），仅适合无并发写时做粗略估计，**不得作为同步/流控依据**。迁移时按要求保持原行为，未加锁。

## 类

### `class ThreadPool`

- **职责**：固定线程数的通用任务池；`enqueue()` 投递可调用对象，工作线程从任务队列取出并执行。
- **线程安全语义**：`enqueue()` 可多线程并发调用；任务在**释放锁之后**执行，故任务内部再投递任务是安全的（不会自死锁）。析构后调用 `enqueue()` 抛 `std::runtime_error`。

## 函数（`ThreadPool`）

### `ThreadPool(size_t num_threads)`

- **作用**：构造线程池并创建 `num_threads` 个工作线程。
- **参数**：`num_threads` 工作线程数量。
- **返回**：无。
- **备注**：工作线程循环：`condition.wait` 等待 `stop || !tasks.empty()`；`stop && tasks.empty()` 时退出；否则取出任务并在锁外执行。

### `~ThreadPool()`

- **作用**：置 `stop = true`、清空未执行的任务队列、`notify_all()` 并 `join` 全部工作线程。
- **参数**：无。
- **返回**：无。
- **备注**：仍在执行中的任务会执行完（join 语义）；**尚未开始的任务被直接丢弃**，不做等待。

### `template <class F> void enqueue(F && f)`

- **作用**：向线程池投递一个任务。
- **参数**：`f` 任意可调用对象（lambda / 函数对象 / 函数指针），完美转发后入队。
- **返回**：无。
- **备注**：原注释保留：`// 添加任务到任务队列`。线程池已停止（`stop == true`）时抛 `std::runtime_error("enqueue on stopped ThreadPool")`；入队后 `notify_one()` 唤醒一个空闲线程。

## 成员变量

### `OrderedQueue`

| 成员 | 含义 |
| --- | --- |
| `std::queue<FrameT> main_queue_` | 已按 id 排好序、等待消费的主队列 |
| `std::unordered_map<int, FrameT> buffer_` | 乱序（id 超前）元素暂存区，键为元素 id |
| `int current_id_` | 下一个期望的 id；构造为 1，析构置 0 |
| `std::mutex mutex_` | 保护 `main_queue_` / `buffer_` / `current_id_` |
| `std::condition_variable cond_var_` | "主队列非空"条件变量，供 `dequeue()` 等待 |

### `ThreadPool`

| 成员 | 含义 |
| --- | --- |
| `std::vector<std::thread> workers` | 工作线程 |
| `std::queue<std::function<void()>> tasks` | 任务队列 |
| `std::mutex queue_mutex` | 任务队列互斥锁 |
| `std::condition_variable condition` | 条件变量，用于等待任务 |
| `bool stop` | 是否停止线程池 |

## 与 sp_vision_25 的差异（分层解耦）

原文件 `sp_vision_25/tools/thread_pool.hpp` 中还包含 `struct Frame`（成员为 `int id; cv::Mat img; std::chrono::steady_clock::time_point t; Eigen::Quaterniond q; std::list<auto_aim::Armor> armors;`）、`create_yolo11s()` 与 `create_yolov8s()`（均返回 `std::vector<auto_aim::YOLO>`），它们 `#include "tasks/auto_aim/yolo.hpp"`，属于 auto_aim 层内容，违反本项目「tools 是最底层、不得依赖上层模块」的分层原则。

| 变更 | 内容 | 理由 |
| --- | --- | --- |
| 1. `OrderedQueue` 模板化 | `class OrderedQueue` → `template <typename FrameT> class OrderedQueue`；`std::queue<tools::Frame>` → `std::queue<FrameT>`，`std::unordered_map<int, tools::Frame>` → `std::unordered_map<int, FrameT>` | 移除对 `tools::Frame`（即对 auto_aim 的间接依赖），由使用方以 auto_aim 层的 `Frame` 实例化；其余逻辑（enqueue/dequeue/try_dequeue/get_size、析构清空、按 id 排序与缓冲、告警日志）完全一致 |
| 2. 删除 `struct Frame` | 移交 auto_aim 层（本仓库**未建公共头**；唯一使用者 `tests/camera_thread_test.cpp` 已在文件内自定义 `Frame`，见 [troubleshooting B-6](../../troubleshooting.md)） | 含 `cv::Mat` / `Eigen::Quaterniond` / `std::list<auto_aim::Armor>`，属 auto_aim 层数据模型 |
| 3. 删除 `create_yolo11s()` / `create_yolov8s()` | 移交 auto_aim 层（同上；`camera_thread_test.cpp` 内以 `create_yolos()` 等价实现） | 工厂函数返回 `std::vector<auto_aim::YOLO>`，属 auto_aim 层 |
| 4. 移除 `#include "tasks/auto_aim/yolo.hpp"` | 同时补充 `#include <unordered_map>` 与 `#include <stdexcept>` | 解耦；`buffer_` 使用 `std::unordered_map`、`ThreadPool::enqueue()` 抛出 `std::runtime_error`，原文件均依赖间接包含，现显式包含（同为标准库头文件，非新依赖），使本头文件可独立编译 |
| 5. 局部变量微调 | `std::queue<tools::Frame>()` → `std::queue<FrameT>()` | 随模板化同步调整，行为不变 |

**回溯路径**：`sp_vision_25` 仓库 `tools/thread_pool.hpp` 第 16–43 行（`struct Frame` 与两个 `create_yolo*` 所在位置）。

**未改变项**：命名空间 `tools`、类名 `ThreadPool` / `OrderedQueue`、全部成员名、头文件保护宏 `TOOLS__THREAD_POOL_HPP`、include 路径风格（如 `"tools/logger.hpp"`）、`get_size()` 的无锁行为。
