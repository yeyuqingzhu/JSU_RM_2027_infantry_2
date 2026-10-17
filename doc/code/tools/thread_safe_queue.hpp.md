# tools/thread_safe_queue.hpp

**概要**：提供线程安全的有界队列模板 `tools::ThreadSafeQueue<T, PopWhenFull>`，以互斥锁 + 条件变量实现多生产者/多消费者访问，队满时按策略"丢弃最旧元素"或"调用满回调后丢弃新元素"。

> **迁移说明**：由 sp_vision_25 的 `tools/thread_safe_queue.hpp` **原样迁移**（纯标准库模板，不含任何项目耦合），仅补充中文注释，代码逻辑、签名、成员名、头文件保护宏与 include 风格均未改动。

## 类

### `template <typename T, bool PopWhenFull = false> class ThreadSafeQueue`

- **职责**：跨线程传递数据的定长队列；队列空时消费端阻塞等待，队列满时生产端按策略丢包而**不阻塞**。
- **线程安全语义**：除构造/析构外所有公有成员函数内部以 `mutex_` 加锁，可并发调用；`pop()` / `pop(T &)` / `front()` 为阻塞接口，`push()` / `back()` / `empty()` / `clear()` 为非阻塞接口。
- **模板参数**：
  - `T`：元素类型，需可拷贝赋值（`pop(T &)` 与 `back()` 依赖拷贝赋值；仅可移动类型请用 `T pop()`）。
  - `PopWhenFull`：队满策略。`true` = 弹出队首（丢最旧）后入队；`false` = 调用 `full_handler_` 后丢弃本次元素（丢最新）。

## 函数

### `ThreadSafeQueue(size_t max_size, std::function<void(void)> full_handler = [] {})`

- **作用**：构造队列，设定容量上限与满队列回调。
- **参数**：`max_size` 队列容量上限，仅用于 `push()` 判断是否已满；`full_handler` 满队列且 `PopWhenFull == false` 时调用的回调，默认空 lambda（静默丢包）。
- **返回**：无。
- **备注**：`full_handler_` 在**持锁状态**下被调用，回调内禁止再访问本队列（会自死锁），且应保持轻量。

### `void push(const T & value)`

- **作用**：入队一个元素；队满时按 `PopWhenFull` 策略处理。
- **参数**：`value` 待入队元素（按拷贝入队）。
- **返回**：无。
- **备注**：非阻塞。`PopWhenFull == true` 时先 `queue_.pop()` 再入队；否则调用 `full_handler_()` 后 `return`（本次元素被丢弃）。入队成功后 `notify_all()` 唤醒等待中的消费线程。

### `void pop(T & value)`

- **作用**：阻塞式出队，把队首元素拷贝给 `value` 后弹出。
- **参数**：`value` 输出引用，需支持拷贝赋值。
- **返回**：无。
- **备注**：队列为空时在 `not_empty_condition_` 上阻塞，直到被 `push()` 或 `clear()` 唤醒。`wait` 返回后理论上队列必非空，保留的空队列分支仅作兜底并向 `std::cerr` 打印错误（原实现行为）。

### `T pop()`

- **作用**：阻塞式出队并返回队首元素（移动语义）。
- **参数**：无。
- **返回**：队首元素（`std::move` 取出）。
- **备注**：语义同 `pop(T &)`，适用于只能移动的元素类型。

### `T front()`

- **作用**：阻塞式读取队首元素，**不出队**。
- **参数**：无。
- **返回**：队首元素的拷贝。
- **备注**：队列为空时阻塞等待。多消费者场景下同一元素可能被重复读取，不构成消费语义。

### `void back(T & value)`

- **作用**：读取队尾元素（非阻塞、不出队）。
- **参数**：`value` 输出引用，接收队尾元素。
- **返回**：无。
- **备注**：队列为空时**不等待**，仅向 `std::cerr` 打印错误并返回，`value` 不被修改。

### `bool empty()`

- **作用**：判断队列当前是否为空。
- **参数**：无。
- **返回**：`true` 表示空。
- **备注**：加锁但结果瞬时有效，并发下可能立刻过期，仅作参考判断。

### `void clear()`

- **作用**：清空队列内所有元素。
- **参数**：无。
- **返回**：无。
- **备注**：非阻塞；清空后 `notify_all()` 唤醒等待线程（唤醒后因谓词仍不满足会继续等待，不会取到空元素）。原注释：`// 如果其他线程正在等待队列不为空，这样可以唤醒它们`。

## 成员变量

| 成员 | 含义 |
| --- | --- |
| `std::queue<T> queue_` | 底层元素队列，由 `mutex_` 保护 |
| `size_t max_size_` | 容量上限，`push()` 判满依据 |
| `mutable std::mutex mutex_` | 保护 `queue_` 的互斥锁 |
| `std::condition_variable not_empty_condition_` | "队列非空"条件变量，供阻塞接口等待 |
| `std::function<void(void)> full_handler_` | 满队列且 `PopWhenFull == false` 时的回调（持锁调用） |
