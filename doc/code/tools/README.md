# `tools/` 逐文件函数说明

- **对应源码目录**：`tools/`
- **规划职责**：底层工具与函数：数学、时间、滤波、日志、配置解析、并发容器、调试可视化等通用件（阶段 1 重点）
- **当前状态**：**阶段 1 迁移已完成** —— 25 个源文件自 sp_vision_25（同济大学 SuperPower 战队 25 赛季视觉框架）的 `tools/` 逐个迁移，并补充详细中文注释。

## 文件清单

| 源文件 | 说明文档 | 概要 |
| --- | --- | --- |
| `crc.hpp` | [crc.hpp.md](crc.hpp.md) | CRC8/CRC16 校验算法接口（用于下位机通信帧校验） |
| `crc.cpp` | [crc.cpp.md](crc.cpp.md) | CRC8/CRC16 计算与校验实现 |
| `exiter.hpp` | [exiter.hpp.md](exiter.hpp.md) | 优雅退出标志（Exiter）接口 |
| `exiter.cpp` | [exiter.cpp.md](exiter.cpp.md) | 注册 SIGINT 处理，捕获 Ctrl+C 后置退出标志 |
| `extended_kalman_filter.hpp` | [extended_kalman_filter.hpp.md](extended_kalman_filter.hpp.md) | 扩展卡尔曼滤波器（线性/非线性 predict 与 update 重载、卡方检验数据） |
| `extended_kalman_filter.cpp` | [extended_kalman_filter.cpp.md](extended_kalman_filter.cpp.md) | EKF 的预测/更新实现（含 NIS 卡方检验与统计） |
| `img_tools.hpp` | [img_tools.hpp.md](img_tools.hpp.md) | OpenCV 图像绘制辅助接口（点、点集、文字） |
| `img_tools.cpp` | [img_tools.cpp.md](img_tools.cpp.md) | 绘制辅助实现 |
| `logger.hpp` | [logger.hpp.md](logger.hpp.md) | 全局日志器接口（spdlog） |
| `logger.cpp` | [logger.cpp.md](logger.cpp.md) | 日志器实现：文件 + 彩色控制台双 sink |
| `math_tools.hpp` | [math_tools.hpp.md](math_tools.hpp.md) | 数学工具接口：角度归一化、四元数/旋转矩阵/欧拉角互转、XYZ↔YPD 及其雅可比、时间差、向量夹角、square |
| `math_tools.cpp` | [math_tools.cpp.md](math_tools.cpp.md) | 数学工具实现 |
| `pid.hpp` | [pid.hpp.md](pid.hpp.md) | PID 控制器接口（支持角度模式，暴露 p/i/d 分量便于调试） |
| `pid.cpp` | [pid.cpp.md](pid.cpp.md) | PID 实现（输出限幅、积分限幅、角度环绕处理） |
| `plotter.hpp` | [plotter.hpp.md](plotter.hpp.md) | 调参可视化接口：经 UDP 把 JSON 数据发给外部绘图端 |
| `plotter.cpp` | [plotter.cpp.md](plotter.cpp.md) | Plotter 实现（POSIX socket + nlohmann/json 序列化） |
| `ransac_sine_fitter.hpp` | [ransac_sine_fitter.hpp.md](ransac_sine_fitter.hpp.md) | 正弦曲线 RANSAC 拟合器接口（转速/相位估计） |
| `ransac_sine_fitter.cpp` | [ransac_sine_fitter.cpp.md](ransac_sine_fitter.cpp.md) | 拟合实现：随机采样 + 最小二乘求解 + 内点评估 |
| `recorder.hpp` | [recorder.hpp.md](recorder.hpp.md) | 录制器接口：同时录制视频与姿态文本，异步写盘 |
| `recorder.cpp` | [recorder.cpp.md](recorder.cpp.md) | 录制器实现（独立保存线程 + ThreadSafeQueue） |
| `thread_pool.hpp` | [thread_pool.hpp.md](thread_pool.hpp.md) | 线程池 ThreadPool + 保序队列模板 OrderedQueue（**与 sp_vision_25 有解耦差异，见该文档**） |
| `thread_safe_queue.hpp` | [thread_safe_queue.hpp.md](thread_safe_queue.hpp.md) | 线程安全队列模板（可选满队列策略与回调） |
| `trajectory.hpp` | [trajectory.hpp.md](trajectory.hpp.md) | 弹道解算接口（不考虑空气阻力，给出 pitch 与飞行时间） |
| `trajectory.cpp` | [trajectory.cpp.md](trajectory.cpp.md) | 弹道解算实现（抛体运动求解，含无解判定） |
| `yaml.hpp` | [yaml.hpp.md](yaml.hpp.md) | YAML 配置读取辅助（`load` / `read`，键缺失时记录日志并退出） |

> `tools/CMakeLists.txt` 是构建脚本而非源文件，故无逐文件说明；其组织见 [`../../overview.md`](../../overview.md) 与文件内注释。

## 约定

新增源文件时，在本目录建立与源文件同名的 `<文件名>.md`，内容模板与维护规则见 [`../../README.md`](../../README.md)。

## 与 sp_vision_25 的差异汇总

| 文件 | 差异 | 原因 |
| --- | --- | --- |
| `thread_pool.hpp` | `OrderedQueue` 模板化为 `OrderedQueue<FrameT>`；移除 `struct Frame` 与 `create_yolo11s`/`create_yolov8s`；补 `#include <unordered_map>`、`<stdexcept>` | 原文件 `#include "tasks/auto_aim/yolo.hpp"`，违反"tools 不得依赖上层模块"的分层原则；相关内容移交 `auto_function/auto_aim/`（尚未实现）。补 include 使头文件可自包含 |
| `recorder.hpp` | 补 `#include <atomic>` | 原文件使用 `std::atomic` 却依赖间接包含 |
| `recorder.cpp` | 构造函数初始化列表顺序改为与成员声明顺序一致（`init_ → stop_thread_ → fps_ → queue_`） | 消除 `-Wreorder` 告警；四个成员互不依赖，初始化顺序不影响行为 |
| `extended_kalman_filter.hpp` | `double last_nis;` → `double last_nis = 0.0;` | 源文件该成员未初始化，首次 `update` 前读取属未定义行为 |
| `trajectory.hpp` | `unsolvable/fly_time/pitch` 补默认成员初值 `true/0.0/0.0` | 源文件在无解分支直接 `return`，三个成员均未赋值（实测会读到上一对象的栈残留值） |
| `math_tools.cpp` | `eulers` 中循环变量 `i` 改名 `idx` | 源文件中与外层 `auto i = axis0` 同名，`-Wshadow` 告警；作用域与语义不变 |
| `crc.cpp` | `i = (crc16 ^ byte) & 0x00ff;` 加显式 `static_cast<uint8_t>` | 消除 `-Wconversion` 告警；结果恒在 [0,255]，取值不变 |
| `yaml.hpp` | 补 `#include <cstdlib>`；`load()` 与 `read<T>(key)` 各补一处不可达 `return`（`YAML::Node{}` / `T{}`） | 声明 `std::exit` 的头文件应显式包含；不可达 `return` 为 `-Werror` 等严格配置下的防御（GCC 15 实测不告警）。注意 `T{}` 要求类型可默认构造 |

除上述之外，所有文件的函数签名、类名、成员名、命名空间（`tools`）、头文件保护宏、`#include` 路径风格与算法逻辑均与源文件保持一致（已用"去注释逐行 diff"逐文件核验）。
