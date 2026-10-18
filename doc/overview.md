# 项目总体详细说明

> 项目：`JSU_RM_2027_infantry_2`
> 定位：RoboMaster 2027 赛季步兵自瞄（auto-aim）算法项目（infantry 2 号机）
> 本文档随项目推进更新；更新触发条件与范围见 [`AGENT.md` §5](../AGENT.md#5-维护规则重要)。

---

## 1. 背景与定位

本项目是 RoboMaster 步兵机器人的视觉自瞄算法工程。核心诉求是**可维护性**：算法的每一个环节、对外接口、底层通用件都应当是可独立替换的模块，而不是互相耦合的一团代码。

当前状态：**阶段 1（`tools/` 底层工具与函数）已完成**；随后把参考项目 sp_vision_25 的**剩余全部代码**（`io/ tasks/ src/ calibration/ tests/ configs/ assets/` 等）原封不动拷入本仓库作为起点（阶段 2 的输入）。目录结构现为 **sp_vision_25 的实际结构**；2026-10-17 已把阶段 0 计划骨架留下的空目录全部删除（见 §5.2），仓库中不再有任何 `.gitkeep`。上游代码的审核/裁剪/改造尚未开始。

## 2. 总目标

建立一个能够作为 RoboMaster 步兵自瞄算法载体的项目，其中：

- **算法部分**（识别、跟踪、预测、弹道解算等）与平台无关；
- **接口部分**（相机、串口/下位机、调试输出）与算法解耦，可单独替换与测试；
- **底层工具与函数**（数学、坐标变换、时间、滤波、日志、配置等）统一沉淀、供上层复用；
- 三者均模块化，方便维护。

**前期目标**：完成**底层工具与函数**以及**纯算法**的迁移。

**最终目标**（源自学习笔记 `docs/技术规划（续）.md`）：写自己的自瞄算法。技术路线判断为——**为性能以传统视觉为主、YOLO 为辅**；不追求一次成型，先有基本方案再优化；2027 规则（英雄与工程合并为重装机器人、无人机小型化）意味着步兵前期要承担更多输出，需同步提升命中、生存、机动，并考虑防空方案。

## 3. 阶段划分

| 阶段 | 内容 | 状态 |
| --- | --- | --- |
| 阶段 0 | 目录骨架 + 文档体系（`AGENT.md`、`doc/`） | **已完成**（计划骨架空目录已于 2026-10-17 清理，见 §5.2） |
| 阶段 1 | 底层工具与函数迁移（数学、时间、滤波、日志、配置、并发、可视化等） | **已完成**（`tools/` 迁移 25 个源文件，含中文注释 + 43 项冒烟测试；另有 2026-10-18 新增的 `file_logger.{hpp,cpp}`，非上游代码） |
| 阶段 2 | 纯算法迁移（识别/跟踪/解算/决策） | **进行中**：sp_vision_25 的算法代码已全量拷入（`tasks/`），尚未按本项目需求审核、裁剪、改造 |
| 阶段 3 | 接口与后端接入（相机、串口、推理后端按确认结果绑定） | 未开始（代码已存在，但版本/后端待确认） |
| 阶段 4 | 整机联调、性能优化与实测验证 | 未开始 |

## 4. 项目全景：三大支柱与一次开火的数据流

整个工程的思路可以概括为：**把"像素"翻译成"角度"，再把"角度"翻译成"云台运动"**。

```
输入（采集）  io::Camera（海康/迈德/大恒/USB）→ cv::Mat + 时间戳
              io::Gimbal / io::CBoard（IMU 四元数 + 云台姿态 + 弹速）
        ↓
思考（算法）  YOLO / Detector（检测）→ Solver（PnP + 坐标变换）
              → Tracker（状态机）→ Target（11 维 EKF）→ Aimer / Planner(MPC)
        ↓
执行（输出）  io::Command → io::Gimbal.send（串口，含速度/加速度前馈）
                          → io::CBoard.send（CAN，yaw/pitch 定点）
```

- **采集层**（`io/`）：把"像素 + 姿态"变成算法可用的输入——相机给图，云台/C 板给"我此刻朝哪"和"子弹多快"。
- **算法层**（`tasks/`）：四步走——**看到**（检测）、**算准**（PnP/坐标变换）、**跟住**（EKF 跟踪预测）、**打到**（决策/规划）。
- **执行层**：把决策结果发给云台，让它转过去并开火。
- **工具层**（`tools/`）：谁都不依赖，谁都用得上（数学、队列、日志、调试）。

三种决策架构并存，是理解本项目分叉的关键：

| 决策器 | 位置 | 产出 | 使用者 |
| --- | --- | --- | --- |
| `Aimer`（规则式，legacy） | `tasks/auto_aim/aimer.*` | `io::Command` | `standard`、`uav`、`sentry*` |
| `Planner`（MPC 轨迹优化，现代） | `tasks/auto_aim/planner/*` | `auto_aim::Plan` | `standard_mpc`、`auto_aim_debug_mpc`、打符 `mpc_aim` |
| `CommandGener`（多线程独立线程） | `tasks/auto_aim/multithread/*` | `io::Command`（直接发 C 板） | `mt_standard`、`mt_auto_aim_debug` |

## 5. 模块划分与目录职责

### 5.1 实际结构（随 sp_vision_25 拷入后）

| 目录 | 职责 | 状态 |
| --- | --- | --- |
| `tools/` | 底层工具与函数（数学、滤波、控制、并发、日志、配置、调试可视化） | **已实现**（阶段 1，含中文注释） |
| `io/` | 硬件 I/O 层：相机工厂 + 三家相机 SDK 封装 + USB 相机 + 云台串口 + C 板 CAN + IMU + ROS2 桥 | 代码已存在（上游原样），未审核 |
| `io/serial/` | vendored libserial（串口库） | 代码已存在 |
| `tasks/auto_aim/` | 自瞄主流程：装甲板检测（传统 CV / YOLOv5·v8·11）→ PnP 解算 → 跟踪 → EKF → 决策（Aimer / MPC Planner） | 代码已存在，未审核 |
| `tasks/auto_buff/` | 打符（能量机关）：检测 → PnP → 转速估计 → 决策（legacy / MPC） | 代码已存在，未审核 |
| `tasks/omniperception/` | 哨兵全向感知：多相机并行检测 → 去重排序 → 决策 | 代码已存在，未审核 |
| `src/` | 13 个可执行入口（步兵/无人机/哨兵/各调试版） | 代码已存在，未审核 |
| `calibration/` | 相机内参、手眼标定、数据采集、视频切片（6 个可执行） | 代码已存在，未审核 |
| `tests/` | 22 个独立 `main()` 测试（无测试框架） | 代码已存在，未审核 |
| `configs/` | 13 份 YAML（每台机器人一份 + 相机/标定配置），类构造函数内部解析 | 代码已存在 |
| `assets/` | OpenVINO IR 模型、ONNX 模型、素材与训练数据集（约 2.2 GB） | 已拷入（注意体积） |
| `doc/` | 文档体系（本目录） | 已建立 |

### 5.2 已清理的阶段 0 计划骨架目录（2026-10-17）

阶段 0 曾按「auto_function / main / measure / test / yolo / io/driver / io/uart」规划目录，但只有 `.gitkeep` 占位。上游代码拷入后职能已被实际结构完全覆盖，因此**已全部删除**：`auto_function/`（→ `tasks/`）、`main/`（→ `src/`）、`measure/`（→ `calibration/`）、`test/`（→ `tests/`）、`yolo/`（→ `tasks/auto_aim/yolos/`）、`io/driver|uart/`（→ `io/hikrobot|mindvision|daheng|usbcamera/`、`io/serial/`、`io/gimbal/`），连空目录 `troubleshooting-doc-pattern/` 一起清掉；同时删除了 `configs/.gitkeep`、`src/.gitkeep`、`tests/.gitkeep`（目录已有真实文件）。**仓库现在没有任何 `.gitkeep`**，`doc/code/` 下对应的 7 个旧占位目录也已删除。详细映射表见 [`directory_tree.md` §2.10](directory_tree.md)。

### 5.3 `tools/` 已迁移的文件（来源：sp_vision_25）

| 分类 | 文件 |
| --- | --- |
| 数学 | `math_tools.hpp/.cpp`（角度归一化、四元数/旋转矩阵/欧拉角互转、XYZ↔YPD 及雅可比、时间差、向量夹角） |
| 滤波与拟合 | `extended_kalman_filter.hpp/.cpp`（EKF + NIS 卡方检验）、`ransac_sine_fitter.hpp/.cpp`（正弦 RANSAC 拟合） |
| 控制与弹道 | `pid.hpp/.cpp`（PID，含角度模式）、`trajectory.hpp/.cpp`（抛体弹道解算） |
| 并发 | `thread_pool.hpp`（ThreadPool + `OrderedQueue<FrameT>`）、`thread_safe_queue.hpp`（线程安全队列模板） |
| 日志与配置 | `logger.hpp/.cpp`（spdlog 双 sink）、`yaml.hpp`（YAML 读取）、`exiter.hpp/.cpp`（Ctrl+C 优雅退出） |
| 通信与调试 | `crc.hpp/.cpp`（CRC8/16）、`img_tools.hpp/.cpp`（图像绘制）、`plotter.hpp/.cpp`（UDP JSON 调参可视化）、`recorder.hpp/.cpp`（视频+姿态录制） |

逐文件函数说明见 [`code/tools/`](code/tools/README.md)。

## 6. 构建与依赖

### 6.1 构建方式（sp_vision_25 版）

```bash
cmake -B build -S .
make -C build/ -j$(nproc)
```

顶层 [`CMakeLists.txt`](../CMakeLists.txt) 按依赖方向单向装配：

```
tools (OBJECT，零依赖)
  ← io (STATIC，+ serial + 三家相机 SDK + yaml-cpp + libusb)
    ← auto_aim (OBJECT，+ openvino::runtime + tinympcstatic)
      ← auto_buff (OBJECT，+ Ceres（声明未用）)
omniperception (OBJECT，+ openvino::runtime)
可执行文件（src/*、calibration/*、tests/*）← 上述库 + OpenCV + fmt + yaml-cpp
```

三个可选开关：ROS2（`sentry*` 与 3 个 ROS2 测试，条件满足才生成）、相机 SDK 架构分支（`x86_64`/`aarch64`）、TinyMPC `USING_CODEGEN`（从未启用）。

### 6.2 本机实测状态（2026-10-17）

| 项 | 值/结论 |
| --- | --- |
| C++ 标准 | C++17（`CMAKE_BUILD_TYPE` 固定 `Release`） |
| 编译选项 | 上游顶层脚本**未加 `-Wall -Wextra`**（仅 `tools/CMakeLists.txt` 阶段 1 手写版对 `tools` 加了） |
| OpenVINO | 脚本内 4 处写死 `/opt/intel/openvino_2024.6.0/runtime/cmake`；本机只装 `/opt/intel/openvino_2023.3.0`，`find_package(OpenVINO REQUIRED)` 回退搜索即命中。**2026-10-17 已实测：2023.3.0 编译全量工程零 openvino 错误，路径无需改** |
| 其它依赖 | OpenCV4（`/usr/local/lib/cmake/opencv4`）、Eigen3 3.4.0、spdlog 1.15.3（外部 fmt）、fmt 10.1.0、yaml-cpp 0.8.0、nlohmann_json 3.11.3、Ceres 2.2.0、libusb-1.0 1.0.29 |
| 相机 SDK | 三家 SDK **均已 vendored 在 `io/*/include` 与 `io/*/lib`**，无需系统安装 |
| ROS2 | 未安装 → configure 打印 `ROS2 environment not found, skipping ROS2-related code.`，`sentry*` 与 3 个 ROS2 测试不生成 |
| **全量构建** | ✅ **2026-10-17 已跑通**：`cp -r` 补拷上游 `color_track_demo/main.cpp`（[A-5](troubleshooting.md#a-5-顶层-cmakelists-引用未拷入的-color_track_demo-导致-configure-失败)）后，`cmake -B build_full -S . && cmake --build build_full -j32` **零错误**，产出 **35 个可执行文件**（42 个 `add_executable` 中 7 个因缺 ROS2 不生成）。唯一编译错误 `tests/camera_thread_test.cpp`（引用了分层重构时移出 `tools` 的 `Frame`/`create_yolov8s`）已修，见 [B-6](troubleshooting.md#b-6-上游测试引用了分层重构时已移出-tools-的类型与工厂函数) |
| 阶段 1 遗留产物 | `build/` 内仍有 `tools` 编译输出与 `build/smoke_tools.cpp`（43 项冒烟测试，当时全过）；**全量构建产物在 `build_full/`**（含 `build.log`） |

## 7. 参考资料（sp_vision_25 原始文档）

上游仓库 `docs/` 与协作配置已通读，是本项目最权威的"项目地图"，位于
`/home/yeyuqingzhu/C++/localcreated/summertraining/original_source/sp_vision_25/`：

| 文件 | 内容 | 用途 |
| --- | --- | --- |
| `docs/项目全景与教学指南.md`（1318 行） | 板块一 项目结构与全部代码清单（按文件）；板块二 第三方库与函数清单；板块三 构建思路与算法搭配（一次自瞄开火的全过程）；板块四 数学思路；板块五 创新内容实现（轨迹视角自瞄、决策器从规则到优化、切板跳变、图像-四元数时间对齐、YOLO 关键点+传统 CV 融合、多线程流水线） | **本文件 §4/§5 的依据**；查任何函数/文件职责的首选 |
| `docs/auto_aim_算法逐行解析.md`（1251 行） | `standard_mpc.cpp` 主程序、`armor.hpp` 数据结构、`tracker.cpp`、`target.cpp` 核心 EKF、`aimer.cpp`、`solver.cpp` 位姿解算逐行讲解 | 读算法细节的入口 |
| `docs/汇总.md`（15 行） | 学习笔记：识别器→解算器→跟踪器→预测器；坐标系转换；固定 pitch 穷举 yaw 重投影；本项目改造 YOLO 输出张量直接输出装甲板四角点；待学（训练改头、跟踪器轨迹视角实现） | 理解"为什么这么改 YOLO" |
| `docs/技术规划（续）.md`（21 行） | 2027 赛季规则变化与对步兵的影响；技术路线（传统视觉为主、YOLO 为辅） | 决定本项目后续取舍 |
| `AGENTS.md`（54 行） | 上游构建方式、依赖、测试清单、C++ 风格、命名约定、库依赖图、设计模式 | 编码规范参照（本仓库有 `.clang-format`） |
| `opencode.json` + `.opencode/command/context-update.md` | 上游用 opencode 的 context-update 命令扫描仓库并增量维护 `.opencode/context/**` | 仅作参考：本仓库不采用该机制（对应 `.opencode/context/` 与 skill 本体在上游仓库中亦不存在） |

**建议阅读顺序**（来自上游文档）：先 `项目全景` 板块三 → 板块四 → 板块五 → 板块一（先懂大局，再查字典）；代码入门从 `src/standard_mpc.cpp` 开始。

## 8. 设计原则

1. **分层**：`tools/`（通用件）← 算法层 ← 接口/驱动层；上层可依赖下层，下层不得反向依赖上层。
   - 实例：sp_vision_25 的 `tools/thread_pool.hpp` 原本 `#include "tasks/auto_aim/yolo.hpp"`，迁移时已解耦（`OrderedQueue` 模板化、`Frame` 与 `create_yolo*` 移交 auto_aim 层），见 [`code/tools/thread_pool.hpp.md`](code/tools/thread_pool.hpp.md)。
   - 与上游一致的单向结构：`tools ← io ← auto_aim ← auto_buff`、`omniperception ← openvino`。
2. **平台无关优先**：算法层不得直接包含平台/后端头文件；平台差异收敛在 `io/` 与推理封装（`yolo*`、`classifier`）内部。
3. **接口可替换**：相机（`CameraBase`）、推理后端（`YOLOBase`）、决策器（Aimer/Planner/CommandGener）均有明确边界，便于平替。
4. **可测试**：纯算法与工具函数应能脱离硬件验证——上游已有 `auto_aim_test`（回放视频）、`planner_test_offline`、`auto_buff_test`、`detector_video_test` 四个无硬件测试。
5. **配置外置**：参数放 `configs/`，类的构造函数内部解析 YAML，不在代码里硬编码。
6. **踩坑必记**：已踩过的问题一律落进 [`troubleshooting.md`](troubleshooting.md)，遇到即按方案执行、不重复思考。

## 9. 待确认事项

| # | 事项 | 影响 |
| --- | --- | --- |
| 1 | 目标平台与推理后端 | 决定 `yolo*`、相机封装的保留/替换方式与构建配置（当前默认 OpenVINO） |
| 2 | 交叉编译与部署方式 | 决定顶层 CMake 是否需要工具链文件、静态链接、目标架构选项 |
| 3 | 阶段 2 迁移清单：上游哪些代码保留、哪些重写、哪些删除 | 决定 `tasks/`、`io/` 的裁剪范围 |
| 4 | 相机型号与取流方式、下位机串口协议 | 决定保留哪家相机封装、保留 `gimbal`（串口）还是 `cboard`（CAN）。**串口协议已按 lt-2027 哨兵下位机定为 v3.0**（`0xFF`/`0x0D` + crc8、28 B/42 B、度制绝对角、921600、请求-响应 + 空闲心跳，见 [`code/io/gimbal/README.md`](code/io/gimbal/README.md)）；相机与 `gimbal`/`cboard` 取舍仍待确认 |
| 5 | `tools/` 迁移中记录的源文件缺陷是否修复（见 [`code/tools/`](code/tools/README.md) 各文件「备注」） | 影响后续稳定性；当前一律保持原行为未改 |
| 6 | ~~计划骨架遗留目录如何处置~~ | **已解决（2026-10-17）**：全部删除，映射见 §5.2 与 [`directory_tree.md` §2.10](directory_tree.md) |
| 7 | ~~顶层 `CMakeLists.txt` 用哪个版本~~ | **已解决（2026-10-17）**：保留上游版并补拷 `color_track_demo/`，全量构建已通过；OpenVINO 2023.3.0 可用，无需改脚本。阶段 1 手写版（仅 `tools`）仍可用 `git show :CMakeLists.txt` 取回 |
| 8 | `.gitignore` 是否恢复 `.dsh-meow/`（现被上游版本覆盖，记忆库变为未跟踪） | 防止误提交 AI 记忆库；见 [troubleshooting A-7](troubleshooting.md#a-7-gitignore-被上游版本覆盖导致-dsh-meow-变成未跟踪) |
| 9 | `readme.md` 是否改写为本项目 README（现为 sp_vision_25 原文） | 影响项目对外可读性 |
| 10 | 本地残留文件（`tasks/auto_aim/*.bak`、`tasks/auto_aim/solver.md`、`tasks/auto_aim/tracker.md`）是否清理 | 影响仓库整洁度与"文档唯一权威"原则（空目录 `troubleshooting-doc-pattern/` 已于 2026-10-17 删除） |
