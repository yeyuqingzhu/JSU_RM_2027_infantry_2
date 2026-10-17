# 项目目录树（含每个文件、文件夹的概要）

> 项目：`JSU_RM_2027_infantry_2`
> 说明：本文件描述仓库的**实际结构**。2026-10-17 已删除全部「仅 `.gitkeep` 的空目录」与多余的 `.gitkeep`（见 §2.10）。
> 维护规则见 [`AGENT.md` §5](../AGENT.md#5-维护规则重要)：仅当函数作用变动、增删函数或被显式指定时更新。
> 现状（2026-10-17）：参考项目 sp_vision_25 的**全部代码**已拷入本仓库（`io/ tasks/ tools/ src/ calibration/ tests/ configs/ assets/` 等），
> 目录结构随之由「阶段 0 的计划骨架」变为「以 sp_vision_25 为准的实际结构」；计划骨架遗留的空目录**已清理完毕**（见 §2.10）。

## 1. 目录树

```text
JSU_RM_2027_infantry_2/
├── .gitignore                     # Git 忽略规则（build/、logs/、records*/、imgs/ 等；注意当前版本不含 .dsh-meow/）
├── .clang-format                  # 代码格式化规则（源自 sp_vision_25：Google 改，100 列）
├── .dsh-meow/                     # AI 记忆库（本机数据，误删不可恢复；当前**未被** .gitignore 覆盖，见 doc/troubleshooting.md A-7）
├── AGENT.md                       # 项目入口：概要、总目标、当前状态、文档维护规则、待确认事项
├── readme.md                      # sp_vision_25 原始 README（351 行，未改写为本项目 README）
├── LICENSE                        # 原项目许可证
├── CMakeLists.txt                 # 顶层构建脚本（sp_vision_25 版：OpenVINO + 三家相机 SDK + 可选 ROS2）
├── autostart.sh                   # 机载自启动脚本
├── buff_layout.xml                # PlotJuggler 打符调试布局
├── mpc_layout.xml                 # PlotJuggler MPC 调试布局
├── build/                         # CMake 构建目录（gitignore；内含阶段 1 的 tools 编译输出与冒烟测试）
│
├── tools/                         # 底层工具与函数（阶段 1 已迁移并补中文注释；后续未被拷入覆盖）
│   ├── CMakeLists.txt
│   ├── crc.hpp / crc.cpp
│   ├── exiter.hpp / exiter.cpp
│   ├── extended_kalman_filter.hpp / extended_kalman_filter.cpp
│   ├── img_tools.hpp / img_tools.cpp
│   ├── logger.hpp / logger.cpp
│   ├── math_tools.hpp / math_tools.cpp
│   ├── pid.hpp / pid.cpp
│   ├── plotter.hpp / plotter.cpp
│   ├── ransac_sine_fitter.hpp / ransac_sine_fitter.cpp
│   ├── recorder.hpp / recorder.cpp
│   ├── thread_pool.hpp            # 头文件即实现（模板）
│   ├── thread_safe_queue.hpp      # 头文件即实现（模板）
│   ├── trajectory.hpp / trajectory.cpp
│   └── yaml.hpp                   # 头文件即实现
│
├── io/                            # 硬件 I/O 层（STATIC 库 io）
│   ├── CMakeLists.txt
│   ├── camera.hpp / camera.cpp
│   ├── command.hpp
│   ├── cboard.hpp / cboard.cpp
│   ├── socketcan.hpp
│   ├── gimbal/
│   │   ├── gimbal.hpp / gimbal.cpp
│   │   └── laser_gimbal.hpp / laser_gimbal.cpp
│   ├── hikrobot/                  # 海康相机封装 + vendored SDK
│   │   ├── hikrobot.hpp / hikrobot.cpp
│   │   ├── include/               # 7 个 SDK 头（MvCameraControl.h、CameraParams.h、PixelType.h …）
│   │   └── lib/{amd64,arm64}/libMvCameraControl.so
│   ├── mindvision/                # 迈德相机封装 + vendored SDK
│   │   ├── mindvision.hpp / mindvision.cpp
│   │   ├── include/               # CameraApi.h、CameraDefine.h、CameraStatus.h
│   │   └── lib/{amd64,arm64}/libMVSDK.so
│   ├── daheng/                    # 大恒相机封装 + vendored SDK
│   │   ├── daheng.hpp / daheng.cpp
│   │   ├── include/               # GxIAPI.h、DxImageProc.h、GXDef.h、GxPixelFormat.h …
│   │   └── lib/x86_64/            # libgxiapi.so、libdxvideoproc.so、libdxmediaproc.so、ffmpeg 系列 .so、*.cti
│   ├── usbcamera/usbcamera.hpp / usbcamera.cpp
│   ├── dm_imu/dm_imu.hpp / dm_imu.cpp
│   ├── ros2/                      # ros2.hpp/.cpp、publish2nav、subscribe2nav（找到 ROS2 才编译）
│   ├── serial/                    # vendored libserial（CMakeLists + include/serial/** + src/**）
│
├── tasks/                         # 算法层
│   ├── auto_aim/                  # 自瞄主流程（OBJECT 库 auto_aim）
│   │   ├── CMakeLists.txt
│   │   ├── armor.hpp / armor.cpp
│   │   ├── detector.hpp / detector.cpp (+ detector.cpp.bak)
│   │   ├── classifier.hpp / classifier.cpp
│   │   ├── solver.hpp / solver.cpp (+ solver.md)
│   │   ├── target.hpp / target.cpp
│   │   ├── tracker.hpp / tracker.cpp (+ tracker.md)
│   │   ├── aimer.hpp / aimer.cpp
│   │   ├── shooter.hpp / shooter.cpp
│   │   ├── voter.hpp / voter.cpp
│   │   ├── yolo.hpp / yolo.cpp
│   │   ├── yolos/yolov5.*、yolov8.*、yolo11.*（各含 .cpp.bak）
│   │   ├── planner/planner.hpp / planner.cpp
│   │   ├── planner/tinympc/       # vendored TinyMPC（STATIC 库 tinympcstatic）
│   │   └── multithread/mt_detector.*、commandgener.*
│   ├── auto_buff/                 # 打符（OBJECT 库 auto_buff）
│   │   ├── CMakeLists.txt
│   │   ├── buff_type.hpp / buff_type.cpp
│   │   ├── buff_detector.hpp / buff_detector.cpp
│   │   ├── yolo11_buff.hpp / yolo11_buff.cpp
│   │   ├── buff_solver.hpp / buff_solver.cpp
│   │   ├── buff_target.hpp / buff_target.cpp
│   │   └── buff_aimer.hpp / buff_aimer.cpp
│   └── omniperception/            # 哨兵全向感知（OBJECT 库 omniperception）
│       ├── CMakeLists.txt
│       ├── detection.hpp
│       ├── perceptron.hpp / perceptron.cpp
│       └── decider.hpp / decider.cpp
│
├── src/                           # 可执行入口（13 个 main）
│   ├── standard.cpp / standard_mpc.cpp / mt_standard.cpp
│   ├── uav.cpp / uav_debug.cpp
│   ├── auto_aim_debug_mpc.cpp / mt_auto_aim_debug.cpp
│   ├── auto_buff_debug.cpp / auto_buff_debug_mpc.cpp
│   └── sentry.cpp / sentry_bp.cpp / sentry_debug.cpp / sentry_multithread.cpp（ROS2 条件编译）
│
├── calibration/                   # 标定与离线工具（6 个可执行）
│   ├── capture.cpp / capture_daheng.cpp
│   ├── calibrate_camera.cpp
│   ├── calibrate_handeye.cpp / calibrate_robotworld_handeye.cpp
│   └── split_video.cpp
│
├── color_track_demo/              # 颜色追踪演示（ONNX Runtime；2026-10-17 从上游补拷，此前缺失导致 configure 失败）
│   └── main.cpp
│
├── tests/                         # 测试程序（22 个独立 main，无测试框架）
│   ├── 无硬件：auto_aim_test / auto_buff_test / detector_video_test / planner_test_offline
│   ├── 单硬件：camera_test / usbcamera_test / cboard_test / gimbal_test / gimbal_response_test / dm_test / fire_test / handeye_test / laser_gimbal_test
│   ├── 组合：camera_thread_test / multi_usbcamera_test / usbcamera_detect_test / camera_detect_test / minimum_vision_system / planner_test
│   └── ROS2 条件编译：publish_test / subscribe_test / topic_loop_test
│
├── configs/                       # 每台机器人一份 YAML（13 个）
│   ├── standard3.yaml / standard4.yaml / sentry.yaml / uav.yaml / ascento.yaml / demo.yaml / example.yaml
│   ├── camera.yaml / mvs.yaml / daheng.yaml
│   └── calibration.yaml / calibration_camera.yaml / calibrate_camera.yaml
│
├── assets/                        # 模型与素材（2.2 GB，其中 PinkTraining/ 约 2.1 GB 为数据集）
│   ├── yolo11.xml|.bin、yolov5.xml|.bin、yolov8.xml|.bin、yolo11_buff_int8.xml|.bin（OpenVINO IR）
│   ├── color_track_model.onnx|.xml|.bin、tiny_resnet.onnx、best2-sim.onnx
│   ├── demo/、img-temp/、img_with_q/、captured_20260725_172207/、PinkTraining/
│   └── standard_fanblade.jpg
│
└── doc/                           # 文档体系
    ├── README.md                  # 文档索引与维护规则
    ├── overview.md                # 项目总体详细说明
    ├── directory_tree.md          # 本文件：目录树与逐项概要
    ├── executables.md             # 可执行程序一览：作用 / 用法 / 默认配置 / 硬件依赖
    ├── troubleshooting.md         # 问题与解决方案（遇到问题先查这里）
    └── code/                      # 逐文件函数作用说明
        ├── README.md              # 命名约定、文档模板与映射表
        ├── tools/                 # 已覆盖：README + 25 篇逐文件说明
        ├── io/gimbal/             # 已覆盖：README + 4 篇逐文件说明
        └── configs/               # 占位（13 份 YAML，无 C++ 源文件）
```

## 2. 逐项概要

### 2.1 根目录文件

| 路径 | 概要 |
| --- | --- |
| `.gitignore` | 忽略 `build/`、`logs/`、`Camera/`、`imgs/`、`records*/`、`testvideos/`、`assets/img_with_q/`、`MvSdkLog/`、`.vscode/`、`.github/`、`watchdog.sh`。**已不含 `.dsh-meow/`**（被上游版本覆盖，见 [troubleshooting A-7](troubleshooting.md#a-7-gitignore-被上游版本覆盖导致-dsh-meow-变成未跟踪)） |
| `.clang-format` | 源自 sp_vision_25：Google 风格改造，指针左/中，类与函数大括号换行，100 列 |
| `.dsh-meow/` | meow-memory 记忆库（`memory.db`），本机数据、误删不可恢复 |
| `AGENT.md` | 项目入口：概要表、总目标、当前状态、文档体系与维护规则、待确认事项 |
| `readme.md` | **sp_vision_25 原始 README 全文**（亮点、模块划分、轨迹视角自瞄理论、命中率等）。本项目尚未改写 |
| `LICENSE` | 原项目许可证文件 |
| `CMakeLists.txt` | sp_vision_25 版顶层构建脚本：`project(sp_vision)`、C++17、Release、OpenVINO 路径写死 `/opt/intel/openvino_2024.6.0/runtime/cmake/`；按 `tools → io → tasks/auto_aim → tasks/auto_buff → tasks/omniperception` 顺序装配，再生成可执行文件。**注意**：阶段 1 的宿主侧 43 行版本已被覆盖，仍可用 `git show :CMakeLists.txt` 取回 |
| `autostart.sh` | 机载开机自启动脚本（拉起重快门的视觉进程） |
| `buff_layout.xml` / `mpc_layout.xml` | PlotJuggler 调试布局：打符曲线 / MPC 曲线 |
| `build_full/` | **全量构建产物目录（gitignore）**：`cmake -B build_full -S . && cmake --build build_full -j32` 在此生成 35 个可执行文件；`build_full/build.log` 留存最近一次全量编译日志 |
| `build/` | 阶段 1 遗留产物目录（gitignore）。含当时 `tools` 的编译输出、`build/smoke_tools.cpp`（43 项冒烟测试）与 `compile_commands.json` |

### 2.2 构建体系：目标一览

| 目标 | 类型 | 源文件 | 依赖 |
| --- | --- | --- | --- |
| `tools` | OBJECT | `tools/*.cpp`（11 个） | Eigen3、spdlog、yaml-cpp、nlohmann_json、fmt、OpenCV、Threads（**阶段 1 手写版本，已补注释**） |
| `serial` | STATIC | `io/serial/src/**`（vendored libserial） | rt、pthread |
| `io` | STATIC | `io/*.cpp` + 3 家相机 + `dm_imu` + `gimbal` | `MvCameraControl`、`MVSDK`、`gxiapi`、`usb-1.0`、`yaml-cpp`、`serial`；PUBLIC 暴露三家相机 `include`/`lib` 目录 |
| `tinympcstatic` | STATIC | `tasks/auto_aim/planner/tinympc/*.cpp` | Eigen（含 `USING_CODEGEN` 分支，从未启用） |
| `auto_aim` | OBJECT | `tasks/auto_aim/**`（16 个 .cpp） | `io`、`openvino::runtime`、`tinympcstatic` |
| `auto_buff` | OBJECT | `tasks/auto_buff/*.cpp` | `auto_aim`、`openvino::runtime`、Ceres（**源码零调用**） |
| `omniperception` | OBJECT | `tasks/omniperception/*.cpp` | `openvino::runtime` |
| 可执行（常规） | — | `src/standard|mt_standard|standard_mpc|auto_aim_debug_mpc|mt_auto_aim_debug|auto_buff_debug|auto_buff_debug_mpc|uav|uav_debug.cpp` | `${OpenCV_LIBS}`、`fmt::fmt`、`yaml-cpp` + 上述库 |
| 可执行（标定） | — | `calibration/*.cpp`（`capture_daheng` 额外链接 `gxiapi dxmediaproc usb-1.0`） | `tools`、`io` 等 |
| 可执行（测试） | — | `tests/*.cpp`（22 个） | `tools`、`io`、`auto_aim`/`auto_buff` |
| `color_track_demo` | — | `color_track_demo/main.cpp`（**2026-10-17 已从上游补拷**，解决 configure 失败） | ONNX Runtime（`$HOME/ProgramFiles/onnxruntime/1.23.2`，本机已装） |
| `sentry*`、`publish_test`、`subscribe_test`、`topic_loop_test` | — | `src/sentry*.cpp`、`tests/publish_test.cpp` 等 | 仅当 `ament_cmake + rclcpp + std_msgs + rosidl_typesupport_cpp + sp_msgs` 全部找到才生成 |

> 子目录脚本：`tools/CMakeLists.txt`（阶段 1 手写）、`io/CMakeLists.txt`、`io/serial/CMakeLists.txt`、`tasks/auto_aim/CMakeLists.txt`、`tasks/auto_aim/planner/tinympc/CMakeLists.txt`、`tasks/auto_buff/CMakeLists.txt`、`tasks/omniperception/CMakeLists.txt`。
> 其中 `auto_aim`/`auto_buff`/`omniperception` 三个脚本各自**重复**写了一遍 `set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")`。

### 2.3 `io/` —— 硬件 I/O 层

| 路径 | 概要 |
| --- | --- |
| `io/CMakeLists.txt` | 编 `io` STATIC 库；按 `CMAKE_SYSTEM_PROCESSOR` 选 `amd64`/`arm64` 库目录（未知架构 `FATAL_ERROR`）；ROS2 找到时追加 `ros2/*.cpp` |
| `camera.hpp` / `camera.cpp` | 相机工厂（PIMPL 门面）：`CameraBase::read(img, timestamp)` 纯虚接口；`Camera(config_path)` 按 `camera_name` 创建 `mindvision`/`hikrobot`/`daheng`，未知品牌抛异常 |
| `command.hpp` | `struct Command { bool control, shoot; double yaw, pitch, horizon_distance; }` —— Aimer / Planner / CommandGener 三种决策器的统一输出 |
| `cboard.hpp` / `cboard.cpp` | RoboMaster C 板（CAN，legacy）：`Mode{idle,auto_aim,small_buff,big_buff,outpost}`、`ShootMode`；`imu_at(t)` 按时间戳 slerp 插值四元数；`send(Command)` 打包 yaw/pitch 定点 `int16(×1e4)` 成 8 字节 CAN 帧 |
| `socketcan.hpp` | SocketCAN 纯头文件封装：`socket(PF_CAN)` + `ioctl(SIOCGIFINDEX)` + epoll 读线程 + 每 100ms 巡检的守护重连线程 |
| `gimbal/gimbal.hpp` / `gimbal.cpp` | **当前主力输出**：串口协议 **v3.0**（2027 赛季 lt-2027 哨兵下位机）`[0xFF][MODE][PAYLOAD][CRC8][0x0D]`，921600 8N1；`UsbCtrl`（28 B 视觉→云台：绝对角 + 速度/加速度前馈，**度制**）、`UsbAutoSendToNuc`（42 B 云台→视觉：mode/四元数/yaw,yaw_vel,pitch,pitch_vel/bullet_speed/bullet_count，度制）、逻辑帧 `VisionToGimbal`（弧度）；`crc8()` 与下位机查表等价；`q(t)` 时间对齐插值、`state()` 快照、七项诊断；请求-响应链路上带 20 Hz 空闲心跳、构造期「敲门 + 2 s 上限」 |
| `gimbal/laser_gimbal.hpp` / `.cpp` | Lracking 激光云台协议：**度制 + 绝对角度**（API 层转弧度），仅 `laser_gimbal_test` 使用 |
| `hikrobot/` | 海康 `MvCameraControl` 封装（`HikRobot : CameraBase`）：SDK 句柄 + 取图线程 + 守护线程（100ms 巡检，USB 断线用 libusb 复位重开）；`include/`、`lib/{amd64,arm64}` 为 vendored SDK |
| `mindvision/` | 迈德 `MVSDK` 同构封装（`CameraSdkInit/CameraPlay/CameraGetImageBuffer/CameraImageProcess`） |
| `daheng/` | 大恒 `gxiapi` 封装（`GXInitLib/GXOpenDevice/GXStreamOn/GXDQBufEx`），SDK 带引用计数可多实例；`lib/x86_64` 内已含 `libgxiapi.so` 与 ffmpeg 依赖链 |
| `usbcamera/usbcamera.hpp` / `.cpp` | V4L2 USB 相机（`cv::VideoCapture` 打开 `/dev/video*`），**不继承 `CameraBase`**；哨兵 4 路全向感知用，`device_name` 由清晰度特征判定 |
| `dm_imu/dm_imu.hpp` / `.cpp` | DM IMU 串口直连驱动（Modbus 风格三帧 + CRC → 四元数，`imu_at` slerp）；仅 `dm_test` 使用 |
| `ros2/` | 哨兵导航桥：`publish2nav`（目标位置）、`subscribe2nav`（无敌状态/自瞄目标）、`ros2.cpp` 节点封装；ROS2 缺失时不编译 |
| `serial/` | vendored libserial（MIT，PIMPL：`Serial` → `SerialImpl`），提供 `Serial`、`Timeout`、`list_ports()` |

> 本目录的逐文件函数说明见 [`doc/code/io/gimbal/`](code/io/gimbal/README.md)（已覆盖 `gimbal` 与 `laser_gimbal`）。

### 2.4 `tasks/` —— 算法层

#### 2.4.1 `tasks/auto_aim/`（自瞄主流程）

| 路径 | 概要 |
| --- | --- |
| `armor.hpp` / `armor.cpp` | 全流水线的"货币"：`Color/ArmorType/ArmorName/ArmorPriority` 枚举；`Lightbar`（中心/上下端点/角点/角度/长宽/长宽比/旋转矩形）、`Armor`（左右灯条/颜色/大小/编号/优先级/像素中心/四角点/几何校验量/置信度/**云台系与世界系的 xyz、ypr、ypd**）；两种构造：传统灯条配对、NN 关键点（yv8/yv11/yv5） |
| `detector.hpp` / `.cpp` | 传统 CV 装甲板检测：二值化 → 灯条提取 → 几何校验 → 两两配对 → 分类器识别编号 → 去重；另提供对已有装甲板 ROI 重提取灯条修正角点。`detector.cpp.bak` 为本地备份残留 |
| `classifier.hpp` / `.cpp` | 装甲板数字分类：`classify()`（`cv::dnn`）+ `ovclassify()`（OpenVINO AUTO/LATENCY），32×32 图案 → 编号 |
| `solver.hpp` / `.cpp` | PnP 位姿解算与坐标变换：`solve(armor)` 由四角点反推位姿 → `相机系→云台系→世界系` → 填 `xyz/ypr/ypd`；`optimize_yaw()` 在 ±140° 内逐度搜索最小重投影误差；另有 `reproject_armor`、`world2pixel`、前哨站顶/面区分。`solver.md` 为本地笔记 |
| `target.hpp` / `.cpp` | 单目标 **11 维 EKF**（位置/速度/朝向/角速度/半径/长轴差/高度差）：`predict`（常加速度+常角速度）、`update`（匹配装甲板 id + ypd 观测）、`armor_xyza_list()`、`diverged/convergened/checkinit`；处理"切板跳变" |
| `tracker.hpp` / `.cpp` | 目标跟踪状态机 `LOST→DETECTING→TRACKING→TEMP_LOST→SWITCHING`：`state_machine()`、`set_target()`（按兵种定半径/板数/P0）、`update_target()`；另有全向感知重载（多相机目标切换）。`tracker.md` 为本地笔记 |
| `aimer.hpp` / `.cpp` | legacy 决策器：`aim()` = 预测 → 选瞄准点（跟板 or 打旋转中心）→ 弹道迭代（≤10 次）→ 输出 `io::Command` |
| `shooter.hpp` / `.cpp` | 发射授权：距离分档容差 + 指令突变抑制（云台未到位不开火）+ 自动开火开关 |
| `voter.hpp` / `.cpp` | 对 (颜色, 名称, 类型) 三元组投票计数（噪声统计/方向投票） |
| `yolo.hpp` / `yolo.cpp` | YOLO 策略模式门面：`YOLOBase` 抽象 + `YOLO(config_path)` 按 `yolo_name` 创建后端，上层只调 `detect()` |
| `yolos/yolov5.*` | YOLOv5 后端：640×640，22 维行输出（xywh+置信度+4 色+9 编号），可选 ROI 与传统检测器二次矫正。`.cpp.bak` 备份残留 |
| `yolos/yolov8.*` | YOLOv8 后端：416×416，2 类 + 8 关键点，关键点排序 + `Classifier` 识别编号。`.cpp.bak` 备份残留 |
| `yolos/yolo11.*` | YOLO11 后端：640×640，38 类（颜色×编号×大小）+ 4 关键点，一次输出带编号装甲板。`.cpp.bak` 备份残留 |
| `planner/planner.hpp` / `.cpp` | **MPC 轨迹规划（现代决策核心）**：`DT=0.01`、`HALF_HORIZON=50`、`HORIZON=100`，`Trajectory` 4×100（yaw/yaw_vel/pitch/pitch_vel）；生成目标轨迹 → yaw/pitch 两个 TinyMPC 求解器 → `Plan{control,fire,target_yaw/pitch,yaw(+vel,acc),pitch(+vel,acc)}` |
| `planner/tinympc/` | vendored TinyMPC（ADMM，C API）：`admm.cpp`（forward/backward pass、slack、dual、termination）、`tiny_api.cpp`（`tiny_setup/tiny_set_x0/tiny_solve/tiny_set_bound_constraints`）、`codegen.*`（未启用）、`rho_benchmark.*`、`types.hpp`、`tiny_api_constants.hpp` |
| `multithread/mt_detector.*` | 异步 YOLO 检测：`push` 预处理 + `start_async` 入队（16 槽）、`pop` 同步取结果，推理与主循环解耦 |
| `multithread/commandgener.*` | 独立后台线程（~500Hz）消费最新目标，跑 Aimer+Shooter 并经 CBoard 发送；`mt_standard`/`mt_auto_aim_debug` 使用 |

#### 2.4.2 `tasks/auto_buff/`（能量机关/打符）

| 路径 | 概要 |
| --- | --- |
| `buff_type.hpp` / `.cpp` | `PowerRune_type{SMALL,BIG}`、`FanBlade_type{_target,_unlight,_light}`、`Track_status`；`FanBlade`（扇叶关键点/角度/宽高/类型）、`PowerRune`（旋转中心/扇叶序列/亮灯数/世界系 xyz/ypr/ypd） |
| `buff_detector.hpp` / `.cpp` | YOLO 检测扇叶 + 传统方法定旋转中心，维护 `TRACK/TEM_LOSE/LOSE` 状态（`LOSE_MAX=20`） |
| `yolo11_buff.hpp` / `.cpp` | 打符专用 YOLO11（OpenVINO）：输出 buff/旋转中心两类 + 关键点，提供多候选与单候选两种取框 |
| `buff_solver.hpp` / `.cpp` | 能量机关 PnP 解算：`OBJECT_POINTS` 为 3D 模型点，`THETA = 2π/5` 扇叶夹角，解世界系位置与旋转角；带重投影调试 |
| `buff_target.hpp` / `.cpp` | 打符 EKF：基类 `Target` + `SmallTarget`（小符转速先验 `SMALL_W=π/3`）+ `BigTarget`（大符用 `RansacSineFitter` 动态估转速）；含旋转方向投票 |
| `buff_aimer.hpp` / `.cpp` | 打符决策：`aim()`（legacy 角度）+ `mpc_aim()`（MPC 轨迹，输出 `auto_aim::Plan`） |

#### 2.4.3 `tasks/omniperception/`（哨兵全向感知）

| 路径 | 概要 |
| --- | --- |
| `detection.hpp` | `DetectionResult{ armors, timestamp, delta_yaw, delta_pitch }` —— 单路相机的检测结果与相对云台的视差 |
| `perceptron.hpp` / `.cpp` | 4 路 USB 相机各配一个 YOLO，逐路线程并行推理，结果入队 |
| `decider.hpp` / `.cpp` | 多相机结果**排序（优先级 mode1/mode2）、过滤（无敌/噪声）、决策**，输出 `io::Command`；`get_target_info()` 供导航 |

### 2.5 `src/` —— 可执行入口

| 文件 | 用途 | 通信 | 检测 | 决策 |
| --- | --- | --- | --- | --- |
| `standard.cpp` | 步兵（legacy） | CBoard CAN | YOLO | Aimer |
| `standard_mpc.cpp` | 步兵（现代，**推荐入门阅读**） | Gimbal 串口 | YOLO | Planner (MPC) + 打符 |
| `mt_standard.cpp` | 步兵（多线程） | CBoard CAN | 异步 `MultiThreadDetector` | CommandGener |
| `uav.cpp` | 无人机 | CBoard CAN | 传统 CV | Aimer + 打符 |
| `uav_debug.cpp` | 无人机调试 | CBoard CAN | 传统 CV + YOLO | Aimer + 可视化 |
| `auto_aim_debug_mpc.cpp` | 自瞄 MPC 调试 | Gimbal 串口 | YOLO | Planner + 曲线 |
| `mt_auto_aim_debug.cpp` | 多线程自瞄调试 | CBoard CAN | `MultiThreadDetector` | CommandGener |
| `auto_buff_debug.cpp` | 打符调试（CAN） | CBoard CAN | YOLO | Buff Aimer |
| `auto_buff_debug_mpc.cpp` | 打符调试（MPC） | Gimbal 串口 | YOLO | Buff `mpc_aim` |
| `sentry.cpp` | 哨兵 | CBoard CAN | YOLO + USB 多相机 | Decider + Aimer + ROS2 |
| `sentry_bp.cpp` | 哨兵（仅后视） | CBoard CAN | YOLO + 后视相机 | Decider + Aimer |
| `sentry_debug.cpp` | 哨兵调试 | CBoard CAN | 全功能 + 可视化 + PlotJuggler | Decider + Aimer |
| `sentry_multithread.cpp` | 哨兵多线程 | CBoard CAN | 4 路 USB + Perceptron | Decider（含 switching） |

> 后 4 个 `sentry*` 仅在 ROS2 环境齐全时才被编译。

### 2.6 `calibration/` —— 标定与离线工具

| 文件 | 概要 |
| --- | --- |
| `capture.cpp` | 采标定数据：按 `s` 存图 + 对应 IMU 四元数 |
| `capture_daheng.cpp` | 大恒相机定时连拍采集（链接 `gxiapi dxmediaproc usb-1.0`） |
| `calibrate_camera.cpp` | 相机内参标定 → `camera_matrix` + `distort_coeffs`（`cv::calibrateCamera` / `cv::findCirclesGrid`） |
| `calibrate_handeye.cpp` | 手眼标定 → `R_camera2gimbal` + `t_camera2gimbal` |
| `calibrate_robotworld_handeye.cpp` | 机器人世界系联合手眼标定（额外输出标定板位置） |
| `split_video.cpp` | 视频切片（截取录制视频的指定帧区间） |

> 流程：`capture` 采数据 → `calibrate_camera` 出内参 → 内参喂 `calibrate_handeye` 出手眼外参 → 写回 `configs/*.yaml`。

### 2.7 `tests/` —— 测试程序（无框架，各自独立 `main()`）

| 分类 | 文件 |
| --- | --- |
| 无硬件 | `auto_aim_test`（回放视频，**最简冒烟**）、`auto_buff_test`、`detector_video_test`、`planner_test_offline` |
| 单硬件 | `camera_test`、`camera_detect_test`、`usbcamera_test`、`usbcamera_detect_test`、`cboard_test`、`gimbal_test`、`gimbal_response_test`、`laser_gimbal_test`、`dm_test`、`fire_test`、`handeye_test` |
| 组合/多线程 | `camera_thread_test`（线程池 + 有序队列；其 `Frame` 与 `create_yolos` 在文件内匿名 namespace 自定义，因 `tools` 已按分层原则删除 `Frame`/`create_yolov8s`，见 [troubleshooting B-6](troubleshooting.md)）、`multi_usbcamera_test`、`minimum_vision_system`、`planner_test`（接真实云台） |
| ROS2（条件编译） | `publish_test`、`subscribe_test`、`topic_loop_test` |

### 2.8 `configs/` —— 运行期配置

| 文件 | 概要 |
| --- | --- |
| `standard3.yaml` / `standard4.yaml` | 两名步兵的整机配置 |
| `sentry.yaml` / `uav.yaml` / `ascento.yaml` | 哨兵 / 无人机 / 平衡步兵 |
| `camera.yaml` / `mvs.yaml` / `daheng.yaml` | 相机参数（迈德 / 海康 / 大恒） |
| `calibration.yaml` / `calibration_camera.yaml` / `calibrate_camera.yaml` | 标定输入与产物 |
| `demo.yaml` / `example.yaml` | 示例/演示配置 |

> 所有类的构造函数都接收 config 路径并**内部解析 YAML**；调参基本只改这里。配置节包括 `enemy_color`、神经网络（模型路径/设备/阈值）、检测器阈值、相机参数、标定矩阵、CAN ID、tracker 阈值、aimer/shooter/planner、buff 参数。

### 2.9 `tools/` —— 已迁移的底层工具与函数（来源：sp_vision_25）

| 路径 | 概要 |
| --- | --- |
| `tools/CMakeLists.txt` | 把 11 个 `.cpp` 编成 `tools` OBJECT 库；`PUBLIC` 链接 Eigen3、spdlog、yaml-cpp、nlohmann_json、fmt、OpenCV、Threads；`PRIVATE -Wall -Wextra` |
| `crc.hpp` / `crc.cpp` | CRC8/CRC16 校验：`get_crc8`/`check_crc8`/`get_crc16`/`check_crc16`，用于下位机通信帧校验 |
| `exiter.hpp` / `exiter.cpp` | `Exiter`：构造时注册 SIGINT 处理，`exit()` 返回是否收到 Ctrl+C，供主循环优雅退出 |
| `extended_kalman_filter.hpp` / `.cpp` | `ExtendedKalmanFilter`：线性/非线性 `predict` 与 `update` 重载，含 NIS/NEES 卡方检验与失败滑动窗口 |
| `img_tools.hpp` / `img_tools.cpp` | OpenCV 绘制辅助：`draw_point`/`draw_points`（Point 与 Point2f 两个重载）/`draw_text` |
| `logger.hpp` / `logger.cpp` | `logger()`：spdlog 全局日志器单例（文件 sink + 彩色控制台 sink，info 触发 flush） |
| `math_tools.hpp` / `math_tools.cpp` | 纯数学工具：`limit_rad`、`eulers`、`rotation_matrix`、`xyz2ypd`/`ypd2xyz` 及雅可比、`delta_time`、`get_abs_angle`、`square`、`limit_min_max` |
| `pid.hpp` / `pid.cpp` | `PID`：位置式，支持角度模式（误差经 `limit_rad` 归一），输出与积分限幅，暴露 p/i/d 分量 |
| `plotter.hpp` / `plotter.cpp` | `Plotter`：`nlohmann::json` 经 UDP 发给绘图端（默认 127.0.0.1:9870） |
| `ransac_sine_fitter.hpp` / `.cpp` | `RansacSineFitter`：对 (t, v) 做 `A·sin(ωt+φ)+C` 的 RANSAC 拟合，输出幅值/角频率/相位/偏置/内点数 |
| `recorder.hpp` / `recorder.cpp` | `Recorder`：图像 + 云台四元数录成 `.avi` + `.txt`，内部独立保存线程 + 容量 1 队列 |
| `thread_pool.hpp` | `ThreadPool`（固定线程数任务池）+ `OrderedQueue<FrameT>`（按 `id` 保序缓冲队列）。**与源文件有解耦差异**，见 [差异汇总](code/tools/README.md#与-sp_vision_25-的差异汇总) |
| `thread_safe_queue.hpp` | `ThreadSafeQueue<T, PopWhenFull>`：带最大容量、满队列策略与回调的线程安全队列模板 |
| `trajectory.hpp` / `trajectory.cpp` | `Trajectory`：不考虑空气阻力的抛体弹道解算（初速/水平距离/高度 → pitch 与飞行时间，含无解判定） |
| `yaml.hpp` | `tools::load` / `tools::read`：YAML 读取辅助，键缺失记日志并 `exit(1)`（带默认值重载可回退） |

> 函数级说明见 [`code/tools/`](code/tools/README.md)（25 篇，与源文件一一对应）。

### 2.10 已清理的阶段 0 计划骨架目录（2026-10-17 删除）

阶段 0 曾按「auto_function / main / measure / test / yolo」规划目录，这些目录只有 `.gitkeep` 占位。上游代码拷入后职能均已由实际结构覆盖，**已于 2026-10-17 全部删除**，映射关系如下（备查）：

| 已删除路径 | 原规划 | 现在的实际位置 |
| --- | --- | --- |
| `auto_function/auto_aim/`、`auto_function/auto_buff/` | 自瞄 / 打符 | `tasks/auto_aim/`、`tasks/auto_buff/` |
| `io/driver/`、`io/uart/` | 相机驱动 / 串口 | `io/hikrobot|mindvision|daheng|usbcamera/`、`io/serial/`、`io/gimbal/` |
| `main/` | 程序入口 | `src/`（13 个 `main`） |
| `measure/` | 标定测量 | `calibration/` + `configs/` |
| `test/` | 测试程序 | `tests/`（22 个独立 `main`） |
| `yolo/` | 推理封装 | `tasks/auto_aim/yolo*`、`classifier`、`mt_detector` |
| `troubleshooting-doc-pattern/` | 无（空目录残留） | — |

同时删除的还有 `configs/.gitkeep`、`src/.gitkeep`、`tests/.gitkeep`（这三个目录已有真实文件，不再需要占位）；
`doc/code/` 下对应的旧占位目录（`main/`、`auto_function/`、`io/driver/`、`io/uart/`、`measure/`、`test/`、`yolo/`）一并删除。

> 现在仓库里**没有任何 `.gitkeep`**：所有目录都因有真实文件而被 git 跟踪。

### 2.11 `assets/` 与其它资源

| 路径 | 概要 |
| --- | --- |
| `assets/yolo11.xml|.bin`、`yolov5.*`、`yolov8.*`、`yolo11_buff_int8.*` | OpenVINO IR 模型（XML 结构 + BIN 权重），对应三个检测后端与打符模型 |
| `assets/color_track_model.onnx|.xml|.bin`、`tiny_resnet.onnx`、`best2-sim.onnx` | 颜色追踪/分类相关模型（`color_track_demo` 用 ONNX） |
| `assets/demo/`、`img-temp/`、`img_with_q/`、`captured_20260725_172207/`、`PinkTraining/` | 演示素材、临时图、带四元数的标定图、采集批次、训练数据集（约 2.1 GB） |
| `.clang-format`、`LICENSE`、`autostart.sh`、`buff_layout.xml`、`mpc_layout.xml` | 见 §2.1 |

### 2.12 文档目录

| 路径 | 概要 |
| --- | --- |
| `doc/` | 文档体系根目录 |
| `doc/README.md` | 文档索引与维护规则摘要 |
| `doc/overview.md` | 项目总体详细说明：定位、总目标、阶段划分、模块划分、设计原则、构建与依赖、参考项目与资料、待确认事项 |
| `doc/directory_tree.md` | 本文件：项目目录树与每个文件/文件夹的概要 |
| `doc/executables.md` | **可执行程序一览**：42 个 `add_executable`（本机产出 35 个，7 个 ROS2 目标条件跳过）的作用、命令行参数与默认值、硬件依赖，附「按需求找程序」速查表 |
| `doc/troubleshooting.md` | **问题与解决方案**：含按症状检索的快速索引与条目模板。遇到问题先查这里；解决新问题后必须追加条目 |
| `doc/code/` | 逐文件函数作用说明 |
| `doc/code/README.md` | 逐文件说明的命名约定、文档模板、维护规则 |
| `doc/code/tools/README.md` + 25 篇 | tools 文件清单、概要、与 sp_vision_25 的差异汇总 |
| `doc/code/io/gimbal/README.md` + 4 篇 | `gimbal.hpp/.cpp`、`laser_gimbal.hpp/.cpp` 的函数作用与已知问题 |
| `doc/code/configs/README.md` | 占位（13 份 YAML，无 C++ 源文件） |

## 3. 变更记录

| 日期 | 变更 |
| --- | --- |
| 2026-10-11 | 建立 `AGENT.md`、`.gitignore`、`doc/` 文档体系；源码目录以 `.gitkeep` 占位 |
| 2026-10-11 | 更正命名：`auto_fuction/`→`auto_function/`、`cmakelists.txt`→`CMakeLists.txt` |
| 2026-10-11 | **阶段 1 完成**：自 sp_vision_25 迁移 `tools/` 全部 25 个源文件并补充中文注释；新增 `CMakeLists.txt` 与 `tools/CMakeLists.txt`；新增 `doc/code/tools/` 25 篇逐文件说明 |
| 2026-10-11 | 新增 `doc/troubleshooting.md`（17 条问题与解决方案 + 快速索引）；`AGENT.md` 新增 §6「问题排查：先查文档，不要重复思考」 |
| 2026-10-17 | **sp_vision_25 剩余部分全量拷入**：新增 `io/`（99 文件）、`tasks/`（71）、`src/`（13）、`calibration/`（6）、`tests/`（22）、`configs/`（13 YAML）、`assets/`（2.2 GB）及 `.clang-format`、`LICENSE`、`autostart.sh`、`buff_layout.xml`、`mpc_layout.xml` |
| 2026-10-17 | 顶层 `CMakeLists.txt`、`.gitignore`、`readme.md` 被上游同名文件覆盖（`tools/` 未被覆盖，仍为注释版）；见 [troubleshooting A-5/A-7/D-5](troubleshooting.md) |
| 2026-10-17 | **清理计划骨架遗留目录**：删除 `auto_function/`、`main/`、`measure/`、`test/`、`yolo/`、`io/driver/`、`io/uart/`、`troubleshooting-doc-pattern/` 与三个多余 `.gitkeep`；同步删除 `doc/code/` 下 7 个旧占位目录（见 §2.10） |
| 2026-10-17 | 新增 `doc/code/io/gimbal/`（README + 4 篇逐文件说明）；`doc/code/README.md` 映射表改为反映真实覆盖情况 |
| 2026-10-17 | 按实际结构重写本文件（目录树 + §2.2 构建目标一览 + `io/ tasks/ src/ calibration/ tests/ configs/` 逐文件概要 + §2.10 骨架遗留目录） |
