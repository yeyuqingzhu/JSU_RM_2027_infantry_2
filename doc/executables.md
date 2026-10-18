# 可执行程序一览（作用 · 用法 · 依赖）

本文件按 [`CMakeLists.txt`](../CMakeLists.txt) 的 `add_executable` 逐条列出本项目所有可执行程序。
**构建产物目录是 `build_full/`**（`build/` 是阶段 1 遗留目录，只有一个 `smoke_tools`，不要用）。

- 顶层共 **42** 个 `add_executable`，本机实际产出 **35** 个可执行：7 个 ROS2 程序因 `find_package(ament_cmake QUIET)` 未命中而整块跳过（见 §四）。
- 每个程序都是 OpenCV `CommandLineParser` 风格：`./build_full/<程序> --help` 打印参数说明，**位置参数（表中写作 `@xxx`）直接跟在程序名后面**。
- 绝大多数程序的唯一必需输入是一份 **yaml 配置**（相机/串口型号、模型路径、标定参数），参数留空的程序必须显式传配置路径。
- 依赖列里的「相机 / CBoard / Gimbal」指真实硬件；没有硬件时这些程序会在构造阶段抛异常或直接卡住，属于环境问题而非构建问题（见 [troubleshooting.md](troubleshooting.md) B 节与 `GXInitLib failed` 条目）。

## 通用用法示例

```bash
cd <仓库根目录>
./build_full/standard --help                          # 打印参数说明
./build_full/standard                                 # 用默认配置 configs/standard3.yaml
./build_full/standard configs/standard4.yaml          # 位置参数覆盖配置
./build_full/auto_aim_test -c configs/demo.yaml assets/demo/demo.avi   # 命名参数 + 位置参数混用
```

---

## 一、主流程程序（`src/`，9 个）

| 程序 | 作用 | 用法 | 硬件依赖 |
| --- | --- | --- | --- |
| `standard` | **单线程主流程**：相机取流 → YOLO 检测 → Solver → Tracker → Aimer/Shooter → `io::CBoard` 下发云台与弹丸指令 | `standard [configs/standard3.yaml]`（默认 `configs/standard3.yaml`） | 相机 + CBoard串口 |
| `mt_standard` | **多线程主流程**：`mt_detector` 检测线程 + `commandgener`，带自瞄/打符模式切换（日志 `Switch to {}`） | `mt_standard <config.yaml>`（**无默认值，必填**） | 相机 + CBoard（可选 DM IMU） |
| `standard_mpc` | **MPC 弹道主流程**：`io::Gimbal` + `auto_aim::Planner` 预瞄，含打符分支；`docs/auto_aim_算法逐行解析.md` 讲的就是它 | `standard_mpc <config.yaml>`（**必填**） | 相机 + Gimbal串口 |
| `auto_aim_debug_mpc` | 自瞄 **MPC 调试版**：全链路 + `imshow` 可视化（画检测框/预测点） | `auto_aim_debug_mpc [configs/sentry.yaml]` | 相机 + Gimbal |
| `mt_auto_aim_debug` | 多线程自瞄 **调试版**：带 `tools::Recorder` 录像与可视化 | `mt_auto_aim_debug [configs/sentry.yaml]` | 相机 + CBoard |
| `auto_buff_debug` | **打符调试版**（CBoard 控制）：`Buff_Detector` → `Solver` → `Aimer` + 可视化 | `auto_buff_debug <config.yaml>`（**必填**） | 相机 + CBoard |
| `auto_buff_debug_mpc` | **打符调试版**（Gimbal 控制） | `auto_buff_debug_mpc <config.yaml>`（**必填**） | 相机 + Gimbal |
| `uav` | **无人机版主流程**：`Detector`/`YOLO` + Tracker + Aimer + 打符分支 | `uav [configs/uav.yaml]` | 相机 + CBoard |
| `uav_debug` | 无人机版 **调试可视化** | `uav_debug [configs/uav.yaml]` | 相机 + CBoard |

> `standard` / `mt_standard` / `standard_mpc` 三点区别：单线程 vs 多线程 vs MPC 预瞄弹道；
> `*_mpc` 与 `*_debug` 后缀分别表示「用 `io::Gimbal`（新云台驱动）+ Planner」与「带 imshow 可视化」。

## 二、标定与素材（`calibration/`，6 个）

标定流程：`capture` 采集 → `calibrate_camera`（内参）→ `calibrate_handeye` / `calibrate_robotworld_handeye`（手眼）。

| 程序 | 作用 | 用法 | 硬件依赖 |
| --- | --- | --- | --- |
| `capture` | **采集标定图**：开相机+云台，按键把当前帧与姿态四元数（顺序 wxyz）存进输出目录 | `capture -c configs/calibration.yaml -o assets/img_with_q`（默认值即此） | 相机 + Gimbal |
| `calibrate_camera` | **相机内参 + 畸变**标定：读 `img_with_q/` 里的标定板图（默认 10 列 7 行），产出相机矩阵/畸变系数 | `calibrate_camera -c configs/calibration.yaml assets/img_with_q` | 无 |
| `calibrate_handeye` | **相机—云台手眼标定**（`R_camera2gimbal`）：用 `R_gimbal2world_list` 与图像求解 | `calibrate_handeye -c configs/calibration.yaml assets/img_with_q` | 无 |
| `calibrate_robotworld_handeye` | **世界—云台手眼标定**（`R_world2gimbal`）：求解机器人世界系到云台系的旋转 | `calibrate_robotworld_handeye -c configs/calibration.yaml assets/img_with_q` | 无 |
| `capture_daheng` | **大恒相机采集 demo**（参数写在 `const` 里，无命令行参数） | `capture_daheng` | 大恒相机（无相机会 `GXInitLib failed`） |
| `split_video` | **视频切片**：把 `xxx.avi` + `xxx.txt`（标注）切成逐帧图片与标签，可裁时间范围 | `split_video -s 0 -e 300 -p records/out records/Big/xxx.avi` | 无 |

> 参数注意：`calibrate_*` 的配置用**命名参数** `-c/--config-path`，输入目录是位置参数；
> `capture` 的 `-c` 与 `-o` 都有默认值，直接运行即可。

## 三、测试与联调程序（`tests/` 19 个 + `color_track_demo/` 1 个）

### 3.1 离线视频链路（无需硬件）

| 程序 | 作用 | 用法 |
| --- | --- | --- |
| `auto_aim_test` | 离线视频跑**自瞄全链路**（YOLO→Solver→Tracker→Aimer），逐帧可视化 | `auto_aim_test -c configs/demo.yaml -s 0 -e 0 assets/demo/demo.avi` |
| `auto_buff_test` | 离线视频跑**打符链路**（Buff_Detector→Solver→Aimer） | `auto_buff_test -c configs/sentry.yaml assets/demo/xxx.avi` |
| `detector_video_test` | 只跑**检测器**：默认 YOLO，`-t` 切传统视觉检测 | `detector_video_test -c configs/sentry.yaml -t 0 assets/demo/xxx.avi` |
| `planner_test_offline` | **MPC 规划离线验证**：给定目标距离 `-d` 与角速度 `-w`，不发云台指令，只打印/曲线 | `planner_test_offline -d 3.0 -w 5.0 <config.yaml>` |

### 3.2 相机取流

| 程序 | 作用 | 用法 | 硬件依赖 |
| --- | --- | --- | --- |
| `camera_test` | 相机取流 + fps 打印，`-d` 开窗口 | `camera_test -c configs/camera.yaml -d` | 相机 |
| `camera_detect_test` | 相机取流 + 检测（`-t` 传统方法），打印 fps | `camera_detect_test [configs/sentry.yaml] -t` | 相机 |
| `camera_thread_test` | **线程池 + 有序队列**演示：取流线程 → `tools::ThreadPool` → `tools::OrderedQueue<Frame>` → 检测，按 id 保序输出。`Frame`/`create_yolos` 在本文件匿名 namespace 自定义（见 [troubleshooting B-6](troubleshooting.md)） | `camera_thread_test [configs/ascento.yaml]` | 相机 + CBoard |
| `multi_usbcamera_test` | **多路 USB 相机**同时取流 | `multi_usbcamera_test [configs/sentry.yaml] -d` | 多路 USB 相机 |
| `usbcamera_test` | 单路 USB 相机取流（`-n` 指定 `/dev/videoN`） | `usbcamera_test -n video0 [configs/sentry.yaml] -d` | USB 相机 |
| `usbcamera_detect_test` | USB 相机取流 + YOLO 检测 | `usbcamera_detect_test -n video0 -d` | USB 相机 |

### 3.3 串口 / 云台 / IMU

| 程序 | 作用 | 用法 | 硬件依赖 |
| --- | --- | --- | --- |
| `cboard_test` | 读 **CBoard** 上报：陀螺 z/y/x 角度、弹速 `bullet speed` | `cboard_test [config.yaml]`（默认 `configs/standard.yaml`，**本仓库不存在，必须显式传**） | CBoard |
| `gimbal_test` | **云台串口链路自检 + 控制**：走 `io::Gimbal`（串口 v3.0，28 B/42 B）。每 9 ms 发一帧 `mode=1`、**绝对角** `yaw=1 rad` / `pitch=0`（固定值，不闭环）；`-f` 时按 1 s 静默 / 0.2 s 开火的占空比切成 `mode=2`；打印模式变化与首次开火耗时（`bullet_count` 增加即判定已发弹）；Plotter 画 `q_yaw/q_pitch/yaw/vyaw/pitch/vpitch/bullet_speed/bullet_count/fired/fire/t`；退出前补发 `mode=0`。**2026-10-18 起不再被首帧卡死**：用 `Gimbal::try_q(t, q, 20)` 取姿态，拿不到就每秒打印一条 `仍在等待首帧: rx_bytes=… tx_frames=… errors=…`，且**照常每 9 ms 发帧**（请求-响应链路靠它保活）；日志在 `logs/tests/gimbal_test.cpp/` 与 `logs/io/gimbal/gimbal.cpp/` | `gimbal_test [-f] <config.yaml>`；**yaml 必须含 `com_port`**（`configs/sentry.yaml` 不含 → 启动即 `[YAML] com_port not found!` 退出） | `io::Gimbal`（USART1 TTL 921600） |
| `gimbal_response_test` | **云台跟随响应测试**：走 `io::CBoard`（SocketCAN）+ `io::Command`，**完全不用 `io::Gimbal`**。按 `-m` 注入目标角，比较「下发指令 vs 实测姿态」：`triangle_wave`（默认，名字如此但**实际是锯齿**——每帧 `cmd += (-a)/(-c×100)` 度、累加到 `-a` 后归零，节拍 `sleep 8 ms`≈100 fps；`-a` 幅值 8、`-c` 周期 0.2 s、`-x` 选 yaw/pitch）、`step`（**只动 yaw**，每 300 帧 ≈2.4 s 阶跃 `-a` 度并 `limit_rad` 限幅 → 看阶跃响应/延迟，`-x`/`-c` 无效）、`circle`（`A=7°`、`T=4 s` 的圆，`pitch = A·sin(2πt/T + π/2) + 18`，`-a/-c/-x` 全部无效；注意 `t` 每 2 s 被额外 `+= 2.4` 一次，轨迹相位会被人为跳变）；开头先发 `io::Command{1,0,0,0}` 并 `sleep 5 s` 等云台归零；姿态反馈来自 CAN 四元数帧（`quaternion_canid`，经 `cboard.imu_at()`；**CAN 不通时构造期就永久阻塞，连 `main` 循环都进不去，一个帧都不会发出**）；`io::CBoard` 只有 `SocketCAN can_` 成员、**没有串口路径**，只接 UART 时它必然无法工作；Plotter 画 `cmd_yaw/cmd_pitch` vs `gimbal_yaw/gimbal_pitch` | `gimbal_response_test -m triangle_wave -a 8 -c 0.2 -x yaw [configs/sentry.yaml]`（yaml 需含 `can_interface`） | `io::CBoard`（CAN / `can0`） |
| `fire_test` | **开火测试**：`io::Gimbal` 单次/连续开火 | `fire_test <config.yaml>`（**必填**） | Gimbal |
| `laser_gimbal_test` | **激光云台串口自检**：打印端口/波特率，打开失败会报 `Failed to open laser gimbal on {}` | `laser_gimbal_test -p /dev/ttyUSB0 -b 115200`（位置参数 `<port>` 优先） | 激光云台（串口） |
| `planner_test` | **MPC 规划接真云台**：给定 `-d` 距离、`-w` 角速度，输出并下发云台角 | `planner_test -d 3.0 -w 5.0 <config.yaml>` | Gimbal |
| `dm_test` | **达妙 IMU 自检**：打印 z/y/x 角度 | `dm_test`（无命令行参数） | DM IMU |
| `handeye_test` | **手眼标定现场验证**：读 `configs/handeye.yaml` 的 `height`/`grid_num`/`grid_size`，用相机+CBoard 现场测角度误差 | `handeye_test -c <config.yaml> -d`（默认 `configs/handeye.yaml`，**本仓库不存在，必须显式传**） | 相机 + CBoard |

### 3.4 端到端

| 程序 | 作用 | 用法 | 硬件依赖 |
| --- | --- | --- | --- |
| `minimum_vision_system` | **最小视觉系统**：相机 → 检测 → 跟踪 → 解算 → 发射的最小可跑闭环，适合新硬件首次联调 | `minimum_vision_system <config.yaml>`（**必填**） | 相机 + CBoard + DM IMU |
| `color_track_demo` | **颜色跟踪 demo**：大恒相机 + 激光云台，自带 ONNX Runtime 推理（CUDA/CPU 回退）与 Ultralytics `[1,N,6]` 输出解析，处理 `color_track_model.onnx` | `color_track_demo`（无命令行参数，参数写在源码内） | 大恒相机 + 激光云台 |

> 本项目 `color_track_demo/main.cpp` 是 520 行的独立第三方 demo，**没有 `--help` 分支**，直接运行就进相机初始化，无相机会以 `GXInitLib failed` 退出（rc=134）。

## 四、ROS2 条件编译程序（7 个，本机不构建）

`CMakeLists.txt:127-137` 的整块被 `find_package(ament_cmake QUIET)` 守卫，本机没有 `/opt/ros` 时 configure 阶段打印
`-- ROS2 environment not found, skipping ROS2-related code.`，这 7 个目标不产生可执行文件。

| 程序 | 作用 | 依赖 |
| --- | --- | --- |
| `sentry` | 哨兵主流程：双相机 + `omniperception::Decider` 决策 + ROS2 发布导航指令 | sensor 相机/USB 相机 + CBoard + ROS2 |
| `sentry_bp` | 哨兵主流程（蓝图/单相机变体） | 相机 + CBoard + ROS2 |
| `sentry_debug` | 哨兵调试可视化版 | 相机 + USB 相机 + CBoard + ROS2 |
| `sentry_multithread` | 哨兵多线程版（`mt_detector` + `omniperception` 决策） | 相机 + USB 相机 + CBoard + ROS2 |
| `publish_test` | ROS2 发布侧最小验证（`io::ros2` + `armor.hpp` 消息） | ROS2 |
| `subscribe_test` | ROS2 订阅侧验证：打印 `invincible enemy ids size is{}` 与各 id | ROS2 |
| `topic_loop_test` | ROS2 话题回环测试（发布→订阅自检） | ROS2 |

装好 ROS2（`ament_cmake`、`rclcpp`、自定义 msg）后重新 configure，这 7 个目标会自动出现，`cmake --build build_full` 即可产出 42 个。

## 五、按需求找程序

| 我想…… | 用哪个 |
| --- | --- |
| 跑真实比赛流程 | `standard` / `mt_standard` / `standard_mpc` |
| 看检测+跟踪效果但不接硬件 | `auto_aim_test`（自瞄）、`auto_buff_test`（打符）、`detector_video_test`（只看检测） |
| 调 MPC 弹道 | `planner_test_offline`（先离线）→ `planner_test`（接真云台）→ `standard_mpc` |
| 验证相机/串口通不通 | `camera_test`、`cboard_test`、`gimbal_test`、`dm_test`、`laser_gimbal_test` |
| 走完整标定流程 | `capture` → `calibrate_camera` → `calibrate_handeye` / `calibrate_robotworld_handeye` → `handeye_test` 复验 |
| 新硬件首次端到端联调 | `minimum_vision_system` |
| 压测多线程流水线 | `camera_thread_test`、`multi_usbcamera_test` |
| 做数据集 | `capture`（带姿态）或 `split_video`（切已有视频） |

## 维护说明

- 新增/删除 `add_executable` 时必须同步本文件的对应表格（属 `AGENT.md` §5 文档维护规则中的「显式指定」类改动）。
- 本文件的参数与默认值取自各 `main()` 的 `const std::string keys`，硬件依赖取自各文件的 `#include "io/..."` 与对象构造；改动这两处后需复核本文件。
