# JSU_RM_2027_infantry_2

> RoboMaster 2027 赛季步兵（2 号机）自瞄算法工程。
>
> **本仓库是由 [同济大学 SuperPower 战队开源的 `sp_vision_25`](https://github.com/TongjiSuperPower/sp_vision_25) 修改而来的派生工程，不是独立实现。**
> 逐字沿用的部分、被修改的部分、以及上游的完整引用关系见 [NOTICE](NOTICE)。

C++17 / CMake / Linux（x86_64 与 aarch64 均可交叉配置） · MIT License

---

## ⚠️ 来源声明（Attribution）

| 项 | 内容 |
| --- | --- |
| 本仓库 | `JSU_RM_2027_infantry_2` —— JSU RoboMaster 战队 2027 赛季步兵自瞄工程 |
| 上游项目 | `sp_vision_25` —— **同济大学（Tongji University）SuperPower 战队** 2025 赛季自瞄算法开源 |
| 上游地址 | <https://github.com/TongjiSuperPower/sp_vision_25> |
| 上游快照 | commit [`604e1194`](https://github.com/TongjiSuperPower/sp_vision_25/commit/604e11943b7dc3a8b466cce6d72079e5d271b2b7)（2025-09-29），2026-10-17 全量拷入本仓库 |
| 上游许可 | MIT，`Copyright (c) 2025 TongjiSuperPower`（见 [LICENSE](LICENSE)） |
| 引用关系 | 本仓库的 `tools/` `io/` `tasks/` `src/` `tests/` `calibration/` `configs/` 等目录**绝大部分源码逐字来自上游**（文件集合与内容逐文件比对一致，仅下表所列 30 个文件有改动）。上游 README 已从本仓库移除，其原始内容请到上游仓库阅读 |
| 上游的上游 | `sp_vision_24`、`sp_vision_23`（同战队往年开源）、[`chenjunnn/rm_vision`](https://github.com/chenjunnn/rm_vision)、[`julyfun/rm.cv.fans`](https://github.com/julyfun/rm.cv.fans)、Ultralytics YOLO、TinyMPC 等，详见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) |

**引用建议**：若要引用本工程的算法思想或复用其代码，请**优先引用上游 `sp_vision_25`**（算法与框架的真正作者）；引用本仓库时请同时注明派生关系，格式见 [CITATION.cff](CITATION.cff)。

---

## 相对上游 `sp_vision_25` 的改动

截至 2026-10-17，与上游快照逐文件比对（`diff -rq`，排除构建产物/模型/数据集）的结果：**30 个文件有内容差异**，其余源码目录文件数与内容完全一致。

| 路径 | 改动 | 说明 |
| --- | --- | --- |
| `tools/` 25 个源文件 + `tools/CMakeLists.txt` | 迁移 + 重写注释 | 底层工具与函数迁移版：补充详细中文注释，**接口与语义未变**；CMake target 增加 `-Wall -Wextra` 并保持零告警 |
| `io/gimbal/gimbal.hpp`、`io/gimbal/gimbal.cpp` | **重写协议** | 云台串口改为协议 **v3.0**，对接 lt-2027 哨兵下位机：921600 8N1、帧 `0xFF … 0x0D` + crc8（poly 0x31）、线上全用度制、28 字节下行 `UsbCtrl` / 42 字节上行 `UsbAutoSendToNuc`；对外 API 仍为弧度制绝对角 |
| `tests/camera_thread_test.cpp` | 编译修复 | 引用的 `Frame` / `create_yolov8s` 已不在 `tools` 层，按新分层改正 |
| `.gitignore` | 重建 | 上游版只忽略少量目录；本仓库只入库源码与文档，模型/数据/厂商 SDK 库/构建产物全部忽略 |
| `doc/`、`AGENT.md` | **新增** | 本项目的文档体系：总体说明、目录树、42 个可执行程序说明、逐文件函数说明、问题排查手册 |
| `README.md`、`NOTICE`、`THIRD_PARTY_NOTICES.md`、`CITATION.cff`、`CONTRIBUTING.md`、`SECURITY.md`、`CODE_OF_CONDUCT.md`、`CHANGELOG.md`、`.gitattributes`、`.github/` | **新增** | 本项目的 GitHub 发布文件体系（上游 `readme.md` 已删除） |

> 注：`io/` 其余部分、`tasks/`（含 vendored TinyMPC）、`src/`、`calibration/`、`tests/` 其余、`configs/`、`color_track_demo/`、顶层 `CMakeLists.txt`、`autostart.sh`、`buff_layout.xml`、`mpc_layout.xml`、`.clang-format` 均与上游一致，尚未改造。

---

## 项目结构

```
JSU_RM_2027_infantry_2/
├── tools/            零依赖底座：数学 / 坐标变换 / 时间 / 滤波 / 日志 / 配置解析 / CRC / 线程池
├── io/               设备层：相机（hikrobot/mindvision/daheng/usbcamera）、云台串口 gimbal/、
│                     cboard CAN、socketcan、dm_imu、serial/、ros2/ 导航桥、command.hpp 决策输出
├── tasks/            算法层
│   ├── auto_aim/         装甲板识别、Solver PnP、Tracker 状态机、Target EKF、决策器
│   │   ├── aimer/            规则决策（legacy）
│   │   ├── planner/          MPC 决策（TinyMPC，DT=0.01，HORIZON=100）
│   │   └── yolos/            YOLOv5 / YOLOv8 / YOLO11 后端
│   ├── auto_buff/        打符
│   └── omniperception/   全向感知
├── src/              13 个可执行入口（standard / mt_standard / standard_mpc / 各 debug / uav / sentry…）
├── calibration/      6 个标定程序（相机内参、手眼等）
├── tests/            22 个各带 main() 的独立测试程序
├── configs/          13 份 YAML 配置
├── color_track_demo/ 颜色目标检测 + 类别判断 + 云台追踪演示（本队自研，ONNX Runtime，编译目标之一）
├── doc/              文档体系（索引见 doc/README.md）
├── AGENT.md          工程入口：概要、总目标、当前状态、文档与排障维护规则
└── CMakeLists.txt    顶层构建（42 个 add_executable）
```

数据流：`io` 采集（Camera / Gimbal / CBoard）→ `tasks` 算法（检测 → Solver 解算 → Tracker → Target EKF → 决策）→ 执行（`io::Command` 经串口下发）。`tools` 为零依赖底座，依赖链 `tools(OBJECT) ← io(STATIC) ← auto_aim(OBJECT) ← auto_buff(OBJECT)`。

---

## 依赖与准备

本仓库**只包含源代码**，下列内容被 `.gitignore` 排除，需在开发机本地准备：

| 缺失内容 | 从哪里来 |
| --- | --- |
| 模型权重 `assets/*.xml` + `*.bin` + `*.onnx` | 上游仓库 `assets/`，或用自己的数据集导出 OpenVINO IR / ONNX。`configs/*.yaml` 中的路径形如 `assets/yolo11.xml`，缺失时检测/打符程序无法启动 |
| 厂商相机 SDK 预编译库 `io/{hikrobot,mindvision,daheng}/lib/` | 各厂商 SDK（海康 MVS、迈德威视 MVSDK、大恒 Galaxy）。头文件与 `CMakeLists.txt` 已在仓库内，仅缺 `lib/` 下的 `.so`/`.cti`。不用某家相机时可在 `io/CMakeLists.txt` 关掉对应选项 |
| 训练数据集 `assets/PinkTraining/`、录制 `records*/`、日志 `logs/` | 本地生成，不入库 |

系统依赖（本机实测版本）：CMake ≥ 3.16、GCC（C++17）、OpenCV4、Eigen3 3.4、Ceres 2.2、yaml-cpp 0.8、**spdlog 1.15（`SPDLOG_FMT_EXTERNAL=1` 编译，链接期必须链 `fmt::fmt`）**、nlohmann_json 3.11、OpenVINO Runtime（脚本默认找 `/opt/intel/openvino_2024.6.0`，`find_package` 有回退，本机 2023.3.0 亦可编译通过）。ROS2 可选：未安装 `/opt/ros` 时 7 个 ROS2 目标自动跳过。

---

## 构建

```bash
cmake -B build -S .
cmake --build build -j"$(nproc)"
```

实测状态（2026-10-17，Ubuntu x86_64，GitHub 上的提交即此状态）：

- **0 error**；**35 个可执行文件**产出（顶层 42 个 `add_executable` 中 7 个 ROS2 目标因未装 ROS2 不生成）。
- 6 条 warning 全部来自 vendored 第三方代码（大恒 `GXDef.h` 的 `typedef` 重定义、`io/daheng/daheng.cpp:130`、TinyMPC `admm.cpp:40` 的 Eigen 弃用 API）；本项目自有代码零告警。
- 运行某个目标前按 `doc/executables.md` 确认默认 YAML 与硬件依赖；部分测试程序的默认配置在本仓库不存在，需显式 `-c <yaml>`。

---

## 文档

| 文档 | 内容 |
| --- | --- |
| [AGENT.md](AGENT.md) | 工程入口：概要、总目标、当前阶段、**文档维护规则**、**排障规则（先查 troubleshooting，命中即照做）** |
| [doc/README.md](doc/README.md) | 文档索引与维护约定 |
| [doc/overview.md](doc/overview.md) | 项目总体说明、模块划分、设计原则、待确认事项 |
| [doc/directory_tree.md](doc/directory_tree.md) | 目录树与逐目录概要 |
| [doc/executables.md](doc/executables.md) | 42 个构建目标的作用、用法、默认配置、硬件依赖 |
| [doc/troubleshooting.md](doc/troubleshooting.md) | 踩坑记录（按症状检索）；**遇到报错先查这里** |
| [doc/code/](doc/code/README.md) | 逐文件函数作用说明 |
| [CONTRIBUTING.md](CONTRIBUTING.md) | 提交与协作约定 |
| [CHANGELOG.md](CHANGELOG.md) | 变更记录 |

---

## 许可证

MIT，见 [LICENSE](LICENSE)。上游声明 `Copyright (c) 2025 TongjiSuperPower` 一并保留；派生关系与改动清单见 [NOTICE](NOTICE)，第三方组件许可见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

- vendored 组件各有自己的许可正文：`io/serial/LICENSE`（MIT）、`tasks/auto_aim/planner/tinympc/LICENSE`（MIT）。
- **三家相机 SDK 的头文件是厂商自有许可，不适用本仓库的 MIT**：见 `io/{daheng,hikrobot,mindvision}/LICENSE-VENDOR.md`；其预编译库不在本仓库内。
- 本队自有代码（如 `color_track_demo/main.cpp`）以 MIT 授权，文件头带 `SPDX-License-Identifier: MIT`。

## 免责声明

本项目为 RoboMaster 参赛队内工程，仅用于比赛与教学研究。涉及云台、发射机构的实机联调请遵守赛事安全规范。
