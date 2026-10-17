# 第三方组件声明（Third-Party Notices）

本仓库自带或依赖下列第三方成果。**各组件以其原始许可为准**；本仓库仅按原样保留，未修改其许可条款。
本仓库根目录的 `LICENSE`（MIT）**不覆盖**第 3 节中标注为厂商许可的内容。

若你是某组件的权利人并认为此处声明有误，请开 issue 或直接联系维护者，我们会立即更正或移除。

---

## 1. 直接上游（本仓库的代码主体来源）

| 组件 | 来源 | 许可 | 在本仓库中的位置 |
| --- | --- | --- | --- |
| **sp_vision_25**（同济大学 SuperPower 战队，2025 赛季自瞄开源） | <https://github.com/TongjiSuperPower/sp_vision_25> | MIT（`Copyright (c) 2025 TongjiSuperPower`） | 本仓库几乎全部源码；派生说明与逐项改动见 [`NOTICE`](NOTICE) |
| sp_vision_24 / sp_vision_23（同战队往年开源，sp_vision_25 的前身） | <https://github.com/TongjiSuperPower/sp_vision_24>、<https://github.com/TongjiSuperPower/sp_vision_23> | 见各自仓库 | 未直接复制；本仓库沿袭其架构与工具函数血统 |

## 2. 上游引用的算法与模型来源（仅思路/权重来源，源码未复制）

| 项目 | 来源 | 说明 |
| --- | --- | --- |
| rm_vision | <https://github.com/chenjunnn/rm_vision> | 四点检测模型（装甲板关键点）思路来源之一，许可见其仓库 |
| rm.cv.fans | <https://github.com/julyfun/rm.cv.fans> | 同上 |
| Ultralytics YOLO（YOLOv5 / YOLOv8 / YOLO11） | <https://github.com/ultralytics/ultralytics> | 检测网络结构；本仓库只含推理侧代码，不含其训练框架源码 |

> **模型权重不在本仓库内**（`assets/` 已被 `.gitignore` 排除）。Ultralytics 系权重为 AGPL-3.0，
> 若将来要把权重一并分发，**必须单独评估许可**，不能以本仓库的 MIT 名义发布。

## 3. vendored 在源码树中的第三方代码

| 组件 | 位置 | 许可 | 许可正文 |
| --- | --- | --- | --- |
| **serial**（跨平台串口库，wjwwood/serial） | `io/serial/` | MIT（`Copyright (c) 2012 William Woodall, John Harrison`；`list_ports*` 另见 `Copyright (c) 2014 Craig Lilley`；`v8stdint.h` 为 V8 项目 BSD-3-Clause） | [`io/serial/LICENSE`](io/serial/LICENSE) |
| **TinyMPC**（MPC 求解器，CMU Robotic Exploration Lab） | `tasks/auto_aim/planner/tinympc/` | MIT（上游 LICENSE 的版权行原文为 `Copyright (c) 2017 Lectem`，疑为上游模板残留，此处照实记录） | [`tasks/auto_aim/planner/tinympc/LICENSE`](tasks/auto_aim/planner/tinympc/LICENSE) |
| 大恒 Galaxy SDK（头文件 + CMake 集成） | `io/daheng/` | **厂商自有许可，非 MIT** | [`io/daheng/LICENSE-VENDOR.md`](io/daheng/LICENSE-VENDOR.md) |
| 海康机器人 MVS SDK（头文件 + CMake 集成） | `io/hikrobot/` | **厂商自有许可，非 MIT** | [`io/hikrobot/LICENSE-VENDOR.md`](io/hikrobot/LICENSE-VENDOR.md) |
| 迈德威视 MVSDK（头文件 + CMake 集成） | `io/mindvision/` | **厂商自有许可，非 MIT** | [`io/mindvision/LICENSE-VENDOR.md`](io/mindvision/LICENSE-VENDOR.md) |
| Qt Designer 布局文件 | `buff_layout.xml`、`mpc_layout.xml` | 随上游，MIT | 上游的调试界面布局 |

> 三家相机 SDK 的**预编译库**（`lib/*.so`、`*.cti`）**不在本仓库内**，需从厂商处自行获取并遵守其许可；
> 本仓库只保留编译期需要的头文件。

## 4. 本队自有代码（非第三方）

| 内容 | 位置 | 许可 | 说明 |
| --- | --- | --- | --- |
| **color_track_demo**（颜色目标检测 + 类别判断 + 云台追踪演示） | `color_track_demo/main.cpp` | MIT（`Copyright (c) 2026 JSU RoboMaster Team`） | 本队编写，编码过程由 opencode 调用 DeepSeek 辅助生成；文件头已带 SPDX 标记。**不是上游代码** |
| `tools/` 中文注释版、`io/gimbal/` 协议 v3.0 改造、`doc/`、`AGENT.md`、本组发布文件 | 全仓库 | MIT | 派生自上游 MIT 代码的修改与新增；上游版权声明保留于根目录 `LICENSE` |
| 本队训练的模型权重 | 不入库 | 另行决定 | 若日后入库，需在此登记训练框架与数据的许可 |

## 5. 运行时系统依赖（未 vendored，需自行安装）

| 依赖 | 用途 | 许可 |
| --- | --- | --- |
| OpenCV 4 | 图像处理 | Apache-2.0 |
| Eigen 3 | 线性代数（只用 core / geometry） | MPL-2.0 |
| Ceres Solver 2.x | 非线性优化（标定） | BSD-3-Clause |
| OpenVINO Runtime | 神经网络推理 | Apache-2.0 |
| ONNX Runtime | `color_track_demo` 的推理后端 | MIT |
| yaml-cpp | 配置解析 | MIT |
| spdlog + fmt | 日志（本机 spdlog 为 `SPDLOG_FMT_EXTERNAL=1`，链接期必须链 `fmt::fmt`） | MIT / MIT |
| nlohmann/json | JSON | MIT |
| ROS2（可选） | 与导航通信（哨兵追击） | Apache-2.0 |

以上均为宽松许可，与 MIT 兼容，无 copyleft 传染问题，因此**无需**在本仓库内为它们增加额外许可文件。

## 6. 兼容性说明

上游与本仓库均为 MIT；第 3 节中被引用的算法来源（rm_vision 系列等）若采用 GPL/AGPL 类许可，
本仓库**仅将其作为思路与预训练权重的外部来源引用，未复制其源代码**。
若后续要把这类代码或权重纳入本仓库，请先确认许可兼容性并在本文件追加条目。
