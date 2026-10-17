# AGENT.md — 项目概要与协作约定

> **本文件是 `JSU_RM_2027_infantry_2` 的入口文件**，存放项目概要、总目标与文档维护规则。
> 每次对项目做出修改后，都必须按 [§5 维护规则](#5-维护规则重要) 维护本文件与 `doc/`；
> **遇到任何报错/异常，先查 [§6 问题排查](#6-问题排查先查文档不要重复思考重要) 指向的 [`doc/troubleshooting.md`](doc/troubleshooting.md)，按既定方案解决，不要重复思考。**

---

## 1. 项目概要

| 项 | 内容 |
| --- | --- |
| 项目名 | `JSU_RM_2027_infantry_2` |
| 定位 | RoboMaster 2027 赛季步兵自瞄（auto-aim）算法项目（infantry 2 号机） |
| 语言 / 形态 | C++17 工程；算法部分、接口部分、底层工具与函数三部分模块化 |
| 参考项目 | [sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25) — 同济大学 SuperPower 战队 25 赛季视觉框架；本地副本 `/home/yeyuqingzhu/C++/localcreated/summertraining/original_source/sp_vision_25` |
| 当前阶段 | **阶段 1 已完成**（`tools/`）；**阶段 2 进行中**：sp_vision_25 剩余全部代码已拷入本仓库作为起点（详见 §3） |
| 构建系统 | 顶层 `CMakeLists.txt` 为 sp_vision_25 版（C++17、OpenVINO、三家相机 SDK、可选 ROS2）；**全量构建已跑通**（2026-10-17，35 个可执行，见 §3.2、[troubleshooting A-5](doc/troubleshooting.md)） |
| 目标平台 / 推理后端 | 未定（代码现状默认 OpenVINO） |

---

## 2. 总目标

建立并维护一个能够作为 RoboMaster 步兵自瞄算法载体的项目，使**算法部分、接口部分、底层工具与函数全部模块化，方便维护**：

1. **算法核心与平台/推理后端解耦** —— 后端确认后可以平替接入，算法层不因换平台而重写；
2. **接口层与算法层分离** —— 相机输入、串口/下位机通信、调试可视化等接口各自独立、可单独测试；
3. **底层工具与函数统一沉淀** —— 数学、坐标变换、时间、滤波、日志、配置解析等通用件供上层复用；
4. **前期目标：完成底层工具与函数、以及纯算法的迁移**（迁移来源为参考项目 sp_vision_25）；
5. **最终目标：写自己的自瞄算法**（技术路线判断：以传统视觉为主、YOLO 为辅；先基本方案再优化）。

---

## 3. 当前状态（阶段 1 已完成，阶段 2 进行中）

### 3.1 已完成

- **阶段 0**（目录骨架 + 文档体系）：已完成（计划骨架空目录已于 2026-10-17 清理，见 §3.3）。
- **阶段 1**（底层工具与函数迁移）：**已完成** —— `tools/` 下 25 个源文件自 sp_vision_25 迁移并补充详细中文注释，
  逐文件说明见 [`doc/code/tools/`](doc/code/tools/README.md)。当时以手写顶层 `CMakeLists.txt` + `tools/CMakeLists.txt`
  在宿主侧（x86 Linux）编译通过，`-Wall -Wextra` 下零告警，14 个头文件均可独立编译，功能冒烟测试 43 项全过（`build/smoke_tools.cpp`）。

### 3.2 阶段 2 起点：sp_vision_25 剩余代码已全量拷入（2026-10-17）

- **新增目录**：`io/`（99 文件，含三家相机 vendored SDK 头与 `.so`）、`tasks/`（71：`auto_aim`/`auto_buff`/`omniperception`，含 vendored TinyMPC）、`src/`（13 个入口）、`calibration/`（6）、`tests/`（22 个独立 `main()`）、`configs/`（13 份 YAML）、`assets/`（2.2 GB：OpenVINO IR / ONNX 模型 + 训练数据集 `PinkTraining/`）。
- **新增根文件**：`.clang-format`、`LICENSE`、`autostart.sh`、`buff_layout.xml`、`mpc_layout.xml`；`readme.md` 现为 sp_vision_25 原始 README（351 行）。
- **被覆盖**：顶层 `CMakeLists.txt`（上游 146 行版取代阶段 1 手写版；旧版可用 `git show :CMakeLists.txt` 取回）、`.gitignore`、`readme.md`。**`tools/` 未被覆盖**，仍是带中文注释的迁移版。
- **构建现状（2026-10-17 已跑通）**：按 [troubleshooting A-5](doc/troubleshooting.md) 方案 1 补拷上游 `color_track_demo/main.cpp` 后，
  `cmake -B build_full -S . && cmake --build build_full -j32` **零错误**，从零构建（`build_verify/`）复现同样结果：
  **35 个可执行文件**（顶层 42 个 `add_executable` 中 7 个 ROS2 目标因未装 ROS2 不生成），6 条告警全部来自 vendored 第三方代码
  （大恒 `GXDef.h`/`daheng.cpp`、TinyMPC `admm.cpp` 的 Eigen 弃用 API），本项目自有代码零告警。
  唯一编译错误 `tests/camera_thread_test.cpp`（引用分层重构时移出 `tools` 的 `Frame`/`create_yolov8s`）已按 [B-6](doc/troubleshooting.md) 修复。
  OpenVINO 2023.3.0 与脚本写死的 2024.6.0 无编译期差异，脚本保持原样。
- **未做**：算法代码的审核/裁剪/改造尚未开始，上游代码按原样保留。

### 3.3 需要留意的现状细节

- **计划骨架遗留目录已全部删除**（2026-10-17）：`auto_function/`、`main/`、`measure/`、`test/`、`yolo/`、`io/driver/`、`io/uart/`、`troubleshooting-doc-pattern/`，以及 `configs/.gitkeep`、`src/.gitkeep`、`tests/.gitkeep`（目录已有真实文件）。**仓库现在没有任何 `.gitkeep`**，映射关系见 [`doc/directory_tree.md` §2.10](doc/directory_tree.md)。
- **本地残留文件**：`tasks/auto_aim/detector.cpp.bak`、`tasks/auto_aim/yolos/*.cpp.bak`（备份）、`tasks/auto_aim/solver.md`、`tasks/auto_aim/tracker.md`（本地笔记）。见 §7 第 9 项。
- `.dsh-meow/` 是 AI 记忆库（本机数据，误删不可恢复）。2026-10-17 已重建 `.gitignore`，它和 `.agents/` 一起被忽略（§3.4），不会再被 `git add -A` 带进提交。
- `build/` 为构建产物目录（已 gitignore），内含阶段 1 的编译输出、`build/smoke_tools.cpp` 与 `compile_commands.json`。
- `doc/code/` 目前覆盖 `tools/`（25 篇）与 `io/gimbal/`（4 篇），另有 `configs/` 占位；`io/` 其余文件、`tasks/`、`src/`、`calibration/`、`tests/` 的逐文件说明尚未创建（旧骨架的 7 个占位目录已删除）。
- **算法全景**（三大支柱与三种决策器）见 [`doc/overview.md` §4](doc/overview.md)；逐文件职责见 [`doc/directory_tree.md`](doc/directory_tree.md)。

### 3.4 仓库发布文件体系（2026-10-17 建立）

本仓库定位为 **sp_vision_25 的派生工程**，GitHub 侧文件如下（上游 `readme.md` 已删除，不再恢复）：

| 文件 | 作用 |
| --- | --- |
| [`README.md`](README.md) | 面向 GitHub 主页：**来源声明（派生自同济大学 SuperPower `sp_vision_25`）**、相对上游的改动清单、项目结构、依赖与准备、构建与实测状态、文档索引 |
| [`NOTICE`](NOTICE) | 上游快照 commit、拷贝时间、逐项改动、未纳入仓库的内容（模型/厂商 SDK 库） |
| [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) | 第三方组件与许可（含 vendored TinyMPC、三家相机 SDK、`color_track_demo` 待核实许可） |
| [`LICENSE`](LICENSE) | MIT，保留上游 `Copyright (c) 2025 TongjiSuperPower` + 本仓库版权行与派生说明 |
| [`CITATION.cff`](CITATION.cff) | 引用信息，含对上游的 `references` 条目 |
| [`CONTRIBUTING.md`](CONTRIBUTING.md) / [`SECURITY.md`](SECURITY.md) / [`CODE_OF_CONDUCT.md`](CODE_OF_CONDUCT.md) / [`CHANGELOG.md`](CHANGELOG.md) | 协作约定、安全问题上报、行为准则、变更记录 |
| `.gitignore` / `.gitattributes` | **只入库源码与文档**：构建产物、`assets/`（模型/数据集）、`io/*/lib/`（厂商 SDK 预编译库）、录制日志、`.agents/`、`.dsh-meow/` 全部忽略；vendored 路径标记 `linguist-vendored` |
| `.github/` | Issue 表单（bug / feature）与 PR 模板，均内置「先查 troubleshooting」「只提交源码」「是否触碰上游文件」检查项 |

> 改动上游文件时必须同步 `NOTICE` 第 3 节；引入第三方代码/权重必须登记 `THIRD_PARTY_NOTICES.md`。

---

## 4. 文档体系（`doc/`）

| 文件 / 目录 | 作用 |
| --- | --- |
| [`doc/README.md`](doc/README.md) | 文档索引与维护规则 |
| [`doc/overview.md`](doc/overview.md) | 项目总体详细说明（定位、阶段划分、项目全景、模块划分、构建与依赖、参考文献、设计原则、待确认事项） |
| [`doc/directory_tree.md`](doc/directory_tree.md) | 项目目录树，包含每个文件、文件夹的概要 |
| [`doc/executables.md`](doc/executables.md) | **可执行程序一览**：每个构建目标的作用 / 用法 / 默认 yaml / 硬件依赖（含「按需求找程序」速查） |
| [`doc/troubleshooting.md`](doc/troubleshooting.md) | **问题与解决方案（构建/编译/测试/环境/工具）**：遇到问题先查这里 |
| [`doc/code/`](doc/code/README.md) | 逐文件函数作用说明，按项目结构建子文件夹（当前仅 `tools/` 已覆盖） |

### 4.1 参考文献（sp_vision_25 原始文档，本项目最权威的"项目地图"）

来源目录：`/home/yeyuqingzhu/C++/localcreated/summertraining/original_source/sp_vision_25/`

| 文件 | 内容 |
| --- | --- |
| `docs/项目全景与教学指南.md`（1318 行） | 项目结构与全部代码清单（按文件）、第三方库与函数清单、构建思路与算法搭配、数学思路、创新内容实现 |
| `docs/auto_aim_算法逐行解析.md`（1251 行） | 主程序 `standard_mpc.cpp`、`armor.hpp`、`tracker.cpp`、`target.cpp`（11 维 EKF）、`aimer.cpp`、`solver.cpp` 逐行讲解 |
| `docs/汇总.md`（15 行） | 学习笔记：四段式架构、坐标系转换、固定 pitch 穷举 yaw 重投影、改造 YOLO 输出为装甲板四角点 |
| `docs/技术规划（续）.md`（21 行） | 2027 规则变化与对步兵的影响；技术路线判断（传统视觉为主、YOLO 为辅） |
| `AGENTS.md`（54 行） | 上游构建方式、依赖清单、测试清单、C++ 风格与命名约定、库依赖图、设计模式 |
| `opencode.json` + `.opencode/command/context-update.md` | 上游 opencode 的 context-update 命令（增量维护 `.opencode/context/**`）；**本仓库不采用该机制** |

> 阅读顺序建议（来自上游文档）：`项目全景` 板块三 → 板块四 → 板块五 → 板块一；代码入门从 `src/standard_mpc.cpp` 开始。

---

## 5. 维护规则（重要）

适用于本文件与 `doc/overview.md`、`doc/directory_tree.md`、`doc/code/**`：

**（一）修改触发条件 —— 当且仅当下列情形之一发生时，才更新文档：**

1. 修改文件**导致函数作用发生变动**；
2. **创建或删除函数**；
3. **显式指定**（用户或协作者明确要求更新文档）。

**（二）修改范围限制：**

- 只修改「详细摘要」中**该文件对应的摘要**，以及**总摘要**；其余条目一律不动。
- 代码变更但函数作用未变（改注释、改格式、改常量取值、重命名局部变量等）→ **不更新文档**。

---

## 6. 问题排查：先查文档，不要重复思考（重要）

- 本项目**已踩过的所有坑**都记录在 [`doc/troubleshooting.md`](doc/troubleshooting.md)，该文档开头有**「快速索引」**，按症状关键词即可定位条目。
- **遇到问题的第一动作是查该文档。命中条目后直接按「解决方案」执行——不要重新推导、不要重复思考同一个问题，也不要再试一遍已经失败的路径。**
- 只有文档中**确无记录**时才自行排查；问题解决后**必须**把新条目（症状/原因/解决方案）追加进 `doc/troubleshooting.md`，并同步其快速索引表，使其成为唯一权威的排查入口。
- 该文档的维护是**独立触发**的：解决新问题即更新，**不受 §5「仅函数作用变动才更新」的限制**。

---

## 7. 待确认事项

| # | 事项 | 说明 |
| --- | --- | --- |
| 1 | 目标平台 / 推理后端 | 未确认；确认前不得在算法层引入新的平台相关依赖（现状默认 OpenVINO） |
| 2 | 构建系统 | 顶层脚本现为 sp_vision_25 版（见第 5 项）；交叉编译与交叉工具链配置待平台确认后补 |
| 3 | ~~目录命名~~ | **已解决**：`auto_fuction/` 已更正为 `auto_function/`，`cmakelists.txt` 已更正为 `CMakeLists.txt` |
| 4 | 迁移清单 | `tools/` 已迁移完成；阶段 2 需逐项列出 `io/ tasks/ src/ calibration/ tests/` 中哪些保留、哪些重写、哪些删除 |
| 5 | ~~顶层 `CMakeLists.txt` 用哪个版本~~ | **已解决（2026-10-17）**：保留上游版 + 补拷 `color_track_demo/`，全量构建通过（35 个可执行）；OpenVINO 2023.3.0 实测可用。阶段 1 手写版（仅 `tools`）仍可用 `git show :CMakeLists.txt` 取回 |
| 6 | ~~计划骨架遗留目录如何处置~~ | **已解决（2026-10-17）**：`auto_function/ main/ measure/ test/ yolo/ io/driver/ io/uart/`、空目录 `troubleshooting-doc-pattern/` 与三个多余 `.gitkeep` 已全部删除，`doc/code/` 对应旧占位目录一并删除；映射见 [`doc/directory_tree.md` §2.10](doc/directory_tree.md) |
| 7 | `.gitignore` 是否恢复 `.dsh-meow/` | 被上游版本覆盖后 AI 记忆库变为未跟踪状态，提交前需注意（见 [troubleshooting A-7](doc/troubleshooting.md)） |
| 8 | `readme.md` 是否改写为本项目 README | 现为 sp_vision_25 原文（351 行） |
| 9 | 本地残留文件是否清理 | `tasks/auto_aim/*.bak`、`tasks/auto_aim/solver.md`、`tasks/auto_aim/tracker.md` |
| 10 | 硬件接口 | 相机型号/接口、下位机协议（云台串口 v2.0 还是 C 板 CAN）均未确认；代码里两条路径都存在 |
| 11 | `assets/` 体积（2.2 GB）是否入库 | 含 `PinkTraining/` 训练数据集约 2.1 GB；当前 `.gitignore` 只忽略 `assets/img_with_q/` |
