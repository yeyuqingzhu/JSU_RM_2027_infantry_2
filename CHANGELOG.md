# 变更记录（CHANGELOG）

格式参考 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。
本项目在赛前不保证 API 稳定，`0.x` 期间以日期标记。

## [Unreleased]

### Added
- GitHub 发布文件体系：`README.md`（重写，含来源声明与改动清单）、`NOTICE`（派生关系与逐项改动）、
  `THIRD_PARTY_NOTICES.md`、`CITATION.cff`、`CONTRIBUTING.md`、`SECURITY.md`、`CODE_OF_CONDUCT.md`、
  `CHANGELOG.md`、`.gitattributes`、`.github/` 模板。
- 补齐 vendored 第三方组件的许可正文与登记：`io/serial/LICENSE`（serial 库，MIT）、
  `tasks/auto_aim/planner/tinympc/LICENSE`（TinyMPC，MIT）、
  `io/{daheng,hikrobot,mindvision}/LICENSE-VENDOR.md`（三家相机 SDK 头文件为厂商自有许可，明确不适用本仓库 MIT）。
- `color_track_demo/main.cpp` 补 MIT 文件头与 `SPDX-License-Identifier`，明确为本队自有代码（opencode + DeepSeek 辅助生成），
  `THIRD_PARTY_NOTICES.md` 新增「本队自有代码」一节。
- `AGENT.md` 增补「仓库发布文件体系」一节。

### Changed
- `.gitignore` 重建：仓库只入库源码与文档；构建产物、模型权重与数据集、厂商 SDK 预编译库、
  运行与录制数据、本地 AI 代理配置（`.agents/`、`.dsh-meow/`）全部忽略。
- `LICENSE`：保留上游 `Copyright (c) 2025 TongjiSuperPower`，追加本仓库版权行与派生说明。

### Removed
- 上游 `readme.md`（同济 SuperPower 战队原版 README）从本仓库移除，原始内容见上游仓库。
- `.agents/` 本地 skill 副本移出版本控制（文件保留在开发机）。

## [0.2.0] — 2026-10-13

### Added
- `tools/` 25 个底层工具与函数：迁移自 `sp_vision_25` 并补充详细中文注释，接口与语义未变；
  `tools/CMakeLists.txt` 增加 `-Wall -Wextra`（零告警）。逐文件说明见 `doc/code/tools/`。
- 上层代码全量拷入：`io/`、`tasks/`（`auto_aim` / `auto_buff` / `omniperception`，含 vendored TinyMPC）、
  `src/`（13 个入口）、`calibration/`（6）、`tests/`（22）、`configs/`（13 份 YAML）。
- 文档体系：`AGENT.md`、`doc/`（README、overview、directory_tree、executables、troubleshooting、code/）。

### Changed
- `io/gimbal` 协议重写为 **v3.0**，对接 lt-2027 哨兵下位机：921600 8N1、帧 `0xFF … 0x0D` + crc8（poly 0x31）、
  线上全用度制、28 字节下行 `UsbCtrl` / 42 字节上行 `UsbAutoSendToNuc`；对外 API 保持弧度制绝对角。
- 顶层 `CMakeLists.txt` 采用上游版本。

### Fixed
- `tests/camera_thread_test.cpp` 编译错误（引用了已移出 `tools` 的 `Frame` / `create_yolov8s`）。
- 补拷上游 `color_track_demo/main.cpp`，顶层 CMake 的 source 缺失错误消除。

### Verified
- 全量构建从零可复现：**0 error，35 个可执行**（42 个 `add_executable` 中 7 个 ROS2 目标因未装 ROS2 跳过），
  6 条告警全部来自 vendored 第三方代码（大恒 `GXDef.h`、`io/daheng/daheng.cpp:130`、TinyMPC `admm.cpp:40`）。

## [0.1.0] — 2026-10-01

### Added
- 初始化仓库与项目骨架：顶层 `CMakeLists.txt`、`tools/` 冒烟测试、`doc/` 与 `AGENT.md` 的初版结构。
