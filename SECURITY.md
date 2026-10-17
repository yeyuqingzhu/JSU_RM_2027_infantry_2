# 安全策略（SECURITY）

## 报告问题

请**不要**用公开 issue 报告安全或安全问题相关的缺陷，改用 GitHub 的私密渠道：

- 首选：[Security Advisories](https://github.com/yeyuqingzhu/JSU_RM_2027_infantry_2/security/advisories/new)（Private vulnerability reporting）
- 或私信维护者：GitHub [@yeyuqingzhu](https://github.com/yeyuqingzhu)

请在报告中包含：受影响文件/提交、复现步骤、影响范围（是否涉及实机控制链路）。我们会在确认后回复处理计划。

## 适用范围

本项目是 RoboMaster 参赛工程，重点关注：

1. **实机控制链路的失控风险** —— 串口帧 CRC/长度校验缺陷、异常帧导致云台失控或误发射。
   （已知：`io/gimbal` 协议 v3.0 只校验帧头帧尾；`tools/crc` 存在长度回绕越界读的历史缺陷讨论，
   见 `doc/troubleshooting.md`。）
2. **不可信输入导致的崩溃** —— 从相机、串口、YAML、模型文件读入的数据未做边界检查（越界读/写、空指针解引用）。
3. **第三方依赖漏洞** —— OpenCV / OpenVINO / spdlog / yaml-cpp / 相机 SDK 的已知 CVE。

## 不适用

- 比赛战术、算法效果、识别率高低 —— 用普通 issue。
- 未安装 ROS2 / OpenVINO 版本差异 / 相机 SDK 缺失导致的构建失败 —— 先查 [`doc/troubleshooting.md`](doc/troubleshooting.md)。

## 支持版本

仅对默认分支最新提交提供修复。赛事期内使用打 tag 的发布版本，请自行评估是否回灌修复。
