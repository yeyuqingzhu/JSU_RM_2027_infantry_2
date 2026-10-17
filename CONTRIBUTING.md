# 贡献指南（CONTRIBUTING）

本仓库是队内工程，同时作为 RoboMaster 2027 赛季步兵自瞄的代码基线。提交前请读完本文件。

## 0. 最重要的一条：这是派生工程

绝大部分源码逐字来自上游 [`sp_vision_25`](https://github.com/TongjiSuperPower/sp_vision_25)（MIT）。
因此：

- **新增/修改文件时，注意区分「逐字沿用上游」与「本队改动」**；凡是改动上游文件，请在 [`NOTICE`](NOTICE) 第 3 节的改动清单里追加一行（路径 + 改了什么）。
- 不要在未确认许可兼容性的前提下把 GPL/AGPL 类代码复制进来；确需引入，先在 [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) 登记许可并确认兼容。
- 上游 `readme.md` 已删除，不要再把它加回来。

## 1. 只提交源码

`.gitignore` 已经把下列内容排除，**不要用 `git add -A` / `git add .` 绕过它**：

| 类别 | 举例 |
| --- | --- |
| 构建产物 | `build/`、`build_full/`、`CMakeFiles/`、`*.o` |
| 模型权重与数据集 | `assets/`（OpenVINO IR / ONNX / `PinkTraining/`）、`*.onnx`、`*.pt` |
| 厂商 SDK 预编译库 | `io/{hikrobot,mindvision,daheng}/lib/`、`*.so`、`*.cti` |
| 运行数据 | `logs/`、`records*/`、`imgs/`、`*.mp4`、`*.csv` |
| 本地备份 / 笔记 / IDE / AI 代理配置 | `*.bak`、`.agents/`、`.dsh-meow/`、`.vscode/` |

新增这类文件前先想着把它写进 `.gitignore`，而不是提交进去。

## 2. 提交前自检

```bash
# 1) 全量构建必须通过（0 error；告警只允许来自 vendored 第三方代码）
cmake -B build -S . && cmake --build build -j"$(nproc)"

# 2) 格式化（仓库自带 .clang-format）
clang-format -i <改动过的 .cpp/.hpp>

# 3) 确认没有把二进制/模型带进暂存区
git diff --cached --stat
git diff --cached --name-only | xargs -r du -h | sort -h | tail
```

提交信息说明**改了什么、为什么改**；涉及上游文件时写清与上游的差异。

## 3. 文档与排障规则（强制）

本项目把「踩过的坑」当作资产：

- **遇到任何报错/异常，第一动作是查 [`doc/troubleshooting.md`](doc/troubleshooting.md)**（含按症状检索的快速索引）。命中条目就直接按其中「解决方案」执行，**不要重新推导、不要重复思考同一个问题**。
- 解决了一个文档里没有的新问题，**必须追加一条**（症状含报错原文 / 原因 / 解决方案）并同步快速索引。
- `doc/overview.md`、`doc/directory_tree.md`、`doc/code/**` 的更新条件是：**①改动导致函数作用变动 ②新增或删除函数 ③维护者显式要求**。只改实现不改语义时不动这三处文档。
- 完整规则见 [`AGENT.md`](AGENT.md) §5、§6。

## 4. 分支与 PR

- 分支命名：`feature/<简述>`、`fix/<简述>`、`proto/<简述>`（实验性改动）。
- PR 描述里请写：改了什么 / 为什么 / 怎么验证（构建与测试命令、实测结果）/ 是否触碰上游文件（触碰则更新 `NOTICE`）。
- 涉及云台、发射机构、串口协议的改动，请在 PR 里注明**是否已在实机上验证**；未验证的标记为 `未上机`。
- 一个 PR 只做一件事；格式化、重命名这类纯机械改动请单独提交，避免与逻辑改动混在一起。

## 5. 代码风格

- C++17，遵循仓库根目录 `.clang-format`（Google 风格改装，列宽 100）。
- 命名空间：`tools` / `io` / `auto_aim` / `auto_buff`；头文件保护宏形如 `MODULE__FILE_HPP`。
- 用 `spdlog` 记日志；错误处理用早返回，不用异常。
- 加注释写「为什么」，不写「做了什么」。
