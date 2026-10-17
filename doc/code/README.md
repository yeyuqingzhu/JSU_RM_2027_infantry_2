# 逐文件函数作用说明（`doc/code/`）

本目录存放**每个源文件的函数作用说明**，按项目结构创建子文件夹。

## 1. 目录映射约定

- 子文件夹结构与源码目录**一一对应**，例如：
  - 源码 `tools/math/xxx.cpp` → 说明 `doc/code/tools/math/xxx.cpp.md`
  - 源码 `io/gimbal/gimbal.cpp` → 说明 `doc/code/io/gimbal/gimbal.cpp.md`
- 每个源文件对应**一个 Markdown 文件**，文件名 = 源文件名 + `.md`。
- 新增源码目录时，在此建立对应的映射子文件夹（目录映射见 §4 与 [`../directory_tree.md`](../directory_tree.md)）。
- **目前只覆盖 `tools/`（25 篇）与 `io/gimbal/`（4 篇）**；其余源码目录（`io/` 其余文件、`tasks/`、`src/`、`calibration/`、`tests/`）的说明文档**尚未创建**，需要时按上表逐步补齐。

## 2. 维护规则（与 `AGENT.md` §5 一致）

- **当且仅当**下列情形之一发生时，才更新本目录下的说明：
  1. 修改文件导致**函数作用发生变动**；
  2. **创建或删除函数**；
  3. **显式指定**。
- 更新时**只修改该文件对应的摘要与总摘要**，其余条目不动。
- 代码变更但函数作用未变（改注释、改格式、改常量、改局部变量名）→ 不更新。

## 3. 文件说明模板

```markdown
# <源文件相对路径>

**概要**：<该文件整体做什么，一句话>

## 函数

### `返回类型 函数名(参数列表)`

- **作用**：
- **参数**：
- **返回**：
- **备注**：<线程安全、调用时机、依赖等，可选>
```

> **总摘要**：当某文件包含多个函数时，文件顶部「概要」即为该文件的总摘要；上层的模块总摘要记录在 [`../overview.md`](../overview.md) 与 [`../directory_tree.md`](../directory_tree.md)。

## 4. 映射文件夹一览

| 说明目录 | 对应源码目录 | 当前状态 |
| --- | --- | --- |
| `tools/` | `tools/` | **已完成**：25 个源文件 25 篇 |
| `io/gimbal/` | `io/gimbal/` | **已完成**：`gimbal.hpp/.cpp`、`laser_gimbal.hpp/.cpp` 共 4 篇 |
| `configs/` | `configs/` | 占位（13 份 YAML 配置，无 C++ 源文件，暂不需要逐文件说明） |

> **待补齐的映射**（源码已存在、说明文档尚未创建，按需逐步添加）：`io/`（`camera`、`cboard`、`command.hpp`、`socketcan.hpp`、`serial/`、`hikrobot/`、`mindvision/`、`daheng/`、`usbcamera/`、`dm_imu/`、`ros2/`）、`tasks/auto_aim/`（16 源文件 + `yolos/`、`multithread/`、`planner/`）、`tasks/auto_buff/`、`tasks/omniperception/`、`src/`（13 个入口）、`calibration/`（6）、`tests/`（22）。
