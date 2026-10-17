# Tracker 目标跟踪详解

> 文件位置：`tasks/auto_aim/tracker.hpp` / `tasks/auto_aim/tracker.cpp`
> 命名空间：`auto_aim`

Tracker 承接 `Detector` 与 `Solver` 的输出，负责**多目标筛选、目标确认、EKF 平滑跟踪与状态机管理**。其核心产出是唯一的 `Target`（含经 EKF 平滑后的位置/速度/朝向），供上游 `Planner`/`Aimer` 使用。

核心职责：

1. 过滤（颜色）、排序（图像中心距离、兵种优先级）装甲板列表；
2. 通过**状态机**（`lost / detecting / tracking / temp_lost / switching`）确认目标，抑制单帧误检与短暂漏检；
3. 用 `set_target`/`update_target` 实现"发现时初始化 EKF"与"跟踪时预测+更新 EKF"两阶段流程；
4. 进行**发散检测**与 **NIS（归一化新息平方）收敛检测**，异常时强制回到 `lost`；
5. 提供全向感知（sentry 多相机）场景下的**目标切换**（`switching` 状态）。

---

## 1. 成员变量总览（含义与用途）

### 1.1 状态机相关

| 变量 | 类型 | 含义 | 用途 |
|---|---|---|---|
| `state_` | `std::string` | 当前状态 | 取值 `lost/detecting/tracking/temp_lost/switching`，是状态机的"当前指针" |
| `pre_state_` | `std::string` | 上一帧状态 | 用于识别"刚从 switching 过渡来的 detecting"，触发重新初始化（`set_target`，见 tracker.cpp:155） |
| `detect_count_` | `int` | 连续检测帧计数 | 从 `lost→detecting` 起计数，达到 `min_detect_count_` 才升为 `tracking`；中途任何一帧丢失归零 |
| `temp_lost_count_` | `int` | 临时丢失帧计数 | `tracking` 中一帧丢失即进入 `temp_lost` 并置 1，每次丢失累加，超过 `max_temp_lost_count_` 才彻底判 `lost` |

### 1.2 阈值（构造时从 YAML 读取）

| 变量 | 类型 | 含义 | 用途 |
|---|---|---|---|
| `min_detect_count_` | `int` | 确认进入 `tracking` 的最少连续检测帧数 | 防止单帧误检直接进入跟踪 |
| `max_temp_lost_count_` | `int` | 临时丢失允许的最大帧数 | 容忍短暂遮挡/漏检，超过即判 `lost` |
| `outpost_max_temp_lost_count_` | `int` | 前哨站的临时丢失阈值 | 前哨站装甲板探测困难，需要更大的容错帧数 |
| `normal_temp_lost_count_` | `int` | 普通目标的临时丢失阈值副本 | 保存 `max_temp_lost_count_` 的普通值；因为后者会因目标是否为 outpost 被改写，需要此副本以便切回普通目标时恢复（`tracker.cpp:26, 220-226`） |

### 1.3 目标与滤波

| 变量 | 类型 | 含义 | 用途 |
|---|---|---|---|
| `target_` | `Target` | 当前跟踪目标（EKF 状态估计器） | `track()` 返回值；内含位置、速度、yaw、yaw 角速度、装甲板半径/高度等状态 |
| `last_timestamp_` | `time_point` | 上一帧时间戳 | 计算 `dt = t - last_timestamp_`（EKF 预测步长）；`dt > 0.1s` 判定相机离线并强制 `lost` |

### 1.4 参数与外部依赖

| 变量 | 类型 | 含义 | 用途 |
|---|---|---|---|
| `solver_` | `Solver &` | 位姿解算器引用 | 每个候选装甲板调用 `solver_.solve(armor)` 解算 3D 位姿后再喂给 EKF |
| `enemy_color_` | `Color` | 敌方颜色（red/blue） | 构造时由 `enemy_color` 解析；`tracker.cpp:43` 用于 `remove_if` 过滤掉非敌方装甲板 |
| `omni_target_priority_` | `ArmorPriority` | 全向感知切换目标的优先级 | `switching` 状态中判断主相机画面是否已出现对应优先级目标；缺省初始化为 `ArmorPriority::fifth`（哨兵最低优先级） |

> 构造初始化：`state_ = pre_state_ = "lost"`，`detect_count_ = temp_lost_count_ = 0`，`last_timestamp_ = now()`。

---

## 2. 目标状态机

### 2.1 五态定义

```
        ┌──────── lost ────────┐
        │    ▲                 │ detect_count_ >= min_detect_count_
        │    │ detect_count_=0 │        │
        │    │ (连续丢帧)       │        ▼
   temp_lost_<─── tracking◄────┤       detecting
        ▲        │   │
        └────────┘   │ 一帧丢失
      temp_lost_count_++
      > max_temp_lost_count_ → lost

   aside: tracking --(全向感知更高优先级)--> switching --(主相机确认)--> detecting
```

| 状态 | 进入条件 | 离开条件 |
|---|---|---|
| `lost` | 初始态；任何状态超阈值/发散/收敛失败 | 检测到目标 → `detecting`（`detect_count_=1`） |
| `detecting` | idle→检测到目标 | `detect_count_ ≥ min_detect_count_` → `tracking`；丢帧 → `lost` |
| `tracking` | 确认目标 | 单帧丢失 → `temp_lost`；检测到更高优先级 → `switching`（仅全向感知版） |
| `temp_lost` | tracking 丢失一帧 | 重新检测到 → `tracking`；超阈值 → `lost` |
| `switching` | tracking 中全向相机出现更高优先级目标 | 主相机确认该优先级 → `detecting`；`temp_lost_count_ > 200` → `lost` |

### 2.2 `state_machine(bool found)`（tracker.cpp:180）

`state_machine` 是唯一修改 `state_` 与计数的函数，逐状态逻辑：

1. **`lost`**：`found` 为真 → `detecting`，`detect_count_ = 1`；否则保持不变（不返回目标）。
2. **`detecting`**：`found` → `detect_count_++`，达标即升 `tracking`；`!found` → `detect_count_ = 0`，回 `lost`。
3. **`tracking`**：`found` 保持；`!found` → `temp_lost_count_ = 1`，进入 `temp_lost`。
4. **`switching`**：`found` → 回 `detecting`；`!found` → `temp_lost_count_++`，超过 **200 帧**判 `lost`。
5. **`temp_lost`**：`found` → 回 `tracking`；`!found` → `temp_lost_count_++`，并根据目标是否为前哨站动态选择阈值：
   ```cpp
   if (target_.name == ArmorName::outpost)
     max_temp_lost_count_ = outpost_max_temp_lost_count_;
   else
     max_temp_lost_count_ = normal_temp_lost_count_;
   if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
   ```

---

## 3. `track` 主流程（单相机版，tracker.cpp:31）

**代码实现过程：**

1. **计算 dt 并更新时间戳**：`dt = delta_time(t, last_timestamp_)`，`last_timestamp_ = t`。该 dt 供 EKF 预测步使用，但注意——它由调用者统一传入，并非 EKF 内部自主计时。
2. **相机离线检测**：`state_ != "lost" && dt > 0.1s` → 记 `warn` 日志并强制 `state_ = "lost"`。时间间隔过长说明相机掉线，避免用陈旧数据继续跟踪。
3. **颜色过滤**：`armors.remove_if(a.color != enemy_color_)`，只保留敌方装甲板。
4. **前哨站顶部装甲板过滤（已注释）**：备用的 `oupost_reprojection_error` 判别方案，用于区分前哨站"正对"与"顶部"装甲板，默认关闭。
5. **按图像中心距离排序**：
   ```cpp
   armors.sort(距离图像中心近的在前);  // 图像中心 (1440/2, 1080/2)，含 TODO
   ```
   距离中心越近越容易被最先选中（画面中央目标通常最稳定）。
6. **按兵种优先级排序**：`armors.sort(a.priority < b.priority)`——**数字越小优先级越高**，最高优先级在前，保证 `front()` 始终是首选目标。
7. **状态分支**：`state_ == "lost"` → `set_target`（初始化），否则 → `update_target`（预测+更新），得到 `found`。
8. **状态机更新**：`state_machine(found)`。
9. **发散检测**：`!lost && target_.diverged()` → 记 `debug` 日志，置 `state_ = "lost"`，返回空列表。`diverged()` 表示 EKF 状态出现异常（位置/速度发散到不合理范围）。
10. **收敛检测**：统计 `target_.ekf().recent_nis_failures` 中失败的次数，超过 **40% × window_size** 即判定"收敛效果差"，同样强制 `lost`。
    - **NIS（Normalized Innovation Squared）** 是 EKF 的统计一致性指标：`新息ᵀ·S⁻¹·新息`，服从卡方分布。失败率高说明模型/协方差与观测失配。
11. 最终若 `state_ == "lost"` 返回空；否则返回 `{target_}`。

---

## 4. `track` 全向感知版（sentry 多相机，tracker.cpp:98）

该重载返回 `{switch_target, {target_}}` 二元组——`switch_target` 是"是否/切到哪个全向目标"的信号，供哨兵导航与决策使用。

**代码实现过程：**

1. 初始化 `switch_target`（默认空）与 `temp_target`：从 `detection_queue` 队首取出的全向感知检测结果（含该视角的装甲板、时间戳、`delta_yaw/delta_pitch` 云台补偿角）。
2. 同单相机版的前置处理：dt 计算、离线检测、排序（按中心距离、按优先级）。
3. **状态分支**扩展为 5 路：
   - `lost` → `set_target`；
   - `tracking` 且主相机出现**更高优先级**装甲板（`armors.front().priority < target_.priority`）→ `set_target` 直接切目标；
   - `tracking` 且**全向相机出现更高优先级**目标（`temp_target.armors` 非空、优先级更高、且当前目标已收敛 `target_.convergened()`）→
     ```cpp
     state_ = "switching";
     switch_target = {temp_target.armors, t, temp_target.delta_yaw, temp_target.delta_pitch};
     omni_target_priority_ = temp_target.armors.front().priority;
     found = false;
     ```
     记录 `switch_target` 与 `omni_target_priority_`，等待主相机画面中该优先级目标出现；
   - `switching` → `found = 主相机出现了 omni_target_priority_ 优先级的装甲板`；
   - `detecting` 且 `pre_state_ == "switching"` → `set_target`（用主相机新目标重新初始化 EKF）。
4. `pre_state_ = state_` 记录切换前的状态（供上一步判别）。
5. `state_machine(found)` 更新状态。
6. 发散检测 / `lost` 时返回 `{switch_target, {}}`（保留切换信号尽力完成导引）。
7. 成功则返回 `{switch_target, {target_}}`。

---

## 5. `set_target`：初始化目标（tracker.cpp:231）

**数学逻辑要点：**

- 从排序后列表取 `armors.front()`（优先级最高），调用 `solver_.solve(armor)` 解算位姿。
- 根据**兵种**选择 EKF 的初始化参数，关键差异在初始协方差 `P0_dig` 与旋转半径 `radius`、编号 `armor_num`：

```cpp
bool is_balance = 大装甲板 && (三 || 四 || 五号);   // 平衡步兵
```

| 目标 | 半径 radius | armor_num | P0_diag 差异 | 理由 |
|---|---|---|---|---|
| 平衡步兵 | 0.2 | 2 | 速度项 64、yaw 角速度项 100、半径/高度项 1 | 姿态变化剧烈，yaw 角速度初值置信度低 |
| 前哨站 outpost | 0.2765 | 3 | 半径项 81，旋转项 1e-4，yaw 角速度为 0 | 前哨站推进缓慢，几乎不转；半径先验更稳 |
| 基地 base | 0.3205 | 3 | 半径项 64，旋转项 1e-4，yaw 角速度为 0 | 基地静止，无旋转 |
| 普通步兵 | 0.2 | 4 | 半径项 64，旋转项 1，全为 1 | 通用配置，4 个灯条状态 |

> `P0`（11 维对角协方差初值）维度与 `ExtendedKalmanFilter` 状态向量一致：`[x,y,z,vx,vy,vz,yaw,vyaw,r,h,?]`，过大的初值表示"该状态很难确定、让滤波器去估"。

**代码实现过程：**

1. `armors.empty()` → 返回 `false`。
2. 取 `front()`，`solver_.solve(armor)`。
3. 按上述 4 分支构造 `Target(armor, t, radius, armor_num, P0_diag)`。
4. 返回 `true` 表示初始化成功。

---

## 6. `update_target`：预测 + 更新（tracker.cpp:266)

**数学逻辑要点（最近邻关联）：**

- 只用与当前目标**同名、同类型**的装甲板（`armor.name == target_.name && armor.type == target_.armor_type`）参与关联，避免把英雄装甲板误接入普通步兵目标。
- 对匹配到的装甲板逐一 `solver_.solve` → `target_.update(armor)`，支持"同一目标多块装甲板"（如大装甲板 4 面）时多个观测依次更新 EKF。

**代码实现过程：**

1. `target_.predict(t)`：EKF 利用上一帧状态与 `dt` 外推当前时刻的先验状态（含运动模型）。
2. 统计匹配个数 `found_count`；`0` → 返回 `false`（状态机据此走丢失分支）。
3. 对每个匹配装甲板：`solve` 后 `update(armor)`，用观测改进后验状态。
4. 返回 `true`。

> 注：`min_x`（画面最左侧）被计算但关联条件 `armor.center.x != min_x` 被注释，属遗留调试代码。

---

## 7. 与 `Target`/`ExtendedKalmanFilter` 的协同

Tracker 不自己维护 EKF，而是持有 `Target target_`。关键接口：

| 接口 | 作用 |
|---|---|
| `target_.predict(t)` | EKF 预测步（运动模型外推） |
| `target_.update(armor)` | EKF 更新步（观测方程 + 马氏/最近邻） |
| `target_.diverged()` | 状态发散（位置/速度越界）检测 |
| `target_.convergened()` | 目标是否已收敛（全向切换的前提条件） |
| `target_.ekf().recent_nis_failures` / `window_size` | NIS 滑动窗口收敛健康度 |
| `target_.priority` / `name` | 目标兵种优先级与装甲板编号 |

---

## 8. 函数调用关系总览（单相机）

```
track(armors, t)
  ├─ dt 计算 + 相机离线检测(lost)
  ├─ 颜色过滤 / 中心距离排序 / 优先级排序
  ├─ state=="lost"  ? set_target    : update_target
  │      │                 │             │
  │      │ solve(armor)    │ predict(t)  │
  │      │ Target(...)     │ solve+update│
  │      ▼                 ▼             ▼
  ├─ state_machine(found)
  ├─ 发散检测(diverged → lost)
  ├─ NIS 收敛检测(失败率>40% → lost)
  └─ lost ? {} : {target_}
```

---

## 9. 关键逻辑要点总结

| 环节 | 要点 |
|---|---|
| 状态机 | 五态抑制误检/漏检：`detecting` 攒帧确认、`temp_lost` 容忍遮挡、`switching` 实现换目标 |
| 目标排序 | 先中心距离（稳定优先）再兵种优先级（`priority` 数字越小越高） |
| 颜色过滤 | 单一敌人颜色假设，`remove_if` 直接剔除 |
| 相机离线 | `dt > 0.1s` 强判 `lost`，防止陈旧数据污染滤波器 |
| 参数分化 | 按兵种设置 EKF `P0`、半径、旋转先验，平衡步兵/前哨站/基地各有专值 |
| 收敛保护 | `diverged()` 状态发散 + NIS 失败率双重护栏，异常即回到 `lost` |
| 全向切换 | 仅在目标已收敛时才允许被更高优先级目标打断，并保留 `switch_target` 信令 |

## 10. 备注与待办

- 图像中心 `(1440, 1080)` 为写死的分辨率，代码中标注 `TODO`。
- `oupost_reprojection_error` 前哨站过滤逻辑被注释，目前通过 `outpost_max_temp_lost_count_` 单独容错。
- `min_x` 最左装甲板关联条件被注释，为遗留调试代码。
- `max_temp_lost_count_` 会被动态改写，故需保留 `normal_temp_lost_count_` 副本（易被忽略的隐式成员）。