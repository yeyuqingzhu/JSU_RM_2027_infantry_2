# Solver 解算器详解

> 文件位置：`tasks/auto_aim/solver.cpp` / `tasks/auto_aim/solver.hpp`
> 命名空间：`auto_aim`

Solver 是整个自瞄（Auto Aim）系统中"从 2D 图像到 3D 空间位姿"的核心模块。它完成：

1. 通过 **PnP（Perspective-n-Point）** 从装甲板四个角点的像素坐标解算出装甲板在相机坐标系下的位置与姿态；
2. 通过 **坐标系链式变换**（相机 → 云台 → 世界）把位姿转换到云台/世界坐标系；
3. 通过 **重投影误差穷举** 对 PnP 解出的 yaw 角进行优化，消除 PnP 对"装甲板朝向"（尤其是 yaw）解算的歧义。

---

## 1. 关键常量与坐标系定义

### 1.1 物理常量（单位均为米）

```cpp
constexpr double LIGHTBAR_LENGTH = 56e-3;   // 灯条长度 56mm
constexpr double BIG_ARMOR_WIDTH   = 230e-3;  // 大装甲板宽度 230mm
constexpr double SMALL_ARMOR_WIDTH = 135e-3;  // 小装甲板宽度 135mm
```

### 1.2 装甲板 3D 模型点（以装甲板中心为原点）

```
        z (由灯条长度决定)
        ▲
        │   ●─────────●    ← 灯条上沿
        │  /           \
        │ ╱             ╲
        └──────────────→ y (由装甲板宽度决定)

  坐标系约定：x 指向装甲板法线（朝外），y 沿装甲板宽度方向，z 沿灯条长度方向。
  4 个点按顺序对应图像中左上、右上、右下、左下 4 个角点。
```

```cpp
BIG_ARMOR_POINTS = {
  {0, +BIG_ARMOR_WIDTH/2, +LIGHTBAR_LENGTH/2}, // 左上
  {0, -BIG_ARMOR_WIDTH/2, +LIGHTBAR_LENGTH/2}, // 右上
  {0, -BIG_ARMOR_WIDTH/2, -LIGHTBAR_LENGTH/2}, // 右下
  {0, +BIG_ARMOR_WIDTH/2, -LIGHTBAR_LENGTH/2}}; // 左下
```

由于装甲板法线方向（x 轴）没有对应的图像信息，PnP 只能确定"哪个面朝相机"，而无法唯一确定装甲板绕 x 轴转过的角度 —— **这就是 yaw 歧义问题的根源**，也是后续 `optimize_yaw` 存在的意义。

### 1.3 涉及的坐标系与变换

| 变换 | 矩阵/向量成员 | 含义 |
|---|---|---|
| 相机 → 云台 | `R_camera2gimbal_`, `t_camera2gimbal_` | 相机外参（标定所得） |
| IMU本体 → IMU绝对 | `R_imubody2imuabs`（由四元数实时得到） | 当前 IMU 姿态 |
| IMU本体 → 云台 | `R_gimbal2imubody_` | 外参（标定所得） |
| 云台 → 世界 | `R_gimbal2world_` | 实时更新 |

> 坐标统一约定：`p_world = R_gimbal2world_ * p_gimbal`，即所有矩阵都是从"源坐标系到目标坐标系"的旋转。

---

## 2. 构造函数：内外参加载

```cpp
Solver::Solver(const std::string & config_path)
  : R_gimbal2world_(Eigen::Matrix3d::Identity())
```

**数学逻辑要点：**

- 成员 `R_gimbal2world_` 初始化为**单位阵**，即默认假定"云台坐标系 == 世界坐标系"，在无 IMU 数据的离线调试场景下也能工作。
- YAML 中存储的外参 `R_gimbal2imubody`、`R_camera2gimbal` 是一维数组（9 个数），必须显式按 **RowMajor（行优先）** 构造 3×3 矩阵，否则数据顺序会错乱。

**代码实现过程：**

1. `YAML::LoadFile(config_path)` 加载 YAML。
2. 依次读取外参 `R_gimbal2imubody`、`R_camera2gimbal`、`t_camera2gimbal`（`std::vector<double>`）。
3. 用 `Eigen::Matrix<double,3,3,Eigen::RowMajor>(data.data())` 把 9 个数按行优先填入旋转矩阵；平移向量按 `Eigen::Matrix<double,3,1>` 构造。
4. 读取内参 `camera_matrix`（3×3）与畸变系数 `distort_coeffs`（1×5，k1,k2,p1,p2,k3）。
5. 用 `cv::eigen2cv` 把 Eigen 格式内参转为 OpenCV 的 `cv::Mat`，供 `solvePnP`/`projectPoints` 使用。

---

## 3. `set_R_gimbal2world`：IMU 姿态重投影

```cpp
void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}
```

**数学逻辑要点：**

- 该函数做的是**相似变换（similarity transform）**：把 IMU 在绝对系的姿态，搬运到云台坐标系下表达。
- 核心思路：
  - 先坐"IMU 的车"到 IMU 本体系：`R_gimbal2imubody_.transpose()`（外参本身约定 IMU 本体 → 云台，故转置得 云台 → IMU本体）。
  - 再执行 IMU 自身的旋转 `R_imubody2imuabs`。
  - 最后坐到"云台的车"上：`R_gimbal2imubody_`。
- 结果：即使云台相对 IMU 有固定安装偏差，也能把 IMU 的姿态正确表达为云台姿态。

**代码实现过程：**

1. 四元数 `q.toRotationMatrix()` 转旋转矩阵。
2. 左乘 `R_gimbal2imubody_.transpose()`，右乘 `R_gimbal2imubody_`，得到并保存 `R_gimbal2world_`。

> 该函数由外部（主循环）在每帧 IMU 数据到达时调用，保证 `R_gimbal2world_` 实时更新。

---

## 4. `solve`：核心解算流程

这是算法核心函数，负责从 2D 平面解算出 3D 坐标，最终为装甲板对象提供位置（xyz）与方向（ypr）。

**数学逻辑要点（坐标变换链）：**

```
像素点 --PnP--> 相机系(xyz, R_armor2camera)
    |  R_camera2gimbal_ * p_cam + t_camera2gimbal_
    v
云台系(xyz_in_gimbal, R_armor2gimbal)
    |  R_gimbal2world_ * p_gimbal
    v
世界系(xyz_in_world, R_armor2world)
    |  eulers(R, 2,1,0) + xyz2ypd
    v
ypr_in_world + ypd_in_world
```

**代码实现过程：**

1. 根据装甲板类型（大/小）选择对应的 3D 模型点 `object_points`。
2. **PnP 解算**：调用
   ```cpp
   cv::solvePnP(object_points, armor.points, camera_matrix_,
                distort_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE);
   ```
   - `rvec`：旋转向量（Rodrigues 形式），表示相机系下装甲板的姿态；
   - `tvec`：平移向量，即装甲板中心在相机系下的坐标。
   - 使用 `SOLVEPNP_IPPE`：要求 4 个角点**共面**（装甲板正是平面目标），能用解析闭式解，速度快且适合平面位姿估计。
3. **位置变换**：
   - `xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_`
   - `xyz_in_world  = R_gimbal2world_  * xyz_in_gimbal`
4. **姿态变换**：
   - `cv::Rodrigues(rvec, rmat)`：旋转向量 → 3×3 旋转矩阵 `R_armor2camera`。
   - `R_armor2gimbal = R_camera2gimbal_ * R_armor2camera`
   - `R_armor2world  = R_gimbal2world_  * R_armor2gimbal`
   - `eulers(R, 2, 1, 0)`：旋转矩阵 → 欧拉角。参数 `2,1,0` 指定旋转顺序 **先绕 Z 轴（yaw，0 下标）、再绕 Y 轴（pitch，1 下标）、最后绕 X 轴（roll，2 下标）**，得到 `ypr_in_gimbal` / `ypr_in_world`。
5. **直角→球坐标**：`armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world)`，得到 `(yaw, pitch, distance)`——距离信息供射击/弹道规划使用。
6. **平衡步兵特判**：若装甲板是平衡步兵（大装甲板且编号为三/四/五），则直接 return，跳过 yaw 优化。原因见注释：**"平衡不做 yaw 优化，因为 pitch 假设不成立"**。
   ```cpp
   auto is_balance = (armor.type == ArmorType::big) &&
                     (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                      armor.name == ArmorName::five);
   ```
   （`optimize_yaw` 依赖"装甲板 pitch 为 ±15°"的强假设，平衡步兵姿态剧烈变化时该假设失效。）
7. 否则调用 `optimize_yaw(armor)` 修正 yaw。

---

## 5. `reproject_armor`：反向重投影

把"假设的世界坐标 + 假定的 yaw/pitch"反算回像素坐标，供 yaw 优化时计算误差。

**数学逻辑要点：**

1. **生成假想姿态矩阵** `R_armor2world`：由假定 yaw 与固定 pitch 构建。
   - 假定 **pitch = 15°**（outpost 前哨站为 **-15°**），这是哨兵站立的先验条件。
   - 按 ZYX 欧拉角顺序构造旋转矩阵：
     ```
     R_armor2world =
       [ cosY·cosP  -sinY  cosY·sinP ]
       [ sinY·cosP   cosY  sinY·sinP ]
       [  -sinP       0      cosP     ]
     ```
2. **反向变换回相机系**（数学上为正向链的逆）：
   - `R_armor2camera = R_camera2gimbal_.T * R_gimbal2world_.T * R_armor2world`
   - `t_armor2camera = R_camera2gimbal_.T * (R_gimbal2world_.T * t_armor2world - t_camera2gimbal_)`
3. **旋转矩阵 → 旋转向量**：`cv::Rodrigues(R_armor2camera_cv, rvec)`。
4. **针孔投影**：`cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points)`，把假想装甲板投影回 2D 平面。

**代码实现过程（逐行）：**

1. 计算 `sin_yaw/cos_yaw`、`sin_pitch/cos_pitch`。
2. 按名字判断 pitch 符号（`outpost` 为 -15°）。
3. 填 3×3 矩阵 `R_armor2world`。
4. 计算 `R_armor2camera` 和 `t_armor2camera`。
5. `eigen2cv` 转成 OpenCV，`Rodrigues` 求 rvec，构造 `cv::Vec3d tvec`。
6. 依类型选模型点，`projectPoints` 返回 4 个像素点。

---

## 6. `optimize_yaw`：穷举 yaw 获取最优解

**数学逻辑要点：**

- **问题**：PnP 解出的 `R_armor2camera` 中，yaw 方向（装甲板面朝向）存在歧义，接缝角度误差大。
- **思想**：以未知的 yaw 为变量，固定 pitch（±15° 先验），把装甲板重新投影回图像；当重投影结果与实测角点**像素误差最小**时，即认为该 yaw 最可信。本质是 **1 维穷举搜索的最小二乘 / 重投影误差最小化**。
- **搜索范围**：以当前云台 yaw（`gimbal_ypr[0]`）为中心，±70°（共 140°），**每 1° 一个采样点**。
  - `SEARCH_RANGE = 140`（度）
  - `yaw0 = limit_rad(gimbal_ypr[0] - SEARCH_RANGE/2 * π/180)`——记录起始角（弧度制）。
  - `limit_rad` 把角度限制到 `(-π, π]`。

**代码实现过程：**

1. 用 `eulers(R_gimbal2world_, 2, 1, 0)` 取当前云台 yaw 作为搜索中心。
2. 计算出起始角 `yaw0`。
3. 初始化最优记录：`min_error = 1e10`，`best_yaw = armor.ypr_in_world[0]`（默认保留 PnP 结果）。
4. 循环 140 次，每次取 `yaw = limit_rad(yaw0 + i * π/180)`：
   - 调用 `armor_reprojection_error(armor, yaw, (i - SEARCH_RANGE/2)*π/180)`
   - 若误差小于当前最优，则更新 `min_error` 与 `best_yaw`。
   - 注意：第三个参数 `inclined` 实际是 `(i - 70)°`，即**以 -70° → +70° 递增的"倾斜角"**，用于 SJTU 代价函数（当前默认未启用）。
5. 输出：保存原始 yaw 到 `armor.yaw_raw`，用最优 `best_yaw` 覆盖 `armor.ypr_in_world[0]`。

> `armor.yaw_raw` 保留 PnP 原始解，`ypr_in_world[0]` 为优化后的最终 yaw，二者对比可用于调试。

---

## 7. 误差计算：`armor_reprojection_error` 与 `SJTU_cost`

### 7.1 `armor_reprojection_error`

```cpp
double Solver::armor_reprojection_error(const Armor & armor, double yaw, const double & inclined) const
{
  auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);
  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  // auto error = SJTU_cost(image_points, armor.points, inclined);
  return error;
}
```

**数学逻辑要点：**

- 对 4 个角点累加**观测像素点与重投影像素点的欧氏距离**，作为当前 yaw 假设下的代价。
- 误差越小 → 假设的 yaw 越接近真实。
- 默认使用朴素的"点到点欧氏距离和"；`SJTU_cost` 版本被注释掉、未启用。

### 7.2 `SJTU_cost`（备用实现）

来自上海交通大学的代价函数，把"形状保真"拆成两部分：

```cpp
pixel_dis   = ( 0.5 * (|start 点差| + |end 点差|) + |标准边长 - 实际边长| ) / 标准边长
angular_dis = 标准边长 * |边方向夹角| / 标准边长   (= |边方向夹角|，abs_angle 范围 0~π)
cost_i = (pixel_dis·sin(inclined))² + (angular_dis·cos(inclined))² · 2
cost   = Σ sqrt(cost_i)
```

**数学逻辑要点：**

- `pixel_dis`：**位移-尺度代价** = 起点/终点中点平均像素偏差 + 线段长度差，除以标准边长归一化。
  - 反映：起始点是否重合 + 线段伸缩是否一致（近似平移 + 缩放）。
- `angular_dis`：**角度代价** = 两条对应线段的方向夹角（`get_abs_angle` 返回 `0~π`），再乘标准边长归一化。
- `inclined`（倾斜角加权）：
  - 用 `sin(inclined)` 调制位移项、`cos(inclined)` 调制角度项——**置信度随观察倾角在"平移/缩放"与"旋转"之间权衡**。
  - 角度项额外乘以 2.0，加重方向一致性权重。
  - 注释说明：**"重投影像素误差越大，越相信斜率"**——即观测越不精确，越依赖线段方向信息。
- 每条边取 `sqrt(cost_i)`（L2 化）再求和，抑制单条边误差的过强影响。

> 该函数当前在 `armor_reprojection_error` 中被注释，作为可切换的进阶方案保留。

---

## 8. `world2pixel`：世界坐标 → 像素坐标

**数学逻辑要点：**

- 计算世界→相机的刚体变换（正向链的逆）：
  - `R_world2camera = R_camera2gimbal_.T * R_gimbal2world_.T`
  - `t_world2camera = -R_camera2gimbal_.T * t_camera2gimbal_`
- 投影前先做**可见性剔除**：只保留在相机前方（`camera_point.z() > 0`）的点，避免投影出负深度/不可见的伪像素。

**代码实现过程：**

1. 组装 `R_world2camera` 与 `t_world2camera`。
2. `eigen2cv` 转 OpenCV，得到 rvec/tvec。
3. 遍历世界点，逐个变换到相机系；`z > 0` 才保留。
4. 若全部不可见，返回空 `vector<cv::Point2f>`。
5. `cv::projectPoints` 投影有效点，返回像素坐标。

---

## 9. 函数调用关系总览

```
主循环（每帧）
  │ IMU 四元数
  ├─► set_R_gimbal2world(q)          // 更新 R_gimbal2world_
  │ 检测器输出的 armor
  └─► solve(armor)
        │  模型点 + armor.points
        ├─► cv::solvePnP(IPPE)       // rvec, tvec
        ├─► 平移/旋转链式变换        // gimbal / world 两套坐标
        ├─► eulers / xyz2ypd        // ypr / ypd
        ├─► 平衡步兵? ──是──► return　// 跳过优化
        │      否
        └─► optimize_yaw(armor)
              │ 以云台 yaw 为中心 ±70° 逐度穷举
              └─► armor_reprojection_error ─► reproject_armor
                                             （[可选] SJTU_cost 加权误差）
```

---

## 10. 数学逻辑要点总结

| 环节 | 核心数学思想 |
|---|---|
| PnP | 4 点共面约束 + IPPE 闭式解，从像素恢复 3D 位姿（`rvec`, `tvec`） |
| 坐标系变换 | 组合刚体变换：`R·p + t`，相机→云台→世界逐级传递 |
| IMU 姿态投影 | 相似变换 `Rᵀ·R_imu·R` 把绝对姿态表达在云台系 |
| yaw 歧义求解 | 固定 pitch(±15°) 先验，1-D 穷举（±70°，步长 1°）最小化重投影误差 |
| 重投影误差 | 模型点经 `R_armor2world`(ZYX) → 逆变换回相机系 → 针孔投影 → 与观测点比欧氏距离 |
| SJTU 代价(备用) | 位移/缩放代价 + 旋转代价，以观察倾角 `sin/cos` 加权，角度项权重 ×2 |
| 球坐标输出 | `xyz2ypd` 给出距离，供弹道/射击决策 |
| 平衡步兵 | 强假设失效，特判跳过 yaw 优化 |

---

## 11. 备注与待办

- `oupost_reprojection_error` 函数已实现但**未在本文件内被调用**（注释注明"函数本身没有被真正用到"），逻辑与 `reproject_armor` + `armor_reprojection_error` 重复，仅 pitch 可传入，疑似为前哨站调试预留。
- `SJTU_cost` 被注释，当前 yaw 优化使用朴素欧氏距离误差。
- 假定 pitch 为 ±15° 是**强先验**，仅对哨兵/步兵等常规姿态成立，平衡步兵（三/四/五号大装甲板）因此被跳过优化。