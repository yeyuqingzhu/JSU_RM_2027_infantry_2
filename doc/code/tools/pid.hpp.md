# tools/pid.hpp

**概要**：位置式 PID 控制器通用件的对外接口声明——输出 = P + I + D，I 项带独立限幅（抗饱和），总输出再限幅；D 项作用在反馈量上（对反馈微分），设定值突变不产生微分冲击；构造参数 `angular` 可切到角度模式，误差与反馈增量按 (-π, π] 归一。

## 类

### `tools::PID`

- **职责**：保存控制周期与三组增益，在每个控制周期调用一次 `calc(set, fdb)` 得到控制量；内部维护 I 项累加值与上次反馈值；三项分输出对外可见，便于整定与调试。
- **典型用法**：
  ```cpp
  tools::PID pid(0.005, 8.0, 0.0, 0.02, 10.0, 3.0, true);  // 角度环，5 ms 周期
  float u = pid.calc(yaw_set, yaw_fdb);                     // 每个控制周期调用一次
  ```
- **备注**：持有状态（`iout` / `last_fdb_`），**非线程安全**；同一实例只能由控制线程以恒定间隔调用。所有参数成员均为 `const`，改增益需重建对象。

### 成员变量

| 成员 | 类型 | 含义 |
| --- | --- | --- |
| `pout` / `iout` / `dout`（public） | `float` | P / I / D 三项分输出，用于调试与整定 |
| `dt_` | `const float` | 控制周期，单位 s；同时是 `calc` 的期望调用间隔，D 项除以它 |
| `kp_` / `ki_` / `kd_` | `const float` | P / I / D 系数（ki 量纲含 1/s，kd 量纲含 s） |
| `max_out_` | `const float` | 总输出限幅（对称），单位同返回的控制量 |
| `max_iout_` | `const float` | I 项限幅（对称），用于防止积分饱和 |
| `angular_` | `const bool` | 是否角度模式：`true` 时用 `tools::limit_rad` 把误差与反馈增量归一到 (-π, π] |
| `last_fdb_` | `float` | 上次反馈值，供 D 项差分使用，初值 0 |

## 函数

### `PID(float dt, float kp, float ki, float kd, float max_out, float max_iout, bool angular = false)`

- **作用**：构造函数，用初始化列表保存控制周期、三组增益、两个限幅值与角度模式标志；内部状态由成员初值给出（0）。
- **参数**：
  - `dt`：控制周期，单位 s，必须 > 0（D 项要除以它）。
  - `kp` / `ki` / `kd`：P / I / D 项系数，量纲取决于 set / fdb 的物理量。
  - `max_out`：总输出限幅（对称），输出被限制在 `[−max_out, +max_out]`。
  - `max_iout`：I 项单独限幅（对称），防止积分饱和。
  - `angular`：是否角度模式，默认 `false`；为 `true` 时误差与反馈增量按 (-π, π] 归一。
- **返回**：无（构造函数）。
- **备注**：不做参数校验；`dt = 0` 会在 `calc` 中除零，`max_iout = 0` 使 I 项恒为 0，`max_out = 0` 使输出恒为 0。单位约定：角度模式下 set / fdb 为 rad。

### `float calc(float set, float fdb)`

- **作用**：执行一次 PID 计算：`e = set − fdb`（角度模式先归一），`de = last_fdb_ − fdb`（角度模式先归一），`pout = kp·e`，`iout = clamp(iout + ki·dt·e, ±max_iout)`，`dout = kd·de/dt`，返回 `clamp(pout + iout + dout, ±max_out)`。
- **参数**：`set` 目标值（设定值），量纲同 fdb，角度模式单位 rad；`fdb` 反馈值（当前测量值），量纲同 set。
- **返回**：限幅后的控制输出，范围 `[−max_out, +max_out]`（max_out > 0 时）。
- **备注 / 边界条件**：
  - `dt_ = 0` → D 项为 ±inf / NaN，返回值不可用（构造函数不校验）；
  - D 项取自「反馈的负变化」，与误差无关，因此改变 set 不会在 D 项上产生尖峰；kd > 0 时表现为阻尼；
  - 首拍时 `last_fdb_ = 0`，若第一次调用时反馈量不为 0，则 `de = 0 − fdb` 是一次虚假跳变，D 项会有冲击；需要平滑启动时应先空跑一次或保证反馈初值为 0（本类未提供初始化 `last_fdb_` 的接口）；
  - 积分抗饱和是 clamp 型（先累加后限幅），输出饱和时积分仍在累加，不是 back-calculation；
  - 调用本函数会修改 `pout` / `iout` / `dout` 与 `last_fdb_`，非线程安全。
