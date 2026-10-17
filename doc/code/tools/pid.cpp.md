# tools/pid.cpp

**概要**：`tools/pid.hpp` 中 `PID` 的实现——构造函数保存参数，`calc()` 完成一次位置式 PID 计算（P + I + D，I 项限幅、总输出限幅），另有文件内的对称限幅辅助函数 `clip()`。

## 函数

### `float clip(float value, float min, float max)`

- **作用**：对称限幅辅助函数，把 `value` 限制到 `[min, max]` 区间：`std::max(min, std::min(max, value))`。
- **参数**：`value` 待限幅的值；`min` 下界；`max` 上界（要求 min ≤ max）。
- **返回**：限幅后的值——小于下界返回 `min`，大于上界返回 `max`，否则原样返回。
- **备注**：
  - 定义在**全局命名空间**（不在 `namespace tools` 内），且源文件未加 `static`，因此会导出为外部链接符号，存在与其他翻译单元同名符号冲突（ODR）的隐患，也会触发 `-Wmissing-declarations` 告警；按「行为完全一致」的要求迁移时保持原样，详见迁移报告的缺陷清单；
  - 参数名 `min` / `max` 与 `std::min` / `std::max` 同名，但函数体内用的是带 `std::` 限定的版本，无影响；
  - `min > max` 时返回 `min`；`value` 为 NaN 时比较全为假，同样返回 `min`。

### `PID::PID(float dt, float kp, float ki, float kd, float max_out, float max_iout, bool angular)`

- **作用**：实现要点——初始化列表按成员声明顺序依次赋值 `dt_`、`kp_`、`ki_`、`kd_`、`max_out_`、`max_iout_`、`angular_`；函数体为空。
- **参数**：`dt` 控制周期（s）；`kp` / `ki` / `kd` P/I/D 系数；`max_out` 总输出限幅；`max_iout` I 项限幅；`angular` 是否角度模式。
- **返回**：无。
- **备注**：不做参数校验；所有参数成员为 `const`，构造后不可改。内部状态 `iout`（公有成员 `iout`）与 `last_fdb_` 由成员初值初始化为 0。

### `float PID::calc(float set, float fdb)`

- **作用**：实现要点（关键步骤，顺序即源码顺序）：
  1. `float e = angular_ ? limit_rad(set - fdb) : (set - fdb);` —— 误差项，角度模式用 `tools::limit_rad` 归一到 (-π, π]，避免跨越 ±π 时输出反向；
  2. `float de = angular_ ? limit_rad(last_fdb_ - fdb) : (last_fdb_ - fdb);` —— 微分项，取「上次反馈 − 本次反馈」，即**对反馈微分**而非对误差微分，设定值突变不会在 D 项产生尖峰；角度模式同样归一；
  3. `last_fdb_ = fdb;` —— 保存本次反馈，供下一拍差分；
  4. `pout = e * kp_;`
  5. `iout = clip(iout + e * dt_ * ki_, -max_iout_, max_iout_);` —— 先累加再限幅（clamp 型抗饱和）；
  6. `dout = de / dt_ * kd_;`
  7. `return clip(pout + iout + dout, -max_out_, max_out_);` —— 三项之和再夹到 ±max_out。
- **参数**：`set` 目标值；`fdb` 反馈值（量纲同 set，角度模式单位 rad）。
- **返回**：限幅后的控制输出，范围 `[−max_out, +max_out]`（max_out > 0 时）。
- **备注 / 边界条件**：
  - 依赖 `tools::limit_rad`（来自同目录 `math_tools.hpp` / `math_tools.cpp`），角度模式必须链接该实现；
  - `dt_ = 0` 时第 6 步除零，得到 ±inf / NaN，本实现不保护；
  - 首次调用时 `last_fdb_ = 0`，可能产生一次 D 项冲击；
  - 返回值恒在 ±max_out 内，即使内部某项已饱和；
  - 本函数修改对象状态（`iout`、`last_fdb_`、`pout`、`dout`），非线程安全，须以恒定周期在同一控制线程内调用。
