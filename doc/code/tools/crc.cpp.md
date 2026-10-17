# tools/crc.cpp

**概要**：用 256 项查找表实现 `tools/crc.hpp` 声明的 CRC8 / CRC16 计算与整帧校验，适合放在串口收发的实时路径上调用。

- **来源**：sp_vision_25 的 `tools/crc.cpp`（表为 RoboMaster 生态通用表，逐字节照搬）
- **依赖**：无第三方依赖（仅 `<cstdint>` 类型）
- **线程安全**：两张表都是 `const` 只读数据（内部链接），函数无状态，可并发调用
- **实现要点与关键步骤**：查表法比逐位运算快约一个数量级；CRC 参数与下位机协议绑定，**表与初值不得改动**
  - CRC8：`crc = CRC8_TABLE[crc ^ byte]`，初值 `0xff`
  - CRC16：`i = (crc ^ byte) & 0xff; crc = (crc >> 8) ^ CRC16_TABLE[i]`，初值 `0xffff`

## 常量与数据

### `constexpr uint8_t CRC8_INIT = 0xff;`

- **作用**：CRC8 初值。
- **备注**：`constexpr`，具有内部链接，仅本编译单元可见。

### `const uint8_t CRC8_TABLE[256];`

- **作用**：CRC8 查找表，由反射多项式 0x8C、低位优先逐位生成的 256 项表。
- **备注**：表内容与源工程逐字节一致，已用独立脚本按“反射多项式 0x8C”重新生成并比对通过。

### `constexpr uint16_t CRC16_INIT = 0xffff;`

- **作用**：CRC16 初值。

### `const uint16_t CRC16_TABLE[256];`

- **作用**：CRC16 查找表，由反射多项式 0x8408（0x1021 位反转）、低位优先生成。
- **备注**：表内容与源工程逐字节一致；按该表实现的算法即 CRC-16/MCRF4XX（参考向量 `"123456789"` → `0x6F91`，已验证）。

## 函数

### `uint8_t get_crc8(const uint8_t * data, uint16_t len)`

- **作用**：对前 `len` 字节做查表 CRC8 计算（不含 CRC8 本身）。
- **参数**：`data` 数据首地址（须可读 `len` 字节）；`len` 参与计算的字节数。
- **返回**：CRC8 值；`len == 0` 时返回初值 0xff。
- **备注**：实现要点——`while (len--)` 逐字节 `i = crc ^ byte; crc = CRC8_TABLE[i]`，无最终异或；不校验空指针。

### `bool check_crc8(const uint8_t * data, uint16_t len)`

- **作用**：比较“前 `len-1` 字节的 CRC8”与“最后一字节”，判断整帧是否完好。
- **参数**：`data` 帧首地址；`len` 帧总字节数（含 CRC8），要求 ≥ 1。
- **返回**：`true` 校验通过；`false` 数据被破坏。
- **备注**：`len == 0` 会回绕并越界读取，必须由调用方避免。

### `uint16_t get_crc16(const uint8_t * data, uint32_t len)`

- **作用**：对前 `len` 字节做查表 CRC16 计算（不含 CRC16 本身）。
- **参数**：`data` 数据首地址；`len` 参与计算的字节数（32 位）。
- **返回**：CRC16 值；`len == 0` 时返回初值 0xffff。
- **备注**：实现要点——`crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ byte) & 0xff]`，无最终异或。

### `bool check_crc16(const uint8_t * data, uint32_t len)`

- **作用**：从帧尾取出 2 字节 CRC16（小端）与“前 `len-2` 字节的 CRC16”比较。
- **参数**：`data` 帧首地址；`len` 帧总字节数（含 2 字节 CRC16），要求 ≥ 2。
- **返回**：`true` 校验通过；`false` 数据被破坏。
- **备注**：`len < 2` 会回绕并越界读取，必须由调用方避免。

## 迁移修补（行为无关）

1. `get_crc16` 中 `i = (crc16 ^ byte) & 0x00ff;` 改为显式窄化转换 `static_cast<uint8_t>(...)`：结果恒在 [0, 255]，取值不变，仅消除 `-Wconversion` 告警。
