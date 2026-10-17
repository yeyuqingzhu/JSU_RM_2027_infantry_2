# 大恒图像 Galaxy SDK — 厂商许可说明（非 MIT）

本目录（`io/daheng/`）中的 **SDK 头文件**（`include/*.h`）与 **预编译库**（`lib/`）由
**大恒图像（Daheng Imaging）** 以自有许可发布，**不适用**本仓库根目录的 MIT License，
本仓库亦不对其作出任何再许可。

| 内容 | 现状 | 授权 |
| --- | --- | --- |
| `include/*.h`（`GxIAPI.h`、`DxImageProc.h` 等） | **在本仓库内**，随上游 sp_vision_25 一并拷入，仅作编译期接口声明之用 | 版权归大恒图像，按厂商 SDK 许可使用 |
| `lib/*.so`、`*.cti` | **不在本仓库内**（`.gitignore` 已排除） | 需自行从大恒官网下载 Galaxy 相机 SDK，遵守其许可协议 |
| `daheng.cpp` / `daheng.hpp` / `CMakeLists.txt` | 本项目（上游）的胶水代码 | **MIT**（本仓库根目录 LICENSE） |

若你是权利人并认为本目录内容使用不当，请联系维护者，我们会立即移除。
