# 问题与解决方案（Known Issues / Troubleshooting）

> 本文件记录本项目在**构建、编译、测试、运行、环境与工具**上实际踩过的坑。
> **使用规则：遇到问题时先查这里。已记录的问题直接按「解决方案」执行，不要重新推导、不要重复思考。**
> **维护规则：解决了一个此前没有记录的新问题后，必须在此追加条目（见文末模板）。**

---

## 0. 快速索引（按症状查）

| 症状关键词 | 条目 |
| --- | --- |
| `cmake` 不读构建脚本 / 找不到目标 | [A-1](#a-1-cmakeliststxt-大小写导致-cmake-不识别) |
| `pkg-config --modversion opencv4` 失败 | [A-2](#a-2-本机-opencv-没有-pkg-config-文件) |
| `undefined reference to fmt::...` / spdlog 链接失败 | [A-3](#a-3-fmt-未链接spdlog-是-fmt_external-构建) |
| `find_package` 找不到目标名 | [A-4](#a-4-各依赖的-cmake-target-名) |
| `Cannot find source file: color_track_demo/main.cpp` / `CMake Generate step failed` | [A-5](#a-5-顶层-cmakelists-引用未拷入的-color_track_demo-导致-configure-失败) |
| `openvino_2024.6.0` 路径不存在 / OpenVINO 版本不符 | [A-6](#a-6-openvino-版本路径被写死) |
| `tools::Frame` / `create_yolov8s` 未声明 / `class template argument deduction failed` | [B-6](#b-6-上游测试引用了分层重构时已移出-tools-的类型与工厂函数) |
| `?? .dsh-meow/` 记忆库变成未跟踪 | [A-7](#a-7-gitignore-被上游版本覆盖导致-dsh-meow-变成未跟踪) |
| 头文件单独包含就编译失败 / 换个顺序才报错 | [B-1](#b-1-隐藏-include-依赖头文件不自包含) |
| `-Wreorder` 告警 | [B-2](#b-2-wreorder初始化列表顺序与成员声明不一致) |
| `-Wshadow` / `-Wconversion` 告警 | [B-3](#b-3-wshadow--wconversion) |
| `request for member 'id' in ... non-class type` | [B-4](#b-4-orderedqueueframet-要求-framet-带-int-id) |
| `redefinition of ...` / `default argument given twice` | [B-5](#b-5-重复声明重复定义并行写入同一文件) |
| Eigen `Assertion index >= 0 && index < size() failed` / 退出码 134 | [C-1](#c-1-ekf-update-要求量测维--4) |
| 程序 abort 但看不到任何输出 | [C-2](#c-2-abort-时-printf-输出丢失) |
| 命令跑很久被移到后台 / 超时 | [C-3](#c-3-单条命令串太多步骤导致超时) |
| 视觉端收不到云台帧 / 构造卡在 `First q received` 前 / 找不到 `0x5A 0xA5` / crc8 一直失败 | [C-4](#c-4-视觉端收不到云台帧下位机协议版本不匹配) |
| 改完 gimbal(v3.0) 仍连不上下位机 / `[YAML] com_port not found!` / `Failed to open serial` / `SocketCAN::open() failed` / 用错测试程序 | [C-5](#c-5-改完-iogimbalv30-适配后仍连不上下位机) |
| `gimbal_test` 一直收不到首帧 / 进程静默卡死一行输出都没有 / 每秒刷 `No serial data for extended period` 与 `High TX latency: 551ms` | [C-6](#c-6-gimbal_test-一直收不到首帧且进程静默卡死) |
| `/tmp` 里的临时文件下一次调用就没了 | [D-1](#d-1-tmp-在每次-bash-调用之间不保留) |
| 写工作区外的文件被拒绝（沙箱） | [D-2](#d-2-沙箱拒绝写工作区外的文件) |
| `file has not been read` / `file changed since it was read` | [D-3](#d-3-编辑被拒未读过或文件已被外部改动) |
| 空目录 clone 后消失 | [D-4](#d-4-git-不跟踪空目录) |
| 拷贝后本地已改文件被覆盖 / `git status` 出现 `MM`、`AM` | [D-5](#d-5-整目录拷贝静默覆盖本地已定制文件) |
| 删除已暂存的目录后 `git status` 变 `AD` / 清理占位目录 | [D-6](#d-6-删除已-git-add-但未提交的目录用-git-rm--r--f) |
| 解压 `session.v4.jsonl.zstd` 只得几百字节 | [E-1](#e-1-dsh-会话转录是多帧-zstd) |

---

## A. 构建与依赖

### A-1 `cmakelists.txt` 大小写导致 CMake 不识别

- **症状**：仓库里有构建脚本文件，但 `cmake` 完全不读它（或提示找不到 `CMakeLists.txt`）。
- **原因**：Linux 文件名大小写敏感，CMake 只识别 **`CMakeLists.txt`**；本项目初始提交里是 0 字节的 `cmakelists.txt`。
- **解决方案**：`git mv cmakelists.txt CMakeLists.txt`（同步修正所有文档引用）。同理 `readme.md` 的惯例大小写是 `README.md`。
- **已处理**：2026-10-11 已改名。

### A-2 本机 OpenCV 没有 pkg-config 文件

- **症状**：`pkg-config --modversion opencv4` → `Package opencv4 was not found`。
- **原因**：本机 OpenCV 4.10.0 是装在 `/usr/local` 的自编译版本，只提供 **CMake config**（`/usr/local/lib/cmake/opencv4/OpenCVConfig.cmake`），没有 `.pc` 文件。
- **解决方案**：**一律用 CMake 的 `find_package(OpenCV REQUIRED)`**，不要用 pkg-config。手工编译时头文件加 `-I/usr/local/include/opencv4`、库加 `-L/usr/local/lib -lopencv_core -lopencv_imgproc -lopencv_videoio`，运行时需要 `LD_LIBRARY_PATH=/usr/local/lib`。

### A-3 `fmt` 未链接（spdlog 是 FMT_EXTERNAL 构建）

- **症状**：链接期 `undefined reference to fmt::v11::...`（即使只用了 spdlog 的日志宏也可能出现）。
- **原因**：本机 `libspdlog-dev` 1.15.3 是 `SPDLOG_FMT_EXTERNAL=1` 构建版，**任何** spdlog 格式化在链接期都要 libfmt；且本项目 `logger.cpp`、`recorder.cpp` 直接使用 `fmt::format` 与 `<fmt/chrono.h>`。
- **解决方案**：`tools/CMakeLists.txt` 中显式 `find_package(fmt REQUIRED)` + 链接 `fmt::fmt`。**不要移除这个依赖。**

### A-4 各依赖的 CMake target 名

- **症状**：`find_package` 成功但 `target_link_libraries` 报目标不存在。
- **解决方案**：本项目已确认可用的写法（照抄即可）：

| 依赖 | 版本 | target |
| --- | --- | --- |
| OpenCV | 4.10.0（`/usr/local`） | `${OpenCV_LIBS}` |
| Eigen3 | 3.4 | `Eigen3::Eigen` |
| yaml-cpp | 0.8.0 | `yaml-cpp::yaml-cpp` |
| spdlog | 1.15.3 | `spdlog::spdlog` |
| nlohmann_json | 3.11.3 | `nlohmann_json::nlohmann_json` |
| fmt | 系统包 | `fmt::fmt` |
| 线程 | — | `Threads::Threads` |

### A-5 顶层 CMakeLists 引用未拷入的 color_track_demo 导致 configure 失败

- **症状**：`cmake -B build -S .` 在 Generate 阶段直接失败：

  ```
  CMake Error at CMakeLists.txt:113 (add_executable):
    Cannot find source file:

      color_track_demo/main.cpp
  ...
  CMake Generate step failed.  Build files cannot be regenerated correctly.
  ```

- **原因**：顶层 `CMakeLists.txt` 是 sp_vision_25 原版，第 113 行 `add_executable(color_track_demo color_track_demo/main.cpp)`；本次整目录拷贝**没有带入 `color_track_demo/` 目录**。CMake 在 configure/generate 阶段要求 `add_executable` 的源文件**全部存在**，缺一个就整个工程无法生成构建文件（不是只跳过该 target）。
- **解决方案**（三选一，按需）：
  1. **补拷目录**：`cp -r <源仓库>/color_track_demo .`（另需 ONNX Runtime 1.23.2 位于 `$HOME/ProgramFiles/onnxruntime/1.23.2`，脚本里写死的 `ORT_DIR`）；
  2. **注释掉该段**：注释 `CMakeLists.txt` 第 108–118 行的 `##################color_track_demo##################` 整段；
  3. **换回阶段 1 脚本**：`git show :CMakeLists.txt > CMakeLists.txt`（只构建 `tools`，用于宿主侧验证）。
- **自查方式**：`grep -n 'add_executable' CMakeLists.txt` 后逐一 `ls` 源文件；本项目实测**只有 `color_track_demo/main.cpp` 一个缺失**。
- **已处理（2026-10-17）**：采用**方案 1**——
  `cp -r /home/yeyuqingzhu/C++/localcreated/summertraining/original_source/sp_vision_25/color_track_demo .`
  上游该目录只有 `main.cpp` 一个文件；本机 ONNX Runtime 1.23.2 已在 `$HOME/ProgramFiles/onnxruntime/1.23.2`，无需另装。
  随后 `cmake -B build_full -S .` **configure + generate 全部通过**，方案 2（注释该段）与方案 3（换回阶段 1 脚本）都不再需要。
- **验证**：`cmake --build build_full -j32` 全量编译通过（35 个可执行，见 [overview §6.2](overview.md)）。

### A-6 OpenVINO 版本路径被写死

- **症状**：`grep -rn 'openvino_2024' --include=CMakeLists.txt .` 命中 4 处（顶层 `CMakeLists.txt` 与 `tasks/auto_aim`、`tasks/auto_buff`、`tasks/omniperception` 各自一份 `set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")`），而本机 `ls /opt/intel` 只有 `openvino_2023.3.0`。
- **原因**：上游在 OpenVINO 2024.6.0 环境开发，把 SDK 路径硬编码进了 4 个构建脚本（未做版本探测）。
- **实测**：直接 `cmake -B ... -S .` 时 **configure 阶段并未因 OpenVINO 报错**（`find_package(OpenVINO REQUIRED COMPONENTS Runtime)` 通过，说明 `OpenVINO_DIR` 指向不存在目录时会回退到 CMake 默认搜索路径并命中本机已安装版本）。**2023.3.0 与 2024.6.0 的 API 差异已实测（2026-10-17 全量构建）：`auto_aim`/`auto_buff`/`omniperception` 与全部可执行文件零 openvino 相关错误，无需改脚本。**
- **解决方案**：先按现状 configure；只有在编译期报 `openvino` 相关 `not declared` / `no member` 时才处理——把 4 处 `set(OpenVINO_DIR ...)` 改成实际安装路径（`/opt/intel/openvino_2023.3.0/runtime/cmake`），或先 `source /opt/intel/openvino_2023.3.0/setupvars.sh` 再 configure。**改构建脚本前先确认 §7 第 5 项的脚本版本选择，不要两边都改。**

### A-7 .gitignore 被上游版本覆盖导致 .dsh-meow 变成未跟踪

- **症状**：`git status --short` 出现 `?? .dsh-meow/`，但 `AGENT.md`/文档写着"AI 记忆库已 gitignore"；`.gitignore` 状态为 `AM`。
- **原因**：整目录拷贝把上游 `.gitignore` 覆盖到了工作区，上游版本**不含 `.dsh-meow/`**；阶段 1 那份（仍保存在 index 中）首行就有该规则。`git show :.gitignore` 可见原版，`git show HEAD:.gitignore` 报"不在 HEAD 中"（初始提交未包含该文件）。
- **解决方案**：在 `.gitignore` 末尾补一行 `.dsh-meow/`；或整体恢复原版 `git show :.gitignore > .gitignore`（原版只多了记忆库等本地规则，与上游规则不冲突）。**不要把 `.dsh-meow/` 提交进 git。**
- **已处理**：未处理（文档已标注风险，提交前必须确认）。

---

## B. 编译与告警

### B-1 隐藏 include 依赖（头文件不自包含）

- **症状**：单独 `#include "tools/xxx.hpp"` 编译失败；或同样的头文件换个包含顺序就报错；或换编译器版本后突然报错。
- **原因**：头文件使用了某类型却依赖别的头文件间接引入（sp_vision_25 原文件普遍如此）。
- **实际案例**：`recorder.hpp` 用 `std::atomic` 未含 `<atomic>`；`thread_pool.hpp` 用 `std::unordered_map`/`std::runtime_error` 未含 `<unordered_map>`/`<stdexcept>`；`yaml.hpp` 用 `std::exit` 未含 `<cstdlib>`；`extended_kalman_filter.hpp` 用 `std::string` 未含 `<string>`；`ransac_sine_fitter.cpp` 用 `std::iota` 未含 `<numeric>`。
- **解决方案**：头部补齐自己的 include。**检查命令**（每个头文件都要过一遍）：

  ```bash
  echo '#include "tools/xxx.hpp"' | g++ -fsyntax-only -x c++ - -I. -I/usr/local/include/opencv4 -I/usr/include/eigen3 -std=c++17 -Wall -Wextra
  ```

### B-2 `-Wreorder`：初始化列表顺序与成员声明不一致

- **症状**：`warning: 'X::member_' will be initialized after ... [-Wreorder]`。
- **原因**：构造函数初始化列表的顺序与类成员**声明**顺序不一致（实际初始化顺序永远按声明顺序）。
- **解决方案**：把初始化列表改成与声明顺序一致即可（成员之间若互不依赖，这是纯行为中性的修复）。
- **已处理**：`recorder.cpp` 的 `Recorder::Recorder`。

### B-3 `-Wshadow` / `-Wconversion`

- **症状**：`declaration of 'i' shadows a previous local`；`conversion from 'int' to 'uint8_t' may change value`。
- **解决方案**：循环变量改名（`i` → `idx`）；窄化处加显式转换 `static_cast<uint8_t>(...)`（确认取值范围不变）。
- **已处理**：`math_tools.cpp`（idx）、`crc.cpp`（static_cast）。

### B-4 `OrderedQueue<FrameT>` 要求 `FrameT` 带 `int id`

- **症状**：`error: request for member 'id' in 'item', which is of non-class type 'const int'`。
- **原因**：`tools::OrderedQueue` 是按 id 保序的队列，内部固定访问 `item.id`；**不能用 `int` 等裸类型实例化**。
- **解决方案**：模板参数传入带公有 `int id` 成员的结构体，例如：

  ```cpp
  struct FrameLike { int id; /* 载荷 */ };
  tools::OrderedQueue<FrameLike> q;
  ```

- **注意**：id 必须从 `1` 开始连续投递，否则后续元素会永久滞留在 buffer_ 中（见 [doc/code/tools/thread_pool.hpp.md](code/tools/thread_pool.hpp.md)）。

### B-5 重复声明/重复定义（并行写入同一文件）

- **症状**：`redefinition of 'template<class T> T tools::square(...)'`、`default argument given for parameter 5 ... [-fpermissive]`。
- **原因**：**多个 agent/进程并行写同一个文件**，一方在另一方写完后又整体覆盖，产生重复内容。
- **解决方案**：
  1. 并行分工时保证**文件级互斥**（一组文件只由一方负责）；
  2. 任一方写入期间，其它方**不要编译/编辑**同一文件；
  3. **所有并行写入结束后，必须由主 agent 统一 clean 重编译验收**，不能只信各方的自测。
- **已处理**：2026-10-11 阶段 1 迁移时 `math_tools.hpp` 发生此问题。

### B-6 上游测试引用了分层重构时已移出 `tools` 的类型与工厂函数

- **症状**：全量构建时**只有 `tests/camera_thread_test.cpp` 一个目标失败**，报错（`build_full/build.log` 原文，13 条 error 同源）：

  ```
  tests/camera_thread_test.cpp:29:21: error: class template argument deduction failed:
  tests/camera_thread_test.cpp:29:21: error: no matching function for call to 'OrderedQueue()'
  tests/camera_thread_test.cpp:32:26: error: 'Frame' is not a member of 'tools'
  tests/camera_thread_test.cpp:71:23: error: 'create_yolov8s' is not a member of 'tools'
  ```
- **原因**：`tools/thread_pool.hpp` 在阶段 1 迁移时按分层原则**删除了 `struct Frame`、`create_yolo11s()`、`create_yolov8s()`**（它们依赖 `tasks/auto_aim/yolo.hpp`，属 auto_aim 层），并把 `OrderedQueue` 改成模板 `OrderedQueue<FrameT>`。上游 `tests/camera_thread_test.cpp` 仍按老 API 写（`tools::OrderedQueue frame_queue;`、`tools::Frame`、`tools::create_yolov8s`），于是全量构建时必挂。**这是文档已预告的后果，不是环境问题**（见 [code/tools/thread_pool.hpp.md](code/tools/thread_pool.hpp.md) 文末差异表）。
- **解决方案**（最小改动，保持 tools 分层不变）：**在测试文件内部按使用处补齐**，不给 `tools` 加回上层依赖，也不急着给 auto_aim 加公共 API：
  1. 文件顶部匿名 namespace 里定义 `struct Frame { int id = 0; cv::Mat img; std::chrono::steady_clock::time_point t; Eigen::Quaterniond q; std::list<auto_aim::Armor> armors; };`（`id` 必须从 1 起连续，见 B-4）；
  2. 同处写 `create_yolos(config_path, number, debug)`：循环 `push_back(auto_aim::YOLO(config_path, debug))`，与上游 `create_yolov8s` 等价；
  3. 全局队列改成模板实例化 `tools::OrderedQueue<Frame> frame_queue;`，`detect_frame(Frame &&, auto_aim::YOLO &)`；
  4. `tools::Frame frame{frame_id, img.clone(), t};` 改为默认构造后逐字段赋值（聚合初始化只给 3 个成员在 `-Wmissing-field-initializers` 下不干净）；
  5. 补 `#include <Eigen/Geometry>`（`Eigen::Quaterniond`）与 `#include <list>`，避免依赖间接包含（见 B-1）。
- **已处理**：2026-10-17，`tests/camera_thread_test.cpp`。改后 `cmake --build build_full --target camera_thread_test` 通过，全量构建 35/35 可执行全部生成。
- **迁移到别处的判据**：以后若其它文件也需要 `Frame`/`create_yolo*`，再把它们提升为 `tasks/auto_aim/` 的公共头（例如 `tasks/auto_aim/frame.hpp`），**不要**放回 `tools/`。

---

## C. 运行与测试

### C-1 EKF `update` 要求量测维 ≥ 4

- **症状**：运行期 `Assertion 'index >= 0 && index < size()' failed`（Eigen），进程 abort，退出码 134。
- **原因**：`ExtendedKalmanFilter::update` 末尾固定写 `data["residual_yaw/pitch/distance/angle"] = residual[0..3]`，**隐含要求量测维数 m ≥ 4**；m < 4 时 debug 下断言、Release 下越界读。
- **解决方案**：调用 `update` 时保证量测向量维数 ≥ 4（yaw/pitch/distance/angle）。
- **附带**：默认构造的 `ExtendedKalmanFilter`（`= default`）其 `x_add` 是空 `std::function`，调用 `predict/update` 会抛 `std::bad_function_call`——要用带 `x0/P0` 的构造函数。

### C-2 abort 时 printf 输出丢失

- **症状**：程序崩了，但终端/日志里**什么输出都没有**，无法定位崩在哪一步。
- **原因**：stdout 被重定向到管道/文件时是**块缓冲**，进程 abort 不会 flush，缓冲区内容全部丢失。
- **解决方案**：用 `stdbuf -o0 -e0 ./程序` 运行；或在 `main` 开头 `setvbuf(stdout, NULL, _IONBF, 0);`。

### C-3 单条命令串太多步骤导致超时

- **症状**：命令被判定为超时并"移到后台"，看起来像卡死；或者误以为程序死锁。
- **原因**：把「全量 clean 重编译 + 14 个头文件语法检查 + 冒烟测试编译并运行」串在同一条 shell 命令里，总耗时超过默认超时。
- **解决方案**：拆分成多条命令分别执行；确需长时间运行时用后台方式启动（`run_in_background`）再用 `job_output` 取结果，**不要盲目中断**。

### C-4 视觉端收不到云台帧（下位机协议版本不匹配）

- **症状**：`io::Gimbal` 构造后长时间没有 `[Gimbal] First q received.`（v2.0 是永久卡死，v3.0 是 2 s 后 warn）；日志里 `[Gimbal] Bad frame candidate: head=...`；`error_count` 持续增长；`q(t)` 一直阻塞；或者波特率对、线也接对，但下位机/视觉端「各自都在发、谁也没收到」。
- **原因**：更换下位机（2027 赛季 lt-2027 哨兵板）后**线协议整体换了**，旧实现找不到任何合法帧：
  - 帧界：旧 `0x5A 0xA5` / `0x7F 0xFE` → 新 `0xFF` / `0x0D` + crc8；
  - 长度：旧 43 B（云台→视觉）/ 29 B（视觉→云台）→ 新 42 B / **28 B**；
  - 单位：旧**弧度** → 新**度**；旧下行是**相对增量**，新下行要**绝对角**；
  - 波特率：旧 115200 → 新 **921600**（下位机注释里的 460800 是错的，以 `Src/usart.c` 的 `huart1.Init.BaudRate` 为准）；
  - 链路是**请求-响应式**：下位机只有成功解析到一帧 `UsbCtrl` 才回发遥测，视觉端不发就永远收不到（旧实现把「等首帧」写成无超时阻塞 → 永久卡死在构造函数）。
- **解决方案**：
  1. 本项目 `io/gimbal/gimbal.{hpp,cpp}` 已整体重写为 v3.0（`UsbCtrl` 28 B / `UsbAutoSendToNuc` 42 B、度制绝对角、crc8、921600、构造期「敲门 + 2 s 上限」、发线程 10 Hz（100 ms）`mode=0` 心跳）。**不要**再用 v2.0 的帧常量去改别的文件。
  2. YAML 里 `com_port` 必填；`baudrate`（默认 921600）、`crc8_check`（默认 `true`）可选。若对端固件的 crc8 未填/校验策略不同，先设 `crc8_check: false` 验证链路，再回头查 crc；若误设成 115200 会表现为「能收到字节但永远拼不出帧」。
  3. 用 pty 假下位机做离线回环（`build/gimbal_verify/`：`fake_board.py` + `proto_test` + `loopback_test` + `run_loopback.sh`；**属构建产物、不入库**，重装环境后需重建）即可验证「打包/换算/心跳/解析」四条流程，不必接硬件。
- **已修复**：2026-10-17，`io/gimbal/gimbal.hpp` 与 `io/gimbal/gimbal.cpp` 按 lt-2027 下位机协议重写（详见 [`code/io/gimbal/README.md`](code/io/gimbal/README.md)）。

### C-5 改完 `io/gimbal`(v3.0) 适配后仍连不上下位机

- **症状**（2026-10-18 在本机 `build_full/` 实测的原文）：
  - `./gimbal_test configs/sentry.yaml` → `[error] [YAML] com_port not found!`，**进程立刻退出**（`tools::read` 缺键即 `exit(1)`）。
  - `./gimbal_test configs/standard4.yaml` → `[error] [Gimbal] Failed to open serial: IO Exception (2): No such file or directory, file .../io/serial/src/impl/unix.cc, line 151.`（`com_port: /dev/gimbal` 不存在，`resolve_serial_port()` 没找到任何 `ttyUSB*/ttyACM*` 可回退 → 原样返回 → open 抛异常 → `exit(1)`）。
  - `./gimbal_response_test configs/sentry.yaml` → 反复 `[warning] SocketCAN::open() failed: Error getting interface index!` + `[info] [Cboard] Waiting for q...`，然后**永久卡死在构造函数里的 `queue_.pop(data_ahead_)` / `queue_.pop(data_behind_)`**（`io/cboard.cpp:17-18`，无超时，日志永远等不到 `[Cboard] Opened.`）——连 `main` 循环都进不去，一个 28 B 帧都不会发出。
  - 接上真板仍是 2 s 后 `[Gimbal] No telemetry frame within 2000ms (baud=..., crc8_check=...)`。
- **原因**（按"最容易误判"排序）：
  1. **用错测试程序**：`gimbal_response_test` **不使用** `io::Gimbal`，它走 `io::CBoard`（SocketCAN/CAN 帧 + `io::Command`）；`src/sentry*.cpp`、`src/standard.cpp`、`src/uav.cpp`、`src/mt_standard.cpp`、`tests/cboard_test` 同理。**改串口库对它们零影响**——拿它们验证串口适配，怎么改都"连不上"。串口路径的验证程序只有 `gimbal_test`、`fire_test`、`planner_test`、`*_mpc`。
  2. **yaml 选错**：`configs/sentry.yaml` 只有 CAN 键（`can_interface`/`quaternion_canid`/`bullet_speed_canid`/`send_canid`），**没有 `com_port`**。串口路径要用 `configs/{standard3,standard4,demo,ascento}.yaml` 之一，或给 yaml 补 `com_port`。
  3. **设备节点不存在**：仓库不提供任何 udev 规则（`find . -name '*.rules'` 为空），`/dev/gimbal` 只是上游约定名。本机 `/dev/gimbal`、`/dev/ttyUSB*`、`/dev/ttyACM*` 全不存在。
  4. **接错物理口**：下位机自瞄通道是 STM32 **USART1 TTL 921600**（`application/cmd/robot_cmd.c:133` `USBVisionInit(&huart1)`；`Src/usart.c:49` `huart1.Init.BaudRate = 921600`），板载 MicroUSB 走的是**导航** DAOHANG 18 B/29 B（`MICROUSBVisionInit()`；`master_process.c:408` 注释「自瞄走 USART1，导航走 USB VCP」）。把板载 USB 插上位机 → 端口能打开但线上永远没有 `0xFF…0x0D` 的 28 B/42 B 帧 → 表现为 2 s warn 而不是打开失败。
  5. **握手节奏压在下位机离线阈值上**：下位机 daemon 100 Hz、`reload_count = 5`（`master_process.c:275` + `application/robot_task.h:101-118` 的 `osDelay(10)`）→ **50 ms 收不到 `UsbCtrl` 就判离线**并 `USARTServiceInit()` 重启 DMA 接收；本层构造期敲门间隔正好 50 ms、空闲心跳 `kIdleHeartbeatMs = 100`（`gimbal.cpp` 的 `tx_thread()`）→ 空闲时恒为阈值的 2 倍，下位机侧会周期性刷 vision offline。
- **解决方案**：
  1. **先确认链路**：串口 v3.0 → `gimbal_test` / `fire_test` / `planner_test` / `*_mpc`；CAN → `gimbal_response_test` / `cboard_test` / `sentry*` / `standard` / `uav`。
  2. 串口调试必须给含 `com_port` 的 yaml（或临时 `com_port: "/dev/ttyUSB0"`），否则第一步就死在 YAML。
  3. 打开前先 `ls -l /dev/ttyUSB* /dev/ttyACM*`；不存在 = 物理层没通（转接线 / CH340 驱动 / 用户是否在 `dialout` 组）。需要固定名时自己做 udev 规则映射到 `/dev/gimbal`。
  4. 端口能打开但 2 s 无帧 → 按序查：是否接在 **USART1**（不是板载 MicroUSB）、波特率 921600、固件是否为 seasky 新版 28 B/42 B、`crc8_check: false` 排除 crc。**注意：正常固件不会因 crc 失败**——下行它不校验，上行它按 `crc_8(tx_buf, 40)` 真填，`CRC_START_8 = 0x00` + sht75 表与本地位实现等价。
  5. 代码侧加固（**2026-10-18 已实施**）：构造期敲门间隔由 50 ms 降到 **20 ms**（`kKnockIntervalMs`），稳稳低于下位机 50 ms 离线阈值；`kIdleHeartbeatMs` **保持 100 ms 不动**——它同时是「控制指令有效期」，压到 30 ms 会让发送频率低于 10 Hz 的上层指令被心跳盖掉（loopback 的 C 段就是这么 FAIL 的，已回滚并写进 C-6）。另：静默时不再每秒重连、重连不再持锁 sleep，详见 [C-6](#c-6-gimbal_test-一直收不到首帧且进程静默卡死)。

### C-6 `gimbal_test` 一直收不到首帧，且进程静默卡死

- **症状**（2026-10-18，用户原话：「运行 gimbal_test 一直收不到首帧，但是 uart 已经连接」）：
  - `./build_full/gimbal_test configs/<含 com_port 的 yaml>` 跑起来后**终端几乎一行输出都没有**：只有构造期一条 `[warning]` 或 `[info]`，之后进程活着、CPU 不高、也不退出，`Ctrl-C` 才能停；
  - 与真板/假下位机都复现；日志里可能出现 `[warning] No serial data for extended period, triggering reconnect` 每秒刷一次，以及 `[warning] High TX latency: 551.74ms`；
  - 2 s 后（v3.0）才会打一条 `No telemetry frame within 2000ms (baud=921600, crc8_check=false)`，之后**什么都不再打**。
- **原因**（按"最容易误判"排序）：
  1. **`gimbal_test` 自己把自己锁死了**：主循环里 `auto q = gimbal.q(t);`（原 `tests/gimbal_test.cpp:53`）在姿态队列为空时永久阻塞在 `ThreadSafeQueue::pop/front` 的条件变量上；而它在 `gimbal.send(...)` **之前**——于是一旦首帧丢了，测试程序就**不再发任何下行帧**；下位机是**请求-响应式**（`master_process.c` 的 `DecodeUsb()` 解析成功才 `UsbVisionSend()`），收不到帧就永远不回 → 死锁、静默、无日志。**"收不到首帧"于是变成了"永远收不到首帧"。**
  2. **旧 `read_thread` 的"没数据就重连"**：连续 4000 次空读判链路死 → `reconnect()`；旧 `reconnect()` **持 `serial_mutex_` 睡 500 ms**（把发线程一起冻住，即那条 551 ms 的 TX 延迟），且 open 后立刻 `flushInput()`——可能把刚到的帧清掉。于是"偶发丢帧"会滚成"每秒重连 + 每秒丢帧"。
  3. **零诊断信息**：不知道实际用的是哪个设备节点（`resolve_serial_port()` 静默回退到 `/dev/ttyUSB0` 只打一条 warn）、不知道波特率、不知道发了几帧/收了几字节，因此无法区分"没发出去"、"发出去了但板子没回"、"回了但解不出"。
  4. **一个想当然的修法会引入新 bug**：把心跳 `kIdleHeartbeatMs` 从 100 ms 压到 30 ms 去迎合下位机 50 ms 离线阈值——实测会让"发送频率 < 10 Hz 的上层指令"被心跳（`mode=0`）盖掉，环路直接失效（`build/gimbal_verify` 的 loopback C 段因此 FAIL）。心跳同时是**控制指令有效期**，不能这么压。
- **解决方案**：
  1. **测试/诊断程序改用 `Gimbal::try_q(t, q, timeout_ms)`**（新增，非阻塞、返回 `bool`）：拿不到姿态只打日志，然后**照常 `send()`**。请求-响应链路必须由上层持续发帧来保活，绝不能因为"没数据"停止发送。
  2. **看日志，别猜**：日志已分流到 `logs/<源文件路径>/{run,warn,error}.log` + `state.log`（见 [`code/tools/file_logger.hpp.md`](code/tools/file_logger.hpp.md)）。**先 `cat logs/io/gimbal/gimbal.cpp/state.log`**（每秒覆盖写，最后一行就是崩溃/卡死前一秒的 `rx_bytes`/`tx_frames`/`serial_ok`），再看 `warn.log`。
  3. **按 `rx_bytes` 分流判据**：
     - `rx_bytes == 0` → **物理层**：TX/RX 是否交叉、GND 是否共地、是否接在 **USART1（PB7=RX / PA9=TX，不是常见的 PA9/PA10）**、波特率 921600、板子是否真在跑这套固件；
     - `rx_bytes` 在涨但 `errors` 也在涨 → **协议层**：波特率/线序/协议版本；日志会带前 5 个坏帧的 hex dump，第 50 次还会专门提示"高度怀疑波特率不匹配或 TX/RX 接反"；
     - 出现 `rx_echo: 收到的字节与本端刚发出的下行帧完全相同` → **TX/RX 被短接/自环**（转接板自环或接错线），下位机其实没回帧。
  4. **回环自测法（5 秒定位 PC 侧是否无辜）**：把 USB-TTL 转接板自己的 TX-RX 短接再跑一次——看到 `rx_echo` 就说明"串口驱动 + 本程序 + 转接板"三者全好，问题 100% 在板子那一端（接线/波特率/固件）。
  5. **静默不再疯转重连**：`read_thread` 改成三档——静默 2 s 打一次六条排查清单（只报一次）、之后每 10 s 一条计数、静默 30 s 且**从未收到任何字节**才强制重连一次；设备节点消失则立刻重连。`reconnect()` 也不再持锁 sleep，并会重新 `resolve_serial_port()`（拔插后 `ttyUSB0`/`ttyUSB1` 会互换）。
- **已修复**：2026-10-18，`tests/gimbal_test.cpp`（改 `try_q` + 每秒等待诊断）、`io/gimbal/gimbal.{hpp,cpp}`（`try_q`、`rx_bytes/tx_frames/tx_fail` 诊断字段、首帧/首字节 hex 取证、回环自检、静默三档、`state.log` 快照、`reconnect` 不持锁 + 重解析端口、端口候选身份表）、`tools/file_logger.{hpp,cpp}`（覆盖式分文件日志器）。
- **验证**（离线，`build/gimbal_verify/`，不入库）：重跑 `run_loopback.sh` 18 条断言全过 `loopback_test PASS`；`silent_test` + pty 三种故障模式分别复现出「0 字节 → 物理层清单」「垃圾字节 → 坏帧 hex + 波特率提示」「自环 → rx_echo」；`gimbal_test` 接静默 pty 时每秒打印 `仍在等待首帧` 且 `tx_frames` 持续增长（不再自锁）。

---

## D. 环境与沙箱

### D-1 `/tmp` 在每次 bash 调用之间不保留

- **症状**：上一次调用写好的 `/tmp/xxx.cpp`，下一次调用报"没有那个文件或目录"。
- **原因**：本沙箱每次 bash 调用使用**独立的临时目录**。
- **解决方案**：临时产物放进**工作区内的 gitignore 目录**（本项目用 `build/`，例如 `build/smoke_tools.cpp`、`build/smoke_tools`），不要依赖 `/tmp`。

### D-2 沙箱拒绝写工作区外的文件

- **症状**：`[sandbox: file access denied under workspace-write mode]`。
- **场景**：需要修改工作区外的配置，例如 `~/.dsh/profiles/web/cordis.patch.yml`。
- **解决方案**：对该次操作**一次性**申请更宽权限（`sandbox_permissions`：`workspace-write` 或 `danger-full-access`）并写明理由，由用户批准；不要试图绕道。

### D-3 编辑被拒：未读过或文件已被外部改动

- **症状**：`file has not been read` 或 `file changed since it was read`。
- **原因**：编辑前未在本会话读取过该文件；或文件被外部改动（如 `chmod` 更新了 mtime）。
- **解决方案**：先 `read` 该文件，再 `edit`。批量修改权限/时间戳之后要重新读取。

### D-4 git 不跟踪空目录

- **症状**：骨架目录在本地存在，但 `git status` 看不到，clone 后目录消失。
- **解决方案**：空目录放一个 `.gitkeep` 占位；该目录有真实文件后**删掉 `.gitkeep`**。
- **已处理**：2026-10-17 清理了全部「只含 `.gitkeep`」的目录与多余的 `.gitkeep`（`configs/`、`src/`、`tests/` 三个已有真实文件的目录），仓库现在**没有任何 `.gitkeep`**。新增空目录时才需要重新引入占位文件。

### D-6 删除已 `git add` 但未提交的目录：用 `git rm -r -f`

- **症状**：目录里的文件状态是 `A`（已暂存待提交），直接 `rm -rf` 后 `git status` 变成 `AD`（暂存了新增、工作区又不存在），看起来很脏，`git add -A` 之前索引与磁盘不一致。
- **原因**：文件已进 index，只删工作区文件不会同步移除索引条目。
- **解决方案**：对**已被 git 跟踪/暂存**的路径用 `git rm -r -f <路径>`（同时清 index 与工作区）；对**未跟踪**的空目录（如临时残留）才用 `rm -rf`。批量混合删除时先判定再动手：

  ```bash
  if git ls-files --error-unmatch "$p" >/dev/null 2>&1; then git rm -r -f -q "$p"; else rm -rf "$p"; fi
  ```

- **注意**：执行删除前先把**解析后的绝对路径**打印出来核对一遍，再用 `[ -e "$p" ]` 守卫，避免路径拼错删到别处。
- **已处理**：2026-10-17 清理计划骨架遗留目录时使用该方式。

### D-5 整目录拷贝静默覆盖本地已定制文件

- **症状**：把参考仓库整目录拷入后，本地已改好的文件被上游旧版覆盖；`git status --short` 里出现 `MM`（index 与工作区都与 HEAD 不同）、`AM`（已 add 后工作区又被改）、` M`。
- **原因**：`cp -r <源>/. <目标>/` 对同名文件**无提示覆盖**。本项目 2026-10-17 拷贝时被覆盖的三个文件：`CMakeLists.txt`（阶段 1 手写的 43 行宿主验证版被上游 146 行版取代；`MM`）、`.gitignore`（丢失 `.dsh-meow/` 规则；`AM`）、`readme.md`（0 字节占位变上游 351 行 README；` M`）。`tools/` 因上游同名文件被排除而未被覆盖。
- **解决方案**：
  1. 拷贝前先 `git status --short` 记基线，明确哪些文件是"本项目已定制"；
  2. 拷贝后用 `git status --short` 复核：出现 `M`/`MM`/`AM` 的文件**必须逐个确认**是"想要的更新"还是"误覆盖"；
  3. 被覆盖但已进过 index 的文件可原样取回：`git show :<路径> > <路径>`（index = 上次 `git add` 的内容）；也可 `git diff` 看工作区 vs index、`git diff --cached` 看 index vs HEAD 的差异；
  4. 约定：整目录拷贝时**显式排除已定制的 `tools/`、`AGENT.md`、`doc/`**。
- **已处理**：2026-10-17 三处覆盖已确认并记入文档（[A-7](#a-7-gitignore-被上游版本覆盖导致-dsh-meow-变成未跟踪)、[A-5](#a-5-顶层-cmakelists-引用未拷入的-color_track_demo-导致-configure-失败)、[`AGENT.md` §3.2](../AGENT.md#32-阶段-2-起点sp_vision_25-剩余代码已全量拷入2026-10-17)），是否回滚见 `AGENT.md` §7。

---

## E. 工具与数据处理

### E-1 DSH 会话转录是多帧 zstd

- **症状**：用 `node:zlib.zstdDecompressSync` 解压 `session.v4.jsonl.zstd` 只得到 **237 字节**（只有一条 session 头），而文件本身有几百 KB。
- **原因**：转录是**按轮次追加的多帧 zstd**，Node 的 `zstdDecompressSync` **只解第一帧**。
- **解决方案**：用 `zstd -dc <文件>`（或 `zstdcat`）解压全部帧：

  ```bash
  zstd -dc session.v4.jsonl.zstd | wc -l
  ```

- **相关**：DSH 用量账本在 `~/.dsh/dsh-usage/usage-ledger.json`（按天）与 `~/.dsh/storages/session_projcache/sessions/<id>.json`（每会话 `tokenUsage.totals`）。

---

## 维护规则

1. **解决了一个此前没有记录的新问题后，必须在下面对应分组追加一条**（没有合适分组就新建分组），同时更新 §0 快速索引表。
2. 条目**只增不删**；问题被代码修复后，在条目里标注「**已修复**：<日期/提交>」，保留原因与症状便于检索。
3. 每条必须包含**症状 / 原因 / 解决方案**三项；症状要写成"报错原文或可检索的关键词"。
4. 本文件的更新**不受** `AGENT.md` §5「仅函数作用变动才更新」的限制——它是独立触发的运维文档。

### 条目模板

```markdown
### <编号> <一句话标题>

- **症状**：（报错原文 / 可检索关键词）
- **原因**：
- **解决方案**：
- **已处理/已修复**：（日期或提交；未修复则写"本项目当前按方案规避"）
```
