# =========================================================================
# tools —— 底层通用件模块
#
#   本模块是整个项目的最底层，**不得依赖任何上层模块**（auto_function / io / …），
#   只允许依赖第三方库与标准库。这样保证算法层与接口层都可以自由复用它。
#
#   源文件来源：sp_vision_25（同济大学 SuperPower 战队 25 赛季视觉框架）的 tools/，
#   逐个迁移并补充中文注释，详见 doc/code/tools/ 与 doc/overview.md。
# =========================================================================

add_library(tools OBJECT
  crc.cpp
  exiter.cpp
  extended_kalman_filter.cpp
  file_logger.cpp
  img_tools.cpp
  logger.cpp
  math_tools.cpp
  pid.cpp
  plotter.cpp
  ransac_sine_fitter.cpp
  recorder.cpp
  trajectory.cpp
)

# 以项目根目录为头文件根，使 "tools/xxx.hpp" 形式可用
target_include_directories(tools PUBLIC ${CMAKE_SOURCE_DIR})

# 依赖按 PUBLIC 传递：OBJECT 库本身不产出链接产物，
# 使用方需要自行链接这些库（CMake ≥ 3.12 会把用法要求传播出去）。
target_link_libraries(tools
  PUBLIC
    Eigen3::Eigen
    spdlog::spdlog
    yaml-cpp::yaml-cpp
    nlohmann_json::nlohmann_json
    fmt::fmt
    ${OpenCV_LIBS}
)

# recorder.cpp 使用 fmt::format 与 <fmt/chrono.h> 格式化时间点；
# 本机 spdlog 使用外部 fmt（未内联捆绑），故需显式链接 fmt::fmt。

# 线程：recorder / thread_pool / thread_safe_queue 使用 std::thread
find_package(Threads REQUIRED)
target_link_libraries(tools PUBLIC Threads::Threads)

# 自有代码保持告警可见（第三方头经 imported target 以 -isystem 引入，不受影响）
target_compile_options(tools PRIVATE -Wall -Wextra)
