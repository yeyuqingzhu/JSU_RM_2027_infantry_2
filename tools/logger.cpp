// =============================================================================
// tools/logger.cpp —— 全局日志器（spdlog）的实现
// -----------------------------------------------------------------------------
// 【作用】实现 tools::logger()：首次调用时创建名为 logs/<时间戳>.log 的文件 sink
//         与彩色控制台 sink，级别均为 debug，info 及以上立即 flush。
// 【来源】sp_vision_25 的 tools/logger.cpp。
// 【依赖】spdlog 1.15.3（sinks/basic_file_sink.h、sinks/stdout_color_sinks.h）、
//         fmt（<fmt/chrono.h> 提供 time_point 的格式化）、C++17 标准库 <chrono>、<string>。
//         【迁移说明】与源文件一致，直接使用 fmt::format。本机 spdlog 为
//         SPDLOG_FMT_EXTERNAL=1 构建版，任何 spdlog 格式化在链接期都需要 libfmt，
//         故 tools/CMakeLists.txt 中 fmt::fmt 是必须的依赖（不可移除）。
// 【注意事项】
//   1) 线程安全：spdlog logger 本身线程安全；但全局指针 logger_ 的“检查并创建”不是
//      原子的，多线程首次并发调用可能重复创建，建议在程序入口先调用一次 logger()。
//   2) 单位/时间格式：文件名时间戳取 system_clock 当前时刻，格式 %Y-%m-%d_%H-%M-%S；
//      fmt 对 time_point 的 %S 会带小数秒，故实际文件名形如
//      2026-10-11_09-26-33.397172716.log，几乎不会重名。
//      basic_file_sink_mt 的第二个参数 true = truncate（同名则清空重写）。
//   3) 调用时机：进程当前工作目录决定 logs/ 的实际位置；spdlog 1.15 的 file_helper
//      会自动创建缺失的父目录（os::create_dir），因此 logs/ 不存在也能正常写日志，
//      只有在目录无法创建或无写权限时才抛 spdlog::spdlog_ex。
//   4) 日志级别固定 debug、flush_on(info)，源工程行为保持不变。
// =============================================================================

#include "logger.hpp"

#include <fmt/chrono.h>  // 与源文件一致：直接使用 fmt 格式化 time_point
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <string>

namespace tools
{
// 全局日志器实例（惰性创建）：nullptr 表示尚未初始化。
// 说明：非 static，源工程如此；仅在本文件内被 logger()/set_logger() 使用。
std::shared_ptr<spdlog::logger> logger_ = nullptr;

// ----------------------------------------------------------------------------
// void set_logger() —— 创建并配置全局日志器（仅本文件内部使用）
// ----------------------------------------------------------------------------
// 作用：构造文件 sink 与控制台 sink，装配成全局 logger_。
// 参数：无。
// 返回：无（写入全局 logger_）。
// 边界：logs/ 目录缺失会自动创建；无法创建或无写权限时抛 spdlog::spdlog_ex。
//       重复调用会丢弃旧 logger（旧 shared_ptr 释放，文件句柄关闭）。
// 实现要点：文件名 = logs/<当前系统时间 %Y-%m-%d_%H-%M-%S>.log；
//           文件 sink 使用 _mt（多线程安全）版本并传 true（truncate）；
//           两个 sink 级别都设为 debug，logger 级别 debug，flush_on(info)。
// 备注：该函数未在 logger.hpp 中声明，属实现细节，外部不可见（保持源工程原样）。
// ----------------------------------------------------------------------------
void set_logger()
{
  auto file_name = fmt::format(
    "logs/{:%Y-%m-%d_%H-%M-%S}.log", std::chrono::system_clock::now());
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_name, true);
  file_sink->set_level(spdlog::level::debug);

  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(spdlog::level::debug);

  logger_ = std::make_shared<spdlog::logger>("", spdlog::sinks_init_list{file_sink, console_sink});
  logger_->set_level(spdlog::level::debug);
  logger_->flush_on(spdlog::level::info);
}

// ----------------------------------------------------------------------------
// std::shared_ptr<spdlog::logger> logger() —— 获取全局日志器
// ----------------------------------------------------------------------------
// 作用：返回全局日志器，首次调用时惰性初始化。
// 参数：无。
// 返回：logger_ 的 shared_ptr（永不为 nullptr，除非构造抛异常）。
// 边界：多线程首次并发调用可能重复创建（未加锁），源工程行为保持一致。
// ----------------------------------------------------------------------------
std::shared_ptr<spdlog::logger> logger()
{
  if (!logger_) set_logger();
  return logger_;
}

}  // namespace tools
