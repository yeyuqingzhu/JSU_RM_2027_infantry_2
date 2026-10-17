// =============================================================================
// tools/logger.hpp —— 全局日志器（基于 spdlog）的唯一对外接口
// -----------------------------------------------------------------------------
// 【作用】声明获取全局日志器的函数 tools::logger()，上层模块用它输出日志；
//         日志同时写入 logs/ 目录下的文件与控制台（具体配置在 logger.cpp）。
// 【来源】sp_vision_25 的 tools/logger.hpp。
// 【依赖】spdlog（本机 1.15.3，系统包以 SPDLOG_FMT_EXTERNAL=1 构建，链接 libfmt）。
// 【注意事项】
//   1) 线程安全：spdlog 的 logger 是线程安全的（多线程写同一条日志会被内部互斥保护）；
//      但 tools::logger() 的“首次惰性创建”不是原子的，多线程首次同时调用存在竞态，
//      建议在主线程/程序入口先调用一次完成初始化。
//   2) 单位/格式：日志格式化语法为 spdlog/fmt 的 {} 占位符（非 printf 的 %d）。
//   3) 调用时机：可在任意时刻调用；返回的 shared_ptr 常驻，不会因局部变量析构而失效。
//      首次调用会在当前工作目录下写 logs/<时间戳>.log；spdlog 1.15 会自动创建缺失的
//      logs/ 目录，只有在目录无法创建/无写权限时才抛 spdlog::spdlog_ex。
//   4) 接口签名与源工程一致，不得改动。
// =============================================================================

#ifndef TOOLS__LOGGER_HPP
#define TOOLS__LOGGER_HPP

#include <spdlog/spdlog.h>

namespace tools
{
// 作用：返回全局日志器（单例，惰性初始化）。
// 参数：无。
// 返回：std::shared_ptr<spdlog::logger>；首次调用时创建“文件 + 彩色控制台”双 sink 的
//       日志器（日志级别 debug，flush_on(info)）。
// 边界：logs/ 目录不存在时 spdlog 会自动创建；目录创建失败或无写权限时，
//       底层 basic_file_sink 构造失败并抛 spdlog::spdlog_ex；本函数不捕获，也不会返回 nullptr。
// 备注：返回值可直接当 spdlog 日志器使用：tools::logger()->info("{}", value)。
std::shared_ptr<spdlog::logger> logger();

}  // namespace tools

#endif  // TOOLS__LOGGER_HPP