// =============================================================================
// tools/file_logger.hpp —— 「按源文件分文件夹」的覆盖式日志器
// -----------------------------------------------------------------------------
// 【为什么需要它】tools/logger() 只写一个 logs/<时间戳>.log：所有模块混在一起，
//   出问题后要在几万行里翻；而且文件名带时间戳，跑几十次就堆出几十个文件，
//   既不覆盖也不轮转，磁盘只增不减。排查串口/云台这类"多线程 + 外部硬件"问题时，
//   需要的是：**每个源文件一个文件夹**，文件夹里按严重程度分三档，
//   每档只保留最近的一小段（写满后新覆盖旧），这样事后一看就知道是哪个环节断的。
//
// 【目录结构】日志根目录默认是进程工作目录下的 logs/（可用 file_logger_root 改）。
//   对源文件标识 "io/gimbal/gimbal.cpp"，生成：
//
//     logs/io/gimbal/gimbal.cpp/run.log     —— 运行日志（debug/info/warn/error 全收）
//     logs/io/gimbal/gimbal.cpp/warn.log    —— 警告日志（warn 及以上）
//     logs/io/gimbal/gimbal.cpp/error.log   —— 报错日志（error 及以上）
//     logs/io/gimbal/gimbal.cpp/state.log   —— 状态快照（snapshot()，每次整份覆盖）
//
//   一条 error 会同时出现在三个文件里（三档是"包含"关系，不是互斥分流），
//   这样单独把 error.log 拿出来就能看全部报错，不用跨文件拼。
//
// 【覆盖策略】每个文件用 spdlog::sinks::rotating_file_sink_mt，参数
//   (max_size, max_files = 1)：写满 max_size 后把当前文件改名成 xxx.1.log，
//   下次再满就直接覆盖那份 xxx.1.log —— 即"新的覆盖旧的"，
//   单档磁盘占用上界是 2 × max_size，永久有界，不需要人工清理。
//
// 【线程安全】内部查表用互斥锁，spdlog 的 _mt sink 自身线程安全，可跨线程调用。
//
// 【与 tools/logger() 的关系】互不影响：老的 logger() 仍然写 logs/<时间戳>.log
//   与控制台（其它 13 个可执行程序靠它，不能动）；新接口只是给"要排查的文件"
//   额外分流。两者都挂控制台 sink，所以终端上看到的信息不会变少。
//
// 【用法】
//   #include "tools/file_logger.hpp"
//   static auto logger = tools::file_logger("io/gimbal/gimbal.cpp");  // 用文件级 static
//   ...
//   logger->warn("[Gimbal] ... {}", value);
// =============================================================================

#ifndef TOOLS__FILE_LOGGER_HPP
#define TOOLS__FILE_LOGGER_HPP

#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace tools
{
/**
 * @brief 取得「某个源文件专用」的三档覆盖式日志器（同参数重复调用返回同一实例）。
 *
 * @param source 源文件标识，通常直接写相对项目根的路径（如 "io/gimbal/gimbal.cpp"），
 *               它会成为 logs/ 下的文件夹名；非法字符会被替换、"." ".." 段会被丢弃，
 *               因此传绝对路径也不会跑到日志根目录外面去。
 * @return 非空的 shared_ptr<spdlog::logger>；日志器级别 trace，flush_on(info)。
 *
 * @note 首次调用时会创建 logs/<source>/ 目录与三个文件，目录建不出来时抛
 *       spdlog::spdlog_ex（不吞异常，避免"以为在记日志其实没记"）。
 * @note 日志器实例常驻（静态表持有强引用），析构顺序上不会出现"用的时候已被释放"。
 */
std::shared_ptr<spdlog::logger> file_logger(const std::string & source);

/**
 * @brief 覆盖式写一份「最后一刻状态快照」（logs/<source>/state.log）。
 *
 * 用途：进程卡死 / 被 Ctrl-C / 段错误之后，run.log 可能还没来得及 flush，
 * 而 state.log 每秒被整份重写一次，打开它就是崩溃前一秒的关键计数
 * （收了多少字节、发了几帧、端口是否还开着……）。
 *
 * @param source 同 file_logger()。
 * @param text   快照正文（多行用 '\n' 连接）；整份文件会被 truncate 后重写。
 * @note 单次写入是 O(长度) 的同步写，调用者负责控制频率（建议 ≥ 500 ms 一次）。
 */
void snapshot(const std::string & source, const std::string & text);

/**
 * @brief 修改日志根目录（默认 "logs"，相对进程当前工作目录）。
 *
 * @param root 新的根目录；空字符串表示恢复默认。不会自动创建目录
 *             （创建由 file_logger() 首次调用时的 sink 完成）。
 * @note 仅应在创建任何 file_logger 之前调用；已创建的日志器不受影响。
 *       提供它是为了让离线验证/单测把日志写到临时目录，避免污染 logs/。
 */
void set_file_logger_root(const std::string & root);

}  // namespace tools

#endif  // TOOLS__FILE_LOGGER_HPP
