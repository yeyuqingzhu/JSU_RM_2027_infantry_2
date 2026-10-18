// =============================================================================
// tools/file_logger.cpp —— 「按源文件分文件夹」覆盖式日志器的实现
// -----------------------------------------------------------------------------
// 【实现要点】
//   1) 一张静态表 source → logger（强引用常驻），同一源文件重复调用只建一次；
//   2) 每个源文件 3 个 rotating_file_sink（run/warn/error 三档，级别递增过滤），
//      每档 max_files = 1 → 写满后新文件覆盖旧的备份，磁盘占用有上界；
//   3) 每个源文件额外挂一个彩色控制台 sink，保证终端可见性不比 tools::logger() 差；
//   4) 文件名/目录名做白名单清洗，杜绝 "../" 逃出日志根目录。
//
// 【为什么不复用 tools::logger()】那个是全局单文件 + 时间戳命名，语义是"一次运行
//   一份完整日志"；本文件要的是"多文件分流 + 覆盖有界"，两者共存但不共享实现。
// =============================================================================

#include "file_logger.hpp"

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace tools
{
namespace
{
// ---------------------------------------------------------------------------
// 每档文件的大小上限与备份份数
// ---------------------------------------------------------------------------
// max_files = 1 的含义（spdlog 语义）：当前文件写满 → 改名为 <name>.1.log；
// 已有的 .1.log 直接删除（即被覆盖）。单档实际占用 = max_size × (1 + 1)。
// 取值依据：一次典型排查会跑 1~3 分钟、云台日志被限频到每秒几条，
// run.log 给 512 KB 足够覆盖十几分钟，warn/error 更稀疏所以给得更小。
constexpr std::size_t kRunMaxBytes = 512u * 1024u;    // run.log   512 KB
constexpr std::size_t kWarnMaxBytes = 128u * 1024u;   // warn.log  128 KB
constexpr std::size_t kErrorMaxBytes = 64u * 1024u;   // error.log  64 KB
constexpr std::size_t kMaxBackupFiles = 1;            // 只留 1 份备份：新的覆盖旧的

// 文件里不重复写日志器名字（名字已经是文件夹了），控制台才带名字
constexpr const char * kFilePattern = "[%Y-%m-%d %H:%M:%S.%e] [%l] %v";
constexpr const char * kConsolePattern = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%n] %v";

constexpr const char * kDefaultRoot = "logs";

// 全局状态：根目录 + 已建日志器表 + 保护它们的锁（snapshot 与建表也用它）
std::mutex g_mutex;
std::string g_root = kDefaultRoot;
std::map<std::string, std::shared_ptr<spdlog::logger>> g_loggers;

// ---------------------------------------------------------------------------
// std::string sanitize(const std::string & source)
// ---------------------------------------------------------------------------
// 作用：把任意"源文件标识"清洗成一个安全的相对目录名。
// 规则：按 '/' 切段，丢弃空段、"." 与 ".."段；每段只保留 [A-Za-z0-9._-]，
//       其余字符（中文/空格/冒号等）替换为 '_'。
// 参数：source 原始标识。
// 返回：清洗后的相对路径；若清洗后为空则返回 "unknown"。
// 边界：传入绝对路径时开头的空段被丢弃，因此不会在日志根目录之外建文件。
// ---------------------------------------------------------------------------
std::string sanitize(const std::string & source)
{
  std::vector<std::string> segments;
  std::string current;
  for (const char ch : source) {
    if (ch == '/') {
      if (!current.empty()) segments.push_back(current);
      current.clear();
      continue;
    }
    current.push_back(ch);
  }
  if (!current.empty()) segments.push_back(current);

  std::string out;
  for (const auto & raw : segments) {
    if (raw == "." || raw == "..") continue;  // 禁止向上跳，避免写出日志根目录

    std::string seg;
    seg.reserve(raw.size());
    for (const char ch : raw) {
      const bool keep = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
                        (ch >= '0' && ch <= '9') || ch == '.' || ch == '_' || ch == '-';
      seg.push_back(keep ? ch : '_');
    }
    if (seg.empty() || seg == "." ) continue;
    if (!out.empty()) out.push_back('/');
    out += seg;
  }

  return out.empty() ? std::string("unknown") : out;
}

// 日志器名字：取最后一段（控制台里显示 gimbal.cpp 比显示整条路径清爽）
std::string logger_name(const std::string & sanitized)
{
  auto pos = sanitized.find_last_of('/');
  return pos == std::string::npos ? sanitized : sanitized.substr(pos + 1);
}
}  // namespace

std::shared_ptr<spdlog::logger> file_logger(const std::string & source)
{
  std::lock_guard<std::mutex> lock(g_mutex);

  // 根目录取规范化后的相对路径（"logs/../logs" 之类归一，便于排查时对应真实位置）
  const auto root = std::filesystem::path(g_root.empty() ? kDefaultRoot : g_root).lexically_normal();

  const std::string key = sanitize(source);
  auto it = g_loggers.find(key);
  if (it != g_loggers.end()) return it->second;

  const auto dir = root / key;
  const std::string name = logger_name(key);

  // 1) 三档文件 sink（级别递增过滤，文件之间是"包含"关系）
  auto run_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
    (dir / "run.log").string(), kRunMaxBytes, kMaxBackupFiles);
  run_sink->set_level(spdlog::level::trace);  // 运行日志：全收
  run_sink->set_pattern(kFilePattern);

  auto warn_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
    (dir / "warn.log").string(), kWarnMaxBytes, kMaxBackupFiles);
  warn_sink->set_level(spdlog::level::warn);  // 警告日志：warn/err/critical
  warn_sink->set_pattern(kFilePattern);

  auto error_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
    (dir / "error.log").string(), kErrorMaxBytes, kMaxBackupFiles);
  error_sink->set_level(spdlog::level::err);  // 报错日志：err/critical
  error_sink->set_pattern(kFilePattern);

  // 2) 控制台 sink：info 及以上，带日志器名字，保证终端可见性
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(spdlog::level::info);
  console_sink->set_pattern(kConsolePattern);

  auto logger = std::make_shared<spdlog::logger>(
    name, spdlog::sinks_init_list{run_sink, warn_sink, error_sink, console_sink});
  logger->set_level(spdlog::level::trace);
  // info 就落盘：排查现场时进程常被 Ctrl-C 或直接崩，等 warn 才 flush 会丢掉关键上下文。
  // 本层日志已被限频（见 gimbal.cpp），这里不构成性能负担。
  logger->flush_on(spdlog::level::info);

  g_loggers.emplace(key, logger);
  return logger;
}

void snapshot(const std::string & source, const std::string & text)
{
  std::lock_guard<std::mutex> lock(g_mutex);

  try {
    const auto root =
      std::filesystem::path(g_root.empty() ? kDefaultRoot : g_root).lexically_normal();
    const auto dir = root / sanitize(source);
    std::filesystem::create_directories(dir);
    std::ofstream out(dir / "state.log", std::ios::trunc);
    if (!out.is_open()) return;
    out << text;
    if (!text.empty() && text.back() != '\n') out << '\n';
  } catch (...) {
    // 快照失败不能影响主流程（它本身就是"尽力而为"的诊断手段）
  }
}

void set_file_logger_root(const std::string & root)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  g_root = root.empty() ? std::string(kDefaultRoot) : root;
}

}  // namespace tools
