// =============================================================================
// tools/yaml.hpp —— yaml-cpp 的薄封装：加载文件 + 带默认值的读取
// -----------------------------------------------------------------------------
// 【作用】把 YAML 配置文件的加载与取值收敛到三个 inline 函数：tools::load 读取
//         整个文件，tools::read 按 key 取值（可带默认值）；配置缺失或解析失败时
//         打印错误并直接 exit(1)，让配置文件错误在启动阶段就暴露。
// 【来源】sp_vision_25 的 tools/yaml.hpp。
// 【依赖】yaml-cpp（本机 0.8.0）、tools/logger.hpp（spdlog）。
//         另显式包含 <cstdlib>（std::exit 的声明头，源文件依赖传递包含）。
// 【注意事项】
//   1) 线程安全：函数无共享可写状态；YAML::Node 为值语义，多线程各自读取不同 Node
//      实例是安全的（同一个 Node 并发只读也安全），但本文件的错误路径会终止进程，
//      不要在实时/业务线程中对“预期会缺 key”的配置做探测。
//   2) 单位：本文件只负责取值，不做单位换算；数值的物理量纲由调用方约定。
//   3) 调用时机：仅在程序初始化阶段读取配置；运行期不要重复 LoadFile（无缓存）。
//   4) 行为约定：key 不存在（或节点不是映射）→ 打印 error 并 exit(1)；
//      key 存在但类型不匹配 → yaml-cpp 抛 YAML::TypedBadConversion，本文件不捕获。
// =============================================================================

#ifndef TOOLS__YAML_HPP
#define TOOLS__YAML_HPP

#include <cstdlib>  // 迁移补充：std::exit 的声明头（源文件依赖传递包含）

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"

namespace tools
{
// ----------------------------------------------------------------------------
// inline YAML::Node load(const std::string & path) —— 加载 YAML 文件
// ----------------------------------------------------------------------------
// 作用：读取并解析指定路径的 YAML 文件，返回根节点。
// 参数：path 配置文件路径（相对路径相对于进程当前工作目录），如 "configs/standard.yaml"。
// 返回：YAML::Node 根节点；失败时不会返回（进程已 exit(1)）。
// 边界：文件不存在/无权限 → 捕获 YAML::BadFile，打印错误后 exit(1)；
//       语法错误 → 捕获 YAML::ParserException，打印错误后 exit(1)；
//       其它 yaml-cpp 异常（如 BadConversion）未捕获，会向上抛出。
// 实现要点：YAML::LoadFile(path)；两个 catch 分支都先 logger()->error(...) 再 exit(1)。
// 迁移补充：函数末尾增加一行不可达的 return YAML::Node{}，
//           用于在开启 -Wreturn-type/-Werror 时消除“非 void 函数走到末尾”的告警，
//           语义与行为不变（exit 不会返回）。
// ----------------------------------------------------------------------------
inline YAML::Node load(const std::string & path)
{
  try {
    return YAML::LoadFile(path);
  } catch (const YAML::BadFile & e) {
    logger()->error("[YAML] Failed to load file: {}", e.what());
    exit(1);
  } catch (const YAML::ParserException & e) {
    logger()->error("[YAML] Parser error: {}", e.what());
    exit(1);
  }
  // 迁移补充：以下 return 不可达（两个 catch 分支都以 exit(1) 结束进程），
  // 仅用于在 -Wreturn-type / -Werror 构建下消除“非 void 函数走到末尾”的告警。
  return YAML::Node{};
}

// ----------------------------------------------------------------------------
// T read(const YAML::Node &, const std::string &) —— 必填项读取（缺 key 即终止进程）
// ----------------------------------------------------------------------------
// 作用：读取映射节点 yaml 中 key 对应的值并转换为类型 T，缺失即终止进程。
// 参数：yaml 已加载的映射节点；key 字段名。
// 返回：类型为 T 的值（由 YAML::Node::as<T>() 转换）。
// 边界：key 不存在（或 yaml 不是映射）→ 打印 error 并 exit(1)，不返回；
//       类型不匹配 → yaml-cpp 抛 YAML::TypedBadConversion（本函数不捕获）。
// 实现要点：用 if (yaml[key]) 判断节点有效性（yaml-cpp 0.8.0 中 const 版 operator[]
//           对缺失 key 返回 Undefined 节点，operator bool 为 false，不会抛异常）。
// 迁移补充：末尾增加不可达的 return T{}（要求 T 可默认构造），
//           仅为消除 -Wreturn-type 告警，行为不变。
// ----------------------------------------------------------------------------
template <typename T>
inline T read(const YAML::Node & yaml, const std::string & key)
{
  if (yaml[key]) return yaml[key].as<T>();
  logger()->error("[YAML] {} not found!", key);
  exit(1);
  // 迁移补充：不可达分支，仅为消除 -Wreturn-type 告警；要求 T 可默认构造。
  return T{};
}

// ----------------------------------------------------------------------------
// T read(const YAML::Node &, const std::string &, const T &) —— 可选项读取（缺 key 用默认值）
// ----------------------------------------------------------------------------
// 作用：读取 key 对应的值；缺失时返回调用方给的默认值（不退出进程）。
// 参数：yaml 映射节点；key 字段名；default_value 缺省值，类型必须与 T 一致。
// 返回：key 存在则返回 yaml[key].as<T>()，否则返回 default_value（按值拷贝）。
// 边界：key 缺失是正常流程（返回默认值）；类型不匹配仍会抛 YAML::TypedBadConversion。
// 备注：该重载常用于带默认值的 PID/阈值等可调参数。
// ----------------------------------------------------------------------------
template <typename T>
inline T read(const YAML::Node & yaml, const std::string & key, const T & default_value)
{
  if (yaml[key]) return yaml[key].as<T>();
  return default_value;
}

}  // namespace tools

#endif  // TOOLS__YAML_HPP