// Copyright (c) 2026 Robot Control System
// All rights reserved.
//
// Author: gene.wang
// Date: 2026
//
// Description:
// 日志辅助工具头文件。
// 提供 ANSI 颜色转义码定义、日志前缀标签定义，
// 以及带颜色的 RCLCPP 日志输出宏（LOG_INFO / LOG_ERROR / LOG_WARN / LOG_DEBUG 等）。
// 所有宏均使用全局命名空间中的颜色/标签常量，确保在任意位置均可正常使用。

#ifndef TASK_MANAGER__LOG_HELPER_HPP_
#define TASK_MANAGER__LOG_HELPER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sstream>

// ============================================================================
// ANSI 颜色转义码
// 定义在全局命名空间 task_log_color 中，以便预处理器宏可以正确解析。
// ============================================================================
namespace task_log_color
{
  constexpr const char* RESET       = "\033[0m";
  constexpr const char* RED         = "\033[31m";      ///< 红色 - 用于错误信息
  constexpr const char* GREEN       = "\033[32m";      ///< 绿色 - 用于成功/普通信息
  constexpr const char* YELLOW      = "\033[33m";      ///< 黄色 - 用于警告信息
  constexpr const char* BLUE        = "\033[34m";      ///< 蓝色 - 用于调试信息
  constexpr const char* MAGENTA     = "\033[35m";      ///< 品红 - 用于重要信息
  constexpr const char* CYAN        = "\033[36m";      ///< 青色 - 用于流程跟踪信息
  constexpr const char* WHITE       = "\033[37m";      ///< 白色 - 用于普通信息
  constexpr const char* BOLD_RED    = "\033[1;31m";    ///< 粗体红色 - 用于严重错误
  constexpr const char* BOLD_GREEN  = "\033[1;32m";    ///< 粗体绿色 - 用于重要成功
  constexpr const char* BOLD_YELLOW = "\033[1;33m";    ///< 粗体黄色 - 用于重要警告
  constexpr const char* BOLD_BLUE   = "\033[1;34m";    ///< 粗体蓝色
  constexpr const char* BOLD_CYAN   = "\033[1;36m";    ///< 粗体青色 - 用于标题/分隔线
} // namespace task_log_color

// ============================================================================
// 日志前缀标签
// 定义在全局命名空间 task_log_tag 中，以便预处理器宏可以正确解析。
// ============================================================================
namespace task_log_tag
{
  constexpr const char* CONFIG     = "[Config]";       ///< 配置解析相关日志
  constexpr const char* BT         = "[BehaviorTree]"; ///< 行为树相关日志
  constexpr const char* ACTION     = "[Action]";       ///< 动作执行相关日志
  constexpr const char* FSM        = "[FSM]";          ///< 状态机相关日志
  constexpr const char* MODBUS     = "[Modbus]";       ///< Modbus 通信相关日志
  constexpr const char* VISION     = "[Vision]";       ///< 视觉检测相关日志
  constexpr const char* ARM        = "[Arm]";          ///< 机械臂控制相关日志
  constexpr const char* GRIPPER    = "[Gripper]";      ///< 夹爪控制相关日志
  constexpr const char* SYSTEM     = "[System]";       ///< 系统级日志
  constexpr const char* TIMER      = "[Timer]";        ///< 定时器相关日志
} // namespace task_log_tag

// ============================================================================
// 便捷命名空间别名
// 允许通过 task_log::Color::RED 或 task_log::Tag::CONFIG 的方式访问。
// ============================================================================
namespace task_log
{
  namespace Color = task_log_color;
  namespace Tag = task_log_tag;
} // namespace task_log

// ============================================================================
// 基础带颜色日志宏
// 这些宏将颜色代码和消息拼接后，通过标准 RCLCPP 接口输出。
// ============================================================================

/// 带颜色的 RCLCPP_INFO 输出
#define RCLCPP_INFO_COLOR(logger, color, msg) \
  do { \
    std::stringstream _log_ss; \
    _log_ss << color << msg << task_log_color::RESET; \
    RCLCPP_INFO(logger, "%s", _log_ss.str().c_str()); \
  } while(0)

/// 带颜色的 RCLCPP_ERROR 输出
#define RCLCPP_ERROR_COLOR(logger, color, msg) \
  do { \
    std::stringstream _log_ss; \
    _log_ss << color << msg << task_log_color::RESET; \
    RCLCPP_ERROR(logger, "%s", _log_ss.str().c_str()); \
  } while(0)

/// 带颜色的 RCLCPP_WARN 输出
#define RCLCPP_WARN_COLOR(logger, color, msg) \
  do { \
    std::stringstream _log_ss; \
    _log_ss << color << msg << task_log_color::RESET; \
    RCLCPP_WARN(logger, "%s", _log_ss.str().c_str()); \
  } while(0)

/// 带颜色的 RCLCPP_DEBUG 输出
#define RCLCPP_DEBUG_COLOR(logger, color, msg) \
  do { \
    std::stringstream _log_ss; \
    _log_ss << color << msg << task_log_color::RESET; \
    RCLCPP_DEBUG(logger, "%s", _log_ss.str().c_str()); \
  } while(0)

// ============================================================================
// 便捷日志宏（按日志级别自动着色）
// 使用方式: LOG_INFO(logger, task_log::Tag::SYSTEM, "message " << variable);
// ============================================================================

/// INFO 级别 - 绿色（普通信息）
#define LOG_INFO(logger, tag, msg) \
  RCLCPP_INFO_COLOR(logger, task_log_color::GREEN, tag << " " << msg)

/// INFO 级别 - 青色（流程跟踪信息）
#define LOG_INFO_FLOW(logger, tag, msg) \
  RCLCPP_INFO_COLOR(logger, task_log_color::CYAN, tag << " " << msg)

/// INFO 级别 - 品红（重要信息）
#define LOG_INFO_IMPORTANT(logger, tag, msg) \
  RCLCPP_INFO_COLOR(logger, task_log_color::MAGENTA, tag << " " << msg)

/// INFO 级别 - 粗体青色（标题/分隔线）
#define LOG_INFO_TITLE(logger, tag, msg) \
  RCLCPP_INFO_COLOR(logger, task_log_color::BOLD_CYAN, tag << " " << msg)

/// ERROR 级别 - 红色（一般错误）
#define LOG_ERROR(logger, tag, msg) \
  RCLCPP_ERROR_COLOR(logger, task_log_color::RED, tag << " " << msg)

/// ERROR 级别 - 粗体红色（严重错误）
#define LOG_ERROR_CRITICAL(logger, tag, msg) \
  RCLCPP_ERROR_COLOR(logger, task_log_color::BOLD_RED, tag << " " << msg)

/// WARN 级别 - 黄色（警告信息）
#define LOG_WARN(logger, tag, msg) \
  RCLCPP_WARN_COLOR(logger, task_log_color::YELLOW, tag << " " << msg)

/// DEBUG 级别 - 蓝色（调试信息）
#define LOG_DEBUG(logger, tag, msg) \
  RCLCPP_DEBUG_COLOR(logger, task_log_color::BLUE, tag << " " << msg)

/// SUCCESS 级别 - 粗体绿色（操作成功）
#define LOG_SUCCESS(logger, tag, msg) \
  RCLCPP_INFO_COLOR(logger, task_log_color::BOLD_GREEN, tag << " " << msg)

#endif // TASK_MANAGER__LOG_HELPER_HPP_
