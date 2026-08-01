# Task Manager 架构文档

## 1. 概述

`task_manager` 是一个基于 ROS2 Humble 的机器人任务管理节点，作为机器人系统的**核心调度中枢**。它通过 **Modbus TCP** 与 PLC 通信接收触发信号，利用 **有限状态机（FSM）** 管理系统生命周期，借助 **行为树（Behavior Tree）** 编排和执行复杂任务序列，并通过 **ROS2 Action** 客户端调用各硬件服务（视觉、机械臂、夹爪等）。

### 1.1 核心职责

- 加载 YAML 任务配置文件
- 通过 Modbus TCP 从站与 PLC 通信（接收触发、反馈状态）
- 管理系统的有限状态机（FSM）生命周期
- 调度行为树执行任务（串行/并行）
- 发布心跳和节点状态信息
- 调用视觉检测、机械臂运动、夹爪控制等硬件 Action

---

## 2. 整体架构图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          task_manager_node                                  │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                        TaskManagerNode                                │  │
│  │                        (rclcpp::Node)                                 │  │
│  │                                                                        │  │
│  │  ┌─────────────┐  ┌──────────────┐  ┌───────────────┐  ┌───────────┐ │  │
│  │  │ FSM State   │  │ BehaviorTree │  │ TaskAction    │  │ Modbus    │ │  │
│  │  │ (状态机)     │◄─┤ Engine       │◄─┤ Client        │  │ TcpSlave │ │  │
│  │  │             │  │ (行为树引擎)  │  │ (Action客户端) │  │ (Modbus  │ │  │
│  │  └─────────────┘  └──────────────┘  └───────────────┘  │  从站)    │ │  │
│  │                                                         └───────────┘ │  │
│  │  ┌──────────────────────────────────────────────────────────────────┐  │  │
│  │  │                    TaskConfigParser                               │  │  │
│  │  │                    (YAML 配置解析器)                               │  │  │
│  │  └──────────────────────────────────────────────────────────────────┘  │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                        LogHelper                                      │  │
│  │                    (日志辅助工具 - 全局使用)                           │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
         │                    │                      │
         ▼                    ▼                      ▼
    ┌─────────┐     ┌──────────────────┐     ┌──────────────┐
    │  PLC    │     │  ROS2 Action     │     │  /SafeNode/  │
    │(Modbus  │◄───►│  Servers:        │     │  Heartbeat   │
    │ Master) │     │  - /vision/...   │     │  & status    │
    └─────────┘     │  - /Control/...  │     └──────────────┘
                    └──────────────────┘
```

---

## 3. 模块详解

### 3.1 [`TaskManagerNode`](task_manager/include/task_manager/task_manager_node.hpp) — 核心节点

**文件**: [`task_manager/src/task_manager_node.cpp`](task_manager/src/task_manager_node.cpp)

继承自 `rclcpp::Node`，是整个系统的**主控类**。负责：

- **初始化流程**（[`run()`](task_manager/src/task_manager_node.cpp:36)）：
  1. 创建 `TaskActionClient`、`BehaviorTreeEngine`
  2. 创建行为树 tick 定时器（100ms 周期）
  3. 调用 `init_config()` 加载 YAML 配置
  4. 调用 `init_modbus()` 启动 Modbus 从站
  5. 进入 `rclcpp::spin()`

- **状态机驱动**（[`bt_tick_cb()`](task_manager/src/task_manager_node.cpp:281)）：每个 tick 根据当前 FSM 状态执行对应逻辑
- **Modbus 回调**（[`on_modbus_reg_write()`](task_manager/src/task_manager_node.cpp:223)）：PLC 写入寄存器时触发任务
- **心跳发布**（[`heartbeat_timer_cb()`](task_manager/src/task_manager_node.cpp:190)）：每秒发布心跳和状态

**关键成员**：

| 成员 | 类型 | 说明 |
|------|------|------|
| `cfg_` | `GlobalConfig` | 全局配置（从 YAML 解析） |
| `modbus_slave_` | `unique_ptr<ModbusTcpSlave>` | Modbus TCP 从站 |
| `action_client_` | `unique_ptr<TaskActionClient>` | ROS2 Action 客户端 |
| `bt_engine_` | `unique_ptr<BehaviorTreeEngine>` | 行为树引擎 |
| `current_state_` | `SystemState` | 当前 FSM 状态 |
| `error_flag_` | `uint64_t` | 错误标志位 |

---

### 3.2 [`FSM State`](task_manager/include/task_manager/fsm_state.hpp) — 有限状态机

**文件**: [`task_manager/src/fsm_state.cpp`](task_manager/src/fsm_state.cpp)

定义系统运行状态和错误标志。

#### 状态枚举 [`SystemState`](task_manager/include/task_manager/fsm_state.hpp:21)

```
UNKNOWN (0) ──► INIT (1) ──► SELF_CHECK (2) ──► RESET (3) ──► RESET_DONE (4) ──► IDLE (5)
                                                                                      │
                                                                                      ▼
                                                                                 RUNNING (6)
                                                                                      │
                                                                                      ▼
                                                                                   ERROR (7)
```

| 状态 | 值 | 说明 |
|------|-----|------|
| `UNKNOWN` | 0 | 初始未知状态 |
| `INIT` | 1 | 初始化中，查询安全节点状态 |
| `SELF_CHECK` | 2 | 自检中，执行自检任务序列 |
| `RESET` | 3 | 复位中，执行复位任务序列 |
| `RESET_DONE` | 4 | 复位完成 |
| `IDLE` | 5 | 空闲待命，等待 PLC 触发 |
| `RUNNING` | 6 | 任务执行中 |
| `ERROR` | 7 | 错误状态 |

#### 错误位掩码 [`ErrorBitMask`](task_manager/include/task_manager/fsm_state.hpp:34)

| 标志 | 位 | 说明 |
|------|-----|------|
| `ERR_EMERGENCY` | 0 | 紧急停止 |
| `ERR_MODBUS_FAULT` | 1 | Modbus 通信故障 |
| `ERR_VISION_TIMEOUT` | 2 | 视觉检测超时 |
| `ERR_ARM_PLAN_FAIL` | 3 | 机械臂规划失败 |
| `ERR_GRIPPER_FAULT` | 4 | 夹爪故障 |
| `ERR_CONFIG_PARSE_FAIL` | 5 | 配置文件解析失败 |
| `ERR_BT_EXEC_ERROR` | 6 | 行为树执行错误 |
| `ERR_SERVICE_TIMEOUT` | 7 | 服务调用超时 |

#### 状态转换流程（启动阶段）

```
[bt_tick_cb] 每 100ms 执行一次
    │
    ├─ UNKNOWN: 无操作
    ├─ INIT: boot_check_system_state() → 查询安全节点状态
    │     └─ 安全节点返回 INIT/SELF_CHECK/RESET → 执行自检 → SELF_CHECK
    │     └─ 安全节点返回 IDLE/ERROR → IDLE
    │
    ├─ SELF_CHECK: tick 行为树
    │     └─ 成功 → boot_reset_system_state() → RESET
    │     └─ 失败 → INIT（重试）
    │
    ├─ RESET: tick 行为树
    │     └─ 成功 → RESET_DONE
    │     └─ 失败 → 重试 RESET
    │
    ├─ RESET_DONE: boot_check_system_state() → 再次查询
    ├─ IDLE: 等待 Modbus 触发
    ├─ RUNNING: tick 行为树
    │     └─ 完成 → 写状态寄存器 → IDLE
    └─ ERROR: 打印错误日志
```

---

### 3.3 [`BehaviorTreeEngine`](task_manager/include/task_manager/behavior_tree_engine.hpp) — 行为树引擎

**文件**: [`task_manager/src/behavior_tree_engine.cpp`](task_manager/src/behavior_tree_engine.cpp)

实现了一个**轻量级自定义行为树框架**，不依赖第三方 BT 库。

#### 节点类型

| 节点 | 类名 | 类型 | 说明 |
|------|------|------|------|
| **基类** | `BTNode` | 抽象 | 提供名称、状态、子节点管理 |
| **动作节点** | `ActionNode` | 叶子 | 封装可执行回调，每次 tick 调用 |
| **条件节点** | `ConditionNode` | 叶子 | 封装条件判断回调 |
| **顺序节点** | `SequenceNode` | 控制流 | 按顺序执行子节点，全部成功才成功 |
| **并行节点** | `ParallelNode` | 控制流 | 所有子节点并行执行 |
| **回退节点** | `FallbackNode` | 控制流 | 按顺序尝试，任一成功即返回 |

#### 节点状态 [`BTStatus`](task_manager/include/task_manager/behavior_tree_engine.hpp:32)

```
IDLE ──► RUNNING ──► SUCCESS
                  └──► FAILURE
```

#### 引擎核心方法

| 方法 | 说明 |
|------|------|
| `setRoot()` | 设置根节点 |
| `start()` | 启动行为树（重置状态） |
| `abort()` | 中止执行 |
| `tick_once()` | 执行一次 tick，驱动整棵树 |
| `load_action_group()` | 加载串行动作序列（Sequence 根节点） |
| `load_parallel_task()` | 加载并行动作（Parallel 根节点，每个分支为 Sequence） |

#### 动作加载机制

**串行加载** ([`load_action_group()`](task_manager/src/behavior_tree_engine.cpp:440)):
```
SequenceNode "Action_Group_Root"
    ├── ActionNode "Action_vision_detect"
    ├── ActionNode "Action_move_arm"
    └── ActionNode "Action_gripper_ctrl"
```

**并行加载** ([`load_parallel_task()`](task_manager/src/behavior_tree_engine.cpp:481)):
```
ParallelNode "Parallel_Task_Root"
    ├── SequenceNode "Sequence_Branch_1"
    │     ├── ActionNode "vision_detect1"
    │     └── ActionNode "move_arm1"
    └── SequenceNode "Sequence_Branch_2"
          ├── ActionNode "vision_detect2"
          └── ActionNode "gripper_ctrl2"
```

每个 ActionNode 的回调逻辑：
1. **IDLE** → 调用 `send_goal()` 发送 Action goal → 返回 `RUNNING`
2. **RUNNING** → 检查超时 → 返回 `RUNNING` 或 `FAILURE`
3. **SUCCESS** → 重置状态 → 返回 `SUCCESS`
4. **FAILURE** → 获取详细错误 → 返回 `FAILURE` 或 `RUNNING`（可重试）

---

### 3.4 [`TaskActionClient`](task_manager/include/task_manager/task_action_client.hpp) — Action 客户端

**文件**: [`task_manager/src/task_action_client.cpp`](task_manager/src/task_action_client.cpp)

封装了与各硬件服务通信的 ROS2 Action 客户端。

#### 支持的 Action 类型

| 动作类型 | Action | 服务端 Topic | 说明 |
|----------|--------|-------------|------|
| `vision_detect` | `VisionDetection` | `/vision/detection_pose_cloud` | 视觉目标检测 |
| `move_arm` | `MoveArm` | `/Control/move_arm` | 机械臂运动 |
| `gripper_ctrl` | `ControlGripper` | `/Control/control_gripper` | 夹爪控制 |
| `vision_check_self` | `HardwareCheck` | `/vision/Check_self` | 视觉自检 |
| `control_check_self` | `HardwareCheck` | `/Control/Checkself` | 运控自检 |
| `vision_reset` | `HardwareReset` | `/vision/Reset` | 视觉复位 |
| `control_reset` | `HardwareReset` | `/Control/Reset` | 运控复位 |

#### 调用结果枚举 [`ActionCallResult`](task_manager/include/task_manager/task_action_client.hpp:48)

| 值 | 说明 |
|------|------|
| `SUCCESS` | 调用成功 |
| `FAIL_SERVER_BUSY` | 服务端繁忙 |
| `FAIL_SERVER_TIMEOUT` | 服务端连接超时 |
| `FAIL_GOAL_REJECTED` | 目标被拒绝 |
| `FAIL_PARAM_INVALID` | 参数无效 |
| `FAIL_UNKNOWN_TYPE` | 未知动作类型 |
| `FAIL_TIMEOUT` | 执行超时 |
| `FAIL_RESOURCE_BUSY` | 本地资源被占用 |
| `FAIL_GENERAL` | 通用失败 |

#### 资源锁机制

系统定义了 5 种可互斥访问的硬件资源：

| 资源 | 说明 |
|------|------|
| `VISION` | 视觉相机 |
| `LEFT_ARM` | 左机械臂 |
| `RIGHT_ARM` | 右机械臂 |
| `LEFT_GRIPPER` | 左夹爪 |
| `RIGHT_GRIPPER` | 右夹爪 |

资源锁使用 `std::atomic<bool>` 的 `compare_exchange_strong` 实现无锁竞争。每个 Action 在发送 goal 前先尝试获取所需资源，获取失败则返回 `FAIL_RESOURCE_BUSY`，行为树会在下次 tick 重试。

#### 并行分支支持

通过 `state_key` 机制支持并行执行多个 Action：
- 每个并行分支使用独立的 `state_key`（如 `"seq1_vision_detect_0"`）
- 每个 `state_key` 对应独立的 `ActionContext`（状态、超时、资源）
- 互不干扰，可同时跟踪多个 Action 的执行状态

---

### 3.5 [`ModbusTcpSlave`](task_manager/include/task_manager/modbus_slave.hpp) — Modbus TCP 从站

**文件**: [`task_manager/src/modbus_slave.cpp`](task_manager/src/modbus_slave.cpp)

基于 `libmodbus` 库实现的 Modbus TCP 服务器。

#### 功能

- **监听**: 在配置的 IP:Port 上监听 PLC 连接
- **读保持寄存器**（功能码 0x03）：PLC 读取系统状态
- **写单寄存器**（功能码 0x06）：PLC 触发任务
- **写入回调**: 寄存器被写入时触发 `on_modbus_reg_write()` 回调
- **线程安全**: 使用 `std::mutex` 保护寄存器映射表

#### 寄存器地址映射

| 地址范围 | 说明 |
|----------|------|
| `40001 + offset` | 触发寄存器（PLC 写入触发任务） |
| `40001 + offset + 10000` | 状态寄存器（系统写入反馈状态） |

状态寄存器值含义：
- `1` = 任务执行中
- `2` = 任务执行成功
- `3` = 任务执行失败

---

### 3.6 [`TaskConfigParser`](task_manager/include/task_manager/task_config_parser.hpp) — 配置解析器

**文件**: [`task_manager/src/task_config_parser.cpp`](task_manager/src/task_config_parser.cpp)

负责解析 YAML 格式的任务配置文件。

#### 配置结构

```yaml
node_id: 1                          # 节点 ID
modbus_slave:                        # Modbus 配置
  listen_ip: "0.0.0.0"
  listen_port: 12000
tasks:                               # 任务字典
  task_name:                         # 任务名称
    trigger_reg: 40001               # 触发寄存器地址
    status_reg: 50001                # 状态寄存器地址
    action_sequencenum: 2            # 动作序列数量（支持并行）
    action_sequence1: [...]          # 序列 1（串行动作列表）
    action_sequence2: [...]          # 序列 2（与序列 1 并行执行）
boot_task:                           # 启动任务
  self_check:                        # 开机自检
    action_sequence: [...]
  reset:                             # 开机复位
    action_sequence: [...]
```

#### 动作参数结构体

| 结构体 | 动作类型 | 关键字段 |
|--------|----------|----------|
| `VisionActionParam` | `vision_detect` | target_obj_name, need_6d_pose, downsample_res |
| `MoveArmParam` | `move_arm` | arm_id, enable_collision_check, config_pose, bound_size, use_force_control, hand_action_type, timeout_ms |
| `GripperParam` | `gripper_ctrl` | gripper_id, position, speed, timeout_ms |
| `VisionCheckSelfParam` | `vision_check_self` | timeout_ms |
| `ControlCheckSelfParam` | `control_check_self` | timeout_ms, com_port |
| `VisionResetParam` | `vision_reset` | timeout_ms |
| `ControlResetParam` | `control_reset` | timeout_ms |

---

### 3.7 [`LogHelper`](task_manager/include/task_manager/log_helper.hpp) — 日志辅助工具

**文件**: 纯头文件 [`task_manager/include/task_manager/log_helper.hpp`](task_manager/include/task_manager/log_helper.hpp)

提供带 ANSI 颜色的日志输出宏。

#### 颜色定义

| 颜色 | 用途 |
|------|------|
| `GREEN` | 普通信息 |
| `RED` | 错误信息 |
| `YELLOW` | 警告信息 |
| `BLUE` | 调试信息 |
| `CYAN` | 流程跟踪 |
| `MAGENTA` | 重要信息 |
| `BOLD_CYAN` | 标题/分隔线 |
| `BOLD_RED` | 严重错误 |
| `BOLD_GREEN` | 操作成功 |

#### 日志标签

| 标签 | 说明 |
|------|------|
| `[Config]` | 配置解析 |
| `[BehaviorTree]` | 行为树 |
| `[Action]` | 动作执行 |
| `[FSM]` | 状态机 |
| `[Modbus]` | Modbus 通信 |
| `[Vision]` | 视觉检测 |
| `[Arm]` | 机械臂 |
| `[Gripper]` | 夹爪 |
| `[System]` | 系统级 |
| `[Timer]` | 定时器 |

#### 便捷宏

| 宏 | 颜色 | 级别 |
|-----|------|------|
| `LOG_INFO()` | 绿色 | INFO |
| `LOG_INFO_FLOW()` | 青色 | INFO |
| `LOG_INFO_IMPORTANT()` | 品红 | INFO |
| `LOG_INFO_TITLE()` | 粗体青色 | INFO |
| `LOG_ERROR()` | 红色 | ERROR |
| `LOG_ERROR_CRITICAL()` | 粗体红色 | ERROR |
| `LOG_WARN()` | 黄色 | WARN |
| `LOG_DEBUG()` | 蓝色 | DEBUG |
| `LOG_SUCCESS()` | 粗体绿色 | INFO |

---

## 4. 数据流

### 4.1 启动流程

```
TaskManagerNode::run()
    │
    ├─ 创建 TaskActionClient、BehaviorTreeEngine
    ├─ 创建 bt_tick_timer (100ms)
    ├─ init_config() → TaskConfigParser::load_config()
    │     └─ 解析 YAML → GlobalConfig
    ├─ init_modbus() → ModbusTcpSlave::start()
    │     └─ 启动 Modbus TCP 服务器线程
    └─ rclcpp::spin()
          │
          └─ bt_tick_cb() [每 100ms]
                ├─ INIT → boot_check_system_state()
                │     └─ 查询 /Safety/get_system_status
                │           └─ 响应回调 → 执行自检/复位或进入 IDLE
                ├─ SELF_CHECK → bt_engine_->tick_once()
                ├─ RESET → bt_engine_->tick_once()
                └─ IDLE → 等待 Modbus 触发
```

### 4.2 任务触发流程

```
PLC 写入 Modbus 寄存器 40001 (value=1)
    │
    ▼
ModbusTcpSlave::handle_modbus_request()
    │
    ▼
on_modbus_reg_write(addr=40001, old=0, new=1)
    │
    ▼
handle_trigger_task(trigger_addr=40001)
    │
    ├─ 查找 task_dict 中 trigger_reg==40001 的任务
    ├─ switch_state(RUNNING)
    ├─ bt_engine_->load_parallel_task(action_seq)
    └─ bt_engine_->start()
          │
          ▼
    bt_tick_cb() [每 100ms]
          │
          ├─ bt_engine_->tick_once()
          │     └─ 驱动行为树执行
          │           ├─ ActionNode: send_goal() → TaskActionClient
          │           │     └─ ROS2 Action → 硬件服务
          │           └─ 检查 Action 状态（RUNNING/SUCCESS/FAILURE）
          │
          └─ 任务完成
                ├─ 写状态寄存器 (2=成功, 3=失败)
                └─ switch_state(IDLE)
```

### 4.3 并行任务执行

```
load_parallel_task() 构建:
    ParallelNode
    ├── Sequence_Branch_1 (左臂)
    │     ├── vision_detect (state_key="seq1_vision_detect_0")
    │     └── move_arm     (state_key="seq1_move_arm_1")
    └── Sequence_Branch_2 (右臂)
          ├── vision_detect (state_key="seq2_vision_detect_0")
          └── gripper_ctrl  (state_key="seq2_gripper_ctrl_1")

tick_once() 时:
    ParallelNode 遍历所有分支
    ├── Branch 1: tick Sequence → tick vision_detect
    │     └─ ActionNode 回调 → 检查 state_key="seq1_..." 的状态
    └── Branch 2: tick Sequence → tick vision_detect
          └─ ActionNode 回调 → 检查 state_key="seq2_..." 的状态
```

---

## 5. 外部依赖

| 依赖 | 类型 | 用途 |
|------|------|------|
| `rclcpp` | ROS2 | ROS2 客户端库 |
| `rclcpp_action` | ROS2 | ROS2 Action 客户端 |
| `std_msgs` | ROS2 | 标准消息类型 |
| `geometry_msgs` | ROS2 | 几何消息（PoseStamped） |
| `sensor_msgs` | ROS2 | 传感器消息（PointCloud2） |
| `com_interfaces` | 自定义 | 自定义 Action/消息/服务定义 |
| `yaml-cpp` | 第三方 | YAML 配置文件解析 |
| `libmodbus` | 第三方 | Modbus TCP 通信 |
| `uuid` | 系统 | UUID 生成（任务 ID） |

---

## 6. 文件清单

```
task_manager/
├── CMakeLists.txt                          # 构建配置
├── package.xml                             # 包描述
├── config/
│   └── task_config.yaml                    # 任务配置文件
├── launch/
│   └── task_manager.launch.py              # 启动文件
├── include/task_manager/
│   ├── task_manager_node.hpp               # 核心节点头文件
│   ├── fsm_state.hpp                       # 状态机定义
│   ├── behavior_tree_engine.hpp            # 行为树引擎头文件
│   ├── task_action_client.hpp              # Action 客户端头文件
│   ├── task_config_parser.hpp              # 配置解析器头文件
│   ├── modbus_slave.hpp                    # Modbus 从站头文件
│   └── log_helper.hpp                      # 日志辅助工具
└── src/
    ├── task_manager_node.cpp               # 核心节点实现
    ├── fsm_state.cpp                       # 状态机实现
    ├── behavior_tree_engine.cpp            # 行为树引擎实现
    ├── task_action_client.cpp              # Action 客户端实现
    ├── task_config_parser.cpp              # 配置解析器实现
    └── modbus_slave.cpp                    # Modbus 从站实现
```

---

## 7. 关键设计决策

1. **自定义行为树而非 BT.CPP**：轻量级实现，无第三方依赖，满足当前串行/并行需求
2. **Modbus 寄存器触发**：通过 PLC 写入寄存器触发任务，适合工业自动化场景
3. **资源锁机制**：基于 `atomic` 的无锁竞争，避免死锁，支持并行分支的资源互斥
4. **state_key 并行分支**：通过字符串 key 隔离并行分支的 Action 状态，实现真正的并行执行
5. **FSM + BT 双层调度**：FSM 管理系统生命周期，BT 管理任务执行逻辑，职责分离
