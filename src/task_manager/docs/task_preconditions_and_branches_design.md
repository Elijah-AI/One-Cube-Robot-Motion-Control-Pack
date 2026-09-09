# task_manager 动作前置条件检测与结果分支执行 —— 方案设计

> 版本：v0.2（方案评审稿）
> 范围：仅方案设计，不改动现有代码。
> 目标：为 [`task_config.yaml`](../config/task_config.yaml) 中的**每个原子动作**增加：
> 1. **执行前条件检测（preconditions）**：检测通过才进入动作执行；
> 2. **执行结果分支（on_success / on_failure）**：根据动作执行结果跳转执行分支动作序列；
> 3. **配置文件分级化**：主配置（Modbus 参数 + 寄存器 → 次级配置映射）与次级任务动作配置分离，启动仅加载主配置并级联加载（见 4.1）。

---

## 1. 需求概述

1. **动作级前置条件**：在每一个原子动作真正下发执行之前，先对一组条件进行检测，全部（或按逻辑组合）通过后才进入动作执行。
2. **可配置**：条件类型、阈值、比较方式、等待策略、超时行为、是否启用等均通过 YAML 配置文件声明，不改代码即可调整。
3. **条件清单**（用户指定 10 类）：
   - 电池电量
   - 机器人状态（整机 FSM）
   - 机器人安全状态
   - 左 / 右机械臂是否空闲
   - 头部是否空闲
   - 腰部是否空闲
   - 左 / 右夹爪是否空闲
   - 左 / 右吸气泵是否空闲
   - 左 / 右灵巧手是否空闲
4. **结果分支**：每个动作根据执行结果（成功 / 失败）执行相应的分支动作序列（如：失败后执行"回安全位姿 / 夹爪复位"等补偿动作）。
5. **配置文件分级化**：配置拆分为主配置 + 次级（任务动作）配置两级。启动时仅加载主配置文件；主配置只声明 Modbus slave 参数（slave ID、监听地址 / 端口）与"寄存器地址 → 次级配置文件名 + 状态返回寄存器"映射；task_manager 依据主配置逐级加载各次级配置文件中的动作列表（详见 4.1）。

---

## 2. 现状分析与差距

### 2.1 配置模型

- [`AtomicAction`](../include/task_manager/task_config_parser.hpp:159) 目前只包含：`action_type`、`delay_ms`、各动作参数、`feedback_check`（动作执行**之后**的反馈条件重试）。
- **没有**"执行前条件"与"结果分支"的字段。
- 解析逻辑集中在 [`parse_action()`](../include/task_manager/task_config_parser.hpp:214)。

### 2.2 执行引擎

- [`load_action_group()`](../src/behavior_tree_engine.cpp:953)（串行）与 [`load_parallel_task()`](../src/behavior_tree_engine.cpp:995)（并行）把动作列表转成 `SequenceNode` / `ParallelNode`，每个动作是一个 `ActionNode` 叶子。
- 动作节点回调（[`build_action_node()`](../src/behavior_tree_engine.cpp:629)）基于 `state_key` 查询动作状态：RUNNING→等待、SUCCESS→成功、FAILURE→失败。
- 已有控制流节点：`SequenceNode` / `ParallelNode` / `FallbackNode` / `RetryUntilConditionNode`。
- **已有 `ConditionNode`**（[`behavior_tree_engine.hpp`](../include/task_manager/behavior_tree_engine.hpp:98)），可复用为条件判断叶子。

### 2.3 资源占用（本地资源锁）

- [`ResourceType`](../include/task_manager/task_action_client.hpp:102) 现有：`VISION / LEFT_ARM / RIGHT_ARM / LEFT_GRIPPER / RIGHT_GRIPPER / LEFT_SUCK / RIGHT_SUCK`。
- 已实现 `acquire_resource()` / `release_resource()` / `is_resource_busy()`（见 [`task_action_client.cpp`](../src/task_action_client.cpp:1846)），在 `send_goal` 前自动加锁、结果回调后释放。
- **缺失**：`LEG`、`HEAD`、`WAIST`、`LEFT_HAND`、`RIGHT_HAND`。

### 2.4 状态数据源（关键差距）

| 条件 | 期望数据 | 现状 | 差距 |
|---|---|---|---|
| 电池电量 | 电量百分比 / 电压 | 安全节点设备状态 topic（`/Safety/device_safety_state`，[`SafetyState`](../com_interfaces/msg/SafetyState.msg:1) 现仅 status/error_msg） | 需扩展 `SafetyState.msg` 增加电池字段（`battery_percentage` / `battery_voltage`），由安全节点统一发布 |
| 机器人状态 | 整机 FSM 状态 | 安全监控节点维护整机 FSM（[`SystemFsmState`](../safety_monitor/include/safety_monitor/safety_monitor_node.hpp:23)，0~8）；task_manager 已订阅 `/Safety/device_safety_state`；`/Safety/get_system_status` 服务可用 | 基本满足，**`robot_status` 以安全监控节点为准**（用户确认，见 9.5） |
| 机器人安全状态 | `SafetyState.status` | 已订阅 `/Safety/device_safety_state`（[`task_manager_node.cpp`](../src/task_manager_node.cpp:33)） | 满足，需缓存供条件查询 |
| 左右臂空闲 | 臂忙闲标志 | 本地资源锁已有 `LEFT_ARM / RIGHT_ARM`（`is_resource_busy()`） | 满足，直接读取本地资源锁 |
| 头部空闲 | 头忙闲标志 | 本地无资源锁（头随 MovePose/MoveJoints 派发） | 扩展资源锁 `HEAD`，与 MovePose/MoveJoints 捆绑（不依赖远端接口） |
| 腰部空闲 | 腰忙闲标志 | 本地无资源锁；腰部动作（关节 16-18）由 task_manager 的 `move_joints` 派发（用户确认，见 9.4） | 扩展资源锁 `WAIST`，与 MovePose/MoveJoints 捆绑（不依赖远端接口） |
| 左右夹爪空闲 | 夹爪忙闲标志 | 本地资源锁已有 `LEFT_GRIPPER / RIGHT_GRIPPER` | 满足，直接读取本地资源锁 |
| 左右吸气泵空闲 | 吸泵忙闲标志 | 本地资源锁已有 `LEFT_SUCK / RIGHT_SUCK` | 满足，直接读取本地资源锁 |
| 左右灵巧手空闲 | 灵巧手忙闲标志 | 本地无资源锁（灵巧手随 `MovePose.hand_action_type` / MoveJoints 执行） | 扩展资源锁 `LEFT_HAND / RIGHT_HAND`，与 MovePose/MoveJoints 捆绑（不依赖远端接口） |

> 结论（依据用户 2026-08-26 答复确认）：
> - **机器人状态 `robot_status` 与安全状态 `safety_status` 均以安全监控节点为准**（订阅 `/Safety/device_safety_state`，`robot_status` 采用安全节点 [`SystemFsmState`](../safety_monitor/include/safety_monitor/safety_monitor_node.hpp:23) 口径）；
> - **电池电量由安全节点在其设备状态 topic 上附带发布**（扩展 `SafetyState.msg` 电池字段）；
> - **各部件忙闲由 task_manager 内部维护的本地资源锁提供**，不依赖远端接口反馈；
> - 前提：所有部件动作均由 task_manager 统一派发调度（arm/leg/head/hand ↔ MovePose/MoveJoints、suck ↔ ControlSuck、gripper ↔ ControlGripper）。机器异常时安全节点会接管部分控制逻辑，但**接管前会发布安全状态异常 topic，task_manager 收到后取消当前任务**，故异常接管不会产生 task_manager 不可见的持续占用（见 9.3）。

---

## 3. 总体架构设计

```
┌──────────────────────────────  task_manager ──────────────────────────────┐
│                                                                           │
│  task_config.yaml ──► TaskConfigParser ──► GlobalConfig                  │
│                       （新增解析 preconditions / 结果分支）                  │
│                                                                           │
│  ┌────────────┐   订阅/缓存    ┌───────────────────┐                      │
│  │StateCache  │◄────────────── │ /Safety/device_   │ (SafetyState)       │
│  │(条件数据缓存)│               │ safety_state      │                      │
│  │  safety_    │◄────────────── │ /Safety/device_   │ (SafetyState 扩展)   │
│  │  battery_   │               │ safety_state      │ (status+battery_*)    │
│  │  fsm_state_ │←─安全节点状态   └───────────────────┘ (robot_status 以安    │
│  │  busy_*     │←─本地资源锁     TaskActionClient::is_resource_busy()       │
│  └─────┬──────┘                                                            │
│        │ 提供 evaluate()                                                  │
│        ▼                                                                  │
│  ┌────────────────────────────────────────┐                               │
│  │ BehaviorTreeEngine                     │                               │
│  │  GuardedActionNode（新增控制流节点）      │                               │
│  │  ├─ PreconditionGate（条件检测门）        │                               │
│  │  ├─ ActionNode（主动作，复用现有逻辑）     │                               │
│  │  └─ ResultBranch（on_success / on_failure）│                            │
│  └────────────────────────────────────────┘                               │
│        │                                                                  │
│        ▼                                                                  │
│  TaskActionClient（资源锁扩展 HEAD/WAIST/LEG/HAND；与 MovePose/MoveJoints、  │
│   ControlSuck、ControlGripper 捆绑加锁）──────────► 各远端 Action Server     │
└───────────────────────────────────────────────────────────────────────────┘
```

---

## 4. 配置文件 Schema 设计

### 4.1 配置文件分级化（主 / 次级配置）

#### 4.1.1 分级动机与原则

现有 [`task_config.yaml`](../config/task_config.yaml) 为**单一文件**：`modbus_slave` 运行参数与全部任务动作列表（`tasks.*.action_sequence*`）混在一起。随着任务增多，文件膨胀、难以维护，且"运行环境参数"（slave ID / 监听地址端口 / 寄存器分配）与"业务动作逻辑"强耦合。

分级化原则（用户需求）：
- **主配置文件**：只承载"运行环境与接口映射"——slave ID、监听地址 / 端口、以及"各寄存器地址 → 次级配置文件名 + 状态返回寄存器"，**不含任何动作列表**；
- **次级配置文件**：按任务拆分的"动作列表"，可独立增删改，不影响其它任务与主配置；
- 启动时**只加载主配置文件**，由主配置驱动逐级加载次级文件，保证"单一入口"。

#### 4.1.2 主配置文件 schema

```yaml
# config/task_config.yaml —— 主配置文件（唯一启动入口）
node_id: 1
modbus_slave:
  slave_id: 1                # Modbus 从站 ID（新增；现代码硬编码 slave id=1，见 modbus_slave.cpp:40）
  listen_ip: "0.0.0.0"       # 监听地址
  listen_port: 12000         # 监听端口
tasks:
  move_box:
    trigger_reg: 40001       # 触发寄存器（逻辑地址）
    status_reg: 50001        # 状态返回寄存器
    config_file: "tasks/move_box.yaml"       # 次级配置：该任务的动作列表文件
  go_pick_pose:
    trigger_reg: 40002
    status_reg: 50002
    config_file: "tasks/go_pick_pose.yaml"
  go_idle_pose:
    trigger_reg: 40003
    status_reg: 50003
    config_file: "tasks/go_idle_pose.yaml"
boot_task:
  config_file: "boot_task.yaml"   # 启动任务（自检 / 复位）同样独立成文件
```

- 主配置中**不再出现** `action_sequencenum / action_sequence*` 等动作内容。
- 路径基准：`config_file` **相对主配置文件所在目录解析**（用户确认，见 9.10），**不需要统一的 `config_dir` 前缀**，也不使用绝对路径。

#### 4.1.3 次级（任务动作）配置文件 schema

`config/tasks/move_box.yaml`（动作字段语义与现有一致，并支持新增的条件 / 分支字段）：

```yaml
# 单个任务的动作列表（触发寄存器与状态寄存器见主配置）
action_sequencenum: 1
action_sequence1:
  - action_type: vision_detect
    delay_ms: 0
    preconditions: { ... }     # 可选：执行前条件检测（见 4.2）
    params: { ... }
    on_failure: { ... }        # 可选：结果分支（见 4.3）
  - action_type: move_pose
    delay_ms: 0
    params: { ... }
    feedback_check: { ... }    # 原有反馈条件重试保持不变
action_sequence2:
  - action_type: move_joints
    delay_ms: 0
    params: { ... }
```

`config/boot_task.yaml`（启动任务独立成文件）：

```yaml
self_check:
  action_sequence:
    - action_type: vision_check_self
      params: { timeout_ms: 15000 }
    - action_type: control_check_self
      params: { timeout_ms: 15000 }
reset:
  action_sequence:
    - action_type: vision_reset
      params: { timeout_ms: 15000 }
    - action_type: control_reset
      params: { timeout_ms: 15000 }
```

#### 4.1.4 加载流程

1. 启动时加载**主配置文件**（launch 的 `config_path` 指向它，见 [`task_manager.launch.py`](../launch/task_manager.launch.py:15)）；
2. 解析 `modbus_slave` → 以 `slave_id / listen_ip / listen_port` 构造 `ModbusTcpSlave`（原构造仅 `(ip, port)`，需增加 slave_id 参数）；
3. 遍历 `tasks` 映射：对每个条目按 `config_file`（**相对主配置文件所在目录**解析，用户确认，见 9.10）加载对应**次级任务动作配置**，与 `trigger_reg / status_reg` 合并为 `TaskMapItem`；
4. 加载 `boot_task.config_file` → `BootTaskSet`；
5. 合并为完整 `GlobalConfig`（内存结构不变），进入既有调度逻辑（`start_task_by_reg` 等**无需改动**）。

> 加载时机：**首版仅启动时一次性加载全部主 / 次级配置**，运行期不重载（用户确认，见 9.11）。

```
主配置文件 task_config.yaml
  │ 1. modbus_slave ──► ModbusTcpSlave(slave_id, ip, port)
  │ 2. tasks[].config_file ──► 解析次级任务动作列表 ──► TaskMapItem{trigger_reg, status_reg, action_seq}
  │ 3. boot_task.config_file ──► BootTaskSet
  ▼
GlobalConfig（与现状内存结构一致，供 BehaviorTreeEngine 构建）
```

#### 4.1.5 兼容双格式（内联 / 外链）

为不破坏现有部署，主配置中每个任务条目**二选一**：

| 任务条目形式 | 判定 | 行为 |
|---|---|---|
| 外链（新格式） | 含 `config_file` | 从次级文件加载动作列表 |
| 内联（旧格式） | 含 `action_sequencenum` / `action_sequenceN`（无 `config_file`） | 沿用现有内联解析，完全向后兼容 |

`boot_task` 同理（`config_file` 外链 或 内联 `self_check / reset`）。

#### 4.1.6 目录结构建议

```
task_manager/config/
├── task_config.yaml          # 主配置文件（分级入口，启动时仅加载此文件）
├── boot_task.yaml            # 启动任务动作配置
└── tasks/
    ├── move_box.yaml         # 各任务动作配置
    ├── go_pick_pose.yaml
    └── go_idle_pose.yaml
```

### 4.2 前置条件 `preconditions`

每个原子动作节点下新增可选字段 `preconditions`：

```yaml
- action_type: move_pose
  delay_ms: 0
  preconditions:
    logical_op: "and"          # and=全部满足（默认） / or=任一满足
    policy: "wait"             # 条件不满足时的处理：
                               #   wait      = 周期性重查直至满足或超时（默认）
                               #   fail_fast = 立即判定本动作失败
                               #   skip      = 跳过本动作，视为成功继续
    max_wait_ms: 30000         # wait 模式下最长等待时间（毫秒），默认 0=无限等待
    check_interval_ms: 100     # wait 模式下的重查周期（毫秒），默认 100
    conditions:
      # —— 1) 电池电量（数值条件：判定电量是否满足阈值，与"空闲/忙"无关）——
      - type: battery_level
        enable: true
        operator: ">="         # 比较运算符：>=, >, <=, <, ==, !=
        threshold: 20.0        # 电量阈值（百分比 %），电量 >= 阈值才通过
        on_timeout: "fail"     # wait 超时后：fail(失败) / pass(放行) / abort(中止整个任务)
      # —— 2) 机器人状态（整机 FSM）——
      - type: robot_status
        enable: true
        expected: ["IDLE"]     # 允许的状态（以安全监控节点 SystemFsmState 为准，见 9.5），
                               # 支持枚举名或数值
                               # UNKNOWN=0 INIT=1 SELF_CHECK=2 RESET=3 RESET_DONE=4
                               # IDLE=5 RUN=6 ABNORMAL=7 EMERGENCY=8
        on_timeout: "fail"
      # —— 3) 机器人安全状态 ——
      - type: safety_status
        enable: true
        expected: [0]          # SafetyState.status：0=正常 1=功能受限 2=急停
        on_timeout: "fail"
      # —— 4) 左/右机械臂空闲 ——
      - type: arm_idle
        enable: true
        part: "left"           # left / right / both，默认 both
        expected: true         # 期望空闲(true) / 占用(false)
        on_timeout: "fail"
      # —— 5) 头部空闲 ——
      - type: head_idle
        enable: true
        expected: true
        on_timeout: "fail"
      # —— 6) 腰部空闲 ——
      - type: waist_idle
        enable: true
        expected: true
        on_timeout: "fail"
      # —— 7) 左/右夹爪空闲 ——
      - type: gripper_idle
        enable: true
        part: "left"           # left / right / both
        expected: true
        on_timeout: "fail"
      # —— 8) 左/右吸气泵空闲 ——
      - type: suck_idle
        enable: true
        part: "left"           # left / right / both
        expected: true
        on_timeout: "fail"
      # —— 9) 左/右灵巧手空闲 ——
      - type: dexterous_hand_idle
        enable: true
        part: "left"           # left / right / both
        expected: true
        on_timeout: "fail"
  params:
    ...                        # 原有动作参数不变
```

**条件通用字段**：

| 字段 | 必选 | 说明 |
|---|---|---|
| `type` | 是 | 条件类型，见下表 |
| `enable` | 否 | 是否启用，默认 `true`；`false` 时该条件恒通过 |
| `expected` | 取决于类型 | 期望值/期望状态集合（状态类条件用） |
| `operator` | 取决于类型 | 比较运算符（数值类条件用，默认 `>=`） |
| `threshold` | 取决于类型 | 数值阈值（数值类条件用） |
| `part` | 取决于类型 | 部件方位：`left` / `right` / `both`（部件空闲类条件用） |
| `on_timeout` | 否 | `wait` 超时后的行为：`fail` / `pass` / `abort`，默认 `fail` |

**条件类型一览（10 类）**：

| `type` | 类别 | 数据来源 | 关键字段 |
|---|---|---|---|
| `battery_level` | 数值 | 安全节点设备状态 topic（`/Safety/device_safety_state`，`SafetyState.battery_*` 扩展） | `operator` / `threshold` |
| `robot_status` | 状态集合 | 安全监控节点 [`SystemFsmState`](../safety_monitor/include/safety_monitor/safety_monitor_node.hpp:23) | `expected[]` |
| `safety_status` | 状态集合 | `/Safety/device_safety_state` | `expected[]` |
| `arm_idle` | 部件空闲 | 本地资源锁 `LEFT_ARM / RIGHT_ARM` | `part` / `expected` |
| `head_idle` | 部件空闲 | 本地资源锁 `HEAD`（新增） | `part`(固定 head) / `expected` |
| `waist_idle` | 部件空闲 | 本地资源锁 `WAIST`（新增） | `part`(固定 waist) / `expected` |
| `gripper_idle` | 部件空闲 | 本地资源锁 `LEFT_GRIPPER / RIGHT_GRIPPER` | `part` / `expected` |
| `suck_idle` | 部件空闲 | 本地资源锁 `LEFT_SUCK / RIGHT_SUCK` | `part` / `expected` |
| `dexterous_hand_idle` | 部件空闲 | 本地资源锁 `LEFT_HAND / RIGHT_HAND`（新增） | `part` / `expected` |

> 说明：`expected` 支持用数值或字符串（如 `"IDLE"`、`"RUNNING"`），解析时统一映射为枚举值，便于人工阅读。
> 说明：**部件空闲类条件的忙闲状态由 task_manager 内部维护（本地资源锁），不依赖远端接口反馈**（详见 5.1.1）；资源-接口捆绑关系（arm/leg/head/hand ↔ MovePose/MoveJoints、suck ↔ ControlSuck、gripper ↔ ControlGripper）见 5.4。

### 4.3 结果分支 `on_success` / `on_failure`

每个原子动作节点下新增可选字段：

```yaml
- action_type: move_pose
  delay_ms: 0
  # —— 成功分支：主动作成功后依次执行 ——
  on_success:
    branch_policy: "continue"   # continue=分支执行成功后本动作仍算成功（默认）
                                # override=分支的最终结果作为本动作结果
    actions:
      - action_type: vision_detect
        params: { ... }
  # —— 失败分支：主动作失败后依次执行（补偿/恢复）——
  on_failure:
    branch_policy: "fail"       # fail=分支执行后本动作仍判失败（默认，任务终止）
                                # recover=分支全部成功则本动作转为成功，主序列继续
    actions:
      - action_type: move_joints
        params: { ... }
  params:
    ...
```

**分支语义汇总**：

| 场景 | 分支策略 | 分支执行成功 | 分支执行失败 |
|---|---|---|---|
| `on_success` | `continue`（默认） | 本动作 = 成功，主序列继续 | 本动作 = 失败，主序列终止 |
| `on_success` | `override` | 本动作 = 成功 | 本动作 = 失败 |
| `on_failure` | `fail`（默认） | 本动作 = 失败（已补偿），任务终止 | 本动作 = 失败，任务终止 |
| `on_failure` | `recover` | 本动作 = 成功，主序列继续 | 本动作 = 失败，任务终止 |

> 说明：
> - `on_success` 用于"成功后的附加动作"（如：检测成功后再拍一张点云存档）。
> - `on_failure` 用于"失败后的补偿/恢复动作"（如：吸盘失败后关闭吸气泵、夹爪失败后回安全位姿）。
> - 分支动作序列允许使用**全部现有动作类型**（`vision_detect` / `move_pose` / `move_joints` / `control_suck` / `gripper_ctrl` / 自检 / 复位等）。
> - 分支动作**允许递归**嵌套 `preconditions` / 结果分支（用户确认，见 9.6）：`on_success` / `on_failure` 内的动作同样用 `GuardedActionNode` 包装（见 6.3）。建议实现时增加**递归深度上限**（如 3~5 层）防止配置错误导致无限递归。

### 4.4 完整示例（片段）

基于现有 [`task_config.yaml`](../config/task_config.yaml:5) 的 `move_box` 任务改造示意：

```yaml
tasks:
  move_box:
    trigger_reg: 40001
    status_reg: 50001
    action_sequencenum: 1
    action_sequence1:
      - action_type: vision_detect
        delay_ms: 0
        preconditions:
          logical_op: "and"
          policy: "wait"
          max_wait_ms: 60000
          conditions:
            - { type: battery_level, operator: ">=", threshold: 20.0 }
            - { type: robot_status,  expected: ["IDLE"] }
            - { type: safety_status, expected: [0] }
            - { type: arm_idle, part: left, expected: true }
            - { type: head_idle, expected: true }
        params:
          target_obj_name: "lefta"
          expected_frame_id: "camera_link"
          need_6d_pose: true
          need_env_point_cloud: true
          downsample_res: 0.02
          type: 0.02
          priority: 1
        feedback_check: { ... }   # 原有反馈条件重试保持不变
        on_failure:
          branch_policy: "fail"
          actions:
            - action_type: control_suck
              params: { suck_id: 0, active: [false, false], timeout_ms: 5000 }

      - action_type: control_suck
        delay_ms: 500
        preconditions:
          conditions:
            - { type: arm_idle, part: left, expected: true }
        on_failure:
          branch_policy: "recover"
          actions:
            - action_type: move_joints
              params: { arm_id: 0, joint_name: [...], position: [...], speed: 0.3, timeout_ms: 30000 }
        params:
          suck_id: 0
          active: [true, true]
          timeout_ms: 5000
```

---

## 5. 条件检测机制设计

### 5.1 数据源方案

#### 5.1.1 各部件忙闲状态（task_manager 内部维护）

按用户确认（2026-08-26）：**各部件忙闲由 task_manager 内部维护，不依赖远端接口反馈**。

- 忙闲的权威来源 = `TaskActionClient` 内部的本地资源锁（[`task_action_client.hpp`](../include/task_manager/task_action_client.hpp:102)）。
- 条件评估时直接查询 `is_resource_busy()`：占用即"忙"、空闲即"闲"。
- **不新增** `/Control/device_busy_state` 话题，**不依赖** motion_control 发布忙闲标志。
- **资源-接口捆绑关系**（用户确认，见 9.2）：
  - 手臂 / 腿 / 头 / 灵巧手 ↔ **MovePose / MoveJoints**（按 `arm_id` / 关节范围映射 `LEFT_ARM / RIGHT_ARM / LEG / HEAD / WAIST`；`hand_action_type` 非空时追加 `LEFT_HAND / RIGHT_HAND`）；
  - 吸盘 ↔ **ControlSuck**（`LEFT_SUCK / RIGHT_SUCK`）；
  - 夹爪 ↔ **ControlGripper**（`LEFT_GRIPPER / RIGHT_GRIPPER`）。
- 腰部：关节 16-18 的动作由 task_manager 的 `move_joints` 派发（用户确认，见 9.4），映射 `WAIST` 资源锁。
- 前提与边界：
  - 成立前提：所有部件动作均由 task_manager 统一派发（上述接口均经 `TaskActionClient` 加锁），因此本地资源锁能如实反映"当前是否被 task_manager 占用"。
  - **安全接管边界**（用户确认，见 9.3）：机器异常时安全节点会接管部分控制逻辑，但**接管前会发布安全状态异常 topic**（`/Safety/device_safety_state`），task_manager 收到安全异常后**取消当前任务**并释放资源锁，因此异常接管不会产生 task_manager 不可见的持续占用。
  - 局限性：若存在绕过 task_manager 的独立客户端直接驱动某部件，task_manager 无法感知其占用。后续如需覆盖该场景，可在资源锁机制之上再叠加远端忙闲通道（可选扩展，非本方案必须）。
- 需把 [`ResourceType`](../include/task_manager/task_action_client.hpp:102) 扩展为 `VISION / LEFT_ARM / RIGHT_ARM / LEG / LEFT_GRIPPER / RIGHT_GRIPPER / LEFT_SUCK / RIGHT_SUCK / HEAD / WAIST / LEFT_HAND / RIGHT_HAND`，并在 `get_required_resources()` 中把动作正确映射到对应资源（见 5.4）。
- 条件清单按用户原意保持 **10 类**；`leg_idle`（腿空闲）未列入条件清单，但腿已与 MovePose/MoveJoints 捆绑加锁，如需可扩展 `leg_idle` 条件（见 9.2）。

#### 5.1.2 电池电量

- 数据来源（用户确认，见 9.1）：**安全节点设备状态 topic** —— 扩展 [`SafetyState.msg`](../com_interfaces/msg/SafetyState.msg:1)，增加电池字段（如 `float32 battery_percentage`、`float32 battery_voltage`），由安全监控节点在 `/Safety/device_safety_state` 上统一发布；task_manager 已订阅该话题，直接缓存提取，**无需新增订阅**。
- 条件检测读取 `battery_percentage` 或 `battery_voltage`（可通过 `battery_level` 条件的 `field` 子字段指定，默认 `percentage`）。
- 优点：安全节点作为整机健康信息（安全状态 + 机器人状态 + 电池电量）的统一出口，task_manager 数据源单一。

#### 5.1.3 机器人状态 / 安全状态

- `robot_status`：**以安全监控节点为准**（用户确认，见 9.5）——采用安全节点 [`SystemFsmState`](../safety_monitor/include/safety_monitor/safety_monitor_node.hpp:23) 枚举口径（UNKNOWN=0…IDLE=5 RUN=6 ABNORMAL=7 EMERGENCY=8）；数据来自已订阅的 `/Safety/device_safety_state`（或周期调用 `/Safety/get_system_status` 服务刷新缓存）。**不再以 task_manager 本地 `current_state_` 为准**。
- `safety_status`：复用已订阅的 `/Safety/device_safety_state`，缓存 `msg->status`（0=正常 / 1=功能受限 / 2=急停）。

### 5.2 StateCache（条件数据缓存）组件

新增 `task_manager` 内部组件 `StateCache`（或 `PreconditionManager`）：

```cpp
class PreconditionManager {
 public:
  void init(rclcpp::Node::SharedPtr node);
  // 订阅缓存（单一来源：/Safety/device_safety_state，含 safety.status / battery.* / 安全节点状态）
  void safety_state_cb(const SafetyStateMsg::SharedPtr msg);
  // robot_status 以安全监控节点 SystemFsmState 为准（见 9.5），不再由本地 FSM 同步
  // 忙闲直接读取 TaskActionClient::is_resource_busy()，无需订阅

  // 核心：评估一组前置条件（满足与否、失败原因）
  PrecheckResult evaluate(const task_cfg::PreconditionList& pre,
                          const task_act::TaskActionClient& act_client) const;
  // 单个条件评估
  bool evaluate_one(const task_cfg::PreconditionParam& p,
                    std::string& fail_reason) const;
};
```

- **部件空闲的评估**：直接以本地资源锁为准——`is_resource_busy(resource) == true` 即视为"忙"（仅当 `expected: false` 时通过）；空闲即"闲"（`expected: true` 时通过）。忙闲状态由 task_manager 内部维护，不依赖远端接口。
- 数据缺失保护：若订阅数据尚未收到（或话题未上线），视为"条件不满足"并记录原因，避免在无数据时误放行动作。

### 5.3 等待与超时策略

- `policy: wait`：`GuardedActionNode` 在 `GATE_CHECK` 阶段周期性（`check_interval_ms`）调用 `evaluate()`；满足则进入执行；超过 `max_wait_ms` 后按 `on_timeout` 处理（`fail` / `pass` / `abort`）。
- `policy: fail_fast`：一次评估不满足即判定本动作失败（进入失败分支或直接失败）。
- `policy: skip`：一次评估不满足即跳过本动作，本动作视为成功继续主序列。
- `abort`（超时后）：使整个任务树返回 FAILURE，并在 task_manager 的 FSM 上记录 `ERR_PRECONDITION_FAIL`（新增错误位）及日志。

### 5.4 与现有资源锁的融合

- 扩展 [`ResourceType`](../include/task_manager/task_action_client.hpp:102)：新增 `LEG`、`HEAD`、`LEFT_HAND`、`RIGHT_HAND`（**腰部统一并入 `LEG`**，不单列 `WAIST`，见 9.4 修订）。
- 扩展 [`get_required_resources()`](../src/task_action_client.cpp:1890)，按**资源-接口捆绑关系**映射（用户确认，见 9.2）：
  - `MovePose` / `MoveJoints` → 按 `arm_id` 映射 `LEFT_ARM / RIGHT_ARM / LEG / HEAD`（腰部关节 16-18 由 `move_joints` 以 `arm_id=LEG` 派发，统一计入 `LEG`，见 9.4 修订）；
  - `MovePose.hand_action_type` 非空 → 追加 `LEFT_HAND / RIGHT_HAND`；
  - `ControlSuck` → `LEFT_SUCK / RIGHT_SUCK`；
  - `ControlGripper` → `LEFT_GRIPPER / RIGHT_GRIPPER`。
- **这是忙闲条件的核心数据来源**：`is_resource_busy()` 既用于前置条件评估（检测），也是执行期加锁的依据（执行），二者天然一致。
- 这样"条件检测"与"执行期加锁"一致：条件要求空闲 → 检测通过 → 执行时自动加锁 → 完成释放，保证同一部件不会被并发动作重复占用。

---

## 6. 结果分支执行机制设计

### 6.1 新增控制流节点 `GuardedActionNode`

在 [`behavior_tree_engine.hpp`](../include/task_manager/behavior_tree_engine.hpp:40) 中新增一个控制流节点，负责：前置条件门 + 主动作 + 结果分支。

```cpp
class GuardedActionNode : public BTNode {
 public:
  // 内部阶段状态机
  enum class Phase { GATE_CHECK, EXECUTE, ON_SUCCESS, ON_FAILURE, DONE };

  void setPreconditionGate(BTNode::Ptr gate);   // 条件门（内部可含等待循环）
  void setAction(BTNode::Ptr action);           // 主动作节点（复用 build_action_node）
  void setOnSuccessBranch(BTNode::Ptr branch);  // 成功分支序列（可选）
  void setOnFailureBranch(BTNode::Ptr branch);  // 失败分支序列（可选）
  void setBranchPolicy(ResultPolicy on_success_policy,
                       ResultPolicy on_failure_policy);
  BTStatus tick() override;
  void reset() override;
 private:
  Phase phase_{Phase::GATE_CHECK};
  BTStatus action_result_{BTStatus::IDLE};
  ...
};
```

### 6.2 状态机流程

```
                 ┌──────────────────────────────┐
   GATE_CHECK ──►│ 评估 preconditions（可等待）   │
                 │ 满足 → EXECUTE                │
                 │ fail_fast/skip/超时 → 结算     │
                 └──────────────────────────────┘
                              │
                              ▼
                 ┌──────────────────────────────┐
      EXECUTE ──►│ 执行主动作（ActionNode）       │
                 │ SUCCESS → ON_SUCCESS          │
                 │ FAILURE → ON_FAILURE          │
                 └──────────────────────────────┘
                  /                 \
                 ▼                   ▼
   ┌────────────────────┐   ┌────────────────────┐
   │ ON_SUCCESS 分支     │   │ ON_FAILURE 分支     │
   │（可选，依次执行）     │   │（可选，依次执行）     │
   └────────────────────┘   └────────────────────┘
                 \                 /
                  ▼               ▼
                 ┌──────────────────────────────┐
       DONE ────►│ 按 branch_policy 结算并返回    │
                 │ SUCCESS / FAILURE             │
                 └──────────────────────────────┘
```

### 6.3 构建期集成

- 在 [`load_action_group()`](../src/behavior_tree_engine.cpp:953) 与 [`load_parallel_task()`](../src/behavior_tree_engine.cpp:995) 中：
  1. 若动作同时配置了 `preconditions` / `on_success` / `on_failure`，则用 `GuardedActionNode` 包装；
  2. 否则保持现状（直接 `ActionNode` / `feedback_check` 包装），**完全向后兼容**。
- `PreconditionManager` 通过构建参数注入 `BehaviorTreeEngine`，供条件门回调使用。
- 分支动作序列同样用 `GuardedActionNode` 包装（**递归支持** `preconditions` 与结果分支，用户确认，见 9.6），复用现有 `build_action_node()` 构建叶子（使用独立的 `state_key`，如 `seqN_<act>_<i>_success` / `_failure`）；实现时增加**递归深度上限**（如 3~5 层）防止配置错误导致无限递归。

### 6.4 分支与任务级状态机的衔接

- 分支动作执行中同样受 `TaskActionClient` 的资源锁保护。
- `on_failure` 分支执行完成后：
  - `branch_policy: fail` → 本节点返回 FAILURE → 任务树终止 → task_manager 写 `status_reg = TASK_STATUS_FAILED`（[`bt_tick_cb()`](../src/task_manager_node.cpp:544) 现有逻辑）。
  - `branch_policy: recover` → 本节点返回 SUCCESS → 主序列继续。
- 条件不满足或补偿分支执行后，**不需要通过 `status_reg` 向 PLC 表达**（如特殊状态码），仅以 ROS 日志记录即可（用户确认，见 9.7）。

---

## 7. 代码改造点清单

### 7.1 `com_interfaces`

| 文件 | 改动 |
|---|---|
| [`SafetyState.msg`](../com_interfaces/msg/SafetyState.msg:1) | **增加电池字段**（如 `float32 battery_percentage`、`float32 battery_voltage`），供安全节点在 `/Safety/device_safety_state` 上附带发布（用户确认电池经安全节点设备状态 topic 提供，见 9.1） |
| （无必改项） | 忙闲状态由 task_manager 内部维护，**无需新增** `DeviceBusyState.msg` |

### 7.2 `motion_control_pkg`

| 文件 | 改动 |
|---|---|
| （一般无必改项） | 忙闲判定不再依赖 motion_control 发布；各模块内部 `is_idle()` 标志仅作调试参考，不参与条件评估 |
| （无必改项） | 电池数据改由安全节点在设备状态 topic 上提供，motion_control **无需**转发 `/Power/battery_state` |

### 7.3 `task_manager`

| 文件 | 改动 |
|---|---|
| `task_config_parser.hpp/.cpp` | **分级化**：新增 `load_main_config()` / `load_task_config()` / `load_all()`，主配置解析 Modbus 参数与 reg→`config_file` 映射、次级文件解析动作列表并合并为 `GlobalConfig`；同时新增 `PreconditionParam`、`PreconditionList`、`ResultBranchParam`、`ResultActionParam`，`AtomicAction` 增加 `preconditions` / `on_success` / `on_failure`，`parse_action()` 解析新字段 |
| `task_config_parser.hpp` | `GlobalConfig` 增加 `modbus_slave_id`；`TaskMapItem` 增加 `config_file`（来源记录，便于日志） |
| `modbus_slave.hpp/.cpp` | 构造函数增加 `slave_id` 参数，`modbus_tcp_new` 后调用 `modbus_set_slave(ctx_, slave_id)`（现硬编码 1，见 [`modbus_slave.cpp`](../src/modbus_slave.cpp:40)） |
| 新增 `precondition_manager.hpp/.cpp` | 状态缓存 + 条件评估（见 5.2）；订阅 `/Safety/device_safety_state`（提取 safety.status + battery.* + 安全节点状态）；忙闲条件直接查询 `TaskActionClient::is_resource_busy()` |
| `task_action_client.hpp/.cpp` | `ResourceType` 扩展 LEG/HEAD/WAIST/LEFT_HAND/RIGHT_HAND；`get_required_resources()` 按资源-接口捆绑关系扩展（MovePose/MoveJoints→ARM/LEG/HEAD/WAIST/HAND、ControlSuck→SUCK、ControlGripper→GRIPPER） |
| `behavior_tree_engine.hpp/.cpp` | 新增 `GuardedActionNode`；`load_action_group/load_parallel_task` 增加包装逻辑；构建时注入 `PreconditionManager` |
| `task_manager_node.hpp/.cpp` | 初始化 `PreconditionManager`（订阅 `/Safety/device_safety_state`，提取 safety.status / battery.* / 安全节点状态）；新增错误位 `ERR_PRECONDITION_FAIL`；构建 BT 时传入条件管理器；`config_path` 指向**主配置文件**，启动时仅加载主配置并级联加载次级配置（首版一次性加载，不做热重载，见 9.11） |

---

## 8. 兼容性与迁移策略

1. **配置向后兼容**：`preconditions` / `on_success` / `on_failure` 全部为可选字段。未配置的动作行为与现状完全一致，现有 [`task_config.yaml`](../config/task_config.yaml) **无需改动**即可继续运行。
2. **分级化向后兼容**：主配置任务条目支持"外链（`config_file`）"与"内联（`action_sequence*`）"双格式，旧单文件配置无需改动即可继续运行；迁移到分级结构时仅需将各任务动作列表抽到次级文件，并在主配置中填 `config_file` 与 `status_reg`。`config_file` **相对主配置文件所在目录解析**（用户确认，见 9.10），无需统一 `config_dir` 前缀。
3. **枚举扩展兼容**：`ResourceType` 新增枚举值不影响既有逻辑；不新增话题/接口，对 motion_control / com_interfaces 无侵入。
4. **分步落地**（建议实施顺序）：
   - 第一步：配置文件分级化（主 / 次级解析、`ModbusTcpSlave` 增加 `slave_id`、`GlobalConfig` / `TaskMapItem` 结构扩展）。
   - 第二步：task_manager 扩展 `ResourceType`（LEG/HEAD/WAIST/LEFT_HAND/RIGHT_HAND）与 `get_required_resources()`（资源-接口捆绑映射）。
   - 第三步：配置解析扩展 `preconditions` / `on_success` / `on_failure`。
   - 第四步：新增 `PreconditionManager`（订阅 `/Safety/device_safety_state` 提取 safety/battery/安全节点状态、查询本地资源锁）。
   - 第五步：`GuardedActionNode` 与构建期包装（先只做前置条件门）。
   - 第六步：结果分支（`on_success` / `on_failure`）。
   - 第七步：电池数据源落地（扩展 `SafetyState.msg` 电池字段，安全节点在 `/Safety/device_safety_state` 上发布）。
   - 第八步：逐任务在配置中启用并回归测试。

---

## 9. 确认结论（2026-08-26 用户答复）

下列 11 项原"待确认事项"已由用户逐条答复确认，结论已落实到上文各章节；本节作为权威记录留存。

| # | 待确认事项 | 用户答复 / 结论 | 落点 |
|---|---|---|---|
| 1 | 电池电量数据来源 | **通过监听安全节点的设备状态 topic 发出**：扩展 `SafetyState.msg` 电池字段，安全节点在 `/Safety/device_safety_state` 上发布，task_manager 复用已订阅话题直接缓存 | 5.1.2 / 7.1 |
| 2 | 资源忙闲判定与接口捆绑 | 全部由 task_manager 内部维护；**手臂 / 腿 / 头 / 灵巧手 ↔ MovePose / MoveJoints**、**吸盘 ↔ ControlSuck**、**夹爪 ↔ ControlGripper** | 5.1.1 / 5.4 |
| 3 | 本地资源锁覆盖前提 | 所有部件动作均由 task_manager 统一派发；机器异常时安全节点接管，但**接管前发布安全状态异常 topic，task_manager 收到后取消当前任务**，无不可见的持续占用 | 5.1.1 边界 |
| 4 | 腰部忙闲判定 | 腰部动作（**关节 16-18**）由 task_manager 的 `move_joints` 派发，映射 `LEG` 资源锁（**修订：腰部与腿统一用 `LEG` 表示，不单列 `WAIST`**） | 5.4 / 2.4 |
| 5 | `robot_status` 状态口径 | **以安全监控节点为准**（`SystemFsmState` 口径），来自 `/Safety/device_safety_state` 或 `/Safety/get_system_status` | 5.1.3 / 4.2 |
| 6 | 分支嵌套层级 | **允许递归**：`on_success` / `on_failure` 内的动作可再配置 `preconditions` 与结果分支 | 4.3 / 6.3 |
| 7 | PLC 感知 | 条件不满足或补偿分支执行后，**不需要**通过 `status_reg` 向 PLC 表达特殊状态码，仅 ROS 日志即可 | 6.4 |
| 8 | `on_success` 分支失败处理 | **默认按"整体失败"处理**（符合预期）；`branch_policy` 仍保留可选策略 | 4.3 分支语义表 |
| 9 | Modbus slave ID 语义 | `slave_id` 直接映射 `modbus_set_slave(ctx_, slave_id)`，**满足现有 PLC 协议约定** | 4.1.2 / 7.3 |
| 10 | 次级配置路径基准 | **相对主配置文件所在目录解析**，**不需要统一的 `config_dir` 前缀** | 4.1.2 / 4.1.4 |
| 11 | 分级后热重载 | **首版仅启动时一次性加载**，不做运行时热重载 | 4.1.4 / 7.3 |

仍建议实现评审确认的点（不影响上述结论落地）：
- 分支递归的**深度上限**（建议 3~5 层），防止配置错误导致无限递归（见 4.3 / 6.3）。
- **腿（LEG）空闲**未列入用户给定的 10 类条件清单，但腿已与 MovePose/MoveJoints 捆绑加锁；如需可扩展 `leg_idle` 条件（见 5.1.1 / 5.4）。
