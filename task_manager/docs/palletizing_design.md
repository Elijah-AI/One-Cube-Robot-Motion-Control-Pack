# 拆堆码垛（Depalletizing / Palletizing）方案设计

> 版本：v1.0
> 状态：方案设计（暂不修改代码）
> 适用：基于 `task_manager` 现有架构，实现"拆垛取箱 → 码垛放箱"的循环搬运作业。

---

## 1. 背景与目标

### 1.1 业务需求
在物流/仓储场景中，机器人需要：
- **拆堆（Depalletize）**：从原料垛上逐层、逐箱取下纸箱/料箱，放到输送线或转运托盘；
- **码垛（Palletize）**：从输送线/供料位抓取箱子，按层规则码放到目标垛上；
- **倒垛（可选的组合场景）**：从 A 垛拆下，直接码到 B 垛（垛位转移）。

### 1.2 设计目标
1. **最大化复用现有机制**：以 [`move_box.yaml`](task_manager/config/tasks/move_box.yaml:1) 的"视觉→抓取→放置"链路为基座，不重写架构。
2. **提供两条落地路线**：
   - **路线 A（零代码）**：纯 YAML 配置 + PLC 循环触发，可立即验证流程；
   - **路线 B（推荐，需少量扩展）**：新增循环/偏移/计数能力，一次触发完成整垛，配置简洁。
3. **明确边界**：方案只描述 `task_manager` 编排层；视觉算法、运控 `MovePose` 服务端能力视为外部依赖，方案会列出对其的输入要求。

---

## 2. 现有系统能力盘点（可复用点）

| 能力 | 位置 | 说明 |
|------|------|------|
| 任务触发 | [`task_config.yaml`](task_manager/config/task_config.yaml:73) | PLC 写 Modbus 触发寄存器 → 执行任务 → 回写状态寄存器（1 执行中/2 成功/3 失败） |
| 串行/并行序列 | [`task_config.yaml`](task_manager/config/task_config.yaml:321) | `action_sequencenum` + `action_sequence1..N`，并行分支独立 `state_key` |
| 视觉→运动联动 | [`task_action_client.cpp`](task_manager/src/task_action_client.cpp:1451) | `vision_detect` 结果存入序列级 key，同序列 `move_pose` 自动取 `detected_targets[0].pose` 为目标位姿 |
| 反馈条件重试 | [`behavior_tree_engine.cpp`](task_manager/src/behavior_tree_engine.cpp:1287) | `feedback_check`：探测→校验反馈字段→不满足执行 `retry_actions` 后重试，`on_exhaust: fail/pass` |
| 结果分支 | [`task_config.yaml`](task_manager/config/task_config.yaml:50) | `on_success`（continue/override）、`on_failure`（fail/recover），支持递归 |
| 前置条件 | [`task_config.yaml`](task_manager/config/task_config.yaml:17) | `preconditions`：电量/整机状态/安全状态/部件空闲，`wait/fail_fast/skip` 策略 |
| 资源锁 | [`task_action_client.hpp`](task_manager/include/task_manager/task_action_client.hpp:103) | VISION/LEFT_ARM/RIGHT_ARM/LEG/HEAD/夹爪/吸盘/灵巧手，原子量互斥 |
| 抓取工具 | [`ControlSuck.action`](com_interfaces/action/ControlSuck.action:3)、[`ControlGripper.action`](com_interfaces/action/ControlGripper.action:3) | 吸盘（`control_suck`）、夹爪（`gripper_ctrl`）、灵巧手（`MovePose.hand_action_type`，运控端未实现） |
| 力控 | [`MovePose.action`](com_interfaces/action/MovePose.action:27) | `use_force_control` 到位力反馈，可用于贴合确认 |

### 2.1 关键缺口（拆堆码垛必须补足）
| 缺口 | 现状 | 影响 |
|------|------|------|
| **无循环节点** | 行为树仅 Sequence/Parallel/Fallback/Retry（[`behavior_tree_engine.hpp`](task_manager/include/task_manager/behavior_tree_engine.hpp:34)） | 无法"一次触发搬完整垛"，需静态展开或新增循环 |
| **无位姿偏移** | `move_pose` 目标 = 固定 `config_pose` 或完整视觉位姿 | 无法表达"视觉抓取位 + Z 抬升/下降"两点 |
| **仅取首个目标** | 只使用 `detected_targets[0]` | 一层多箱无法逐个搬 |
| **无层/箱计数** | 无任务级变量 | 无法按层计算放置位/判定完成 |
| **"无目标"语义未定义** | 检测服务返回 `success=true` 但 `detected_targets` 可能为空 | 无法区分"拆堆完成"与"真故障" |

---

## 3. 业务场景与术语

### 3.1 术语
- **搬运单元（Palletizing Unit）**：一个箱子完整的"检测→接近→抓取→抬升→移动→放置→释放→返回"动作序列，是拆堆/码垛的最小可循环块。
- **垛（Pallet/Stack）**：多层、每层若干箱的箱体集合。
- **层（Layer）**：垛的某一层，高度 = 层索引 × 单箱高（或托板+箱高）。
- **抓取位/放置位**：吸盘/夹爪与箱体贴合时的末端位姿。

### 3.2 场景矩阵
| 场景 | 抓取来源 | 放置目标 | 方向 |
|------|----------|----------|------|
| 拆堆 | 原料垛（视觉动态定位） | 输送线/转运位（固定或视觉） | 取 |
| 码垛 | 供料位（固定或视觉） | 目标垛（固定基准+层偏移） | 放 |
| 倒垛 | A 垛（视觉） | B 垛（基准+层偏移） | 取+放 |

> 本文以**拆堆为主场景**详述，码垛是其镜像（交换"抓取/放置"两端即可）。

---

## 4. 总体方案设计

### 4.1 搬运单元 SOP（单箱动作序列，拆堆为例）

```
┌──────────────────────────────────────────────────────────────────┐
│  搬运单元（拆堆：原料垛 → 输送线）                                │
│  ① vision_detect       检测垛顶目标箱（type=0），结果入 seq_key   │
│  ② feedback_check      校验 total_detected_count>=1 && 置信度    │
│                        不满足→ retry_actions 微调重测，耗尽→完成判定│
│  ③ move_pose(接近)     A点：视觉位姿 + Z 抬升（预接近上方）        │
│  ④ move_pose(下降)     B点：下降贴合箱面（低速，可开力控确认）     │
│  ⑤ control_suck(吸)    开启吸盘吸附                                │
│  ⑥ move_pose(抬升)     B→A：吸附后提起至安全高度                  │
│  ⑦ move_pose(转运)     A→C：移动到放置位上方（固定/视觉）          │
│  ⑧ move_pose(放置)     下降到放置位                                │
│  ⑨ control_suck(放)    关闭吸盘释放                                │
│  ⑩ move_pose(返回)     回安全位/初始位，等待下一箱                │
└──────────────────────────────────────────────────────────────────┘
```

**要点**：
- ①—④ 依赖视觉动态位姿；⑥—⑩ 依赖固定配置位姿（或视觉 `type=1` 放置位检测）。
- 若运控端支持 `MovePose` 相对偏移（路线 B），③④⑥⑧ 可由一个视觉位姿 + 偏移表达，大幅简化 YAML。

### 4.2 视觉定位方案

#### 4.2.1 目标箱位姿（type=0）
- 输入要求（视觉侧）：`DetectionPoseCloud` 返回 `detected_targets[]`，其中 `pose` 为**箱体抓取基准点**（须与运控/吸盘中心对齐，建议为箱体上表面中心），附带 `size`（长宽高）与 `confidence`。
- 语义约定（需与视觉团队确认）：抓取基准点是"上表面中心"还是"几何中心"决定 ③④ 的偏移方向与量。
- 排序策略：当一层多个箱时，建议视觉侧按"抓取优先级"排序（如最高/最易抓），当前 `task_manager` 取 `detected_targets[0]`，路线 B 可扩展索引策略。

#### 4.2.2 放置位（type=1 / 固定基准）
- 供料/输送位固定：用 `config_pose` 写死。
- 目标垛：用"垛基准坐标 + 层偏移"（见 4.4），偏移可由 YAML 或 PLC 传入。

#### 4.2.3 拆堆完成判定
- 判定依据：视觉检测 `total_detected_count == 0` 且重试仍为 0 → 判定"无剩余箱"。
- 必须将"完成"与"故障"区分（见第 7 章异常处理）。

### 4.3 抓取与力控方案

| 工具 | 动作 | 适用 | 到位确认 |
|------|------|------|----------|
| 吸盘（默认，复用 move_box 模式） | `control_suck`（[`ControlSuck.action`](com_interfaces/action/ControlSuck.action:3)） | 纸箱/平面硬箱 | `use_force_control=true` 到位力反馈 + 吸气后抬升前可加静默校验 |
| 夹爪 | `gripper_ctrl`（[`ControlGripper.action`](com_interfaces/action/ControlGripper.action:3)） | 托盘/重物/异形箱 | 夹持到位后依据 `final_position/final_torque` 校验 |
| 灵巧手 | `MovePose.hand_action_type`（[`MovePose.action`](com_interfaces/action/MovePose.action:30)） | 柔性/异形件 | 运控端当前**未实现**，仅预留 |

> 建议主方案采用**吸盘**（与现有 [`move_box.yaml`](task_manager/config/tasks/move_box.yaml:134) 一致，成熟可用）；夹爪作为重载备选。

### 4.4 层/箱管理与放置位计算

#### 路线 A（零代码，PLC 侧管理）
- **每触发一次 = 搬一箱**。PLC 负责循环触发与计数。
- 放置位逐层偏移：在 YAML 中为每一层写一组放置位（层高 = 层号 × 箱高），PLC 按层号触发对应"码垛第 N 层"任务；或放置位恒为基准位，PLC 通过寄存器传偏移（需 `task_manager` 支持寄存器→位姿映射，属扩展）。
- 拆堆完成判定：PLC 轮询"最后一箱任务失败/完成"信号，或视觉"无箱"上报。

#### 路线 B（task_manager 内部管理，推荐）
- 新增**任务级循环**：`repeat` 动作组，支持终止条件：
  - `max_count`：最多搬运箱数；
  - `until_no_target`：视觉无目标即成功结束（拆堆完成）；
  - `layer_height` 步进：每箱/每层后更新 Z 基准。
- 新增**放置位生成**：`place_base + layer*[0,0,箱高]`，支持 `config_pose` 或视觉放置位。
- 新增**目标索引策略**：`index` 固定、`top_most` 最高优先、`sequential` 按视觉顺序。

---

## 5. 任务配置示例

### 5.1 路线 A：单箱拆堆任务（零代码，每触发一次搬一箱）

```yaml
# 主配置 task_config.yaml 中注册：
#   depalletize_box:
#     trigger_reg: 40010
#     status_reg: 50010
#     config_file: "tasks/depalletize_box.yaml"

# tasks/depalletize_box.yaml
action_sequencenum: 1
action_sequence1:
  # ① 视觉检测 + ② 完成判定（无目标连续重试后 pass → 任务成功结束，PLC 判定拆堆完成）
  - action_type: vision_detect
    delay_ms: 0
    feedback_check:
      enable: true
      check_field: "total_detected_count"
      operator: ">="
      threshold: 1
      max_retries: 2
      on_exhaust: "pass"            # 无箱 → 通过 → 后置 move_pose 需配合跳过
      retry_actions:
        - action_type: move_pose    # 微调相机视角重测（抬升/移动）
          params:
            arm_id: 3               # HEAD
            use_config_pose: true
            target_pose: [500.0, 0.0, 820.0, 0.0, 0.0, 0.0, 1.0]
            bound_size: [0.0, 0.0, 0.0]
            speed: 0.2
            timeout_ms: 5000
  # ③ 预接近（自动使用视觉位姿 detected_targets[0].pose）
  - action_type: move_pose
    params:
      arm_id: 0                     # ARM_LEFT
      use_config_pose: false        # 关键：关闭固定位姿，采用视觉目标
      bound_size: [0.0, 0.0, 0.15]
      enable_collision_check: true
      speed: 0.3
      timeout_ms: 5000
  # ④ 下降贴合（低速 + 力控确认）
  - action_type: move_pose
    params:
      arm_id: 0
      use_config_pose: false
      use_force_control: true
      bound_size: [0.0, 0.0, 0.15]
      enable_collision_check: true
      speed: 0.1
      timeout_ms: 8000
  # ⑤ 吸气
  - action_type: control_suck
    params:
      suck_id: 0                    # SUCK_LEFT
      active: [true]
      timeout_ms: 5000
  # ⑥ 抬升
  - action_type: move_pose
    params:
      arm_id: 0
      use_config_pose: false
      bound_size: [0.0, 0.0, 0.15]
      enable_collision_check: true
      speed: 0.2
      timeout_ms: 5000
  # ⑦⑧ 转运 + 放置（固定配置位）
  - action_type: move_pose
    params:
      arm_id: 0
      use_config_pose: true
      target_pose: [50.0, 0.0, 700.0, -0.895291, -0.164696, 0.407321, -0.0736177]
      bound_size: [0.0, 0.0, 0.15]
      enable_collision_check: true
      speed: 0.3
      timeout_ms: 5000
  # ⑨ 释放
  - action_type: control_suck
    params:
      suck_id: 0
      active: [false]
      timeout_ms: 5000
  # ⑩ 返回安全位（on_failure: recover 回到此位，保证失败可恢复）
  - action_type: move_pose
    params:
      arm_id: 0
      use_config_pose: true
      target_pose: [500.0, 0.0, 820.0, -0.895291, -0.164696, 0.407321, -0.0736177]
      bound_size: [0.0, 0.0, 0.1]
      enable_collision_check: false
      speed: 0.3
      timeout_ms: 5000
```

**零代码方案注意事项**：
1. `on_exhaust: "pass"` 后，后续 `move_pose` 在 `detected_targets` 为空时会**回退到 config_pose 或无效位姿**（见 [`task_action_client.cpp`](task_manager/src/task_action_client.cpp:1460)）——存在误动作风险。**建议**：无箱场景由 PLC 用独立寄存器触发"拆堆收尾任务"（仅回安全位 + 上报完成），或改 `on_exhaust: "fail"` 让 PLC 把状态 3 解释为"拆堆完成"。二选一，须与 PLC 约定。
2. 每箱一个触发寄存器 → PLC 需维护箱计数与触发节奏；可用 `preconditions` 的 `arm_idle/suck_idle` 防止触发过快。

### 5.2 路线 B：一次触发拆完整垛（示意，需扩展能力后生效）

```yaml
# tasks/depalletize_stack.yaml（架构增强后）
action_sequencenum: 1
action_sequence1:
  - action_type: repeat                 # 【新增】循环搬运单元
    repeat:
      until: "no_target"                # 视觉无目标即结束（拆堆完成）
      max_count: 120                    # 安全上限
      body:
        - action_type: vision_detect
          params:
            target_obj_name: "box"
            expected_frame_id: "camera_link"
            type: 0
        - action_type: move_pose        # 接近：视觉位姿 + 偏移
          params:
            arm_id: 0
            pose_source: "vision"
            pose_offset: [0.0, 0.0, 0.3]   # 【新增】相对偏移
        - action_type: move_pose        # 下降
          params:
            arm_id: 0
            pose_source: "vision"
            pose_offset: [0.0, 0.0, 0.0]
            use_force_control: true
        - action_type: control_suck
          params: { suck_id: 0, active: [true], timeout_ms: 5000 }
        - action_type: move_pose        # 抬升
          params:
            arm_id: 0
            pose_source: "vision"
            pose_offset: [0.0, 0.0, 0.3]
        - action_type: move_pose        # 放置位 = 基准 + 层偏移
          params:
            arm_id: 0
            pose_source: "place_base"
            layer_offset: [0.0, 0.0, 1.0]   # 【新增】层高步进
        - action_type: control_suck
          params: { suck_id: 0, active: [false], timeout_ms: 5000 }
    on_failure:
      branch_policy: "recover"
      actions:
        - action_type: move_pose        # 失败恢复：关吸盘回安全位
          params:
            arm_id: 0
            use_config_pose: true
            target_pose: [500.0, 0.0, 820.0, 0.0, 0.0, 0.0, 1.0]
```

### 5.3 双臂并行码垛（扩展，action_sequence2 并行）

- 左臂负责取/放 A 侧，右臂负责 B 侧；两分支独立 `state_key`（[`architecture.md`](task_manager/docs/architecture.md:453)）。
- **资源锁约束**：左右分支各自占用 `LEFT_ARM+LEFT_SUCK` / `RIGHT_ARM+RIGHT_SUCK`；`VISION` 为共享资源，并行时两侧 `vision_detect` 会互斥（`FAIL_RESOURCE_BUSY` → 下个 tick 重试），需评估是否满足节拍；不满足则改为"左检测→右检测→左右并行搬运"。
- 双臂联动需左右手臂工作空间不重叠，且垛/输送线布局需支持双臂同时作业。

---

## 6. 资源锁与并发设计

| 场景 | 占用资源 | 说明 |
|------|----------|------|
| 单臂拆堆/码垛 | `VISION + LEFT_ARM + LEFT_SUCK`（或 `LEFT_GRIPPER`） | 每箱串行，无并发冲突 |
| 双臂并行 | 分支1：`LEFT_ARM+LEFT_SUCK`；分支2：`RIGHT_ARM+RIGHT_SUCK`；`VISION` 交替共享 | `VISION` 互斥，检测阶段串行化 |
| 灵巧手（预留） | `MovePose.hand_action_type` 非空时追加 `LEFT_HAND/RIGHT_HAND`（[`task_action_client.cpp`](task_manager/src/task_action_client.cpp:1929)） | 与手臂捆绑 |
| 吸盘确认 | `suck_idle` 前置条件（[`task_config.yaml`](task_manager/config/task_config.yaml:43)） | 防止上一箱未释放即下一箱吸气 |

**推荐**：
1. 首版采用**单臂 + 吸盘 + 串行**，资源锁天然安全；
2. 后续节拍不足时再评估双臂并行（先做工作空间/布局可行性验证）。

---

## 7. 异常处理与恢复策略

| 异常 | 检测方式 | 恢复策略 |
|------|----------|----------|
| 视觉检测不到目标（拆堆完成） | `feedback_check` 校验 `total_detected_count` | 路线A：`on_exhaust: pass/fail` + PLC 判定；路线B：`until: no_target` 正常结束 |
| 置信度过低/目标误识别 | `feedback_check` 校验 `confidence` | `retry_actions` 微调视角重测，耗尽后 `fail` 并上报 |
| 吸盘吸附失败 | 抬升前可加"位姿差"校验（需扩展） | `on_failure: recover` → 关吸盘 → 回安全位 → 重试该箱（计数不增） |
| 到位力控异常 | `use_force_control` 结果 + `timeout_ms` | `on_failure: recover` 回安全位，避免硬顶 |
| 急停/中止 | FSM ERROR + action 中止 | 立即取消活动 goal，任务状态 3，PLC 介入人工恢复 |
| 网络/服务超时 | `FAIL_SERVER_TIMEOUT` | 重试（现有 `wait_for_action_server` 已内置 3 次） |

**统一的失败出口**：每个搬运单元配置 `on_failure: recover`，保证失败后吸盘已释放、手臂回到安全位，不残留"抓取中"状态，避免任务重触发时与现场冲突。

---

## 8. 需要新增的架构能力（路线 B 代码扩展点，后续实施）

> 以下均为 `task_manager` 编排层的**增量扩展**，不改变现有动作的执行语义，仅新增配置字段与节点类型，完全向后兼容。

| # | 扩展点 | 建议改动位置 | 说明 |
|---|--------|--------------|------|
| 1 | **Repeat/Loop 节点** | [`behavior_tree_engine.hpp`](task_manager/include/task_manager/behavior_tree_engine.hpp:34) 新增控制节点 | 支持 `max_count` + `until: no_target` 终止条件，内部维护循环状态 |
| 2 | **位姿相对偏移** | [`MovePoseParam`](task_manager/include/task_manager/task_config_parser.hpp:38) 增加 `pose_source`/`pose_offset`；`task_action_client.cpp` 视觉位姿处叠加偏移 | 表达"视觉位 + Z 抬升/下降"两点 |
| 3 | **目标索引/排序策略** | `task_action_client.cpp` `detected_targets[0]` 处 | 支持 `index`/`top_most`/`sequential`，配合一层多箱 |
| 4 | **层/箱计数与放置位生成** | 新增任务级运行时变量表 | `place_base + layer*层高`；`until: no_target` 完成判定 |
| 5 | **"无目标"完成语义** | `behavior_tree_engine.cpp` 循环终止条件 | 与"故障失败"严格区分，状态寄存器可上报"完成/剩余0" |
| 6 | **吸附校验（可选）** | 抬升前后位姿对比或吸盘回读（[`ControlSuck.action`](com_interfaces/action/ControlSuck.action:13) 目前无状态回读） | 确认吸附成功，提升可靠性 |

**依赖外部侧的输入要求**：
- 视觉：`detected_targets[].pose` 的基准点语义、`size` 准确、`confidence` 可信、排序策略；
- 运控：`MovePose` 的 `use_force_control` 到位反馈可用；`hand_action_type` 灵巧手如启用需先实现（[`dual_arm_module.cpp`](motion_control_pkg/src/dual_arm_module.cpp:2353) 当前拒绝）。

---

## 9. 实施路线

| 阶段 | 内容 | 交付 | 改动量 |
|------|------|------|--------|
| **P0 流程验证（零代码）** | 用路线 A 单箱任务 + PLC 循环触发跑通"拆 1 箱→放 1 箱" | 动作模板 + 与 PLC 的寄存器协议 | 无代码，仅 YAML + 现场标定 |
| **P1 完成判定闭环** | 与 PLC 约定"无箱"判定（pass/fail/专用寄存器），跑完整垛 | 拆堆完成协议 | 无代码 |
| **P2 循环增强** | 实现扩展点 1/2（Loop + 偏移），一次触发整垛 | 循环任务 YAML | 小改：BT 引擎 + 解析器 |
| **P3 多箱/层管理** | 实现扩展点 3/4/5，一层多箱顺序搬运、层高步进 | 多箱任务 YAML | 中改：运行时变量 + 索引 |
| **P4 双臂/可靠性** | 双臂并行评估 + 吸附校验（扩展点 6） | 双臂模板 | 视评估结果 |

---

## 10. 风险与注意事项

1. **视觉位姿基准点语义**是首要风险：必须先与视觉团队对齐 `pose` 代表"上表面中心 / 几何中心 / 箱体角点"，否则 ③④ 偏移量错误会导致碰撞或吸附失败。
2. **垛位漂移**：整垛坐标可能随层数/搬运累积漂移，建议每层（或每 N 箱）重新视觉定位垛位，避免累积误差。
3. **"拆堆完成"误判**：单次无目标可能是遮挡/光照问题而非真的搬完，须用"多次重试 + 时间窗"降低误判；`retry_actions` 中可加入"相机抬升/移位重测"。
4. **吸盘吸附不可见**：`ControlSuck` 无状态回读（[`ControlSuck.action`](com_interfaces/action/ControlSuck.action:13) 注释明确未实现），吸附失败只能靠运动层间接推断，建议 P2+ 增加校验。
5. **双臂共享视觉**：`VISION` 资源互斥会串行化两臂的检测，节拍可能不达标；首版建议单臂。
6. **YAML 递归深度**：`repeat` 体内动作若再嵌套 `on_success/on_failure` 分支，受现有递归深度上限（5 层）约束，需注意配置层级。
