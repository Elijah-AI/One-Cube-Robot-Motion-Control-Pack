# 全身逆解函数 `solveWholeBodyIKOnce` 方法详解

> 对应源码：[`motion_control_pkg/src/miracle_m_algo.cpp`](../src/miracle_m_algo.cpp#L1375-L1901)

## 1. 概述

这是一个**基于速度级 QP 的迭代式全身逆运动学求解器**，核心思想是：在每一步用 pinocchio 计算 FK 和雅可比，把"末端位姿误差"转化为一个**加权最小二乘 + 不等式约束的 QP**，解出关节速度 `dq`，再积分一步。与 CLIK（Closed-Loop IK）思路一致，但用 QP 取代了阻尼最小二乘（DLS）的解析解，以处理自碰撞、关节限位等多约束场景。

## 2. 函数签名与职责

```cpp
bool MiracleMAlgo::solveWholeBodyIKOnce(
    const Eigen::VectorXd& q_init,
    const std::optional<Eigen::Isometry3d>& left_goal,
    const std::optional<Eigen::Isometry3d>& right_goal,
    Eigen::VectorXd& q_res,
    const std::array<double, 4>& leg_weights,
    double body_pitch_limit_rad,
    std::chrono::steady_clock::time_point deadline,
    bool enable_collision_avoidance)
```

| 参数 | 含义 |
|---|---|
| `q_init` | 种子构型（全模型 `nq` 维） |
| `left_goal/right_goal` | 可选的左右手目标位姿（至少一个） |
| `leg_weights` | 4 维腿部权重（按 project_memory 约定 `{1.0,1.0,1.0,0.2}`） |
| `deadline` | 单次求解超时点 |
| `enable_collision_avoidance` | 是否在 QP 里加自碰撞约束 |
| 返回 `true` | 仅当误差收敛 **且** 通过终点校验（自碰撞 + CCS 干涉） |

## 3. 流程分阶段

### 阶段 0 — 线程安全与输入校验（L1387-1402）

- 用 `std::recursive_mutex` 串行访问 pinocchio 的 `Data/GeometryData`，因为它们与 FK、碰撞回调共享非线程安全。
- 校验 `q_init` 维度、目标至少一个、目标矩阵有限。

### 阶段 1 — 构造活动关节集（L1404-1449）

- **活动关节** = 被请求手臂（每臂 7 DOF）+ 腿部/躯干 4 DOF。头部不参与。
- 用 `push_unique` 去重构造 `active_v`（速度索引列表）。
- 把 `idx_v` 映射到 `idx_q`（单自由度关节二者相等，这里仍显式查表以兼容球副等未来扩展）。
- 同时取该关节的位置上下限，后续做盒约束。

### 阶段 2 — 任务空间与权重（L1422-1474）

- 每个末端 6 维误差（位置 3 + 旋转向量 3），任务维 `n_task = 6*n_ee`。
- **姿态正则权重**：
  - 手臂 `arm_reg = 0.05`（自由运动）
  - 腿部按 `leg_weights`（大权重，约 1.0~20 倍于手臂，作用是"腿部尽量保持在 `q_init` 附近"——保证手臂可达性的同时躯干不乱晃）
- 单独取 `body_joint`（躯干俯仰）的 q 索引，供后续躯干限位（当前代码段被注释掉了）。

### 阶段 3 — 迭代参数（L1477-1497）

关键常量（与 project_memory 一致）：

| 常量 | 值 | 含义 |
|---|---|---|
| `eps` | `1.4e-4` | 6D 误差范数收敛阈值（约 0.1mm + 0.008°） |
| `IT_MAX` | `600` | 单次最大迭代（外层多种子叠加可达 1200） |
| `safety_margin` | `0.01 rad` | 关节限位安全余量 |
| `d_safe` | `0.04 m` | 自碰撞安全距离阈值 |
| `coll_gain` | `1.0` | 自碰撞约束增益 |

**自适应步长** `step_limit(err)`：

| 误差范数 | 步长上限 |
|---|---|
| `err < 0.01` | `0.01` |
| `err < 0.05` | `0.05` |
| 其他 | `0.30` |

原因：接近目标时二阶非线性占比大，大步长会过冲成极限环（mm 级震荡）。

`skew` lambda：反对称矩阵算子，用于最近点线速度雅可比推导。

### 阶段 4 — 主迭代循环（L1508-1869）

每步做 8 件事：

#### ① FK + 雅可比（L1516-1529）

- `computeJointJacobians` + `updateFramePlacements`
- 对每个末端：用 `log6(iMd)` 计算 6D 误差（`LOCAL` 帧表示，与雅可比一致），`getFrameJacobian(LOCAL)` 抽取活动列组成 `J_full`

#### ② 收敛判定（L1602）

- `err.norm() < eps` 即成功跳出。
- 注意：均匀分布下 `eps=1.4e-4` 对应位置约 0.1mm、姿态约 0.008°。

#### ③ 自碰撞约束构造（L1604-1667）

详见 [第 4 节：自碰撞约束构造的数学推导](#4-自碰撞约束构造的数学推导)。

#### ④ 6/7 轴组合干涉约束 CCS（L1669-1702）

- 约束式：`|j7| ≤ 110.5° - 1.025·|j6|`，仅 `|j6| > 20°` 激活
- 同样一阶线性化：slack 不足时限制 `dq` 不得进一步逼近边界
- 左右臂各加一行约束

#### ⑤ QP 目标函数（L1704-1709）

$$
\min \; \frac{1}{2} dq^\top H \, dq + dq^\top g
$$

$$
H = 2 \cdot (J^\top J + \text{diag}(W) + 10^{-9} \cdot I), \quad g = -2 \cdot J^\top e
$$

- 第一项 `JᵀJ`：任务空间追踪（最小化末端误差）
- 第二项 `diag(W)`：关节正则（手臂 0.05，腿部按 leg_weights）→ 阻尼最小二乘的"阻尼"角色
- `1e-9`：数值稳定，保证 H 正定
- 因子 2.0 是为了与标准 QP 形式 `0.5·xᵀHx + xᵀg` 对齐梯度 `-Jᵀe`。

#### ⑥ 盒约束（关节位置限位 → dq 边界）（L1711-1723）

- `lo = lower_pos + safety_margin - qj`，`up = upper_pos - safety_margin - qj`
- 再裁剪到 `[-step_now, +step_now]`，融合"位置限位"与"自适应步长"
- `safety_margin = 0.01rad` 留余量防止触底

#### ⑦ 解 QP（qpOASES）（L1739-1805）

- `setToFast()`：实时优先
- `nWSR=50`，`cputime=0.02s`：单步硬上限
- 接受 `SUCCESSFUL_RETURN` 或 `RET_MAX_NWSR_REACHED`
- **回退策略**：QP 不可行（约束冲突/奇异）时，退化为阻尼最小二乘解析解
  $$
  dq = J^\top (J J^\top + 10^{-4} I)^{-1} e
  $$
  再裁剪到盒约束。保证鲁棒性。

#### ⑧ 回溯线搜索（L1807-1852）

- 8 级 `α = 1, 0.5, ..., 1/128`
- 每级：积分 `q_next`、关节限位裁剪、自碰撞硬校验、CCS 硬校验
- 全部失败时：仍接受 `α = 1/256` 极小步前进（project_memory 提到的"8级+α=1/256兜底"）
- 目的：渡过腿部大角度旋转时的临时自碰撞

### 阶段 5 — 终点校验（L1873-1899）

收敛（误差达标）不等于解有效，还要做严格校验：

1. **自碰撞**：`isFullQValid(q)` 失败则置 `success=false`（仅在 `enable_collision_avoidance` 时）
2. **CCS 干涉**：左右臂独立校验，失败置 `success=false`

这两个校验独立于迭代内的"线性化近似"——后者是引导搜索的软约束，前者是最终判据。

## 4. 自碰撞约束构造的数学推导

> 对应源码：[`miracle_m_algo.cpp:1604-1667`](../src/miracle_m_algo.cpp#L1604-L1667)

### 4.1 问题陈述

给定当前位形 `q`，两个碰撞体 obj0（附着在关节 j0）、obj1（附着在关节 j1），它们的最近距离 `d(q)` 是关于 `q` 的非线性函数。QP 要求**线性**不等式约束，所以需要一阶线性化：

$$
d(q + \Delta q) \approx d(q) + \nabla_q d \cdot \Delta q \geq d_{safe}
$$

核心难点是求 $\nabla_q d$（距离对关节速度的梯度，1×n_v 行向量）。

### 4.2 距离函数的几何量

设最近点对：
- `p1(q)` ∈ obj0 表面，世界系坐标
- `p2(q)` ∈ obj1 表面，世界系坐标

距离与法向量（coal/hpp-fcl 约定 normal 由 o1 指向 o2）：

$$
d = \|p_2 - p_1\|, \quad n = \frac{p_2 - p_1}{d}
$$

代码 L1619-1627 对应：
```cpp
const Eigen::Vector3d& p1 = dr.nearest_points[0];
const Eigen::Vector3d& p2 = dr.nearest_points[1];
Eigen::Vector3d nrm = dr.normal;        // = (p2-p1)/d
```

### 4.3 距离变化率 ddot

对时间求导（$\dot p_i = v_{p_i}$ 是最近点线速度）：

$$
\dot d = \frac{d}{dt}\|p_2 - p_1\| = n \cdot (v_{p_2} - v_{p_1}) = n^\top (v_{p_2} - v_{p_1})
$$

推导：$\frac{d}{dt}\|r\| = \frac{r^\top \dot r}{\|r\|} = n^\top \dot r$，这里 $r = p_2 - p_1$。

### 4.4 最近点线速度 v_p

刚体上一点 P 的线速度（O 为刚体原点，即关节位置）：

$$
v_P = v_O + \omega \times (P - O) = v_O + \omega \times r, \quad r = P - O
$$

用雅可比表示（$\dot q$ 是关节速度）：
- $v_O = J_v \dot q$（线速度雅可比，3×n_v）
- $\omega = J_\omega \dot q$（角速度雅可比，3×n_v）

代入：

$$
v_P = J_v \dot q + (J_\omega \dot q) \times r = J_v \dot q - r \times (J_\omega \dot q) = \big(J_v - [r]_\times J_\omega\big) \dot q
$$

其中 $[r]_\times$ 是反对称矩阵（skew），满足 $[r]_\times \omega = r \times \omega$。

定义**最近点雅可比**：

$$
J_{p_i} = J_v^{(i)} - [r_i]_\times J_\omega^{(i)}, \quad r_i = p_i - o_i
$$

代码 L1639-1656 完全对应：
```cpp
Eigen::MatrixXd Jp0 = Eigen::MatrixXd::Zero(3, model_.nv);
Eigen::MatrixXd Jp1 = Eigen::MatrixXd::Zero(3, model_.nv);
if (j0 > 0 && j0 < model_.joints.size()) {
    J_joint.setZero();
    pinocchio::getJointJacobian(model_, *data_, j0, pinocchio::WORLD, J_joint);
    const Eigen::Vector3d o0 = data_->oMi[j0].translation();
    const Eigen::Vector3d r0 = p1 - o0;
    Jp0 = J_joint.topRows(3) - skew(r0) * J_joint.bottomRows(3);
}
```

关键点：
- `getJointJacobian(WORLD)` 返回世界系下的 6×n_v 雅可比，**topRows(3) = $J_v$（LINEAR），bottomRows(3) = $J_\omega$（ANGULAR）**（pinocchio 行序约定，注释 L1638 也写明了）
- `o0 = data_->oMi[j0].translation()`：关节 j0 原点在世界系的位置
- `r0 = p1 - o0`：从关节原点到最近点的向量
- `skew(r0) * J_joint.bottomRows(3)` 即 $[r_0]_\times J_\omega^{(0)}$

注意 `getJointJacobian` 给的是**关节原点**的雅可比，而最近点在关节上偏移了 $r$，所以要用叉积修正。

### 4.5 距离梯度 ∇_q d

把 $v_{p_i} = J_{p_i} \dot q$ 代入 $\dot d$：

$$
\dot d = n^\top (J_{p_1} - J_{p_0}) \dot q
$$

所以：

$$
\nabla_q d = n^\top (J_{p_1} - J_{p_0}) \quad (1 \times n_v)
$$

代码 L1658：
```cpp
const Eigen::RowVectorXd A_full = nrm.transpose() * (Jp1 - Jp0);
```

### 4.6 约束不等式

要求下一步 $d(q + \Delta q) \geq d_{safe}$：

$$
d(q) + \nabla_q d \cdot \Delta q \geq d_{safe}
$$

$$
\nabla_q d \cdot \Delta q \geq d_{safe} - d(q) = -(d(q) - d_{safe})
$$

定义 **slack 裕量** $\text{slack} = d(q) - d_{safe}$：
- $\text{slack} > 0$：当前有裕量，约束要求 $\Delta q$ 不要让距离减少太多
- $\text{slack} < 0$：当前已侵入安全距离，约束要求 $\Delta q$ 使距离增加（拉开）
- $\text{slack} = 0$：恰好在边界，$\Delta q$ 至少保持距离不变

约束化为：

$$
A \cdot \Delta q \geq -\text{slack}, \quad A = \nabla_q d
$$

加入增益 $k$（`coll_gain`）控制保守程度：

$$
A \cdot \Delta q \geq -k \cdot \text{slack} = -k \cdot (d(q) - d_{safe})
$$

代码 L1664-1665：
```cpp
lbA_rows.push_back(-coll_gain * (d - d_safe));  // 强制不越过安全距离
ubA_rows.push_back(std::numeric_limits<double>::infinity());
```

- `lbA = -coll_gain * (d - d_safe) = -coll_gain * slack`
- `ubA = +∞`（单向约束，只限制"不要进一步靠近"，不限制"远离"）

**关于 coll_gain 的物理意义**：$k=1$ 是线性化下的精确约束（下一步恰好到 $d_{safe}$）；$k>1$ 更保守，要求下一步距离比 $d_{safe}$ 更远（更快推开）；代码用 $k=1$，注释 L1488 说明"线性化下不越过安全距离"。

### 4.7 抽取活动关节列

`A_full` 是 1×n_v（全模型速度维），但 QP 变量 `dq` 是 n_active 维（仅手臂+腿部活动关节）。需要抽取活动列：

$$
A_{act} = A_{full}[:, \text{active\_v}]
$$

代码 L1659-1660：
```cpp
Eigen::RowVectorXd A_act(1, n_active);
for (int j = 0; j < n_active; ++j) A_act[j] = A_full[active_v[j]];
```

### 4.8 约束激活策略

代码 L1617：`if (d >= d_safe) continue;`

只有 $d < d_{safe}$ 的碰撞对才构造约束，距离足够远的对跳过。这是**动态 active set** 构造——每步迭代时激活集随位形变化，QP 规模精简，避免对几十上百个碰撞对都建约束。

### 4.9 完整数学总结

| 量 | 表达式 | 代码位置 |
|---|---|---|
| 距离 $d$ | $\|p_2 - p_1\|$ | L1616 `dr.min_distance` |
| 法向量 $n$ | $(p_2-p_1)/d$ | L1622 `dr.normal` |
| 关节原点 $o_i$ | `data_->oMi[j_i].translation()` | L1645, L1654 |
| 偏移 $r_i$ | $p_i - o_i$ | L1646, L1655 |
| 关节雅可比 $[J_v; J_\omega]$ | `getJointJacobian(WORLD)` | L1643, L1651 |
| 最近点雅可比 $J_{p_i}$ | $J_v - [r_i]_\times J_\omega$ | L1647, L1655 |
| 距离梯度 $A$ | $n^\top(J_{p_1} - J_{p_0})$ | L1658 |
| 约束下界 | $-k(d - d_{safe})$ | L1664 |
| 约束上界 | $+\infty$ | L1665 |

### 4.10 线性化的局限与补救

一阶 Taylor 展开只在 $q$ 附近有效，大步长会失真（线性化预测"安全"但真实碰撞已发生）。这就是为什么代码有两层补救：

1. **回溯线搜索**（L1807-1840）：每级步长都用 `computeCollisions` 真实检测，不信任线性化预测
2. **终点硬校验**（L1873-1882）：收敛后用 `isFullQValid(q)` 严格判定，线性化约束是"软引导"，真值校验是"硬判据"

这种"QP 线性化引导 + 真实检测兜底"是处理非线性约束的工程标准做法。

## 5. 工程要点总结

| 维度 | 设计选择 | 原因 |
|---|---|---|
| 求解框架 | 速度级 QP + CLIK 迭代 | 可统一处理任务追踪+关节限位+自碰撞+CCS，比解析 DLS 更灵活 |
| 误差表示 | `log6` + LOCAL 帧雅可比 | 旋转向量无周期性歧义，与雅可比表示一致 |
| 步长控制 | 三档自适应 + 回溯线搜索 + 1/256 兜底 | 接近目标防过冲；大位移时防临时自碰撞卡死 |
| 约束线性化 | 一阶最近点雅可比 | QP 要求线性约束，每步重线性化近似收敛 |
| 终点判据 | 收敛 + 自碰撞 + CCS 双重硬校验 | 迭代内是软约束，必须用真值校验兜底 |
| 鲁棒性 | QP 失败回退到 DLS 解析解 | 奇异/约束冲突时不致完全停摆 |
| 线程安全 | `recursive_mutex` 串行 pinocchio | Data/GeometryData 非线程安全，与 FK/碰撞回调共享 |

### 姿态正则项 W 的双重作用

**姿态正则项 `W`** 既做"关节阻尼"又做"腿部保形"。腿部大权重（1.0~0.2）使躯干在手臂够目标时几乎不动，但 `JᵀJ` 在腿部行可能远小于手臂行，`W` 既能改善 H 的条件数（project_memory 提到"过高腿权重 50.0 导致 QP 病态"），也保证手臂优先运动。这是数值与物理的折中。
