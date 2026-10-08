# 放置托盘六槽中心 MoveP 命令

生成日期：2026-09-28。依据 `src/task_manager/config/tasks/laptop_assembly_two_piece.yaml` 当前放置姿态映射和槽位标定。
右臂＋腰部到第1、2、3槽，左臂＋腰部到第4、5、6槽；命令彼此独立，需逐条执行。

## 目标定义

这里“槽中心”取槽内长中点 `x=0.205 m`，**高度沿用任务的放置点：上表面上方10 cm（局部z=+0.10 m）**，不是上表面接触点（z=0），也不是25 cm接近高点。
本文件使用下列固定的视觉右下角位姿，坐标系 `base_link`：

```yaml
position: {x: 0.7314735651016235, y: -0.040704675018787384, z: 0.7919204235076904}
orientation: {x: -0.005905137397348881, y: -0.011516675353050232, z: 0.02253052406013012, w: 0.9996623992919922}
```

视觉右下角比真实右下角沿视觉+Y偏0.005 m，所以一次性补偿：

```text
p_tcp = p_vision + R(q_vision) * [0.205, y_slot - 0.005, 0.10]
q_tcp = normalize(q_vision * q_place_offset)
右臂 q_place_offset = [sqrt(0.5), sqrt(0.5), 0, 0]
左臂 q_place_offset = [sqrt(0.5), -sqrt(0.5), 0, 0]
```

补偿已包含在下面目标中，不要再次减0.005 m。放置检测目标 `tray` 的角点（同名缓存 `tray`）不应用抓取检测目标 `rack`（同名缓存 `rack`）的180度修正。
全部命令为单臂＋腰部 MoveP：`whole_body_waist_only=true`，三个腿关节保持启动值；另一手臂保持关节角，会随腰转动，TCP不会固定在空间中。
速度为0.05，超时100秒；保留自碰撞检查。仅生成命令，未验证在线IK或路径，无接触力检测，也不控制夹爪。
确认反馈完整、新鲜且机器人静止后执行；等待每条动作成功并静止，再运行下一条。失败后先检查反馈与错误，不连续重试。

更新：第4、5、6槽沿视觉局部+Y各增加0.004 m；下面坐标已重新计算，第1、2、3槽不变。

## 目标汇总

所有长度单位为m，四元数顺序xyzw。

| 槽号 | 部件 | 相对真实角点Y | 补偿后局部Y | TCP基坐标xyz |
|---|---|---:|---:|---|
| 1 | 右臂＋腰部 | 0.0180 | 0.0130 | 0.933298065, -0.017327768, 0.896392388 |
| 2 | 右臂＋腰部 | 0.0440 | 0.0390 | 0.932130410, 0.008644023, 0.896071932 |
| 3 | 右臂＋腰部 | 0.0663 | 0.0613 | 0.931128921, 0.030919827, 0.895797079 |
| 4 | 左臂＋腰部 | 0.0970 | 0.0920 | 0.929750189, 0.061586518, 0.895418694 |
| 5 | 左臂＋腰部 | 0.1270 | 0.1220 | 0.928402895, 0.091553969, 0.895048937 |
| 6 | 左臂＋腰部 | 0.1530 | 0.1480 | 0.927235240, 0.117525759, 0.894728481 |

## 右臂＋腰部到第1槽中心上方10 cm

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  "task_id": "laptop_right_waist_place_slot1_center_10cm",
  "arm_id": 6,
  "control_mode": 0,
  "whole_body_waist_only": true,
  "target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.933298065024,
        "y": -0.017327767538,
        "z": 0.896392388301
      },
      "orientation": {
        "x": 0.690936560742,
        "y": 0.722799532774,
        "z": 0.003967956459,
        "w": 0.012319081681
      }
    }
  },
  "bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "enable_collision_check": 1,
  "use_force_control": false,
  "enable_move_line": false,
  "speed": 0.05,
  "timeout_ms": 100000
}' --feedback
```

## 右臂＋腰部到第2槽中心上方10 cm

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  "task_id": "laptop_right_waist_place_slot2_center_10cm",
  "arm_id": 6,
  "control_mode": 0,
  "whole_body_waist_only": true,
  "target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.932130409743,
        "y": 0.008644022715,
        "z": 0.896071932045
      },
      "orientation": {
        "x": 0.690936560742,
        "y": 0.722799532774,
        "z": 0.003967956459,
        "w": 0.012319081681
      }
    }
  },
  "bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "enable_collision_check": 1,
  "use_force_control": false,
  "enable_move_line": false,
  "speed": 0.05,
  "timeout_ms": 100000
}' --feedback
```

## 右臂＋腰部到第3槽中心上方10 cm

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  "task_id": "laptop_right_waist_place_slot3_center_10cm",
  "arm_id": 6,
  "control_mode": 0,
  "whole_body_waist_only": true,
  "target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.93112892079,
        "y": 0.030919827432,
        "z": 0.89579707918
      },
      "orientation": {
        "x": 0.690936560742,
        "y": 0.722799532774,
        "z": 0.003967956459,
        "w": 0.012319081681
      }
    }
  },
  "bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "enable_collision_check": 1,
  "use_force_control": false,
  "enable_move_line": false,
  "speed": 0.05,
  "timeout_ms": 100000
}' --feedback
```

## 左臂＋腰部到第4槽中心上方10 cm

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  "task_id": "laptop_left_waist_place_slot4_center_10cm",
  "arm_id": 5,
  "control_mode": 0,
  "whole_body_waist_only": true,
  "target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.929750189361,
        "y": 0.06158651823,
        "z": 0.895418694294
      },
      "orientation": {
        "x": 0.722799532774,
        "y": -0.690936560742,
        "z": 0.012319081681,
        "w": -0.003967956459
      }
    }
  },
  "bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "enable_collision_check": 1,
  "use_force_control": false,
  "enable_move_line": false,
  "speed": 0.05,
  "timeout_ms": 100000
}' --feedback
```

## 左臂＋腰部到第5槽中心上方10 cm

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  "task_id": "laptop_left_waist_place_slot5_center_10cm",
  "arm_id": 5,
  "control_mode": 0,
  "whole_body_waist_only": true,
  "target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.928402894806,
        "y": 0.091553968522,
        "z": 0.895048937077
      },
      "orientation": {
        "x": 0.722799532774,
        "y": -0.690936560742,
        "z": 0.012319081681,
        "w": -0.003967956459
      }
    }
  },
  "bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "enable_collision_check": 1,
  "use_force_control": false,
  "enable_move_line": false,
  "speed": 0.05,
  "timeout_ms": 100000
}' --feedback
```

## 左臂＋腰部到第6槽中心上方10 cm

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  "task_id": "laptop_left_waist_place_slot6_center_10cm",
  "arm_id": 5,
  "control_mode": 0,
  "whole_body_waist_only": true,
  "target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.927235239525,
        "y": 0.117525758775,
        "z": 0.894728480821
      },
      "orientation": {
        "x": 0.722799532774,
        "y": -0.690936560742,
        "z": 0.012319081681,
        "w": -0.003967956459
      }
    }
  },
  "bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "enable_collision_check": 1,
  "use_force_control": false,
  "enable_move_line": false,
  "speed": 0.05,
  "timeout_ms": 100000
}' --feedback
```
