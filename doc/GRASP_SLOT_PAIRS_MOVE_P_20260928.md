# 双臂＋腰部抓取槽位 MoveP 命令

生成日期：2026-09-28。依据 `src/task_manager/config/tasks/laptop_assembly_two_piece.yaml` 当前抓取槽距、视觉翻转修正规则和左右 TCP 姿态映射。
六组槽位分别为左2/右8、左3/右9、左4/右10、左5/右11、左6/右12、左7/右13。
每组提供“上方2 cm”和“与上表面齐平”两条命令，共12条。

## 本次视觉输入与计算约定

视觉坐标为真实 `base_link` 位姿，表示抓取箱子上表面中心：

```yaml
position: {x: 0.3857915997505188, y: 0.5054003000259399, z: 1.0285412073135376}
orientation: {x: -0.009750808589160442, y: -0.0023174739908427, z: 0.7061440944671631, w: 0.70799720287323}
```

视觉局部+Y与基座-X的点积为 `0.999850922540`；本组**无需180度翻转修正**。
沿用任务槽序：14槽中心对称，槽序沿修正后的视觉局部-Y。

```text
pitch = (0.530 - 0.025) / 13 = 0.03884615384615385 m
y(slot) = (7.5 - slot) * pitch
y_left(slot) = y(slot) - 0.008
y_right(slot) = y(slot)
p_left_tcp = p_vision + R(q_corrected) * [0, y_left(slot), h]
p_right_tcp = p_vision + R(q_corrected) * [0, y_right(slot), h]
q_tcp = normalize(q_corrected * q_pick_offset)
左臂 q_pick_offset = [sqrt(0.5), sqrt(0.5), 0, 0]
右臂 q_pick_offset = [sqrt(0.5), -sqrt(0.5), 0, 0]
h = +0.02 m：槽上方2 cm
h = 0 m：TCP与托盘上表面齐平
```

“齐平”指视觉局部Z=0，保留上表面倾斜；不强制左右TCP的基坐标z相同，也不把夹爪姿态改成基座水平。
抓取检测目标为 `rack`（同名缓存 `rack`），不应用放置检测目标 `tray`（同名缓存 `tray`）的-Y 0.005 m补偿。

2026-09-28 左臂实测偏向槽序负方向 0.8 cm，所有左臂目标已沿槽序正方向补偿 0.008 m（修正后的视觉局部 -Y）；右臂不变。补偿已包含在汇总表和命令中，不要重复叠加。

## 执行说明

- 所有命令均为双臂＋腰部 MoveP：`arm_id=7`、`whole_body_waist_only=true`，三个腿关节保持启动角度。
- `synchronization_mode=0`，双臂同步开始和结束；速度0.1、超时100秒，保留自碰撞检查。
- 按需要选择一组执行；先确认处于准备关节位且反馈完整、新鲜、静止，再发送上方命令；成功到位后才发送该组齐平命令。不要把12条作为批处理连续运行。
- 两种高度均按请求使用 MoveP（`enable_move_line=false`），不保证两点之间直线下降。齐平目标与表面高度相同，但不提供接触检测或接触力控制。
- 不包含准备位MoveJ、夹爪开合或表面以下抓取动作。更换槽位组前先退出到合适准备位；失败后检查原因，不继续发送下一条。
- 本文件仅完成坐标换算和格式校验，未执行，未验证这些目标的IK和避碰路径。固定目标已包含本次视觉修正，手工命令不再读取实时视觉。

## 目标坐标汇总

位置单位m，姿态四元数顺序xyzw。

| 左槽 / 右槽 | 高度 | 左TCP xyz | 右TCP xyz |
|---|---|---|---|
| 2 / 8 | 上方2 cm | 0.179827362, 0.506131462, 1.045024615 | 0.404870731, 0.505561826, 1.048868936 |
| 2 / 8 | 上表面齐平 | 0.180168412, 0.505920779, 1.025028633 | 0.405211781, 0.505351143, 1.028872954 |
| 3 / 9 | 上方2 cm | 0.218667725, 0.506033148, 1.045688109 | 0.443711094, 0.505463512, 1.049532430 |
| 3 / 9 | 上表面齐平 | 0.219008775, 0.505822465, 1.025692127 | 0.444052144, 0.505252829, 1.029536448 |
| 4 / 10 | 上方2 cm | 0.257508088, 0.505934834, 1.046351602 | 0.482551457, 0.505365199, 1.050195923 |
| 4 / 10 | 上表面齐平 | 0.257849137, 0.505724151, 1.026355620 | 0.482892507, 0.505154516, 1.030199941 |
| 5 / 11 | 上方2 cm | 0.296348451, 0.505836521, 1.047015096 | 0.521391820, 0.505266885, 1.050859417 |
| 5 / 11 | 上表面齐平 | 0.296689500, 0.505625838, 1.027019114 | 0.521732869, 0.505056202, 1.030863435 |
| 6 / 12 | 上方2 cm | 0.335188813, 0.505738207, 1.047678589 | 0.560232183, 0.505168571, 1.051522910 |
| 6 / 12 | 上表面齐平 | 0.335529863, 0.505527524, 1.027682607 | 0.560573232, 0.504957888, 1.031526928 |
| 7 / 13 | 上方2 cm | 0.374029176, 0.505639893, 1.048342083 | 0.599072545, 0.505070257, 1.052186404 |
| 7 / 13 | 上表面齐平 | 0.374370226, 0.505429210, 1.028346101 | 0.599413595, 0.504859574, 1.032190422 |

## 左第2槽、右第8槽：上方2 cm

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  "task_id": "laptop_dual_left2_right8_above_2cm",
  "arm_id": 7,
  "control_mode": 0,
  "whole_body_waist_only": true,
  "left_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.179827362299,
        "y": 0.506131461975,
        "z": 1.045024615069
      },
      "orientation": {
        "x": 0.00131034554,
        "y": 0.999948915714,
        "z": -0.005256161379,
        "w": 0.008533564576
      }
    }
  },
  "right_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.40487073148,
        "y": 0.505561826131,
        "z": 1.048868936132
      },
      "orientation": {
        "x": 0.999948915714,
        "y": -0.00131034554,
        "z": 0.008533564576,
        "w": 0.005256161379
      }
    }
  },
  "left_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "right_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "synchronization_mode": 0,
  "enable_collision_check": 1,
  "use_force_control": false,
  "enable_move_line": false,
  "speed": 0.1,
  "timeout_ms": 100000
}' --feedback
```

## 左第2槽、右第8槽：与上表面齐平

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  "task_id": "laptop_dual_left2_right8_surface",
  "arm_id": 7,
  "control_mode": 0,
  "whole_body_waist_only": true,
  "left_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.180168411949,
        "y": 0.505920778984,
        "z": 1.025028633027
      },
      "orientation": {
        "x": 0.00131034554,
        "y": 0.999948915714,
        "z": -0.005256161379,
        "w": 0.008533564576
      }
    }
  },
  "right_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.405211781131,
        "y": 0.505351143139,
        "z": 1.02887295409
      },
      "orientation": {
        "x": 0.999948915714,
        "y": -0.00131034554,
        "z": 0.008533564576,
        "w": 0.005256161379
      }
    }
  },
  "left_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "right_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "synchronization_mode": 0,
  "enable_collision_check": 1,
  "use_force_control": false,
  "enable_move_line": false,
  "speed": 0.1,
  "timeout_ms": 100000
}' --feedback
```

## 左第3槽、右第9槽：上方2 cm

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  "task_id": "laptop_dual_left3_right9_above_2cm",
  "arm_id": 7,
  "control_mode": 0,
  "whole_body_waist_only": true,
  "left_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.218667725059,
        "y": 0.506033148202,
        "z": 1.045688108622
      },
      "orientation": {
        "x": 0.00131034554,
        "y": 0.999948915714,
        "z": -0.005256161379,
        "w": 0.008533564576
      }
    }
  },
  "right_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.443711094241,
        "y": 0.505463512357,
        "z": 1.049532429685
      },
      "orientation": {
        "x": 0.999948915714,
        "y": -0.00131034554,
        "z": 0.008533564576,
        "w": 0.005256161379
      }
    }
  },
  "left_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "right_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "synchronization_mode": 0,
  "enable_collision_check": 1,
  "use_force_control": false,
  "enable_move_line": false,
  "speed": 0.1,
  "timeout_ms": 100000
}' --feedback
```

## 左第3槽、右第9槽：与上表面齐平

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  "task_id": "laptop_dual_left3_right9_surface",
  "arm_id": 7,
  "control_mode": 0,
  "whole_body_waist_only": true,
  "left_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.21900877471,
        "y": 0.50582246521,
        "z": 1.025692126581
      },
      "orientation": {
        "x": 0.00131034554,
        "y": 0.999948915714,
        "z": -0.005256161379,
        "w": 0.008533564576
      }
    }
  },
  "right_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.444052143891,
        "y": 0.505252829366,
        "z": 1.029536447643
      },
      "orientation": {
        "x": 0.999948915714,
        "y": -0.00131034554,
        "z": 0.008533564576,
        "w": 0.005256161379
      }
    }
  },
  "left_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "right_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "synchronization_mode": 0,
  "enable_collision_check": 1,
  "use_force_control": false,
  "enable_move_line": false,
  "speed": 0.1,
  "timeout_ms": 100000
}' --feedback
```

## 左第4槽、右第10槽：上方2 cm

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  "task_id": "laptop_dual_left4_right10_above_2cm",
  "arm_id": 7,
  "control_mode": 0,
  "whole_body_waist_only": true,
  "left_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.257508087819,
        "y": 0.505934834428,
        "z": 1.046351602176
      },
      "orientation": {
        "x": 0.00131034554,
        "y": 0.999948915714,
        "z": -0.005256161379,
        "w": 0.008533564576
      }
    }
  },
  "right_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.482551457001,
        "y": 0.505365198583,
        "z": 1.050195923239
      },
      "orientation": {
        "x": 0.999948915714,
        "y": -0.00131034554,
        "z": 0.008533564576,
        "w": 0.005256161379
      }
    }
  },
  "left_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "right_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "synchronization_mode": 0,
  "enable_collision_check": 1,
  "use_force_control": false,
  "enable_move_line": false,
  "speed": 0.1,
  "timeout_ms": 100000
}' --feedback
```

## 左第4槽、右第10槽：与上表面齐平

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  "task_id": "laptop_dual_left4_right10_surface",
  "arm_id": 7,
  "control_mode": 0,
  "whole_body_waist_only": true,
  "left_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.25784913747,
        "y": 0.505724151437,
        "z": 1.026355620134
      },
      "orientation": {
        "x": 0.00131034554,
        "y": 0.999948915714,
        "z": -0.005256161379,
        "w": 0.008533564576
      }
    }
  },
  "right_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.482892506651,
        "y": 0.505154515592,
        "z": 1.030199941197
      },
      "orientation": {
        "x": 0.999948915714,
        "y": -0.00131034554,
        "z": 0.008533564576,
        "w": 0.005256161379
      }
    }
  },
  "left_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "right_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "synchronization_mode": 0,
  "enable_collision_check": 1,
  "use_force_control": false,
  "enable_move_line": false,
  "speed": 0.1,
  "timeout_ms": 100000
}' --feedback
```

## 左第5槽、右第11槽：上方2 cm

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  "task_id": "laptop_dual_left5_right11_above_2cm",
  "arm_id": 7,
  "control_mode": 0,
  "whole_body_waist_only": true,
  "left_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.29634845058,
        "y": 0.505836520655,
        "z": 1.047015095729
      },
      "orientation": {
        "x": 0.00131034554,
        "y": 0.999948915714,
        "z": -0.005256161379,
        "w": 0.008533564576
      }
    }
  },
  "right_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.521391819761,
        "y": 0.50526688481,
        "z": 1.050859416792
      },
      "orientation": {
        "x": 0.999948915714,
        "y": -0.00131034554,
        "z": 0.008533564576,
        "w": 0.005256161379
      }
    }
  },
  "left_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "right_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "synchronization_mode": 0,
  "enable_collision_check": 1,
  "use_force_control": false,
  "enable_move_line": false,
  "speed": 0.1,
  "timeout_ms": 100000
}' --feedback
```

## 左第5槽、右第11槽：与上表面齐平

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  "task_id": "laptop_dual_left5_right11_surface",
  "arm_id": 7,
  "control_mode": 0,
  "whole_body_waist_only": true,
  "left_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.29668950023,
        "y": 0.505625837663,
        "z": 1.027019113687
      },
      "orientation": {
        "x": 0.00131034554,
        "y": 0.999948915714,
        "z": -0.005256161379,
        "w": 0.008533564576
      }
    }
  },
  "right_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.521732869411,
        "y": 0.505056201818,
        "z": 1.03086343475
      },
      "orientation": {
        "x": 0.999948915714,
        "y": -0.00131034554,
        "z": 0.008533564576,
        "w": 0.005256161379
      }
    }
  },
  "left_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "right_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "synchronization_mode": 0,
  "enable_collision_check": 1,
  "use_force_control": false,
  "enable_move_line": false,
  "speed": 0.1,
  "timeout_ms": 100000
}' --feedback
```

## 左第6槽、右第12槽：上方2 cm

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  "task_id": "laptop_dual_left6_right12_above_2cm",
  "arm_id": 7,
  "control_mode": 0,
  "whole_body_waist_only": true,
  "left_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.33518881334,
        "y": 0.505738206881,
        "z": 1.047678589282
      },
      "orientation": {
        "x": 0.00131034554,
        "y": 0.999948915714,
        "z": -0.005256161379,
        "w": 0.008533564576
      }
    }
  },
  "right_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.560232182521,
        "y": 0.505168571036,
        "z": 1.051522910345
      },
      "orientation": {
        "x": 0.999948915714,
        "y": -0.00131034554,
        "z": 0.008533564576,
        "w": 0.005256161379
      }
    }
  },
  "left_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "right_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "synchronization_mode": 0,
  "enable_collision_check": 1,
  "use_force_control": false,
  "enable_move_line": false,
  "speed": 0.1,
  "timeout_ms": 100000
}' --feedback
```

## 左第6槽、右第12槽：与上表面齐平

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  "task_id": "laptop_dual_left6_right12_surface",
  "arm_id": 7,
  "control_mode": 0,
  "whole_body_waist_only": true,
  "left_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.33552986299,
        "y": 0.505527523889,
        "z": 1.027682607241
      },
      "orientation": {
        "x": 0.00131034554,
        "y": 0.999948915714,
        "z": -0.005256161379,
        "w": 0.008533564576
      }
    }
  },
  "right_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.560573232172,
        "y": 0.504957888045,
        "z": 1.031526928303
      },
      "orientation": {
        "x": 0.999948915714,
        "y": -0.00131034554,
        "z": 0.008533564576,
        "w": 0.005256161379
      }
    }
  },
  "left_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "right_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "synchronization_mode": 0,
  "enable_collision_check": 1,
  "use_force_control": false,
  "enable_move_line": false,
  "speed": 0.1,
  "timeout_ms": 100000
}' --feedback
```

## 左第7槽、右第13槽：上方2 cm

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  "task_id": "laptop_dual_left7_right13_above_2cm",
  "arm_id": 7,
  "control_mode": 0,
  "whole_body_waist_only": true,
  "left_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.3740291761,
        "y": 0.505639893107,
        "z": 1.048342082836
      },
      "orientation": {
        "x": 0.00131034554,
        "y": 0.999948915714,
        "z": -0.005256161379,
        "w": 0.008533564576
      }
    }
  },
  "right_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.599072545281,
        "y": 0.505070257263,
        "z": 1.052186403898
      },
      "orientation": {
        "x": 0.999948915714,
        "y": -0.00131034554,
        "z": 0.008533564576,
        "w": 0.005256161379
      }
    }
  },
  "left_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "right_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "synchronization_mode": 0,
  "enable_collision_check": 1,
  "use_force_control": false,
  "enable_move_line": false,
  "speed": 0.1,
  "timeout_ms": 100000
}' --feedback
```

## 左第7槽、右第13槽：与上表面齐平

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  "task_id": "laptop_dual_left7_right13_surface",
  "arm_id": 7,
  "control_mode": 0,
  "whole_body_waist_only": true,
  "left_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.37437022575,
        "y": 0.505429210116,
        "z": 1.028346100794
      },
      "orientation": {
        "x": 0.00131034554,
        "y": 0.999948915714,
        "z": -0.005256161379,
        "w": 0.008533564576
      }
    }
  },
  "right_target_pose": {
    "header": {
      "frame_id": "base_link"
    },
    "pose": {
      "position": {
        "x": 0.599413594932,
        "y": 0.504859574271,
        "z": 1.032190421857
      },
      "orientation": {
        "x": 0.999948915714,
        "y": -0.00131034554,
        "z": 0.008533564576,
        "w": 0.005256161379
      }
    }
  },
  "left_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "right_bound_size": [
    0.0,
    0.0,
    0.0
  ],
  "synchronization_mode": 0,
  "enable_collision_check": 1,
  "use_force_control": false,
  "enable_move_line": false,
  "speed": 0.1,
  "timeout_ms": 100000
}' --feedback
```
