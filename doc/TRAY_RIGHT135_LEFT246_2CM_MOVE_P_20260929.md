# 最新 tray 右1/3/5、左2/4/6槽上方2 cm：单臂 MoveP

2026-09-29；仅生成命令，未执行运动，未进行IK或路径测试；任务YAML未修改。

视觉输入已经是base_link位姿，单位m，四元数xyzw：
```yaml
position: [0.7544771432876587, -0.033196281641721725, 0.7934128642082214]
orientation: [-0.006275577936321497, -0.009659134782850742, 0.0024261947255581617, 0.999930739402771]
```

使用当前任务补偿后的局部X=0.205 m；槽1..6局部Y=[0.019,0.030,0.0653,0.077,0.132,0.142] m；用户确认校准高度为视觉局部Z=+0.020 m。

p_tcp=p_tray+R(q_tray)*[0.205,y_slot,0.020]；q_tcp=normalize(q_tray*q_offset)。右臂q_offset=[sqrt(0.5),-sqrt(0.5),0,0]，左臂q_offset=[sqrt(0.5),sqrt(0.5),0,0]。

六条均为固定腰腿的单臂MoveP；每次单独执行一条。速度0.05、自碰撞检查1；不包含预转腰、调腿或夹爪动作。

## 右臂第1槽

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  task_id: tray_right_slot1_2cm,
  arm_id: 1,
  control_mode: 0,
  target_pose: {
    header: {frame_id: base_link},
    pose: {
      position: {x: 0.958959644096, y: -0.012928409838, z: 0.817121939178},
      orientation: {x: 0.708773366939, y: -0.705342209542, z: 0.011267543128, w: -0.002392535929}
    }
  },
  bound_size: [0.0, 0.0, 0.0],
  enable_collision_check: 1,
  use_force_control: false,
  enable_move_line: false,
  speed: 0.05,
  timeout_ms: 60000
}' --feedback
```

## 右臂第3槽

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  task_id: tray_right_slot3_2cm,
  arm_id: 1,
  control_mode: 0,
  target_pose: {
    header: {frame_id: base_link},
    pose: {
      position: {x: 0.958740607138, y: 0.033367398225, z: 0.816538690864},
      orientation: {x: 0.708773366939, y: -0.705342209542, z: 0.011267543128, w: -0.002392535929}
    }
  },
  bound_size: [0.0, 0.0, 0.0],
  enable_collision_check: 1,
  use_force_control: false,
  enable_move_line: false,
  speed: 0.05,
  timeout_ms: 60000
}' --feedback
```

## 右臂第5槽

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  task_id: tray_right_slot5_2cm,
  arm_id: 1,
  control_mode: 0,
  target_pose: {
    header: {frame_id: base_link},
    pose: {
      position: {x: 0.958425061456, y: 0.100061359301, z: 0.815698460572},
      orientation: {x: 0.708773366939, y: -0.705342209542, z: 0.011267543128, w: -0.002392535929}
    }
  },
  bound_size: [0.0, 0.0, 0.0],
  enable_collision_check: 1,
  use_force_control: false,
  enable_move_line: false,
  speed: 0.05,
  timeout_ms: 60000
}' --feedback
```

## 左臂第2槽

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  task_id: tray_left_slot2_2cm,
  arm_id: 0,
  control_mode: 0,
  target_pose: {
    header: {frame_id: base_link},
    pose: {
      position: {x: 0.958907605078, y: -0.001929405763, z: 0.816983370464},
      orientation: {x: 0.705342209542, y: 0.708773366939, z: 0.002392535929, w: 0.011267543128}
    }
  },
  bound_size: [0.0, 0.0, 0.0],
  enable_collision_check: 1,
  use_force_control: false,
  enable_move_line: false,
  speed: 0.05,
  timeout_ms: 60000
}' --feedback
```

## 左臂第4槽

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  task_id: tray_left_slot4_2cm,
  arm_id: 0,
  control_mode: 0,
  target_pose: {
    header: {frame_id: base_link},
    pose: {
      position: {x: 0.958685256546, y: 0.045066338924, z: 0.816391304141},
      orientation: {x: 0.705342209542, y: 0.708773366939, z: 0.002392535929, w: 0.011267543128}
    }
  },
  bound_size: [0.0, 0.0, 0.0],
  enable_collision_check: 1,
  use_force_control: false,
  enable_move_line: false,
  speed: 0.05,
  timeout_ms: 60000
}' --feedback
```

## 左臂第6槽

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  task_id: tray_left_slot6_2cm,
  arm_id: 0,
  control_mode: 0,
  target_pose: {
    header: {frame_id: base_link},
    pose: {
      position: {x: 0.958377753258, y: 0.110060453915, z: 0.815572489014},
      orientation: {x: 0.705342209542, y: 0.708773366939, z: 0.002392535929, w: 0.011267543128}
    }
  },
  bound_size: [0.0, 0.0, 0.0],
  enable_collision_check: 1,
  use_force_control: false,
  enable_move_line: false,
  speed: 0.05,
  timeout_ms: 60000
}' --feedback
```
