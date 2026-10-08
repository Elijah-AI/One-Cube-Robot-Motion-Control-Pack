# Tray 六槽上方2 cm：单臂 MoveP

2026-09-29；仅生成命令，未执行运动、未运行IK或路径测试；不修改任务YAML。

视觉输入为base_link下的tray右下角位姿：
```yaml
position: [0.7616061568260193, -0.05435311049222946, 0.7936766743659973]
orientation_xyzw: [-0.00993711594492197, -0.009078592993319035, 0.013018704019486904, 0.9998246431350708]
```

沿用当前任务：局部X=0.205 m，局部Y分别为[0.008,0.034,0.0563,0.087,0.117,0.143] m，局部Z=0.020 m。已含视觉Y方向−0.010 m整体补偿及第4–6槽既有修正，不要重复补偿。

p_tcp=p_tray+R(q_tray)*[0.205,y_slot,0.020]；右臂q_offset=[sqrt(0.5),-sqrt(0.5),0,0]，左臂q_offset=[sqrt(0.5),sqrt(0.5),0,0]，q_tcp=normalize(q_tray*q_offset)。

右臂到1/2/3槽，左臂到4/5/6槽；均为单臂MoveP，腰腿保持发送前的位置。不包含预转腰、调腿或夹爪动作。速度0.05，保留自碰撞检查。每次单独执行一条；MoveP不保证TCP沿直线到达。

## 右臂：第1槽

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  task_id: tray_right_slot1_2cm,
  arm_id: 1,
  control_mode: 0,
  whole_body_waist_only: false,
  target_pose: {
    header: {frame_id: base_link},
    pose: {
      position: {x: 0.965927800657, y: -0.040590993961, z: 0.817177100122},
      orientation: {x: 0.716188409510, y: -0.697777181452, z: 0.013446136936, w: 0.000607067410}
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

## 右臂：第2槽

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  task_id: tray_right_slot2_2cm,
  arm_id: 1,
  control_mode: 0,
  whole_body_waist_only: false,
  target_pose: {
    header: {frame_id: base_link},
    pose: {
      position: {x: 0.965255637922, y: -0.014604942074, z: 0.816654314731},
      orientation: {x: 0.716188409510, y: -0.697777181452, z: 0.013446136936, w: 0.000607067410}
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

## 右臂：第3槽

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  task_id: tray_right_slot3_2cm,
  arm_id: 1,
  control_mode: 0,
  whole_body_waist_only: false,
  target_pose: {
    header: {frame_id: base_link},
    pose: {
      position: {x: 0.964679129114, y: 0.007683094737, z: 0.816205925723},
      orientation: {x: 0.716188409510, y: -0.697777181452, z: 0.013446136936, w: 0.000607067410}
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

## 左臂：第4槽

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  task_id: tray_left_slot4_2cm,
  arm_id: 0,
  control_mode: 0,
  whole_body_waist_only: false,
  target_pose: {
    header: {frame_id: base_link},
    pose: {
      position: {x: 0.963885460039, y: 0.038366625235, z: 0.815588636819},
      orientation: {x: 0.697777181452, y: 0.716188409510, z: -0.000607067410, w: 0.013446136936}
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
1槽 y +0.006
2槽 y +0.006
3槽 y +0.009
4槽 y -0.005
5槽 y -0.005
6槽 y -0.005
## 左臂：第5槽

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  task_id: tray_left_slot5_2cm,
  arm_id: 0,
  control_mode: 0,
  whole_body_waist_only: false,
  target_pose: {
    header: {frame_id: base_link},
    pose: {
      position: {x: 0.963109887652, y: 0.068350531259, z: 0.814985422906},
      orientation: {x: 0.697777181452, y: 0.716188409510, z: -0.000607067410, w: 0.013446136936}
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

## 左臂：第6槽

```bash
ros2 action send_goal /Control/move_pose com_interfaces/action/MovePose \
'{
  task_id: tray_left_slot6_2cm,
  arm_id: 0,
  control_mode: 0,
  whole_body_waist_only: false,
  target_pose: {
    header: {frame_id: base_link},
    pose: {
      position: {x: 0.962437724917, y: 0.094336583146, z: 0.814462637515},
      orientation: {x: 0.697777181452, y: 0.716188409510, z: -0.000607067410, w: 0.013446136936}
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
