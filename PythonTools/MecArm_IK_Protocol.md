# 机械臂世界坐标协议说明

当前上位机到下位机的运动任务只发送 4 个 `float`：

```text
world_x, world_y, world_z, suc_dir_deg
```

帧格式：

```text
0x5A 0xA5 | cmd | payload | checksum | 0x0D 0x0A
```

- `cmd = 0x00`：急停，无 payload。
- `cmd = 0x01`：运动任务，payload 为 4 个 little-endian float。
- `cmd = 0x02`：自动收敛，无 payload，机械臂回到默认固定收敛姿态。
- `checksum = (cmd + sum(payload_bytes)) & 0xFF`。

坐标定义：

- 原点为云台中心在地面上的投影。
- `+Y` 为云台虚拟 `0 deg`。
- `+X` 为云台虚拟 `90 deg`。
- `+Z` 为竖直向上。
- `suc_dir_deg` 为吸盘在目标竖直平面内的绝对方向，竖直向下为 `0 deg`，平面深度正方向为 `90 deg`，竖直向上为 `180 deg`。

下位机会自动完成：

- 云台平面角计算。
- 吸盘末端到第四舵机轴心的反推。
- 二连杆 IK。
- 舵机标定映射与限位检查。
- 运动规划：同侧目标直达；只有 `world_x` 正负号跨过零点时才执行折叠、转平面、展开。
- 机械臂初始化后直接进入与跨零规划共用的固定收敛姿态。

当前执行层默认使用速度型同步命令 `ARM_USE_SYNC_SERVO_SPEED_CMD`，不使用阻塞延时。详细解算、折叠姿态、调参和上位机使用说明见：

```text
PythonTools/MecArm_IK_Planning_Report.md
```

## 上位机 controller 调用

通信层已封装为：

```text
PythonTools/ARM_controller.py
```

其它上位机程序可直接调用：

```python
from ARM_controller import ArmCommunicationController

def on_msg(line):
    print("RX:", line)

arm = ArmCommunicationController(on_message=on_msg)
arm.connect("COM3")
arm.send_target(0.0, 30.0, 18.0, 0.0)
arm.send_fold()
arm.send_stop()
arm.close()
```

通信、预览、默认目标、机械参数和限位参数先集中列在：

```text
PythonTools/ARM_communication.yml
```

后续可以从该文件中删除不希望外置的条目，再把对应值硬编码回上位机代码。
