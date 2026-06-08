# 机械臂解算与跨零折叠规划报告

## 1. 通讯协议

上位机到下位机的运动协议保持为 4 个 `float`，也就是只发送吸盘末端的三维空间坐标和吸盘在目标竖直平面内的朝向：

```text
0x5A 0xA5 | cmd | payload | checksum | 0x0D 0x0A
```

- `cmd = 0x00`：急停，无 payload。
- `cmd = 0x01`：运动任务，payload 为 4 个 little-endian float：
- `cmd = 0x02`：自动收敛，无 payload，用于让机械臂回到默认固定收敛姿态。

```text
world_x, world_y, world_z, suc_dir_deg
```

坐标定义：

- 原点：云台中心在地面上的投影。
- `+Y`：云台虚拟 `0 deg`。
- `+X`：云台虚拟 `90 deg`。
- `+Z`：竖直向上。
- `suc_dir_deg`：吸盘在目标竖直平面内的绝对朝向，竖直向下为 `0 deg`，平面深度正方向为 `90 deg`，竖直向上为 `180 deg`。

## 2. 逆解流程

下位机机械参数：

```text
ARM_H0 = 8.8 cm
ARM_H1 = 6.5 cm
ARM_R  = 3.0 cm
ARM_L1 = 17.5 cm
ARM_C  = 2.5 cm
ARM_L2 = 20.5 cm
ARM_L3 = 11.0 cm
```

逆解步骤：

1. 由世界坐标确定目标竖直平面：

```text
plane_x = sqrt(world_x^2 + world_y^2)
th0 = atan2(world_x, world_y)
plane_z = world_z
```

2. 根据吸盘朝向反推第四舵机轴心：

```text
suc_dx = sin(suc_dir_deg)
suc_dz = -cos(suc_dir_deg)
wrist_x = plane_x - ARM_L3 * suc_dx
wrist_z = plane_z - ARM_L3 * suc_dz
```

3. 对 `wrist_x/wrist_z` 做二连杆 IK，臂 1 加上机械偏置后使用等效长度：

```text
L_eff = sqrt(ARM_L1^2 + ARM_C^2)
phi = atan2(ARM_C, ARM_L1)
```

4. 由吸盘绝对朝向求第四舵机相对角：

```text
suc_std_dir = suc_dir_deg - 90 deg
th3 = normalize(suc_std_dir - th1 - th2)
```

5. 将虚拟角映射到真实舵机角并检查硬限位。目标不可达或越界时，下位机拒绝执行。

腕部 `S3` 的软件角度范围已放开到 `[-180 deg, 180 deg]`，用于支持吸盘末端竖直向下的 `0 deg` 姿态。其它关节仍按各自标定限位检查。

## 3. 规划策略

本版不再每次都收敛到中间状态。下位机只根据当前记录位置和新目标位置的 `world_x` 符号决定是否折叠：

```text
同侧或靠近 x=0：当前姿态 -> 目标姿态
跨过 x=0：当前姿态 -> 当前平面折叠姿态 -> 目标平面折叠姿态 -> 目标姿态
```

正负侧判断带有死区，避免目标在零点附近反复触发折叠：

```c
#define ARM_X_SIDE_EPS_CM 1.0f
```

只有当前侧和目标侧都不为 0，且正负号相反，才认为需要“跨零折叠”。如果当前目标在死区内，或者新目标在死区内，就直接去目标姿态。

## 4. 折叠姿态

旧版本的收敛状态使用高位空间目标点，机械臂会明显抬高。本版改为固定虚拟关节角折叠姿态，让臂 1 和臂 2 尽量贴合，缩短水平占用后再转动云台：

```c
#define ARM_FOLD_TH1_DEG       120.0f
#define ARM_FOLD_TH1_MAX_DEG   150.0f
#define ARM_FOLD_TH2_DEG     -145.0f
#define ARM_FOLD_TH3_DEG       75.0f
#define ARM_FOLD_SAFE_MARGIN  10.0f
```

该姿态映射到舵机角约为：

```text
S1 = -173 deg
S2 = 132 deg
S3 = -169 deg
```

该姿态故意把 `th1` 控制在保守角度，避免初始化时第一关节越过实物机械行程。代码还额外设置了 `ARM_FOLD_TH1_MAX_DEG`，若收敛姿态的第一关节虚拟角被误调得过大，会直接拒绝生成初始化姿态。折叠姿态的吸盘末端大约位于目标平面深度 `22 cm`、高度 `15 cm` 附近，第二段机械臂保持回折，同时吸盘末端相比最早的收敛姿态略向上抬。初始化也直接进入同一套固定收敛姿态。

跨零时的三个阶段含义：

- `fold`：在当前竖直平面内折叠，让臂 1 和臂 2 贴近。
- `rotate`：保持折叠关节角，只改变云台角到目标平面。
- `expand`：在目标竖直平面内展开到最终吸盘坐标和朝向。

同侧变化时只生成 `direct` 阶段，减少无意义状态切换。

自动收敛命令使用同一套固定收敛姿态。上位机发送 `cmd=0x02` 后，下位机清空当前运动计划，并按以下策略执行：

```text
当前姿态 -> 当前平面收敛姿态 -> 默认平面收敛姿态
```

默认平面由下位机 `ARM_DEFAULT_FOLD_BASE_DEG` 决定，当前为 `90 deg`。如果机械臂已经处于默认收敛姿态附近，则不会重复发送多余阶段，会直接记录状态并回传完成。

## 5. 非阻塞执行与同步速度命令

下位机仍然不使用 `HAL_Delay()`。`Arm_Task_Loop()` 通过 `HAL_GetTick()` 推进阶段，每个阶段只发送一次舵机速度命令，然后根据最大舵机角变化估算本阶段等待时间：

```text
wait_ms ~= max_servo_delta / speed + extra_margin
```

相关宏：

```c
#define ARM_SERVO_MOVE_SPEED       180U
#define ARM_PLANE_ROTATE_SKIP_DEG  0.05f
#define ARM_PHASE_SETTLE_EXTRA_MS  200U
#define ARM_PHASE_MIN_INTERVAL_MS  350U
#define ARM_PHASE_MAX_INTERVAL_MS  15000U
```

默认使用速度型同步命令：

```c
#define ARM_USE_SYNC_SERVO_SPEED_CMD 2
```

由于单圈和多圈同步命令数据长度不同，下位机分两帧发送：

- `Servo_Sync_Adv_Multi_Turn_Speed()`：发送 `S1`。
- `Servo_Sync_Adv_Single_Turn_Speed()`：发送 `S0/S2/S3`。

如果实车测试发现同步命令不可用，将该宏改为 `0` 即可回退到逐个舵机速度命令。

## 6. 前方细杆处理

当前假设车前方只有一根竖直细杆，参考位置为：

```c
#define ARM_FRONT_ROD_Y_CM       25.0f
#define ARM_FRONT_ROD_RADIUS_CM  0.5f
```

策略保持简单：

- 最终目标不能贴近细杆中心附近。
- 只有跨过 `world_x=0` 时才折叠并经过中间区域。
- 折叠姿态缩短水平伸出长度，从中间区域通过，避免为了细杆规划大量中间点。

## 7. 上位机使用说明

运行：

```text
PythonTools/ARMcom.py
```

通信封装：

```text
PythonTools/ARM_controller.py
PythonTools/ARM_communication.yml
```

`ARM_controller.py` 提供 `ArmCommunicationController`，可供其它上位机程序直接调用；`ARM_communication.yml` 暂时集中列出串口、协议、默认目标、机械参数、舵机映射和限位参数，后续可按实际需要删减。

使用步骤：

1. 选择串口并连接，波特率为 `115200`。
2. 调整 `X/Y/Z/Dir` 四个滑块；上位机默认 `Dir=0 deg`，便于直接测试吸盘竖直向下。
3. 右侧 3D 图显示目标点、机械臂姿态、前方细杆参考线，以及目标平面内的绿色折叠参考点。
4. 状态栏显示 `直达` 或 `跨零折叠` 提示，以及当前目标的 `th0/th1/th2/th3` 解算结果。
5. 若目标太靠近前方细杆，上位机会阻止发送。
6. 点击发送任务后，下位机回传 `ACCEPTED`；阶段完成后回传 `SUCCESS`。
7. 点击自动收敛后，上位机发送 `cmd=0x02`；下位机回传 `FOLDING` 表示开始收敛，完成后回传 `FOLDED`。
8. 点击紧急停止后，下位机清除当前计划并锁止四个舵机。

## 8. 调参建议

- 同侧动作仍然过慢：提高 `ARM_SERVO_MOVE_SPEED`。
- 跨零时折叠不够紧：可以适当减小 `ARM_FOLD_TH2_DEG`，但必须重新检查 `S2` 到最大限位的余量。
- 折叠姿态太低：适当增大 `ARM_FOLD_TH1_DEG` 或调整 `ARM_FOLD_TH3_DEG`，并重新检查限位。
- 零点附近频繁误判：增大 `ARM_X_SIDE_EPS_CM`。
- 阶段完成判断偏早：增大 `ARM_PHASE_SETTLE_EXTRA_MS`。
