# 机械臂运动范围与通信清理报告

## 1. 本次修改范围

本次只处理机械臂相关逻辑，以及统一协议中对机械臂旧回包模式的一处耦合调用。底盘、气泵和其它外设控制逻辑未做功能修改。

涉及文件：

- `Code/ChaseNum1_MecArm.c`
- `Code/ChaseNum1_MecArm.h`
- `Code/ChaseNum1_UnifiedProtocol.c`

## 2. 禁止静止到达区域

新增机械臂吸盘末端禁止静止到达区域：

```text
-13 < x < 13
0   < y < 30
0   < z < 20
```

代码中使用严格不等式实现，也就是说边界点本身不按禁入区处理，只有真正落在内部才拒绝。

对应宏定义：

```c
#define ARM_FORBID_X_MIN_CM            -13.0f
#define ARM_FORBID_X_MAX_CM            13.0f
#define ARM_FORBID_Y_MIN_CM            0.0f
#define ARM_FORBID_Y_MAX_CM            30.0f
#define ARM_FORBID_Z_MIN_CM            0.0f
#define ARM_FORBID_Z_MAX_CM            20.0f
```

检查函数为 `Arm_Target_In_Forbidden_Zone()`，目前在三处使用：

- `Arm_Inverse_Kinematics()`：直接调用逆解时先拒绝禁入区目标。
- `Arm_Build_Motion_Plan()`：上位机或调度层投递目标时先拒绝禁入区目标。
- `Arm_Build_Folded_Pose()`：固定收敛姿态正解后的末端点也必须不在禁入区。

## 3. 新收敛姿态计算

收敛姿态改为：

```text
th1 = 120 deg
th2 = -120 deg
th3 = -90 deg
th0 = 当前云台平面角或默认收敛云台角
```

吸盘方向计算：

```text
dir = th1 + th2 + th3 + 90 = 0 deg
```

因此收敛姿态下吸盘末端为竖直向下。

吸盘末端在对应竖直平面内的正解结果约为：

```text
平面深度 plane_x = 13.618 cm
高度     plane_z = 23.055 cm
```

转换到世界坐标：

```text
x = 13.618 * sin(th0)
y = 13.618 * cos(th0)
z = 23.055
```

因为 `z = 23.055 cm > 20 cm`，所以无论云台虚拟角 `th0` 在 `-180~180` 内如何旋转，吸盘末端都不会落入 `0 < z < 20` 的禁入高度范围，收敛姿态天然避开该区域。

该姿态对应的主要舵机物理角约为：

```text
S1 = -160 deg
S2 = 107 deg
S3 = -5 deg
```

它们均满足当前软件限位，并保留 `ARM_FOLD_SAFE_MARGIN` 指定的安全余量。

## 4. 云台角覆盖处理

云台 0 号舵机保留原来的 `S0_OFFSET = 2 deg` 标定。为了保证虚拟云台角 `-180~180` 的等效方向都能使用，新增 `Arm_Normalize_Base_Servo_Angle()`：

```c
while (actual_s0 > S0_MAX) actual_s0 -= 360.0f;
while (actual_s0 < S0_MIN) actual_s0 += 360.0f;
```

例如：

```text
th0 = -180.0 -> S0 = -178.0
th0 =   90.0 -> S0 =   92.0
th0 =  179.9 -> S0 = -178.1
```

这样不会因为 2 度标定偏置导致虚拟角接近正 180 度时被软件限位误拒绝。

## 5. 机械臂通信与回包清理

已从机械臂文件中删除或断开以下旧通信路径：

- 删除 `ARM_RX_Callback()` 旧机械臂专用串口解析状态机。
- 删除 `ProtocolState_t` 旧解析状态枚举。
- 删除机械臂回包模式宏：`ARM_HOST_RESPONSE_TEXT / UNIFIED / PACKET / BOTH`。
- 删除 `Arm_Set_Host_Response_Mode()`。
- 删除 `Arm_Send_Host_Text()` 文本回包。
- 删除 `Arm_Send_Host_Event()` 结构化事件回包。
- 删除机械臂文件内所有 `sprintf()` 回包拼接。
- 删除机械臂文件内直接 UART 文本输出路径。

仍保留 `g_HostCmd` 作为临时内部命令邮箱，因为现有 `UnifiedProtocol.c` 仍通过该变量把机械臂目标投递给 `Arm_Task_Loop()`。后续重构统一状态机和统一协议时，建议把这个邮箱替换成统一的 `Arm_Command_t` 或模块命令队列。

同时在 `UnifiedProtocol.c` 中删除了旧的：

```c
Arm_Set_Host_Response_Mode(ARM_UNIFIED_RESPONSE_MODE);
```

机械臂自身现在不再主动向上位机发送文本或事件包。后续统一协议应由系统级状态机统一决定 ACK、状态包和事件包的发送。

## 6. 编译检查

已使用 Keil ARMCC 单独编译：

- `Code/ChaseNum1_MecArm.c`
- `Code/ChaseNum1_UnifiedProtocol.c`

结果：

```text
0 errors
```

剩余警告为工程既有的 `ChaseNum1_Typedef.h` 中 `#pragma region / endregion` 不被 ARMCC 识别，和本次修改无关。
