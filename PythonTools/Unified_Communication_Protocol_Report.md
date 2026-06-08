# 底盘与机械臂统一协议完整报告

本文档站在上位机角度说明当前统一通信协议、机械臂运动范围、上下位机回包机制，以及 `UnifiedCommunicationController` 的状态变量用法。统一协议使用 `UART3`，同时覆盖底盘控制、机械臂控制、状态查询、周期反馈和调试日志。

## 1. 结论概览

| 问题 | 结论 |
| --- | --- |
| 底盘命令是否有回应 | 默认有。合法命令立即回 `ACK(0x80)`，多数底盘/系统命令随后回 `STATUS(0x82)`；`STATUS` 自动回包可由宏关闭，显式 `GET_STATUS` 不受影响。 |
| 机械臂命令是否有回应 | 默认有。统一协议层先回 `ACK(0x80)`，表示命令已投递；即时 `ACK` 可由宏关闭，动作最终结果看 `ARM_EVENT(0x84)`。 |
| 文本回应是否保留 | 保留。机械臂执行结果由 `ARM_UNIFIED_RESPONSE_MODE` 选择文本、数据包或双发；上位机程序建议优先解析 `ARM_EVENT`。 |
| 上位机是否能直接显示调试文本 | 能。controller 会打印 `TEXT(0x83)`，也兼容同串口偶发的裸 `printf` 文本，裸文本缓存在 `state["raw_texts"]`。 |
| 机械臂工作范围怎么判断 | 准确判断以下位机逆解、舵机限位和前方细杆避让为准；报告中的范围是根据当前参数推导和采样的参考范围。 |

## 2. 相关文件

| 文件 | 作用 |
| --- | --- |
| `Code/ChaseNum1_UnifiedProtocol.c/.h` | 下位机统一协议解析、命令分发、统一帧回传。 |
| `Code/ChaseNum1_MecArm.c/.h` | 机械臂解算、路径规划、执行和机械臂事件回包。 |
| `USER/main.c` | 主循环调用 `Unified_COM_Task_Loop()` 和 `Arm_Task_Loop()`。 |
| `PythonTools/Unified_controller.py` | 上位机 controller 类，负责发包、收包、状态缓存和终端打印。 |
| `PythonTools/Unified_communication.yml` | 上位机可配置协议参数、命令字、响应字、范围参考。 |

## 3. 机械臂坐标和运动范围

世界坐标定义如下：

| 坐标 | 含义 |
| --- | --- |
| `world_x_cm` | 水平面 X 坐标，云台虚拟 `90 deg` 指向 `+X`。 |
| `world_y_cm` | 水平面 Y 坐标，云台虚拟 `0 deg` 指向 `+Y`。 |
| `world_z_cm` | 竖直高度，向上为正，原点为云台中心在地面上的投影。 |
| `suc_dir_deg` | 吸盘在目标竖直平面内的绝对朝向，`0 deg` 竖直向下，`90 deg` 沿平面深度正方向，`180 deg` 竖直向上。 |

当前机械臂参数来自 `Code/ChaseNum1_MecArm.c`：

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `ARM_H0` | `8.8 cm` | 云台中心距地面高度。 |
| `ARM_H1` | `6.5 cm` | 云台中心到大齿轮关节高度差。 |
| `ARM_R` | `3.0 cm` | 云台中心到大齿轮关节水平偏置。 |
| `ARM_L1` | `17.5 cm` | 第一段机械臂名义长度。 |
| `ARM_C` | `2.5 cm` | 第二关节相对第一段的垂直机械偏置。 |
| `ARM_L2` | `20.5 cm` | 第二段机械臂长度。 |
| `ARM_L3` | `11.0 cm` | 第四舵机轴心到吸盘末端长度。 |

由标定和舵机限位换算得到的虚拟关节角范围为：

| 虚拟角 | 约束范围 |
| --- | --- |
| `th0` 云台角 | `[-180, 178] deg` |
| `th1` 大臂角 | `[0, 180] deg` |
| `th2` 小臂角 | `[-163, 77] deg` |
| `th3` 末端角 | `[-105, 95] deg` |

二连杆腕点等效长度：

| 项目 | 值 |
| --- | --- |
| 第一段等效长度 `sqrt(L1^2 + C^2)` | `17.678 cm` |
| 腕点最小可达距离 | `2.82 cm` |
| 腕点最大可达距离 | `38.18 cm` |

这里的“腕点”是扣除吸盘长度和方向约束之后，第四舵机轴心需要到达的位置。最终吸盘末端是否可达，还必须同时满足末端方向、四个舵机限位和前方细杆避让。

按 `z >= 0` 采样得到的参考工作空间如下，`rho = sqrt(x^2 + y^2)`：

| 吸盘方向 | `rho` 参考范围 | `z` 参考范围 |
| --- | --- | --- |
| `0 deg` 竖直向下 | `0.9 - 41.0 cm` | `0.0 - 27.2 cm` |
| `30 deg` | `5.4 - 46.4 cm` | `0.0 - 37.9 cm` |
| `60 deg` | `0.2 - 50.5 cm` | `0.0 - 47.2 cm` |
| `90 deg` 水平向外 | `0.0 - 52.0 cm` | `0.0 - 53.4 cm` |
| `120 deg` | `0.0 - 50.5 cm` | `4.8 - 58.9 cm` |
| `150 deg` | `0.0 - 46.4 cm` | `15.6 - 63.0 cm` |
| `180 deg` 竖直向上 | `0.0 - 41.0 cm` | `27.2 - 64.4 cm` |

注意：上表不是一个矩形包络，不表示范围内任意组合都可达。实际执行时，下位机先根据目标解算 `th0/th1/th2/th3`，再检查标定后的真实舵机角是否在限位内。

前方细杆约束：

| 参数 | 值 |
| --- | --- |
| 细杆中心参考位置 | `world_y = 25.0 cm, world_x = 0.0 cm` |
| 保守避让半径 | `0.5 cm` |
| 判断方式 | 若 `abs(x) <= 0.5` 且 `abs(y - 25.0) <= 0.5`，目标被拒绝。 |

## 4. 协议帧格式

统一协议所有上位机发包和下位机回包都使用同一种二进制帧：

```text
AA 55 | version | seq | cmd | len | payload | checksum | 0D 0A
```

| 字段 | 长度 | 说明 |
| --- | --- | --- |
| `AA 55` | 2 字节 | 固定帧头。 |
| `version` | 1 字节 | 协议版本，当前为 `0x02`。 |
| `seq` | 1 字节 | 发送方递增序号，溢出回到 `0x00`。 |
| `cmd` | 1 字节 | 命令字或响应字。 |
| `len` | 1 字节 | payload 长度，当前最大 `192` 字节。 |
| `payload` | `len` 字节 | 载荷，小端格式。 |
| `checksum` | 1 字节 | `(version + seq + cmd + len + sum(payload)) & 0xFF`。 |
| `0D 0A` | 2 字节 | 固定帧尾。 |

所有多字节整数使用 little-endian。所有 `float` 为 IEEE754 float32 小端。

## 5. 上位机命令

| 命令 | cmd | payload | controller 接口 | 典型回包 |
| --- | --- | --- | --- | --- |
| 通信握手 | `0x01` | 无 | `send_hello()` | `ACK + STATUS` |
| 获取综合状态 | `0x02` | 无 | `get_status()` | `STATUS` |
| 设置周期反馈 | `0x03` | `uint16 interval_ms` | `set_feedback(interval_ms)` | `ACK + STATUS` |
| 设置逻辑航向角 | `0x04` | `float yaw_deg` | `set_yaw(yaw_deg)` | `ACK + STATUS` |
| 底盘刹车 | `0x10` | 无 | `chassis_brake()` | `ACK + STATUS` |
| 底盘速度运行 | `0x11` | `float vx_cm_s, vy_cm_s, wz_deg_s` | `chassis_run(vx, vy, wz)` | `ACK + STATUS`，超时另有 `ERR` |
| 机械臂急停 | `0x20` | 无 | `arm_stop()` | `ACK + ARM_EVENT`，可伴随 `TEXT` |
| 机械臂目标 | `0x21` | `float x_cm, y_cm, z_cm, suc_dir_deg` | `arm_target(x, y, z, dir)` | `ACK + ARM_EVENT`，可伴随 `TEXT` |
| 机械臂自动收敛 | `0x22` | 无 | `arm_fold()` | `ACK + ARM_EVENT`，可伴随 `TEXT` |

底盘速度命令有 `500 ms` 超时保护。若上位机想持续运动，应周期发送 `chassis_run()`；若超时，下位机会自动刹车并回 `ERR source_cmd=0x11 error=TIMEOUT`。

## 6. 下位机响应

| 响应 | cmd | payload | 用途 |
| --- | --- | --- | --- |
| `ACK` | `0x80` | `uint8 source_cmd, uint8 error` | 命令已被统一协议层接收。 |
| `ERR` | `0x81` | `uint8 source_cmd, uint8 error` | 帧长度、未知命令、底盘超时等协议层错误。 |
| `STATUS` | `0x82` | 固定 36 字节 | 综合状态快照。 |
| `TEXT` | `0x83` | UTF-8/ASCII 文本 | 调试日志。 |
| `ARM_EVENT` | `0x84` | 固定 44 字节 | 机械臂动作结果与状态，推荐上位机主逻辑解析这个包。 |

错误码：

| error | 名称 | 含义 |
| --- | --- | --- |
| `0x00` | `NONE` | 无错误。 |
| `0x01` | `BAD_CMD` | 未知命令。 |
| `0x02` | `BAD_LEN` | payload 长度错误。 |
| `0x03` | `BAD_CHECK` | 校验错误，当前解析失败时一般直接丢帧。 |
| `0x04` | `BUSY` | 模块忙，预留。 |
| `0x05` | `TIMEOUT` | 底盘速度命令超时，已自动刹车。 |
| `0x06` | `REJECTED` | 执行被拒绝。 |

## 7. STATUS 数据包

`STATUS` payload 固定 36 字节：

```python
"<BBBBfffffffI"
```

| 字段 | 类型 | 单位 | 含义 |
| --- | --- | --- | --- |
| `chassis_mode` | `uint8` | - | 底盘模式，`0=Breaking`，其他值与底盘模块枚举一致。 |
| `chassis_motion_active` | `uint8` | - | 底盘速度命令是否正在保持。 |
| `arm_cmd_type` | `uint8` | - | 机械臂内部最近命令类型。 |
| `arm_new_cmd_flag` | `uint8` | - | 机械臂是否还有待处理命令。 |
| `yaw_deg` | `float` | deg | 当前逻辑航向角。 |
| `target_vx_cm_s` | `float` | cm/s | 当前底盘 X 方向目标速度。 |
| `target_vy_cm_s` | `float` | cm/s | 当前底盘 Y 方向目标速度。 |
| `target_wz_deg_s` | `float` | deg/s | 当前底盘角速度目标。 |
| `arm_world_x_cm` | `float` | cm | 最近一次机械臂目标 X。 |
| `arm_world_y_cm` | `float` | cm | 最近一次机械臂目标 Y。 |
| `arm_world_z_cm` | `float` | cm | 最近一次机械臂目标 Z。 |
| `timestamp_ms` | `uint32` | ms | 下位机系统毫秒时间。 |

## 8. ARM_EVENT 数据包

`ARM_EVENT` payload 固定 44 字节：

```python
"<BBBBBBBBffffffffI"
```

| 字段 | 类型 | 单位 | 含义 |
| --- | --- | --- | --- |
| `event` | `uint8` | - | 机械臂事件码。 |
| `source_cmd` | `uint8` | - | 对应统一协议命令，通常是 `0x20/0x21/0x22`。 |
| `state` | `uint8` | - | 机械臂状态码。 |
| `err` | `uint8` | - | 错误码。 |
| `phase_index` | `uint8` | - | 当前或完成时的阶段下标。 |
| `phase_count` | `uint8` | - | 本次规划总阶段数。 |
| `planner_active` | `uint8` | - | 非阻塞规划器是否仍在运行。 |
| `reserved` | `uint8` | - | 保留。 |
| `target_x/y/z/dir` | `float * 4` | cm/deg | 本次目标。 |
| `th0/th1/th2/th3` | `float * 4` | deg | 解算出的虚拟关节角。 |
| `timestamp_ms` | `uint32` | ms | 下位机系统毫秒时间。 |

事件码：

| event | 名称 | 含义 |
| --- | --- | --- |
| `0x01` | `ACCEPTED` | 机械臂命令已被规划器接受，后续还会有完成或错误事件。 |
| `0x02` | `COMPLETED` | 机械臂命令执行完成。 |
| `0x03` | `REJECTED` | 目标不可达、越限、过近前方细杆或规划失败。 |
| `0x04` | `STOPPED` | 急停并锁持舵机。 |
| `0x05` | `PHASE_ERROR` | 阶段切换执行失败。 |

状态码：

| state | 名称 |
| --- | --- |
| `0x00` | `IDLE` |
| `0x01` | `RUNNING` |
| `0x02` | `DONE` |
| `0x03` | `REJECTED` |
| `0x04` | `STOPPED` |
| `0x05` | `ERROR` |

## 9. 文本和数据包模式切换

下位机在 `Code/ChaseNum1_UnifiedProtocol.h` 中提供统一协议回包开关：

```c
#define UP_CHASSIS_STATUS_REPLY_ENABLE     1U
#define UP_ARM_ACK_REPLY_ENABLE            1U
```

| 宏 | 默认 | 作用 |
| --- | --- | --- |
| `UP_CHASSIS_STATUS_REPLY_ENABLE` | `1` | 控制底盘/系统命令后的自动 `STATUS(0x82)` 回包，也控制周期 `STATUS` 反馈；`GET_STATUS(0x02)` 显式查询仍会回 `STATUS`。 |
| `UP_ARM_ACK_REPLY_ENABLE` | `1` | 控制机械臂命令 `0x20/0x21/0x22` 收到后是否立即回 `ACK(0x80)`；机械臂最终事件不受这个宏影响。 |

下位机在 `Code/ChaseNum1_MecArm.h` 中提供一键切换宏：

```c
#define ARM_HOST_RESPONSE_TEXT     0U
#define ARM_HOST_RESPONSE_UNIFIED  1U
#define ARM_HOST_RESPONSE_PACKET   2U
#define ARM_HOST_RESPONSE_BOTH     3U
#define ARM_UNIFIED_RESPONSE_MODE  ARM_HOST_RESPONSE_TEXT
```

建议：

| 模式 | 效果 | 使用场景 |
| --- | --- | --- |
| `ARM_HOST_RESPONSE_TEXT` | 裸串口文本 | 当前源码默认，人工串口调试方便。 |
| `ARM_HOST_RESPONSE_UNIFIED` | 只发 `TEXT(0x83)` | 保留文本内容，同时放进统一协议帧。 |
| `ARM_HOST_RESPONSE_PACKET` | 只发 `ARM_EVENT(0x84)` | 上位机正式控制，减少文本解析。 |
| `ARM_HOST_RESPONSE_BOTH` | 同时发 `TEXT` 和 `ARM_EVENT` | 调试和程序解析兼顾。 |

统一协议路径会在接收机械臂命令时调用 `Arm_Set_Host_Response_Mode(ARM_UNIFIED_RESPONSE_MODE)`。因此只要改宏并重新编译，就能在文本包、数据包和双发之间切换。

## 10. Controller 状态变量

`UnifiedCommunicationController` 收到下位机回包后会自动更新 `ctrl.state`。可用：

```python
state = ctrl.get_state_snapshot()
```

主要字段：

| 字段 | 含义 |
| --- | --- |
| `state["connected"]` | 串口连接状态。 |
| `state["last_frame"]` | 最近一帧合法统一协议数据。 |
| `state["last_ack"]` | 最近一个 ACK。 |
| `state["last_error_frame"]` | 最近一个 ERR。 |
| `state["last_status"]` | 最近一个 STATUS 解包结果。 |
| `state["last_text"]` | 最近一个 `TEXT(0x83)` 文本。 |
| `state["last_raw_text"]` | 最近一段裸 `printf` 文本。 |
| `state["last_arm_event"]` | 最近一个机械臂事件完整数据。 |
| `state["texts"]` | 最近缓存的 `TEXT` 文本列表。 |
| `state["raw_texts"]` | 最近缓存的裸文本列表。 |
| `state["counters"]` | 各类回包计数。 |
| `state["arm"]` | 机械臂当前可解析状态。 |

`state["arm"]` 结构：

| 字段 | 含义 |
| --- | --- |
| `state_name` | `IDLE/RUNNING/DONE/REJECTED/STOPPED/ERROR`。 |
| `last_event_name` | `ACCEPTED/COMPLETED/REJECTED/STOPPED/PHASE_ERROR`。 |
| `source_cmd_name` | 来源命令名，例如 `arm_target`。 |
| `error_name` | 错误名。 |
| `phase_index / phase_count` | 当前阶段和总阶段数。 |
| `planner_active` | 规划器是否还在运行。 |
| `target` | 当前目标 `x/y/z/suc_dir`。 |
| `joint` | 当前事件对应的 `th0/th1/th2/th3`。 |
| `timestamp_ms` | 下位机时间。 |

等待机械臂终态示例：

```python
start_count = ctrl.get_state_snapshot()["counters"]["arm_event"]
ctrl.arm_target(0.0, 30.0, 18.0, 0.0)
event = ctrl.wait_for_arm_terminal_event(timeout_s=8.0, min_event_count=start_count)
if event is None:
    print("机械臂等待超时")
else:
    print(event["event_name"], event["state_name"], event["error_name"])
```

## 11. Controller 最小使用示例

```python
from PythonTools.Unified_controller import UnifiedCommunicationController

def on_arm_event(event):
    print("arm:", event["event_name"], event["state_name"], event["error_name"])

ctrl = UnifiedCommunicationController(on_arm_event=on_arm_event)
ctrl.connect("COM3")

ctrl.send_hello()
ctrl.chassis_run(0.0, 20.0, 0.0)
ctrl.arm_target(0.0, 30.0, 18.0, 0.0)
```

默认情况下，controller 会把收到的 `ACK/ERR/STATUS/TEXT/ARM_EVENT` 和裸 `printf` 文本打印到终端。若不想打印：

```python
ctrl = UnifiedCommunicationController(print_received=False)
```

## 12. 发包和回包示例

以下示例均假设 `seq=0x00`。

### 12.1 握手

上位机发送：

```text
AA 55 02 00 01 00 03 0D 0A
```

典型回包：

```text
ACK source=hello err=NONE
STATUS ...
```

### 12.2 获取状态

上位机发送：

```text
AA 55 02 00 02 00 04 0D 0A
```

典型回包：

```text
STATUS ...
```

### 12.3 设置周期反馈为 100 ms

上位机发送：

```text
AA 55 02 00 03 02 64 00 6B 0D 0A
```

典型回包：

```text
ACK source=set_feedback err=NONE
STATUS ...
```

### 12.4 设置航向角为 0 deg

上位机发送：

```text
AA 55 02 00 04 04 00 00 00 00 0A 0D 0A
```

典型回包：

```text
ACK source=set_yaw err=NONE
STATUS ...
```

### 12.5 底盘刹车

上位机发送：

```text
AA 55 02 00 10 00 12 0D 0A
```

典型回包：

```text
ACK source=chassis_brake err=NONE
STATUS chassis_mode=0
```

### 12.6 底盘速度运行

发送 `vx=0.0, vy=20.0, wz=0.0`：

```text
AA 55 02 00 11 0C 00 00 00 00 00 00 A0 41 00 00 00 00 00 0D 0A
```

典型回包：

```text
ACK source=chassis_run err=NONE
STATUS target_vy_cm_s=20.0
```

若超过 `500 ms` 未刷新速度命令，典型回包：

```text
ERR source=chassis_run err=TIMEOUT
```

### 12.7 机械臂急停

上位机发送：

```text
AA 55 02 00 20 00 22 0D 0A
```

典型回包：

```text
ACK source=arm_stop err=NONE
ARM_EVENT event=STOPPED state=STOPPED err=NONE
TEXT STOPPED: All servos locked.
```

### 12.8 机械臂目标

发送 `x=0.0, y=30.0, z=18.0, dir=0.0`：

```text
AA 55 02 00 21 10 00 00 00 00 00 00 F0 41 00 00 90 41 00 00 00 00 35 0D 0A
```

若需要规划运动，典型回包：

```text
ACK source=arm_target err=NONE
ARM_EVENT event=ACCEPTED state=RUNNING err=NONE phase=0/N
TEXT ACCEPTED: ...
ARM_EVENT event=COMPLETED state=DONE err=NONE phase=N/N
TEXT SUCCESS: ...
```

若目标不可达或越限，典型回包：

```text
ACK source=arm_target err=NONE
ARM_EVENT event=REJECTED state=REJECTED err=REJECTED
TEXT ERROR: Target (...) rejected: target unreachable or angle-limited
```

### 12.9 机械臂自动收敛

上位机发送：

```text
AA 55 02 00 22 00 24 0D 0A
```

典型回包：

```text
ACK source=arm_fold err=NONE
ARM_EVENT event=ACCEPTED state=RUNNING err=NONE
TEXT FOLDING: ...
ARM_EVENT event=COMPLETED state=DONE err=NONE
TEXT FOLDED: ...
```

## 13. 一帧机械臂完成事件示例

下面是一帧示例 `ARM_EVENT COMPLETED`，对应目标 `x=0, y=30, z=18, dir=0`，关节角只是示例值：

```text
AA 55 02 00 84 2C 02 21 02 00 01 01 00 00 00 00 00 00 00 00 F0 41 00 00 90 41 00 00 00 00 00 00 00 00 00 00 70 42 00 00 A0 C2 00 00 A0 41 40 E2 01 00 F3 0D 0A
```

解析结果：

| 字段 | 值 |
| --- | --- |
| `event` | `COMPLETED` |
| `source_cmd` | `arm_target(0x21)` |
| `state` | `DONE` |
| `error` | `NONE` |
| `phase_index/phase_count` | `1/1` |
| `planner_active` | `0` |
| `target` | `(0.0, 30.0, 18.0, 0.0)` |
| `joint` | `(0.0, 60.0, -80.0, 20.0)` |

## 14. 注意事项

1. `ACK` 只表示统一协议层接收到了命令，不代表机械臂动作完成。
2. 机械臂上位机主逻辑应以 `ARM_EVENT` 为准，`TEXT` 和裸 `printf` 只用于调试显示。
3. 底盘持续运动必须周期刷新 `chassis_run()`，否则超时自动刹车。
4. 修改 `ARM_UNIFIED_RESPONSE_MODE` 后需要重新编译下位机。
5. 若使用 Keil 手动工程文件，需要确认 `ChaseNum1_UnifiedProtocol.c/.h` 已加入工程。
