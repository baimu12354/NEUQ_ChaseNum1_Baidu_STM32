# 上位机通信协议报告

本文档站在上位机角度整理当前工程内的通信接口。当前主链路是 `UART2 + NANO 协议`，对应下位机文件：

- `Code/ChaseNum1_COM.c`
- `Code/ChaseNum1_NanoProtocol.c`
- `Code/ChaseNum1_NanoProtocol.h`

工程内还保留了旧的 `UnifiedProtocol` 和 `VOFA` 调试协议，见文末说明。新上位机程序建议只调用 `PythonTools/controller.py` 中的 `NanoController`。

## 1. 当前主协议：NANO UART2

### 1.1 串口参数

| 项目 | 值 |
|---|---|
| 串口 | UART2 |
| 波特率 | 115200 |
| 数据位 | 8 |
| 校验 | None |
| 停止位 | 1 |
| 下位机接收入口 | `Nano_COM_RX_Callback(uint8_t dat)` |

### 1.2 帧格式

所有多字节数据均为小端格式。

| 字段 | 长度 | 说明 |
|---|---:|---|
| HEAD | 2 | 固定 `AA 55` |
| VERSION | 1 | 当前 `0x01` |
| SEQ | 1 | 上位机递增序号，回包沿用该序号 |
| MODULE | 1 | 模块号 |
| CMD | 1 | 命令或响应号 |
| LEN | 1 | payload 字节数 |
| PAYLOAD | LEN | 载荷 |
| CHECK | 1 | 累加校验 |
| TAIL | 2 | 固定 `0D 0A` |

校验：

```text
CHECK = (VERSION + SEQ + MODULE + CMD + LEN + sum(PAYLOAD)) & 0xFF
```

## 2. 模块号

| 模块 | 值 | 说明 |
|---|---:|---|
| SYSTEM | `0x01` | 系统状态、复位、急停 |
| CHASSIS | `0x02` | 底盘运动 |
| ARM | `0x03` | 机械臂 |
| PUMP | `0x04` | 气泵 |

## 3. 通用回包

### 3.1 ACK `0x80`

payload 长度 5：

```text
uint8 source_seq
uint8 source_module
uint8 source_cmd
uint8 err
uint8 state
```

表示命令已接收。`err` 正常为 `0x00`。

### 3.2 NACK `0x81`

payload 同 ACK。表示命令被拒绝。

常见错误码：

| 错误 | 值 |
|---|---:|
| OK | `0x00` |
| BAD_CMD | `0x01` |
| BAD_LEN | `0x02` |
| BAD_CHECK | `0x03` |
| BAD_STATE | `0x04` |
| BUSY | `0x05` |
| RANGE_LIMIT | `0x06` |
| FORBIDDEN_ZONE | `0x07` |
| TIMEOUT | `0x08` |
| DEVICE | `0x09` |

### 3.3 EVENT `0x83`

payload 长度 5：

```text
uint8 source_seq
uint8 module
uint8 event
uint8 state
uint8 result
```

常见事件：

| 事件 | 值 | 说明 |
|---|---:|---|
| SYS_READY | `0x01` | 下位机初始化完成 |
| SYS_RESET_DONE | `0x02` | 外设复位完成 |
| SYS_ESTOP | `0x03` | 已进入急停 |
| CHASSIS_ROTATE_DONE | `0x10` | 底盘定角旋转完成 |
| CHASSIS_TRANSLATE_DONE | `0x11` | 底盘定向平移完成 |
| CHASSIS_TIMEOUT_STOP | `0x12` | RUN 周期命令超时停止 |
| ARM_DONE | `0x20` | 机械臂目标完成 |
| ARM_FOLD_DONE | `0x21` | 机械臂默认休息姿态到位 |
| ARM_REJECTED | `0x22` | 机械臂目标被拒绝 |
| PUMP_SUCK_DONE | `0x30` | 气泵吸附完成 |
| PUMP_RELEASE_DONE | `0x31` | 气泵释放完成 |
| PUMP_TIMEOUT_STOP | `0x32` | 气泵连续工作 30s 后自动停泵 |

## 4. SYSTEM 指令

| 命令 | 值 | payload | 回包 |
|---|---:|---|---|
| HELLO | `0x01` | 空 | ACK + STATUS |
| RESET | `0x02` | 空 | ACK，完成后 EVENT |
| ESTOP | `0x03` | 空 | ACK + EVENT |
| GET_STATUS | `0x04` | 空 | STATUS |

STATUS `0x82` payload 长度 16：

```text
uint8 system_state
uint8 chassis_state
uint8 arm_state
uint8 pump_state
uint8 movement_type
uint8 pump_on
uint8 sucked
uint8 arm_planner_active
uint32 tick_ms
float yaw_deg
```

## 5. CHASSIS 指令

### 5.1 底盘变量范围

下表为上位机发送范围。下位机内部会自动映射。

| 模式 | 上位机变量 | 上位机范围 | 下位机内部范围 | 说明 |
|---|---|---:|---:|---|
| RUN | `vx` | `-100 ~ 100` | 非零 `±1 ~ ±20` | 横向速度百分量，正负表示方向 |
| RUN | `vy` | `-100 ~ 100` | 非零 `±1 ~ ±20` | 纵向速度百分量，正负表示方向 |
| RUN | `wz` | `-100 ~ 100` | `-200 ~ 200` | RUN 模式航向目标变化率，仍经过航向角环 |
| ROTATE | `angle_deg` | 任意 float | 原值 | 相对当前角度旋转 |
| ROTATE | `max_wz` | `1 ~ 100` | `1 ~ 10` | 定角旋转最大角速度 |
| TRANSLATE | `dx_cm` | `-1000 ~ 1000` | `-50000 ~ 50000` | 相对当前位置 X 平移距离 |
| TRANSLATE | `dy_cm` | `-1000 ~ 1000` | `-50000 ~ 50000` | 相对当前位置 Y 平移距离 |
| TRANSLATE | `speed` | `1 ~ 100` | `1 ~ 20` | 定向平移速度 |

RUN 命令必须周期发送。超时时间为 `500 ms`，超时后下位机发送 `CHASSIS_TIMEOUT_STOP`。

### 5.2 RUN `0x10`

payload 长度 12：

```text
float vx
float vy
float wz
```

示例：前进并右旋。

```python
ctl.chassis_run(vx=0, vy=60, wz=30)
```

### 5.3 STOP `0x11`

payload 空。进入零速度姿态控制。

```python
ctl.chassis_stop()
```

### 5.4 BRAKE `0x12`

payload 空。进入刹停流程。

```python
ctl.chassis_brake()
```

### 5.5 ROTATE_TO `0x13`

payload 长度 8：

```text
float angle_deg
float max_wz
```

示例：相对当前角度旋转 90 度。

```python
ctl.chassis_rotate_to(angle_deg=90, max_wz=80)
event = ctl.wait_for_event(event_code=0x10, timeout_s=5)
```

### 5.6 TRANSLATE_REL `0x14`

payload 长度 12：

```text
float dx_cm
float dy_cm
float speed
```

示例：向前平移 50 cm。

```python
ctl.chassis_translate_rel(dx_cm=0, dy_cm=50, speed=80)
event = ctl.wait_for_event(event_code=0x11, timeout_s=10)
```

### 5.7 GET_STATE `0x15`

payload 空。回包为 CHASSIS_STATE `0x85`，payload 长度 22：

```text
uint8 chassis_state
uint8 movement_type
float yaw_deg
float vx
float vy
float wz
float distance_cm
```

`vx/vy/wz/distance_cm` 已反向映射为上位机量程。

## 6. ARM 指令

### 6.1 TARGET_XYZ_DIR `0x20`

payload 长度 16：

```text
float world_x_cm
float world_y_cm
float world_z_cm
float suction_dir_deg
```

说明：上位机发送吸盘末端三维空间坐标和吸盘在竖直平面内的方向。到达后下位机会发 `ARM_DONE`，目标被拒绝时发 `ARM_REJECTED`。

```python
ctl.arm_target(20, 0, 25, 0)
event = ctl.wait_for_event(event_code=0x20, timeout_s=8)
```

### 6.2 FOLD_HOME `0x21`

payload 空。机械臂回默认休息姿态。下位机可能会先借用跨零用的收敛姿态作为过渡，但最终姿态不是收敛姿态。

```python
ctl.arm_fold_home()
```

### 6.3 STOP_HOLD `0x22`

payload 空。机械臂停止并锁持。

```python
ctl.arm_stop_hold()
```

### 6.4 GET_POSE `0x23`

payload 空。回包 ARM_POSE `0x84`，payload 长度 36：

```text
uint8 arm_state
uint8 reserved0
uint8 planner_active
uint8 reserved1
float x
float y
float z
float suction_dir_deg
float th0
float th1
float th2
float th3
```

## 7. PUMP 指令

| 命令 | 值 | payload | 说明 |
|---|---:|---|---|
| SUCK | `0x30` | 空 | 开始吸附 |
| RELEASE | `0x31` | 空 | 停泵并释放 |
| GET_STATE | `0x32` | 空 | 读取状态 |

吸附流程：上位机发送 `SUCK` 后，下位机先回 `ACK` 并启动气泵；反馈频率降低到吸附阈值以下后回 `PUMP_SUCK_DONE`，此时气泵继续保持开启。上位机发送 `RELEASE` 后下位机停泵，等待释放时间后回 `PUMP_RELEASE_DONE`。如果气泵连续工作超过 `30000 ms`，下位机会自动停泵，并回 `PUMP_TIMEOUT_STOP`，结果码为 `TIMEOUT`。

PUMP_STATE `0x86` payload 长度 10：

```text
uint8 pump_state
uint8 pump_on
uint8 sucked
uint8 reserved
float feedback_hz     // 滤波后的 PE6 反馈频率，单位 Hz
uint8 suck_strength
uint8 reserved
```

## 8. Python Controller

文件：

- `PythonTools/controller.py`
- `PythonTools/communication.yml`

基本用法：

```python
from controller import NanoController

with NanoController(port="COM8", auto_open=True) as ctl:
    ctl.system_hello()
    ctl.chassis_run(0, 60, 20)
    ctl.chassis_brake()
    ctl.arm_target(20, 0, 25, 0)
    ctl.pump_suck()
```

配置文件里列出了重要参数，包括串口、协议常量、命令号、状态名、底盘映射范围、默认值。后续可以根据实际使用决定保留在 `communication.yml`，或硬编码进 `controller.py`。

## 9. 旧协议说明

工程中还保留以下通信文件：

- `Code/ChaseNum1_UnifiedProtocol.c/.h`：旧 UART3 统一协议，版本号 `0x02`，包含系统、底盘 RUN、机械臂、文本回包等功能。当前 `Code/ChaseNum1_COM.c` 已切到 NANO UART2。
- `Code/ChaseNum1_COM.c` 中的 VOFA 解析：用于 UART7 电机 PID 调试，不建议作为正式上位机控制协议。

当前推荐上位机只使用 NANO UART2 协议，避免 UART2/UART3 两套协议并行造成状态不一致。
