# 上位机通信协议报告

本文档站在上位机角度整理当前工程内的通信接口。当前主链路是 `NANO 协议`，实际串口由下位机 `Code/ChaseNum1_COM.h` 中的 `NANO_PROTOCOL_UART_SELECT` 决定，对应下位机文件：

- `Code/ChaseNum1_COM.c`
- `Code/ChaseNum1_NanoProtocol.c`
- `Code/ChaseNum1_NanoProtocol.h`

工程内还保留了旧的 `UnifiedProtocol` 和 `VOFA` 调试协议，见文末说明。新上位机程序建议只调用 `NANO_Control/controller.py` 中的 `NanoController`。

## 1. 当前主协议：NANO

### 1.1 串口参数

| 项目 | 值 |
|---|---|
| 串口 | 由 `NANO_PROTOCOL_UART_SELECT` 选择，当前代码为 UART7 |
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
| SHOOTER | `0x05` | 射击模块 |

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
| ARM_FOLD_DONE | `0x21` | 机械臂收敛完成 |
| ARM_REJECTED | `0x22` | 机械臂目标被拒绝 |
| PUMP_SUCK_DONE | `0x30` | 气泵吸附完成 |
| PUMP_RELEASE_DONE | `0x31` | 气泵释放完成 |
| PUMP_TIMEOUT_STOP | `0x32` | 气泵超时停泵 |
| SHOOTER_DONE | `0x40` | 一次射击动作完成 |
| SHOOTER_RESET_DONE | `0x41` | 射击模块复位完成 |
| SHOOTER_TIMEOUT | `0x42` | 射击动作超时 |

## 4. SYSTEM 指令

| 命令 | 值 | payload | 回包 |
|---|---:|---|---|
| HELLO | `0x01` | 空 | ACK + STATUS |
| RESET | `0x02` | 空 | ACK，完成后 EVENT |
| ESTOP | `0x03` | 空 | ACK + EVENT |
| GET_STATUS | `0x04` | 空 | STATUS |

STATUS `0x82` payload 长度 17：

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
uint8 shooter_state
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
| LOCAL_POS | `x_cm` | 查询回包 | 内部积分 `*0.02` | 相对开始积分位置的局部 X 位移 |
| LOCAL_POS | `y_cm` | 查询回包 | 内部积分 `*0.02` | 相对开始积分位置的局部 Y 位移 |

RUN 命令必须周期发送。超时时间为 `500 ms`，超时后下位机发送 `CHASSIS_TIMEOUT_STOP`。

底盘局部位移积分由上位机显式开启。上位机发送 START_LOCAL_POS `0x16` 时，下位机会清零局部坐标，并记录当前 yaw 角作为局部坐标系的 `+Y` 方向；之后每次读取编码器增量时，先按定向平移相同的方向和系数合成车体系 `delta_x/delta_y`，再根据当前 yaw 相对开始 yaw 的角度差旋转分解到局部坐标系中累计。回包单位为 cm，换算比例与定向平移一致：下位机内部 `50000` 对应上位机 `1000 cm`，即 `1` 个内部位移单位对应 `0.02 cm`。

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

### 5.8 START_LOCAL_POS `0x16`

payload 空。开启局部位移积分，清零坐标，并记录当前 yaw 为局部坐标系 `+Y` 方向。下位机先回 ACK `0x80`，随后回 CHASSIS_LOCAL_POS `0x87`。

```python
ctl.chassis_start_local_pos()
```

### 5.9 GET_LOCAL_POS `0x17`

payload 空。回包为 CHASSIS_LOCAL_POS `0x87`，payload 长度 8：

```text
float x_cm
float y_cm
```

`x_cm/y_cm` 表示当前位置相对开始积分位置的局部坐标，单位为 cm。局部 `+Y` 为开始积分时的车身 yaw 方向，局部 `+X` 与底盘定向平移 X 正方向一致。

```python
ctl.chassis_get_local_pos()
time.sleep(0.05)
print(ctl.chassis_local_pos)
```

### 5.10 STOP_LOCAL_POS `0x18`

payload 空。关闭局部位移积分，并把局部坐标清零。下位机先回 ACK `0x80`，随后回 CHASSIS_LOCAL_POS `0x87`，此时坐标通常为 `0, 0`。

```python
ctl.chassis_stop_local_pos()
```

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

payload 空。机械臂回默认收敛姿态。

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

PUMP_STATE `0x86` payload 长度 10：

```text
uint8 pump_state
uint8 pump_on
uint8 sucked
uint8 reserved
float feedback_hz
uint8 suck_strength
uint8 reserved
```

## 8. SHOOTER 指令

射击模块硬件使用 `PB8/TIM16_CH1` 输出摩擦轮 PWM，`PB7` 输出 DRV8701 方向，`UART5(PB13 TX / PB12 RX)` 控制推弹总线舵机。

| 命令 | 值 | payload | 说明 |
|---|---:|---|---|
| FIRE | `0x40` | 空 | 执行一次完整射击 |
| RESET | `0x41` | 空 | 立即关闭摩擦轮，并让推弹舵机回初始位置 |
| GET_STATE | `0x42` | 空 | 读取射击模块状态 |

FIRE 收到后，下位机流程为：摩擦轮启动并预热 `SHOOTER_SPINUP_MS`，舵机推到 `SHOOTER_SERVO_PUSH_POS`，摩擦轮关闭，舵机回到 `SHOOTER_SERVO_HOME_POS`，最后发送 `SHOOTER_DONE`。

SHOOTER_STATE `0x88` payload 长度 12：

```text
uint8 shooter_state
uint8 friction_on
uint8 busy
uint8 reserved
float duty_percent
int16 servo_pos
uint16 pwm_duty
```

状态枚举：

| 状态 | 值 | 说明 |
|---|---:|---|
| IDLE | `0` | 空闲 |
| SPINUP | `1` | 摩擦轮预热中 |
| PUSHING | `2` | 舵机推弹中 |
| RETURNING | `3` | 舵机回位中 |
| RESETTING | `4` | 复位中 |
| FAULT | `5` | 故障预留 |

示例：

```python
ctl.shooter_fire()
event = ctl.wait_for_event(event_code=0x40, timeout_s=6)
ctl.shooter_get_state()
```

紧急关闭摩擦轮并复位舵机：

```python
ctl.shooter_reset()
```

## 9. Python Controller

文件：

- `NANO_Control/controller.py`
- `NANO_Control/communication.yml`

基本用法：

```python
from controller import NanoController

with NanoController(port="COM8", auto_open=True) as ctl:
    ctl.system_hello()
    ctl.chassis_run(0, 60, 20)
    ctl.chassis_brake()
    ctl.arm_target(20, 0, 25, 0)
    ctl.pump_suck()
    ctl.shooter_fire()
```

配置文件里列出了重要参数，包括串口、协议常量、命令号、状态名、底盘映射范围、默认值。后续可以根据实际使用决定保留在 `communication.yml`，或硬编码进 `controller.py`。

常用射击接口：

| 方法 | 功能 |
|---|---|
| `shooter_fire()` | 执行一次射击动作 |
| `shooter_reset()` | 关闭摩擦轮并让推弹舵机回初始位置 |
| `shooter_get_state()` | 查询射击模块状态 |

## 10. 旧协议说明

工程中还保留以下通信文件：

- `Code/ChaseNum1_UnifiedProtocol.c/.h`：旧 UART3 统一协议，版本号 `0x02`，包含系统、底盘 RUN、机械臂、文本回包等功能。当前 `Code/ChaseNum1_COM.c` 已切到 NANO UART2。
- `Code/ChaseNum1_COM.c` 中的 VOFA 解析：用于串口助手电机 PID 调试；当 `NANO_PROTOCOL_UART_SELECT` 选择 UART7 时，UART7 会作为 NANO 协议入口。

当前推荐上位机只使用 NANO 协议，并让上位机串口选择与 `NANO_PROTOCOL_UART_SELECT` 保持一致，避免多套协议并行造成状态不一致。
