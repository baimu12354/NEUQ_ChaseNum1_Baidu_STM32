# ChaseNum1_COM.c 新增函数说明报告

本文档说明 `Code/ChaseNum1_COM.c` 中本次为底盘测试协议新增或重新实现的函数。每个函数都给出：

- 函数职责
- 下位机触发方式或执行结果
- 对应上位机发送或接收方式
- 具体十六进制帧示例

说明：文档中的 `SEQ` 是示例序号。实际运行时，上位机 `ChassisController.seq` 和下位机 `Main_COM_seq` 分别独立递增，因此实车抓包时序号可能不同。

## 1. 协议基础

### 1.1 串口参数

UART3：`115200 / 8N1 / no flow control`

### 1.2 通用帧格式

| 字段 | 长度 | 说明 |
| --- | ---: | --- |
| HEAD | 2 | 固定 `AA 55` |
| VERSION | 1 | 固定 `01` |
| SEQ | 1 | 发送端递增序号 |
| CMD | 1 | 命令字或响应字 |
| LEN | 1 | payload 长度，最大 64 |
| PAYLOAD | LEN | 载荷 |
| CHECK | 1 | `VERSION + SEQ + CMD + LEN + PAYLOAD所有字节` 的低 8 位 |
| TAIL | 2 | 固定 `0D 0A` |

### 1.3 常用示例帧

| 方向 | 含义 | 示例帧 |
| --- | --- | --- |
| 上位机 -> 下位机 | HELLO, seq=0 | `AA 55 01 00 01 00 02 0D 0A` |
| 下位机 -> 上位机 | ACK HELLO, seq=0 | `AA 55 01 00 81 02 01 00 85 0D 0A` |
| 下位机 -> 上位机 | STATUS, braking, yaw=30.0, timestamp=1234 | `AA 55 01 01 83 15 00 00 00 F0 41 00 00 00 00 00 00 00 00 00 00 00 00 D2 04 00 00 A1 0D 0A` |
| 上位机 -> 下位机 | BRAKE, seq=1 | `AA 55 01 01 02 00 04 0D 0A` |
| 上位机 -> 下位机 | RUN, seq=2, vx=1.5, vy=10.0, wz=20.0 | `AA 55 01 02 03 0C 00 00 C0 3F 00 00 20 41 00 00 A0 41 53 0D 0A` |
| 下位机 -> 上位机 | STATUS, running, yaw=15.0, vx=1.5, vy=10.0, wz=20.0, timestamp=2500 | `AA 55 01 04 83 15 01 00 00 70 41 00 00 C0 3F 00 00 20 41 00 00 A0 41 C4 09 00 00 5D 0D 0A` |

对应上位机调用：

```python
from chassis_controller import ChassisController

with ChassisController() as chassis:
    chassis.hello()
    chassis.run(vx=1.5, vy=10.0, wz_deg_s=20.0)
    status = chassis.get_status()
    chassis.brake()
```

## 2. 全局状态变量

| 变量 | 作用 |
| --- | --- |
| `Package_get` | 串口中断拼出的完整命令包，主循环读取后执行 |
| `Main_COM_seq` | 下位机发送帧时使用的递增序号 |
| `Main_COM_motion_active` | 当前是否处于上位机运动控制状态 |
| `Main_COM_last_motion_ms` | 最近一次收到有效 `RUN` 命令的时间 |
| `Main_COM_last_feedback_ms` | 最近一次周期状态反馈时间 |
| `Main_COM_feedback_interval_ms` | 状态反馈周期，0 表示关闭 |
| `Main_COM_current_mode` | 当前底盘模式：`Movement_Breaking` 或 `Movement_Running` |
| `Main_COM_package_ready` | 串口中断与主循环之间的完整包就绪标志 |
| `Main_COM_rx_state` | 字节接收状态机当前状态 |
| `Main_COM_rx_package` | 正在拼接的接收包 |
| `Main_COM_rx_payload_index` | 当前 payload 接收位置 |
| `Main_COM_rx_seq` | 当前接收帧的上位机序号，当前只保存不回显 |
| `Main_COM_rx_check` | 接收帧校验和累计值 |

## 3. 函数详细说明

### 3.1 `static uint32_t Main_COM_now_ms(void)`

功能：把系统微秒计数 `systime_us` 转成毫秒，供状态反馈时间戳、运动命令超时和周期反馈判断使用。

下位机示例：

- 若 `systime_us = 1234000`，函数返回 `1234`。
- `Main_COM_send_status()` 会把 `1234` 写入 STATUS payload 末尾，little-endian 字节为 `D2 04 00 00`。

下位机发送示例：

```text
AA 55 01 01 83 15 00 00 00 F0 41 00 00 00 00 00 00 00 00 00 00 00 00 D2 04 00 00 A1 0D 0A
```

上位机对应处理：

```python
status = chassis.get_status()
print(status.timestamp_ms)  # 1234
```

上位机解析位置：`ChassisController._parse_status()` 使用 `struct.unpack("<I", payload[17:21])`。

### 3.2 `static void Main_COM_reset_rx(void)`

功能：清空当前接收状态机，重新等待下一帧头 `AA`。

具体操作：

- `Main_COM_rx_state = Main_COM_RX_HEAD1`
- `Main_COM_rx_payload_index = 0`
- `Main_COM_rx_check = 0`
- 清空 `Main_COM_rx_package`

下位机触发示例：

- 一帧接收完成后，调用该函数回到初始状态。
- 接收错误后，也会通过 `Main_COM_restart_rx()` 间接调用。

上位机对应场景：

```python
# 上位机发送一帧完整 HELLO
chassis.hello()
```

对应发送帧：

```text
AA 55 01 00 01 00 02 0D 0A
```

下位机收到并处理完该帧后，调用 `Main_COM_reset_rx()`，准备接收下一帧，例如 BRAKE：

```text
AA 55 01 01 02 00 04 0D 0A
```

### 3.3 `static void Main_COM_restart_rx(uint8_t dat)`

功能：接收出错时重启接收状态机，同时处理“当前错误字节可能正好是新帧头”的情况。

逻辑：

- 先调用 `Main_COM_reset_rx()`。
- 如果当前字节 `dat == 0xAA`，则立即进入 `Main_COM_RX_HEAD2`，等待第二帧头 `0x55`。

下位机接收示例：

```text
00 FF AA 55 01 00 01 00 02 0D 0A
```

前两个噪声字节会被丢弃；当读到 `AA` 时，状态机保留它作为新帧头，继续等待 `55`。

上位机对应行为：

```python
chassis._write_frame(0x01, b"")
```

正确 HELLO 帧：

```text
AA 55 01 00 01 00 02 0D 0A
```

如果线路噪声导致下位机丢帧，上位机侧应在超时后重发命令。

### 3.4 `static uint8_t Main_COM_calc_check(...)`

功能：计算协议校验和。

计算方式：

```text
CHECK = (VERSION + SEQ + CMD + LEN + sum(PAYLOAD)) & 0xFF
```

下位机示例 1：HELLO

```text
VERSION=01, SEQ=00, CMD=01, LEN=00
CHECK = 01 + 00 + 01 + 00 = 02
完整帧 = AA 55 01 00 01 00 02 0D 0A
```

下位机示例 2：RUN `vx=1.5, vy=10.0, wz=20.0`

payload：

```text
00 00 C0 3F 00 00 20 41 00 00 A0 41
```

完整帧：

```text
AA 55 01 02 03 0C 00 00 C0 3F 00 00 20 41 00 00 A0 41 53 0D 0A
```

上位机对应代码：

```python
check = (version + seq + cmd + length + sum(payload)) & 0xFF
```

该逻辑在 `ChassisController._write_frame()` 和 `_pop_frame()` 中分别用于发送与接收校验。

### 3.5 `static uint16_t Main_COM_get_u16_le(const uint8_t *src)`

功能：从 little-endian 字节数组中读取 `uint16_t`。

主要用于解析 `SET_FEEDBACK` 的反馈周期。

下位机解析示例：

```text
payload = 64 00
interval_ms = 0x0064 = 100
```

上位机发送示例：

```python
chassis.set_feedback_interval(100)
```

对应上位机发送帧：

```text
AA 55 01 05 06 02 64 00 72 0D 0A
```

下位机应答：

```text
ACK SET_FEEDBACK = AA 55 01 05 81 02 06 00 8F 0D 0A
```

说明：ACK 的 `SEQ` 由下位机当前 `Main_COM_seq` 决定，示例中假设为 `05`。

### 3.6 `static void Main_COM_put_u32_le(uint8_t *dst, uint32_t value)`

功能：把 `uint32_t` 按 little-endian 写入 payload。

主要用于 STATUS 中的 `timestamp_ms`。

下位机编码示例：

```text
value = 1234 = 0x000004D2
payload bytes = D2 04 00 00
```

下位机 STATUS 示例：

```text
AA 55 01 01 83 15 00 00 00 F0 41 00 00 00 00 00 00 00 00 00 00 00 00 D2 04 00 00 A1 0D 0A
```

上位机对应解析：

```python
timestamp_ms = struct.unpack("<I", payload[17:21])[0]
```

### 3.7 `static float Main_COM_get_float_le(const uint8_t *src)`

功能：从 little-endian IEEE-754 float32 字节中读取 `float`。

主要用于解析：

- `RUN`: `vx, vy, wz_deg_s`
- `SET_YAW`: `yaw_deg`

下位机解析示例：

```text
00 00 C0 3F -> 1.5
00 00 20 41 -> 10.0
00 00 A0 41 -> 20.0
```

上位机发送：

```python
chassis.run(vx=1.5, vy=10.0, wz_deg_s=20.0)
```

对应上位机发送帧：

```text
AA 55 01 02 03 0C 00 00 C0 3F 00 00 20 41 00 00 A0 41 53 0D 0A
```

下位机执行结果：

```c
movement.Car_X_vel = 1.5f;
movement.Car_Y_vel = 10.0f;
movement.Car_angvel = 20.0f;
CN_set_movement(movement, Movement_Running);
```

### 3.8 `static void Main_COM_put_float_le(uint8_t *dst, float value)`

功能：把 `float` 按 little-endian IEEE-754 float32 写入 payload。

主要用于 STATUS 中的：

- `yaw_deg`
- `vx`
- `vy`
- `wz_deg_s`

下位机编码示例：

```text
yaw=15.0 -> 00 00 70 41
vx=1.5 -> 00 00 C0 3F
vy=10.0 -> 00 00 20 41
wz=20.0 -> 00 00 A0 41
```

下位机发送 STATUS：

```text
AA 55 01 04 83 15 01 00 00 70 41 00 00 C0 3F 00 00 20 41 00 00 A0 41 C4 09 00 00 5D 0D 0A
```

上位机解析：

```python
status = chassis.get_status()
print(status.yaw_deg, status.vx, status.vy, status.wz_deg_s)
```

### 3.9 `static float Main_COM_limit_float(float value, float max_abs)`

功能：对上位机输入速度进行限幅，并过滤 NaN。

规则：

- `value` 是 NaN 时返回 `0.0`
- 超过 `+max_abs` 时返回 `+max_abs`
- 低于 `-max_abs` 时返回 `-max_abs`
- 范围内原样返回

下位机示例：

```text
MAIN_COM_MAX_VX = 50.0
上位机原始 vx = 100.0
下位机实际 movement.Car_X_vel = 50.0
```

上位机对应：

```python
# 使用 ChassisController 时，上位机也会先做一次同样限幅
chassis.run(vx=100.0, vy=0.0, wz_deg_s=0.0)
```

对应结果：

- 上位机 controller 会先把 `vx` 限为 `50.0` 再发出。
- 如果上位机不用 controller，直接发 `100.0`，下位机仍会在 `Main_COM_apply_run()` 中限为 `50.0`。

### 3.10 `static void Main_COM_send_ack(uint8_t cmd)`

功能：发送 ACK 响应，表示命令已被下位机接收并执行。

ACK payload 格式：

| 字节 | 说明 |
| ---: | --- |
| 0 | 原始命令字 |
| 1 | 错误码，ACK 固定为 `00` |

下位机应答示例：收到 HELLO 后发送 ACK。

上位机发送：

```text
AA 55 01 00 01 00 02 0D 0A
```

下位机应答：

```text
AA 55 01 00 81 02 01 00 85 0D 0A
```

上位机对应处理：

```python
frame = chassis.read_frame()
# frame -> (0x81, b\"\\x01\\x00\")
```

说明：当前 `hello()` / `run()` / `brake()` 这类高级接口会继续等待 STATUS，因此通常不会把 ACK 返回给用户层。

### 3.11 `static void Main_COM_send_error(uint8_t cmd, uint8_t err)`

功能：发送错误响应，表示命令错误、payload 长度错误或控制超时。

ERR payload 格式：

| 字节 | 说明 |
| ---: | --- |
| 0 | 原始命令字 |
| 1 | 错误码 |

错误码：

| 错误码 | 含义 |
| ---: | --- |
| `01` | 未知命令 |
| `02` | payload 长度错误 |
| `05` | 运动命令超时 |

示例 1：上位机发送未知命令 `0x99`

上位机发送：

```text
AA 55 01 05 99 00 9F 0D 0A
```

下位机应答：

```text
AA 55 01 05 82 02 99 01 24 0D 0A
```

示例 2：上位机发送错误长度 RUN

上位机发送：

```text
AA 55 01 06 03 01 00 0B 0D 0A
```

下位机应答：

```text
AA 55 01 07 82 02 03 02 91 0D 0A
```

上位机对应处理：

```python
try:
    chassis._write_frame(0x03, b\"\\x00\")
    chassis.get_status()
except RuntimeError as e:
    print(e)
```

### 3.12 `static void Main_COM_apply_brake(void)`

功能：调用底盘刹车模式。

下位机执行：

```c
Car_Movement_struct movement = {0};
CN_set_movement(movement, Movement_Breaking);
Main_COM_current_mode = Movement_Breaking;
Main_COM_motion_active = 0;
```

触发来源：

- `Main_COM_init()` 初始化时默认刹车
- 上位机发送 `BRAKE`
- `RUN` 超时保护触发

上位机发送 BRAKE：

```python
chassis.brake()
```

对应发送帧：

```text
AA 55 01 01 02 00 04 0D 0A
```

下位机应答：

```text
ACK = AA 55 01 02 81 02 02 00 88 0D 0A
STATUS = mode=0, vx=0, vy=0, wz=0
```

### 3.13 `static void Main_COM_apply_run(float vx, float vy, float wz_deg_s)`

功能：调用底盘运行模式，与 `RC_control()` 最终进入的控制入口一致。

下位机执行：

```c
movement.Car_X_vel = limit(vx);
movement.Car_Y_vel = limit(vy);
movement.Car_angvel = limit(wz_deg_s);
CN_set_movement(movement, Movement_Running);
Main_COM_current_mode = Movement_Running;
Main_COM_motion_active = 1;
Main_COM_last_motion_ms = Main_COM_now_ms();
```

上位机发送：

```python
chassis.run(vx=1.5, vy=10.0, wz_deg_s=20.0)
```

对应帧：

```text
AA 55 01 02 03 0C 00 00 C0 3F 00 00 20 41 00 00 A0 41 53 0D 0A
```

下位机应答：

```text
ACK RUN = AA 55 01 03 81 02 03 00 8A 0D 0A
STATUS = AA 55 01 04 83 15 01 00 00 70 41 00 00 C0 3F 00 00 20 41 00 00 A0 41 C4 09 00 00 5D 0D 0A
```

说明：连续运动时上位机应以 10-20 Hz 周期发送 `RUN`。如果超过 `MAIN_COM_CMD_TIMEOUT_MS` 没有新 `RUN`，下位机会自动刹车。

### 3.14 `void Main_COM_init(void)`

功能：初始化上位机通信模块。

下位机执行：

```c
CN_UART3_init();
Main_COM_apply_brake();
Main_COM_reset_rx();
```

调用位置：

```c
// USER/main.c
Main_COM_init();
```

上位机对应启动流程：

```python
with ChassisController() as chassis:
    chassis.hello()
```

上位机发送 HELLO：

```text
AA 55 01 00 01 00 02 0D 0A
```

下位机预期应答：

```text
ACK HELLO + STATUS
```

### 3.15 `void Main_COM_send_frame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)`

功能：下位机通用发帧函数。

负责：

- 填写帧头 `AA 55`
- 填写协议版本
- 填写下位机发送序号
- 填写命令字、长度和 payload
- 计算 CHECK
- 填写帧尾 `0D 0A`
- 调用 `CN_UART_write_fifo(CN_UART_3, frame, index)` 发送

下位机示例：发送 ACK HELLO

```c
uint8_t payload[2] = {0x01, 0x00};
Main_COM_send_frame(0x81, payload, 2);
```

下位机发出：

```text
AA 55 01 00 81 02 01 00 85 0D 0A
```

上位机对应函数：

```python
# 上位机镜像函数
chassis._write_frame(cmd, payload)
```

上位机读下位机帧：

```python
cmd, payload = chassis.read_frame()
```

### 3.16 `void Main_COM_send_status(void)`

功能：发送当前底盘状态和航向角反馈。

STATUS payload 长度固定 21 字节：

| 偏移 | 类型 | 字段 |
| ---: | --- | --- |
| 0 | uint8 | mode |
| 1-4 | float | yaw_deg |
| 5-8 | float | vx |
| 9-12 | float | vy |
| 13-16 | float | wz_deg_s |
| 17-20 | uint32 | timestamp_ms |

下位机状态示例：

```text
mode = 1
yaw = 15.0
vx = 1.5
vy = 10.0
wz = 20.0
timestamp = 2500
```

下位机发送：

```text
AA 55 01 04 83 15 01 00 00 70 41 00 00 C0 3F 00 00 20 41 00 00 A0 41 C4 09 00 00 5D 0D 0A
```

上位机请求：

```python
status = chassis.get_status()
```

上位机发送 GET_STATUS：

```text
AA 55 01 03 04 00 08 0D 0A
```

上位机解析结果：

```python
ChassisStatus(mode=1, yaw_deg=15.0, vx=1.5, vy=10.0, wz_deg_s=20.0, timestamp_ms=2500)
```

### 3.17 `static void Main_COM_process_package(const Main_COM_Package_t *package)`

功能：主命令分发器。它根据 `package->CMD` 调用不同业务逻辑。

#### 3.17.1 HELLO

上位机发送：

```python
chassis.hello()
```

帧：

```text
AA 55 01 00 01 00 02 0D 0A
```

下位机动作：

- 检查 payload 长度必须为 0
- 发送 ACK
- 发送 STATUS

#### 3.17.2 BRAKE

上位机发送：

```python
chassis.brake()
```

帧：

```text
AA 55 01 01 02 00 04 0D 0A
```

下位机动作：

- 调用 `Main_COM_apply_brake()`
- 发送 ACK
- 发送 STATUS

#### 3.17.3 RUN

上位机发送：

```python
chassis.run(1.5, 10.0, 20.0)
```

帧：

```text
AA 55 01 02 03 0C 00 00 C0 3F 00 00 20 41 00 00 A0 41 53 0D 0A
```

下位机动作：

- 检查 payload 长度为 12
- 解析三个 float
- 调用 `Main_COM_apply_run(vx, vy, wz)`
- 发送 ACK
- 发送 STATUS

#### 3.17.4 GET_STATUS

上位机发送：

```python
chassis.get_status()
```

帧：

```text
AA 55 01 03 04 00 08 0D 0A
```

下位机动作：

- 直接调用 `Main_COM_send_status()`

#### 3.17.5 SET_YAW

上位机发送：

```python
chassis.set_yaw(90.0)
```

帧：

```text
AA 55 01 04 05 04 00 00 B4 42 04 0D 0A
```

下位机动作：

- 解析 `yaw_deg=90.0`
- 调用 `CN_Set_YAW(90.0)`
- 发送 ACK
- 发送 STATUS

可能的 STATUS 示例：

```text
AA 55 01 09 83 15 00 00 00 B4 42 00 00 00 00 00 00 00 00 00 00 00 00 B8 0B 00 00 5B 0D 0A
```

#### 3.17.6 SET_FEEDBACK

上位机发送：

```python
chassis.set_feedback_interval(100)
```

帧：

```text
AA 55 01 05 06 02 64 00 72 0D 0A
```

下位机动作：

- 解析 `interval_ms=100`
- 若为 0，关闭周期状态反馈
- 若小于最小值，钳制为 `MAIN_COM_FEEDBACK_MIN_MS`
- 若大于最大值，钳制为 `MAIN_COM_FEEDBACK_MAX_MS`
- 发送 ACK
- 发送 STATUS

关闭反馈示例：

```python
chassis.set_feedback_interval(0)
```

帧：

```text
AA 55 01 08 06 02 00 00 11 0D 0A
```

#### 3.17.7 未知命令

上位机错误发送：

```text
AA 55 01 05 99 00 9F 0D 0A
```

下位机应答：

```text
AA 55 01 05 82 02 99 01 24 0D 0A
```

### 3.18 `void Main_COM_Response_CALLBACK(void)`

功能：主循环中周期调用的通信处理函数。

调用位置：

```c
while (1) {
    Main_COM_Response_CALLBACK();
}
```

它做三件事：

1. 如果串口中断拼出了完整包，则复制 `Package_get` 并调用 `Main_COM_process_package()`。
2. 如果底盘处于运动状态，且超过 `MAIN_COM_CMD_TIMEOUT_MS` 没有新 RUN，则自动刹车并发送 ERR。
3. 如果周期反馈开启，且到达反馈周期，则发送 STATUS。

上位机发送 RUN 后，下位机主循环处理示例：

```text
上位机 -> 下位机:
AA 55 01 02 03 0C 00 00 C0 3F 00 00 20 41 00 00 A0 41 53 0D 0A

下位机 -> 上位机:
ACK RUN
STATUS
```

超时示例：

如果上位机发送一次 RUN 后超过 500 ms 不再发送 RUN，下位机发送：

```text
AA 55 01 06 82 02 03 05 93 0D 0A
```

随后发送刹车后的 STATUS。

上位机建议：

```python
# 推荐连续运动方式，周期重发 RUN，结束后自动 brake
chassis.run_for(vx=0.0, vy=10.0, wz_deg_s=0.0, duration_s=2.0, rate_hz=20)
```

### 3.19 `void CN_send_Response(Main_CMD_ENUM CMD, uint8_t *response)`

功能：兼容旧代码中调用 `CN_send_Response()` 的位置。当前实现忽略 `response` 参数，直接把 `CMD` 作为协议帧命令字发送一个空 payload 帧。

下位机示例：

```c
CN_send_Response(Main_CMD_STARTSERVER, NULL);
```

可能发送：

```text
AA 55 01 xx E8 00 CHECK 0D 0A
```

说明：

- `xx` 是当前下位机发送序号。
- `CHECK = 01 + xx + E8 + 00` 的低 8 位。
- 这不是本次底盘测试协议推荐的新接口，新的协议发送应优先使用 `Main_COM_send_frame()`、`Main_COM_send_ack()`、`Main_COM_send_error()`、`Main_COM_send_status()`。

上位机对应：

```python
cmd, payload = chassis.read_frame()
# cmd 可能是 0xE8，payload 为空
```

### 3.20 `void Main_COM_RX_CALLBACK(uint8_t dat)`

功能：UART3 单字节接收状态机。它不直接执行运动命令，只负责把字节拼成完整包。

接收流程：

1. 等待 `AA`
2. 等待 `55`
3. 检查 `VERSION`
4. 读取 `SEQ`
5. 读取 `CMD`
6. 读取 `LEN`
7. 读取 payload
8. 校验 CHECK
9. 检查帧尾 `0D 0A`
10. 完整包写入 `Package_get`，置位 `Main_COM_package_ready`

上位机发送 RUN：

```text
AA 55 01 02 03 0C 00 00 C0 3F 00 00 20 41 00 00 A0 41 53 0D 0A
```

下位机逐字节调用：

```c
Main_COM_RX_CALLBACK(0xAA);
Main_COM_RX_CALLBACK(0x55);
Main_COM_RX_CALLBACK(0x01);
...
Main_COM_RX_CALLBACK(0x0A);
```

收到最后一个字节后：

```c
Package_get.CMD = MAIN_COM_CMD_RUN;
Package_get.load_len = 12;
Package_get.load[0..11] = RUN payload;
Main_COM_package_ready = 1;
```

上位机对应函数：

```python
chassis._write_frame(0x03, struct.pack("<fff", 1.5, 10.0, 20.0))
```

错误场景：

- CHECK 错误：下位机丢弃该帧并重新找帧头。
- 当前实现不会对 CHECK 错误主动回包，因为错误帧可能已经不可信。
- 上位机应靠超时重发。

### 3.21 `void CN_UART3_RX_CALLBACK(uint8_t dat)`

功能：UART3 HAL 接收回调适配层。底层 `HAL_UART_RxCpltCallback()` 收到 UART3 字节后会调用该函数，它再转给 `Main_COM_RX_CALLBACK()`。

下位机调用链：

```text
USART3 IRQ
-> HAL_UART_RxCpltCallback()
-> CN_UART3_RX_CALLBACK(dat)
-> Main_COM_RX_CALLBACK(dat)
```

上位机对应：

```python
chassis.run(1.5, 10.0, 20.0)
```

每个发送出的串口字节都会在下位机侧经过 `CN_UART3_RX_CALLBACK()`。

### 3.22 `void CN_UART1_RX_CALLBACK(uint8_t dat)`

说明：该函数属于 UART1 的 VOFA 电机调参接收链路，不是本次 UART3 上下位机底盘测试协议的新增功能，但它仍保留在 `ChaseNum1_COM.c` 中。

功能：把 UART1 收到的字节交给 `VOFA_Single_Byte_Process()`。

下位机调用链：

```text
USART1 IRQ
-> HAL_UART_RxCpltCallback()
-> CN_UART1_RX_CALLBACK(dat)
-> VOFA_Single_Byte_Process(dat)
```

VOFA 上位机发送示例：设置 `Kp_angle = 0.5`

```text
AA 55 01 00 00 00 3F 5A A5
```

说明：

- `01` 是参数 ID，代表 `Kp_angle`
- `00 00 00 3F` 是 float32 `0.5`
- 该 VOFA 帧没有本次底盘协议的 ACK/STATUS 应答

### 3.23 `void VOFA_Single_Byte_Process(uint8_t rx_byte)`

说明：该函数也是原有 VOFA 调参链路，不属于 UART3 上位机底盘测试协议。

功能：解析固定 9 字节 VOFA 参数帧：

| 字节 | 说明 |
| ---: | --- |
| 0 | `AA` |
| 1 | `55` |
| 2 | 参数 ID |
| 3-6 | float32 参数值，little-endian |
| 7 | `5A` |
| 8 | `A5` |

参数 ID 映射：

| ID | 参数 |
| ---: | --- |
| 0 | `motor_tune.target_angle` |
| 1 | `motor_tune.Kp_angle` |
| 2 | `motor_tune.Ki_angle` |
| 3 | `motor_tune.Kd_angle` |
| 4 | `motor_tune.target_vel` |
| 5 | `motor_tune.Kp_vel` |
| 6 | `motor_tune.Ki_vel` |
| 7 | `motor_tune.Kd_vel` |

示例：设置 `target_vel = 10.0`

VOFA 上位机发送：

```text
AA 55 04 00 00 20 41 5A A5
```

下位机结果：

```c
motor_tune.target_vel = 10.0f;
param_updated_flag = true;
```

该链路没有下位机协议应答；调参软件侧通常通过观察调参效果或调试输出确认。

## 4. 上位机与下位机命令对照

| 上位机方法 | 上位机 CMD | 下位机处理函数 | 下位机底盘动作 | 下位机应答 |
| --- | ---: | --- | --- | --- |
| `hello()` | `0x01` | `Main_COM_process_package()` | 不改变底盘，仅确认在线 | ACK + STATUS |
| `brake()` | `0x02` | `Main_COM_apply_brake()` | `Movement_Breaking` | ACK + STATUS |
| `run(vx, vy, wz)` | `0x03` | `Main_COM_apply_run()` | `Movement_Running` | ACK + STATUS |
| `get_status()` | `0x04` | `Main_COM_send_status()` | 不改变底盘 | STATUS |
| `set_yaw(yaw)` | `0x05` | `CN_Set_YAW()` | 设置逻辑航向角偏置 | ACK + STATUS |
| `set_feedback_interval(ms)` | `0x06` | 修改 `Main_COM_feedback_interval_ms` | 设置周期反馈 | ACK + STATUS |

## 5. 典型收发流程

### 5.1 上电握手

上位机：

```python
status = chassis.hello()
```

上位机发送：

```text
AA 55 01 00 01 00 02 0D 0A
```

下位机应答：

```text
AA 55 01 00 81 02 01 00 85 0D 0A
AA 55 01 01 83 15 ... CHECK 0D 0A
```

### 5.2 连续前进 2 秒

上位机：

```python
chassis.run_for(vx=0.0, vy=10.0, wz_deg_s=0.0, duration_s=2.0, rate_hz=20)
```

上位机会以 20 Hz 周期发送 RUN，最后发送 BRAKE。这样不会触发下位机 500 ms 超时刹车。

### 5.3 请求航向角反馈

上位机：

```python
status = chassis.get_status()
print(status.yaw_deg)
```

上位机发送：

```text
AA 55 01 03 04 00 08 0D 0A
```

下位机应答 STATUS，其中 payload 第 1-4 字节是 `yaw_deg`。

### 5.4 设置逻辑航向角

上位机：

```python
chassis.set_yaw(90.0)
```

上位机发送：

```text
AA 55 01 04 05 04 00 00 B4 42 04 0D 0A
```

下位机执行：

```c
CN_Set_YAW(90.0f);
```

下位机随后 ACK 并发送 STATUS。STATUS 中 `yaw_deg` 应接近 90.0，具体取决于 IMU 当前读数与偏置计算。

## 6. 注意事项

1. `Main_COM_RX_CALLBACK()` 在中断上下文中被间接调用，只拼包，不执行运动控制，这是为了避免在串口中断里直接修改底盘状态。
2. `Main_COM_Response_CALLBACK()` 必须在 `while(1)` 中持续调用，否则完整包不会被执行，周期反馈和超时保护也不会运行。
3. `RUN` 是速度控制命令，不是一次性位移任务。连续运动必须周期发送。
4. 当前校验错误帧会被下位机丢弃，不会主动回复 `ERR_BAD_CHECK`。
5. `CN_send_Response()` 只是兼容旧调用的简单包装，不建议作为新协议主发送接口。
6. UART1 的 VOFA 调参函数和 UART3 上下位机底盘协议相互独立。
