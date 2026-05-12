# 上位机视角底盘通信协议报告

本文只从上位机角度说明如何调用下位机底盘测试协议：如何组包、发送、接收、解析，以及如何使用 `ChassisController` 类和 `communication.yml`。

当前测试阶段建议先只做通信测试，不上车跑。通信测试脚本为 `test_communication.py`，默认不会发送有速度的 RUN 命令。

## 1. 文件说明

| 文件 | 用途 |
| --- | --- |
| `communication.yml` | 上位机通信参数、协议常量、命令字、限幅参数和 RC 映射参数 |
| `chassis_controller.py` | 可直接调用的上位机 controller 类 |
| `test_communication.py` | 上下位机通信自测脚本，打印 TX/RX 原始帧和解析结果 |
| `Upper_Controller_Protocol_Report.md` | 本报告 |

## 2. 上位机需要知道的协议结构

### 2.1 串口参数

默认来自 `communication.yml`：

```yaml
serial:
  port: COM3
  baudrate: 115200
  bytesize: 8
  parity: N
  stopbits: 1
  timeout_s: 0.05
  write_timeout_s: 0.05
```

### 2.2 帧格式

上位机发送和接收都使用同一帧格式：

```text
AA 55 VERSION SEQ CMD LEN PAYLOAD CHECK 0D 0A
```

| 字段 | 长度 | 上位机处理 |
| --- | ---: | --- |
| `AA 55` | 2 | 帧头，固定值 |
| `VERSION` | 1 | 当前固定 `01` |
| `SEQ` | 1 | 上位机发送时自增；接收下位机帧时只用于显示和调试 |
| `CMD` | 1 | 命令字或响应字 |
| `LEN` | 1 | payload 长度 |
| `PAYLOAD` | LEN | 命令参数或响应数据 |
| `CHECK` | 1 | `VERSION + SEQ + CMD + LEN + PAYLOAD` 低 8 位 |
| `0D 0A` | 2 | 帧尾，固定值 |

所有多字节数据为 little-endian；所有 float 为 IEEE-754 float32。

### 2.3 上位机组包示例

以 `RUN(vx=1.5, vy=10.0, wz=20.0)` 为例：

```text
vx=1.5   -> 00 00 C0 3F
vy=10.0  -> 00 00 20 41
wz=20.0  -> 00 00 A0 41
payload  -> 00 00 C0 3F 00 00 20 41 00 00 A0 41
```

完整发送帧：

```text
AA 55 01 02 03 0C 00 00 C0 3F 00 00 20 41 00 00 A0 41 53 0D 0A
```

在 controller 中：

```python
chassis.run(vx=1.5, vy=10.0, wz_deg_s=20.0)
```

## 3. `communication.yml` 参数清单

这些参数都已经外置，便于你决定保留为配置项还是硬编码到代码中。

### 3.1 建议保留在 YAML 的参数

| 参数 | 原因 |
| --- | --- |
| `serial.port` | 不同电脑串口号经常变化 |
| `serial.timeout_s` | 调试阶段需要调整等待窗口 |
| `serial.write_timeout_s` | 不同 USB 转串口可能表现不同 |
| `limits.max_vx/max_vy/max_wz_deg_s` | 早期测试时可能要保守限速 |
| `limits.command_timeout_ms` | 与下位机超时保护对应，测试阶段要明确可见 |
| `limits.default_feedback_interval_ms` | 状态反馈频率常需调整 |
| `rc_mapping` | 如果上位机需要模拟遥控器通道输入，这些比例方便调 |

### 3.2 可以考虑硬编码的参数

| 参数 | 原因 |
| --- | --- |
| `protocol.header` | 协议帧头一般不变 |
| `protocol.tail` | 协议帧尾一般不变 |
| `protocol.version` | 单版本测试时不需要频繁改 |
| `protocol.endian` | 与下位机 C 实现绑定 |
| `protocol.float_format` | 与下位机 C float 解析绑定 |
| `commands` | 命令字变动会影响上下位机一致性，稳定后可硬编码 |
| `responses` | 响应字同上，稳定后可硬编码 |

### 3.3 当前配置内容

```yaml
commands:
  hello: 0x01
  brake: 0x02
  run: 0x03
  get_status: 0x04
  set_yaw: 0x05
  set_feedback: 0x06

responses:
  ack: 0x81
  error: 0x82
  status: 0x83
```

## 4. Controller 类使用说明

### 4.1 初始化与关闭

```python
from chassis_controller import ChassisController

with ChassisController("communication.yml") as chassis:
    status = chassis.hello()
    print(status)
```

如果 `communication.yml` 与 `chassis_controller.py` 在同一目录，也可以不传路径：

```python
with ChassisController() as chassis:
    chassis.hello()
```

### 4.2 覆盖串口号

```python
chassis = ChassisController()
chassis.config["serial"]["port"] = "COM5"
chassis.open()
```

测试脚本也支持：

```bash
python test_communication.py --port COM5
```

### 4.3 查看原始 TX/RX 帧

```python
tx = chassis.send_command("hello")
print(chassis.frame_to_hex(tx))

frames = chassis.collect_frames(0.5)
for frame in frames:
    print(frame.seq, frame.cmd, chassis.frame_to_hex(frame))
```

## 5. 协议包含的所有功能及上位机实例

### 5.1 HELLO：通信握手

功能：确认下位机在线，并获取一次 STATUS。

上位机调用：

```python
status = chassis.hello()
print(status)
```

上位机发送帧示例：

```text
AA 55 01 00 01 00 02 0D 0A
```

下位机预期应答：

```text
ACK:    AA 55 01 00 81 02 01 00 85 0D 0A
STATUS: AA 55 01 01 83 15 ... CHECK 0D 0A
```

上位机解析结果：

```python
ChassisStatus(mode=0, yaw_deg=..., vx=0.0, vy=0.0, wz_deg_s=0.0, timestamp_ms=...)
```

### 5.2 BRAKE：刹车

功能：要求下位机进入 `Movement_Breaking`。通信测试或任何异常后都建议发送一次。

上位机调用：

```python
status = chassis.brake()
```

上位机发送帧示例：

```text
AA 55 01 01 02 00 04 0D 0A
```

下位机预期应答：

```text
ACK:    AA 55 01 02 81 02 02 00 88 0D 0A
STATUS: mode=0, vx=0, vy=0, wz=0
```

### 5.3 RUN：速度运行

功能：发送底盘速度命令，对应下位机 `Movement_Running`。

参数：

| 参数 | 类型 | 含义 |
| --- | --- | --- |
| `vx` | float | 车体横向速度，右为正 |
| `vy` | float | 车体纵向速度，前为正 |
| `wz_deg_s` | float | 航向角速度，单位沿用下位机当前实现 |

上位机调用：

```python
status = chassis.run(vx=1.5, vy=10.0, wz_deg_s=20.0)
```

上位机发送帧示例：

```text
AA 55 01 02 03 0C 00 00 C0 3F 00 00 20 41 00 00 A0 41 53 0D 0A
```

下位机预期应答：

```text
ACK:    AA 55 01 03 81 02 03 00 8A 0D 0A
STATUS: mode=1, vx=1.5, vy=10.0, wz=20.0
```

连续运行示例：

```python
chassis.run_for(vx=0.0, vy=10.0, wz_deg_s=0.0, duration_s=2.0, rate_hz=20)
```

注意：`RUN` 是速度命令，不是持续任务。下位机有 500 ms 超时保护，连续运动必须周期发送。纯通信测试阶段不要发送非零速度。

### 5.4 GET_STATUS：读取状态

功能：请求一次下位机状态，包含航向角反馈。

上位机调用：

```python
status = chassis.get_status()
print(status.yaw_deg)
```

上位机发送帧示例：

```text
AA 55 01 03 04 00 08 0D 0A
```

下位机预期应答：

```text
STATUS: AA 55 01 04 83 15 01 00 00 70 41 00 00 C0 3F 00 00 20 41 00 00 A0 41 C4 09 00 00 5D 0D 0A
```

该 STATUS 示例解析为：

```python
ChassisStatus(
    mode=1,
    yaw_deg=15.0,
    vx=1.5,
    vy=10.0,
    wz_deg_s=20.0,
    timestamp_ms=2500,
)
```

### 5.5 SET_YAW：设置逻辑航向角

功能：设置下位机逻辑航向角偏置。上位机后续读取的 `yaw_deg` 会以这个设定为参考。

上位机调用：

```python
status = chassis.set_yaw(90.0)
```

上位机发送帧示例：

```text
AA 55 01 04 05 04 00 00 B4 42 04 0D 0A
```

下位机预期应答：

```text
ACK:    original_cmd=0x05, error=0x00
STATUS: yaw_deg 接近 90.0
```

### 5.6 SET_FEEDBACK：设置周期状态反馈

功能：让下位机按周期主动发送 STATUS；`0` 表示关闭周期反馈。

上位机调用：

```python
status = chassis.set_feedback_interval(100)
```

上位机发送帧示例：

```text
AA 55 01 05 06 02 64 00 72 0D 0A
```

关闭周期反馈：

```python
chassis.set_feedback_interval(0)
```

发送帧：

```text
AA 55 01 08 06 02 00 00 11 0D 0A
```

监听周期反馈：

```python
def on_status(status):
    print(status)

chassis.set_feedback_interval(100)
chassis.listen_status(on_status, duration_s=2.0)
chassis.set_feedback_interval(0)
```

### 5.7 错误响应解析

下位机错误响应 CMD 为 `0x82`，payload 为：

```text
original_cmd error_code
```

常见错误：

| error_code | 含义 |
| ---: | --- |
| `0x01` | 未知命令 |
| `0x02` | payload 长度错误 |
| `0x05` | RUN 超时，已自动刹车 |

上位机原始读取：

```python
frames = chassis.collect_frames(0.5)
for frame in frames:
    parsed = chassis.parse_frame_payload(frame)
    print(parsed)
```

错误帧示例：

```text
AA 55 01 06 82 02 03 05 93 0D 0A
```

解析为：

```python
ChassisAck(original_cmd=0x03, error=0x05)
```

含义：RUN 命令超时，下位机已经自动刹车。

## 6. 通信自测脚本

脚本路径：

```text
HostController/test_communication.py
```

### 6.1 安装依赖

```bash
pip install -r requirements.txt
```

依赖：

```text
pyserial
PyYAML
```

### 6.2 默认安全测试

默认测试不发送非零速度，不让车运动：

```bash
python test_communication.py --port COM5
```

默认执行：

1. 打开串口
2. 发送 HELLO
3. 请求 GET_STATUS
4. 设置周期反馈为 0，也就是关闭周期反馈
5. 发送 BRAKE
6. 打印所有 TX/RX 原始帧和解析结果

示例输出形态：

```text
Opening COM5 at 115200 baud

TX hello: AA 55 01 00 01 00 02 0D 0A
RX seq=00 cmd=0x81 raw=AA 55 ... ACK(original_cmd=0x01, error=0x00)
RX seq=01 cmd=0x83 raw=AA 55 ... STATUS(mode=0, yaw=..., vx=0.000, vy=0.000, wz=0.000, t=...ms)

TX get_status: AA 55 ...
RX seq=.. cmd=0x83 ... STATUS(...)

TX brake: AA 55 ...
RX seq=.. cmd=0x81 ... ACK(...)
RX seq=.. cmd=0x83 ... STATUS(...)

PASS: serial protocol parsed valid responses.
```

### 6.3 测试设置航向角

```bash
python test_communication.py --port COM5 --set-yaw 90
```

会额外发送 `SET_YAW(90.0)` 并检查 STATUS 是否能正常解析。

### 6.4 测试周期反馈

```bash
python test_communication.py --port COM5 --feedback-ms 100
```

脚本会设置 100 ms 周期反馈，然后在后续监听窗口内打印额外收到的 STATUS。测试结束前建议再手动或脚本发送：

```bash
python test_communication.py --port COM5 --feedback-ms 0
```

默认脚本末尾也会发送 BRAKE。

### 6.5 零速度 RUN 通信测试

默认不发送 RUN。若只想测试 RUN 命令解析，可以发送零速度 RUN：

```bash
python test_communication.py --port COM5 --unsafe-run-test
```

该选项当前发送：

```python
RUN(vx=0.0, vy=0.0, wz=0.0)
```

虽然是零速度，仍建议电机输出安全可控时再测试。

## 7. 上位机开发建议

1. 正常业务层优先使用 `hello()`、`brake()`、`run()`、`get_status()`、`set_yaw()`、`set_feedback_interval()`。
2. 调试抓包时使用 `send_command()`、`collect_frames()`、`frame_to_hex()` 和 `parse_frame_payload()`。
3. 通信测试阶段先运行 `test_communication.py --port COMx`，确认 ACK/STATUS 都能解析。
4. 需要动车前，先确认 `brake()` 能正常收到 STATUS，并确认下位机 500 ms 超时自动刹车有效。
5. 连续运动使用 `run_for()` 或自行以 10-20 Hz 调用 `run()`。
