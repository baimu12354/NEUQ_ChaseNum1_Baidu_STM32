# 机械臂上位机通信协议报告

本文档站在上位机角度整理当前机械臂通信协议的全部功能，包括上位机发包、下位机回包、`controller` 类调用方式、调试 GUI 行为以及 `ARM_communication.yml` 中的关键参数。

## 1. 文件分工

| 文件 | 作用 |
| --- | --- |
| `PythonTools/ARM_controller.py` | 上位机通信封装类，负责串口连接、协议组包、发送命令、后台收包、回调分发。 |
| `PythonTools/ARM_communication.yml` | 上位机通信和调试参数配置文件，集中列出串口、协议、默认目标、机械参数、预览限位等参数。 |
| `PythonTools/ARMcom.py` | 上位机调试 GUI，调用 `ArmCommunicationController` 发目标点或急停，并显示下位机文本回传。 |
| `Code/ChaseNum1_MecArm.c/.h` | 下位机协议解析、机械臂逆解、运动规划、舵机执行和文本状态回传。 |

## 2. 坐标与任务语义

上位机当前只需要发送吸盘末端的三维空间坐标和吸盘在目标竖直平面内的朝向。

| 字段 | 类型 | 单位 | 含义 |
| --- | --- | --- | --- |
| `world_x` | `float` | cm | 吸盘末端世界坐标 X。云台虚拟 `90 deg` 指向水平面 `+X`。 |
| `world_y` | `float` | cm | 吸盘末端世界坐标 Y。云台虚拟 `0 deg` 指向水平面 `+Y`。 |
| `world_z` | `float` | cm | 吸盘末端世界坐标 Z。竖直向上为 `+Z`。 |
| `suc_dir_deg` | `float` | deg | 吸盘在目标竖直平面内的绝对朝向，`0 deg` 为竖直向下，`90 deg` 为平面深度正方向，`180 deg` 为竖直向上。 |

坐标原点定义为云台中心在地面上的投影。下位机会根据 `world_x/world_y` 自动计算云台平面角，即：

```text
Y 方向 -> th0 = 0 deg
+X 方向 -> th0 = 90 deg
th0 = atan2(world_x, world_y)
平面深度 plane_x = sqrt(world_x^2 + world_y^2)
```

上位机不再单独发送云台角度，也不需要发送各关节角。下位机会自动完成云台角、吸盘末端回推、二连杆逆解、舵机标定映射、限位检查和必要的跨零规划。

## 3. 串口基础参数

默认参数来自 `ARM_communication.yml`：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `serial.baudrate` | `115200` | 串口波特率。 |
| `serial.timeout_s` | `0.1` | 串口读超时时间，单位秒。 |
| `serial.read_sleep_s` | `0.01` | 后台接收线程空闲睡眠时间，单位秒。 |
| `serial.encoding` | `utf-8` | 下位机文本回传的解码方式。 |
| `serial.print_rx_to_terminal` | `true` | 是否把下位机文本行直接打印到上位机终端。 |
| `serial.rx_print_prefix` | `[MCU] ` | 终端打印下位机文本时使用的前缀。 |

上位机可以通过 `ArmCommunicationController.connect()` 覆盖波特率和超时时间：

```python
arm.connect("COM3", baudrate=115200, timeout=0.1)
```

## 4. 上位机发包格式

所有上位机发往下位机的命令都使用同一种二进制帧格式：

```text
header | cmd | payload | checksum | tail
```

展开后为：

```text
0x5A 0xA5 | cmd | payload | checksum | 0x0D 0x0A
```

| 字段 | 长度 | 说明 |
| --- | --- | --- |
| `header` | 2 字节 | 固定帧头，默认 `0x5A 0xA5`。 |
| `cmd` | 1 字节 | 命令字，目前支持 `0x00` 急停、`0x01` 运动任务和 `0x02` 自动收敛。 |
| `payload` | 0 或 16 字节 | 命令载荷。急停和自动收敛无载荷，运动任务载荷为 4 个小端 `float`。 |
| `checksum` | 1 字节 | 校验和，计算方式为 `(cmd + sum(payload_bytes)) & 0xFF`。 |
| `tail` | 2 字节 | 固定帧尾，默认 `0x0D 0x0A`。 |

校验和不包含帧头和帧尾，只包含 `cmd` 和 `payload`。

## 5. 上位机可发送命令

### 5.1 急停命令

急停命令用于让下位机停止并锁持四个总线舵机。

| 项目 | 内容 |
| --- | --- |
| 命令字 | `0x00` |
| payload | 无 |
| 总帧长 | 6 字节 |
| controller 接口 | `send_stop()` |
| 下位机行为 | 清空当前规划，调用舵机停止保持。 |

帧结构：

```text
5A A5 00 00 0D 0A
```

其中第 4 字节 `00` 是校验和：`(0x00) & 0xFF = 0x00`。

Python 调用：

```python
from ARM_controller import ArmCommunicationController

arm = ArmCommunicationController()
arm.connect("COM3")
packet = arm.send_stop()
print(packet.hex(" "))
arm.close()
```

### 5.2 运动任务命令

运动任务命令用于发送吸盘末端目标。

| 项目 | 内容 |
| --- | --- |
| 命令字 | `0x01` |
| payload | `world_x, world_y, world_z, suc_dir_deg` |
| payload 格式 | `"<ffff"`，即 4 个小端 IEEE754 单精度浮点数 |
| payload 长度 | 16 字节 |
| 总帧长 | 22 字节 |
| controller 接口 | `send_target(world_x, world_y, world_z, suc_dir_deg)` |
| 下位机行为 | 解析目标，逆解和规划后驱动舵机，或返回错误文本。 |

payload 顺序固定为：

```text
float world_x
float world_y
float world_z
float suc_dir_deg
```

Python 调用：

```python
from ARM_controller import ArmCommunicationController

arm = ArmCommunicationController()
arm.connect("COM3")
packet = arm.send_target(0.0, 30.0, 18.0, 0.0)
print(packet.hex(" "))
arm.close()
```

上例表示：

```text
吸盘末端 X = 0.0 cm
吸盘末端 Y = 30.0 cm
吸盘末端 Z = 18.0 cm
吸盘朝向 = 0.0 deg，即竖直向下
```

### 5.3 自动收敛命令

自动收敛命令用于让机械臂回到默认固定收敛姿态，方便上位机调试、重新定位或任务间复位。

| 项目 | 内容 |
| --- | --- |
| 命令字 | `0x02` |
| payload | 无 |
| 总帧长 | 6 字节 |
| controller 接口 | `send_fold()` |
| 下位机行为 | 清空当前规划，先在当前平面收敛，再保持收敛姿态回到默认收敛平面。 |

帧结构：

```text
5A A5 02 02 0D 0A
```

其中第 4 字节 `02` 是校验和：`(0x02) & 0xFF = 0x02`。

Python 调用：

```python
from ARM_controller import ArmCommunicationController

arm = ArmCommunicationController()
arm.connect("COM3")
packet = arm.send_fold()
print(packet.hex(" "))
arm.close()
```

自动收敛的默认云台平面由下位机 `ARM_DEFAULT_FOLD_BASE_DEG` 决定，当前为 `90 deg`。关节收敛角由下位机 `ARM_FOLD_TH1_DEG/ARM_FOLD_TH2_DEG/ARM_FOLD_TH3_DEG` 决定。

### 5.4 自定义命令组包

如果上位机需要只构造数据帧、不立刻发送，可以直接使用：

```python
payload = arm.build_target_payload(0.0, 30.0, 18.0, 0.0)
packet = arm.build_packet(0x01, payload)
```

如果后续协议增加命令字，也可以复用：

```python
packet = arm.send_packet(cmd_type, payload)
```

当前下位机只解析 `0x00`、`0x01` 和 `0x02`。未知命令字会被下位机状态机丢弃，不会生成有效任务。

## 6. 下位机收包规则

下位机 `ARM_RX_Callback(uint8_t dat)` 按字节状态机解析上位机数据。状态顺序为：

```text
等待 0x5A
等待 0xA5
读取 cmd
按 cmd 决定是否读取 payload
校验 checksum
校验 0x0D
校验 0x0A
命令生效
```

解析规则：

| 情况 | 下位机处理 |
| --- | --- |
| 帧头不匹配 | 回到等待帧头。 |
| 命令字为 `0x00` | 不读取 payload，直接等待校验和与帧尾。 |
| 命令字为 `0x01` | 继续读取 16 字节 payload。 |
| 命令字为 `0x02` | 不读取 payload，直接等待校验和与帧尾。 |
| 命令字未知 | 直接丢弃并回到等待帧头。 |
| 校验失败 | 丢弃当前帧并回到等待帧头。 |
| 帧尾失败 | 丢弃当前帧并回到等待帧头。 |
| 完整急停帧 | 设置 `run_flag = 0`，设置 `new_cmd_flag = 1`。 |
| 完整运动帧 | 解析 4 个 `float`，设置 `run_flag = 1`，设置 `new_cmd_flag = 1`。 |
| 完整收敛帧 | 记录命令字，设置 `new_cmd_flag = 1`，由主循环执行自动收敛。 |

注意：下位机当前没有二进制 ACK 包。命令执行结果通过文本行回传。

## 7. 上位机收包格式

下位机回传为文本日志，通常以 `\r\n` 结尾。上位机 `ArmCommunicationController` 的后台接收线程会：

1. 从串口读取当前可用字节。
2. 按 `serial.encoding` 解码，默认 `utf-8`。
3. 累积到内部字符串缓冲区。
4. 以 `\n` 切分行。
5. 对每一行执行 `strip()`。
6. 若 `print_rx_to_terminal` 开启，先用 `print()` 打印到终端。
7. 非空行通过 `on_message(line)` 回调交给上层程序。

因此，上位机业务层看到的是已经去掉换行符的字符串。

### 7.1 常见回传文本

| 文本前缀 | 示例 | 上位机含义 |
| --- | --- | --- |
| `ACCEPTED:` | `ACCEPTED: X=0.0, Y=30.0, Z=18.0, Dir=0.0, phases=1` | 下位机已接受目标，并启动非阻塞规划。`phases` 表示本次规划阶段数。 |
| `SUCCESS:` | `SUCCESS: X=0.0, Y=30.0, Z=18.0, Dir=0.0 -> th0=0.0, th1=..., th2=..., th3=..., phases=1` | 目标已经执行完成，或无需运动时直接成功。 |
| `FOLDING:` | `FOLDING: th0=90.0, th1=120.0, th2=-145.0, th3=75.0, phases=2` | 自动收敛命令已接受，并开始非阻塞收敛。 |
| `FOLDED:` | `FOLDED: th0=90.0, th1=120.0, th2=-145.0, th3=75.0, phases=2` | 自动收敛完成，或机械臂已经处于默认收敛姿态。 |
| `ERROR:` | `ERROR: Target (X:0.0, Y:30.0, Z:18.0, Dir:0.0) rejected: target unreachable or angle-limited` | 目标被拒绝或执行中发生错误。 |
| `STOPPED:` | `STOPPED: All servos locked.` | 急停命令已生效，舵机保持锁定。 |

### 7.2 可能出现的错误原因

| 错误文本 | 常见原因 |
| --- | --- |
| `target too close to front rod` | 目标点太靠近车前方竖直细杆保护区。 |
| `target unreachable or angle-limited` | 目标超出机械臂可达空间，或逆解后的舵机角超出限位。 |
| `arm planner busy` | 上一个非阻塞规划还没有执行完，又收到新的运动目标。 |
| `current-plane fold pose invalid` | 当前平面的收敛姿态超过舵机安全限位。 |
| `target-plane fold pose invalid` | 目标平面的收敛姿态超过舵机安全限位。 |
| `phase buffer overflow` | 规划阶段数量超过下位机缓冲上限。 |
| `failed to start first phase` | 第一阶段姿态执行失败，通常与限位或舵机命令有关。 |
| `ERROR: Failed to execute next phase!` | 后续规划阶段执行失败。 |
| `ERROR: Arm folded home pose too close to limit!` | 初始化收敛姿态不满足安全余量，通常需要检查收敛角、舵机映射或限位参数。 |
| `ERROR: Fold command rejected: ...` | 自动收敛命令被拒绝，常见原因是收敛姿态越界或规划生成失败。 |

GUI 当前判断规则比较简单：文本中包含 `ERROR` 或 `STOP` 时按错误颜色显示，其余文本按普通接收信息显示。

## 8. `ArmCommunicationController` 调用接口

### 8.1 创建控制器

```python
from ARM_controller import ArmCommunicationController

def on_message(line: str):
    print("RX:", line)

def on_error(exc: Exception):
    print("RX error:", exc)

arm = ArmCommunicationController(on_message=on_message, on_error=on_error)
```

也可以指定配置文件：

```python
arm = ArmCommunicationController(config_path="PythonTools/ARM_communication.yml")
```

### 8.2 串口管理

| 接口 | 功能 |
| --- | --- |
| `list_ports()` | 返回当前系统可用串口列表，例如 `["COM3", "COM5"]`。 |
| `connect(port, baudrate=None, timeout=None, start_receiver=True)` | 打开串口。默认读取配置文件中的波特率和超时时间。 |
| `disconnect()` | 停止后台接收线程并关闭串口。 |
| `close()` | `disconnect()` 的别名。 |
| `is_open` | 属性，判断串口是否已打开。 |
| `__enter__/__exit__` | 支持 `with` 上下文管理，退出时自动关闭串口。 |

示例：

```python
with ArmCommunicationController(on_message=on_message) as arm:
    arm.connect("COM3")
    arm.send_target(0.0, 30.0, 18.0, 0.0)
```

### 8.3 回调管理

| 接口 | 功能 |
| --- | --- |
| `set_message_callback(callback)` | 设置下位机文本行回调。 |
| `set_error_callback(callback)` | 设置接收线程异常回调。 |
| `start_receiver()` | 手动启动后台接收线程。 |
| `stop_receiver()` | 停止后台接收线程。 |

如果上位机想自己管理收包循环，可以连接时关闭自动接收：

```python
arm.connect("COM3", start_receiver=False)
```

然后通过 `arm.serial_port` 自己读取串口。

### 8.4 组包与发送

| 接口 | 功能 |
| --- | --- |
| `build_packet(cmd_type, payload=b"")` | 按协议构造完整二进制帧，不发送。 |
| `build_target_payload(world_x, world_y, world_z, suc_dir_deg)` | 将运动目标打包为 16 字节 payload。 |
| `send_packet(cmd_type, payload=b"")` | 构造并发送任意命令帧，返回实际发送的 `bytes`。 |
| `send_target(world_x, world_y, world_z, suc_dir_deg)` | 发送运动目标，返回实际发送的 `bytes`。 |
| `send_stop()` | 发送急停命令，返回实际发送的 `bytes`。 |
| `send_fold()` | 发送自动收敛命令，返回实际发送的 `bytes`。 |

如果串口未连接，`send_packet()` 会抛出 `RuntimeError`，上层程序应捕获并提示用户。

## 9. 调试 GUI 的协议行为

`PythonTools/ARMcom.py` 是基于 `ArmCommunicationController` 的图形调试程序。

### 9.1 GUI 发包

GUI 左侧滑块对应：

| GUI 控件 | 实际发送字段 |
| --- | --- |
| 末端世界 X | `world_x` |
| 末端世界 Y | `world_y` |
| 末端世界 Z | `world_z` |
| 竖直平面吸盘方向 | `suc_dir_deg` |

点击发送任务时，GUI 会先做本地预检查：

1. 目标是否太靠近前方竖直细杆。
2. Python 侧预览逆解是否可达。
3. 预估舵机角是否在配置限位内。

预检查通过后调用：

```python
self.controller.send_target(x, y, z, dir)
```

同时在日志区显示：

```text
TX (任务): X=..., Y=..., Z=..., Dir=...
```

点击急停时调用：

```python
self.controller.send_stop()
```

点击自动收敛时调用：

```python
self.controller.send_fold()
```

### 9.2 GUI 收包

GUI 将 `controller` 的 `on_message` 回调绑定到日志显示函数。收到下位机文本后显示：

```text
RX: 下位机原始文本
```

其中包含 `ERROR` 或 `STOP` 的文本按错误颜色显示。

## 10. 配置文件参数清单

`ARM_communication.yml` 当前集中列出了上位机需要关注的参数。部分参数也在下位机中有对应宏定义，调试时要保持一致。

### 10.1 `serial`

| 参数 | 含义 | 建议 |
| --- | --- | --- |
| `baudrate` | 串口波特率。 | 保留在 YAML，方便换板或换串口工具。 |
| `timeout_s` | 串口读超时。 | 保留在 YAML。 |
| `read_sleep_s` | 接收线程睡眠间隔。 | 保留在 YAML。 |
| `encoding` | 下位机文本解码方式。 | 保留在 YAML。 |

### 10.2 `protocol`

| 参数 | 含义 | 建议 |
| --- | --- | --- |
| `header` | 二进制帧头。 | 协议稳定后可硬编码。 |
| `tail` | 二进制帧尾。 | 协议稳定后可硬编码。 |
| `cmd_stop` | 急停命令字。 | 协议稳定后可硬编码。 |
| `cmd_move` | 运动任务命令字。 | 协议稳定后可硬编码。 |
| `cmd_fold` | 自动收敛命令字。 | 协议稳定后可硬编码。 |
| `move_payload_format` | 运动 payload 的 `struct.pack` 格式。 | 建议保留或在代码中清晰常量化。 |

### 10.3 `ui_defaults`

| 参数 | 含义 | 建议 |
| --- | --- | --- |
| `world_x_cm` | GUI 启动时默认 X。 | 保留在 YAML。 |
| `world_y_cm` | GUI 启动时默认 Y。 | 保留在 YAML。 |
| `world_z_cm` | GUI 启动时默认 Z。 | 保留在 YAML。 |
| `suc_dir_deg` | GUI 启动时默认吸盘方向。 | 保留在 YAML。 |

### 10.4 `arm_geometry`

| 参数 | 含义 |
| --- | --- |
| `h0_cm` | 云台中心距地面高度。 |
| `h1_cm` | 云台中心到大齿轮关节的高度差。 |
| `r_cm` | 云台中心到大齿轮关节的水平投影偏置。 |
| `l1_cm` | 第一段机械臂名义长度。 |
| `c_cm` | 第二关节相对第一段机械臂的垂直机械偏置。 |
| `l2_cm` | 第二段机械臂长度。 |
| `suction_len_cm` | 第四舵机轴心到吸盘末端距离。 |

这些参数只用于上位机预览和发送前预检查；真正执行以固件里的宏定义为准。调机阶段建议保留在 YAML，机械结构完全固定后可以硬编码或同步生成。

### 10.5 `front_rod`

| 参数 | 含义 | 建议 |
| --- | --- | --- |
| `enabled` | 是否启用前方竖直细杆预检查。 | 保留在 YAML。 |
| `x_cm` | 细杆 X 坐标。 | 保留在 YAML。 |
| `y_cm` | 细杆 Y 坐标。 | 保留在 YAML，并与下位机避障宏一致。 |
| `radius_cm` | 保护半径。 | 保留在 YAML。 |

### 10.6 `planner_preview`

| 参数 | 含义 |
| --- | --- |
| `x_side_eps_cm` | 判断目标在 X 正侧、负侧还是零点附近的死区。 |

该参数用于上位机预览提示跨零收敛，实际规划仍由下位机执行。

### 10.7 `folded_pose`

| 参数 | 含义 |
| --- | --- |
| `th1_deg` | 收敛姿态中 1 号关节虚拟角。 |
| `th2_deg` | 收敛姿态中 2 号关节虚拟角。 |
| `th3_deg` | 收敛姿态中 3 号关节虚拟角。 |
| `default_base_deg` | 上位机预览使用的默认收敛云台平面角。 |

这些参数用于上位机预览收敛姿态。实际初始姿态和跨零收敛姿态以固件宏定义为准。

### 10.8 `servo_mapping`

| 参数 | 含义 |
| --- | --- |
| `s0_offset_deg` | 0 号舵机物理零点偏置。 |
| `s1_gear` | 1 号舵机虚拟角到物理角的比例。 |
| `s1_offset_deg` | 1 号舵机物理零点偏置。 |
| `s2_offset_deg` | 2 号舵机物理零点偏置。 |
| `s3_offset_deg` | 3 号舵机物理零点偏置。 |

这些参数用于 GUI 预检查。调试阶段建议保留在 YAML，方便与下位机标定值对齐。

### 10.9 `servo_limits`

| 参数 | 含义 |
| --- | --- |
| `s0_min_deg/s0_max_deg` | 0 号舵机物理角限位。 |
| `s1_min_deg/s1_max_deg` | 1 号舵机物理角限位。 |
| `s2_min_deg/s2_max_deg` | 2 号舵机物理角限位。 |
| `s3_min_deg/s3_max_deg` | 3 号舵机物理角限位。 |

这些参数只影响上位机预览和发送前拦截。下位机仍会做最终限位检查。

## 11. 上位机推荐使用方式

### 11.1 最小调用示例

```python
from ARM_controller import ArmCommunicationController

def handle_rx(line):
    print("MCU:", line)

def handle_error(exc):
    print("Serial error:", exc)

arm = ArmCommunicationController(on_message=handle_rx, on_error=handle_error)
arm.connect("COM3")

arm.send_target(0.0, 30.0, 18.0, 0.0)

# 程序退出前关闭串口
arm.close()
```

### 11.2 带端口扫描

```python
from ARM_controller import ArmCommunicationController

ports = ArmCommunicationController.list_ports()
print("ports:", ports)

if ports:
    arm = ArmCommunicationController()
    arm.connect(ports[0])
    arm.send_stop()
    arm.close()
```

### 11.3 根据回传判断任务状态

```python
def handle_rx(line):
    if line.startswith("ACCEPTED:"):
        print("任务已接受")
    elif line.startswith("SUCCESS:"):
        print("任务完成")
    elif line.startswith("FOLDING:"):
        print("正在自动收敛")
    elif line.startswith("FOLDED:"):
        print("自动收敛完成")
    elif line.startswith("ERROR:"):
        print("任务失败:", line)
    elif line.startswith("STOPPED:"):
        print("急停生效")
    else:
        print("其他信息:", line)
```

## 12. 通信链路完整流程

运动任务完整链路如下：

```mermaid
flowchart LR
    A["上位机业务程序/GUI"] --> B["ArmCommunicationController.send_target"]
    B --> C["struct.pack('<ffff') 生成 payload"]
    C --> D["build_packet 添加帧头、命令字、校验和、帧尾"]
    D --> E["串口发送 bytes"]
    E --> F["下位机 ARM_RX_Callback 按字节解析"]
    F --> G["写入 g_HostCmd 并置 new_cmd_flag"]
    G --> H["Arm_Task_Loop 逆解、规划、执行"]
    H --> I["下位机串口回传文本行"]
    I --> J["controller 接收线程切行"]
    J --> K["on_message(line) 回调给上位机"]
```

急停链路如下：

```mermaid
flowchart LR
    A["上位机 send_stop"] --> B["发送 cmd=0x00 空 payload 帧"]
    B --> C["下位机设置 run_flag=0/new_cmd_flag=1"]
    C --> D["Arm_Task_Loop 停止并锁持舵机"]
    D --> E["回传 STOPPED 文本"]
    E --> F["上位机 on_message 接收显示"]
```

自动收敛链路如下：

```mermaid
flowchart LR
    A["上位机 send_fold"] --> B["发送 cmd=0x02 空 payload 帧"]
    B --> C["下位机记录自动收敛命令"]
    C --> D["清空当前规划并构建收敛计划"]
    D --> E["当前平面收敛，再转到默认收敛平面"]
    E --> F["回传 FOLDING/FOLDED 文本"]
    F --> G["上位机终端打印并触发 on_message"]
```

## 13. 注意事项

1. 上位机发送的是二进制帧，下位机回传的是文本行，两者不是同一种格式。
2. 运动任务的 4 个 `float` 必须使用小端格式，当前 Python 格式字符串为 `"<ffff"`。
3. `suc_dir_deg = 0` 表示吸盘竖直向下，不应被上位机当作非法角度。
4. GUI 的可达性判断只是预览和拦截，最终是否执行成功以下位机回传为准。
5. 如果收到 `arm planner busy`，说明下位机上一条规划未执行完，上位机应等待 `SUCCESS:` 或发送急停后再下发新目标。
6. 如果调整机械参数、收敛姿态、舵机映射或限位，建议同时检查 YAML 与下位机宏定义，避免 GUI 预览和实际固件不一致。
7. 协议当前没有序号字段。如果上位机连续快速发送多条运动目标，只能根据回传文本和发送顺序人工对应。
