# NANO 通信协议与下位机状态机报告

## 1. 本次修改目标

本次新增一套面向上位机 NANO 的统一通信和下位机状态机，通信串口固定为 `UART2`。

原则：

- 不重写底盘、机械臂、气泵原有功能，只在状态机中调用现有接口。
- 统一使用二进制帧，不使用文本作为协议内容。
- 下位机自动初始化，上位机握手只用于确认通信。
- 底盘速度控制必须周期发送，超时后普通停止，不走刹停。
- 旋转和平移到位后保持零速度姿态控制，并给上位机回完成事件。
- 机械臂支持目标控制、收敛、停止锁持、读取逻辑位姿。
- 气泵只暴露吸附和放下，吸附完成和放下完成用事件回包。

## 2. 修改文件

新增：

- `Code/ChaseNum1_NanoProtocol.c`
- `Code/ChaseNum1_NanoProtocol.h`
- `PythonTools/nano_protocol_tester.py`
- `PythonTools/nano_protocol_ui.py`

修改：

- `Code/ChaseNum1_COM.c`：`Main_COM_init()` 改为初始化 NANO 协议，`CN_UART2_RX_CALLBACK()` 转发给协议解析。
- `Code/ChaseNum1_Include.h`：加入 NANO 协议头文件。
- `Code/ChaseNum1_Uart.c`：给 UART2 挂载发送 FIFO，否则 UART2 不能可靠使用 `CN_UART_write_fifo()`。
- `Code/ChaseNum1_Uart.h`：UART2 注释改为 NANO 上位机通信。
- `Code/ChaseNum1_MecArm.c/.h`：新增逻辑位姿读取、规划器忙闲读取、内部事件读取接口，不改变运动解算。
- `Code/ChaseNum1_Movement.c/.h`：新增当前底盘运动模式读取接口，不改变底盘运动逻辑。
- `USER/main.c`：接入 NANO 状态机主循环和 1ms Tick，恢复底盘运动初始化、气泵初始化和底盘 1ms 控制回调。

## 3. 下位机运行流程

上电后：

```text
1. 初始化电机 PWM 并先停止输出
2. 初始化 PIT、Debug、NANO UART2 通信
3. 初始化编码器
4. 初始化底盘运动模块
5. 初始化机械臂并进入默认收敛姿态
6. 初始化 IMU
7. 初始化气泵，默认关闭
8. 启动 PIT1/PIT2
9. 调用 Nano_COM_Notify_Init_Done()
10. 发送 SYS_READY 事件和 STATUS 状态包
```

主循环中调用：

```c
Nano_COM_Task_Loop();
```

PIT1 每 1ms 调用：

```c
CN_Movement_ctrl_CALLBACK(1);
Nano_COM_1ms_Tick();
```

UART2 收到字节后调用：

```c
Nano_COM_RX_Callback(dat);
```

## 4. 帧格式

所有收包和回包统一使用：

```text
HEAD1 HEAD2 VER SEQ MODULE CMD LEN PAYLOAD CHECK TAIL1 TAIL2
```

字段定义：

| 字段 | 长度 | 当前值/说明 |
| --- | --- | --- |
| `HEAD1` | 1 | `0xAA` |
| `HEAD2` | 1 | `0x55` |
| `VER` | 1 | `0x01` |
| `SEQ` | 1 | 上位机命令序号，回包原样带回 |
| `MODULE` | 1 | 模块 ID |
| `CMD` | 1 | 命令字或响应字 |
| `LEN` | 1 | 载荷长度 |
| `PAYLOAD` | N | 小端载荷 |
| `CHECK` | 1 | 从 `VER` 到 `PAYLOAD` 的 8 位累加和 |
| `TAIL1` | 1 | `0x0D` |
| `TAIL2` | 1 | `0x0A` |

校验计算：

```text
CHECK = (VER + SEQ + MODULE + CMD + LEN + sum(PAYLOAD)) & 0xFF
```

所有 `float` 都是 IEEE754 小端格式。

UART 参数：

```text
UART2
115200
8N1
无硬件流控
```

## 5. 模块 ID

| 模块 | ID |
| --- | --- |
| `SYSTEM` | `0x01` |
| `CHASSIS` | `0x02` |
| `ARM` | `0x03` |
| `PUMP` | `0x04` |

## 6. 系统状态

| 状态 | 值 | 说明 |
| --- | --- | --- |
| `SYS_INIT` | `0` | 下位机初始化中 |
| `SYS_RET` | `1` | 外设复位中 |
| `SYS_IDLE` | `2` | 空闲 |
| `SYS_ACTIVE` | `3` | 至少一个模块正在执行 |
| `SYS_ESTOP` | `4` | 急停锁止 |
| `SYS_FAULT` | `5` | 故障预留 |

急停后只允许：

- `HELLO`
- `GET_STATUS`
- `RESET`

必须 `RESET` 完成后才能继续普通命令。

## 7. 系统命令

| 命令 | CMD | 载荷 | 回包 |
| --- | --- | --- | --- |
| `HELLO` | `0x01` | 无 | `ACK` + `STATUS` |
| `RESET` | `0x02` | 无 | `ACK`，完成后 `EVENT_SYS_RESET_DONE` |
| `ESTOP` | `0x03` | 无 | `ACK` + `EVENT_SYS_ESTOP` |
| `GET_STATUS` | `0x04` | 无 | `STATUS` |

系统事件：

| 事件 | 值 |
| --- | --- |
| `EVENT_SYS_READY` | `0x01` |
| `EVENT_SYS_RESET_DONE` | `0x02` |
| `EVENT_SYS_ESTOP` | `0x03` |

## 8. 底盘命令

| 命令 | CMD | 载荷 | 说明 |
| --- | --- | --- | --- |
| `CHASSIS_RUN` | `0x10` | `float vx, vy, wz` | 周期速度控制 |
| `CHASSIS_STOP` | `0x11` | 无 | 普通停止，零速度姿态控制 |
| `CHASSIS_BRAKE` | `0x12` | 无 | 主动刹车 |
| `CHASSIS_ROTATE_TO` | `0x13` | `float angle, max_wz` | 相对当前角旋转 |
| `CHASSIS_TRANSLATE_REL` | `0x14` | `float dx, dy, speed` | 相对当前位置平移 |
| `CHASSIS_GET_STATE` | `0x15` | 无 | 读取底盘状态 |

底盘状态：

| 状态 | 值 |
| --- | --- |
| `CHASSIS_IDLE` | `0` |
| `CHASSIS_RUN` | `1` |
| `CHASSIS_ROTATE` | `2` |
| `CHASSIS_TRANSLATE` | `3` |
| `CHASSIS_STOP` | `4` |
| `CHASSIS_BRAKE` | `5` |
| `CHASSIS_FAULT` | `6` |

底盘事件：

| 事件 | 值 | 触发条件 |
| --- | --- | --- |
| `EVENT_CHASSIS_ROTATE_DONE` | `0x10` | 旋转到目标角后 |
| `EVENT_CHASSIS_TRANS_DONE` | `0x11` | 平移到目标位置后 |
| `EVENT_CHASSIS_TIMEOUT_STOP` | `0x12` | 周期速度命令超时后 |

`CHASSIS_RUN` 必须由上位机周期发送。当前超时时间：

```c
#define NANO_CHASSIS_RUN_TIMEOUT_MS 500U
```

超时后下位机发送零速度 `Movement_Running`，保持姿态控制，不调用刹车。

## 9. 机械臂命令

| 命令 | CMD | 载荷 | 说明 |
| --- | --- | --- | --- |
| `ARM_TARGET_XYZ_DIR` | `0x20` | `float x, y, z, dir` | 吸盘末端三维坐标和方向 |
| `ARM_FOLD_HOME` | `0x21` | 无 | 回默认收敛姿态 |
| `ARM_STOP_HOLD` | `0x22` | 无 | 停止并锁持 |
| `ARM_GET_POSE` | `0x23` | 无 | 读取逻辑位置和关节角 |

机械臂状态：

| 状态 | 值 |
| --- | --- |
| `ARM_IDLE` | `0` |
| `ARM_MOVING` | `1` |
| `ARM_FOLDING` | `2` |
| `ARM_HOLD` | `3` |
| `ARM_REJECTED` | `4` |
| `ARM_FAULT` | `5` |

机械臂事件：

| 事件 | 值 |
| --- | --- |
| `EVENT_ARM_DONE` | `0x20` |
| `EVENT_ARM_FOLD_DONE` | `0x21` |
| `EVENT_ARM_REJECTED` | `0x22` |

`ARM_GET_POSE` 回包 `RESP_ARM_POSE`，载荷：

```text
arm_state    u8
pose_source  u8   当前为 0，表示逻辑位置
arm_busy     u8
reserved     u8
x            float
y            float
z            float
dir          float
th0          float
th1          float
th2          float
th3          float
```

## 10. 气泵命令

| 命令 | CMD | 载荷 | 说明 |
| --- | --- | --- | --- |
| `PUMP_SUCK` | `0x30` | 无 | 开始吸附 |
| `PUMP_RELEASE` | `0x31` | 无 | 放下工件 |
| `PUMP_GET_STATE` | `0x32` | 无 | 读取气泵状态 |

气泵状态：

| 状态 | 值 |
| --- | --- |
| `PUMP_IDLE` | `0` |
| `PUMP_SUCKING` | `1` |
| `PUMP_SUCKED` | `2` |
| `PUMP_RELEASING` | `3` |
| `PUMP_RELEASED` | `4` |
| `PUMP_FAULT` | `5` |

气泵事件：

| 事件 | 值 | 触发条件 |
| --- | --- | --- |
| `EVENT_PUMP_SUCK_DONE` | `0x30` | `Pump_Handle.IfSucked != 0` |
| `EVENT_PUMP_RELEASE_DONE` | `0x31` | 停泵后等待释放时间结束 |

释放等待时间：

```c
#define NANO_PUMP_RELEASE_WAIT_MS 500U
```

该值后续根据实际工件释放时间调试。

## 11. 通用回包

### ACK / NACK

`ACK = 0x80`，`NACK = 0x81`。

载荷：

```text
ref_seq   u8
module    u8
cmd       u8
result    u8
state     u8
```

`result=0` 表示成功，其它为错误码。

### EVENT

`EVENT = 0x83`。

载荷：

```text
ref_seq   u8
module    u8
event     u8
state     u8
result    u8
```

### STATUS

`STATUS = 0x82`。

载荷：

```text
system_state    u8
chassis_state   u8
arm_state       u8
pump_state      u8
movement_type   u8
pump_on         u8
pump_sucked     u8
arm_busy        u8
tick_ms         u32
yaw_now         float
```

## 12. 错误码

| 错误 | 值 |
| --- | --- |
| `OK` | `0x00` |
| `BAD_CMD` | `0x01` |
| `BAD_LEN` | `0x02` |
| `BAD_CHECK` | `0x03` |
| `BAD_STATE` | `0x04` |
| `BUSY` | `0x05` |
| `RANGE_LIMIT` | `0x06` |
| `FORBIDDEN_ZONE` | `0x07` |
| `TIMEOUT` | `0x08` |
| `DEVICE` | `0x09` |

## 13. Python 测试脚本

脚本位置：

```text
PythonTools/nano_protocol_tester.py
```

依赖：

```bash
pip install pyserial
```

常用测试：

```bash
python PythonTools/nano_protocol_tester.py --port COM8 hello
python PythonTools/nano_protocol_tester.py --port COM8 status
python PythonTools/nano_protocol_tester.py --port COM8 chassis_state
python PythonTools/nano_protocol_tester.py --port COM8 arm_pose
python PythonTools/nano_protocol_tester.py --port COM8 pump_state
```

底盘速度控制，注意需要周期发送：

```bash
python PythonTools/nano_protocol_tester.py --port COM8 run --vx 0 --vy 20 --wz 0
```

旋转和平移：

```bash
python PythonTools/nano_protocol_tester.py --port COM8 rotate --angle 90 --wz 40 --listen 3
python PythonTools/nano_protocol_tester.py --port COM8 translate --dx 20 --dy 0 --speed 20 --listen 5
```

机械臂：

```bash
python PythonTools/nano_protocol_tester.py --port COM8 arm_fold --listen 5
python PythonTools/nano_protocol_tester.py --port COM8 arm_pose
python PythonTools/nano_protocol_tester.py --port COM8 arm_target --x 20 --y 0 --z 25 --dir 0 --listen 5
```

气泵：

```bash
python PythonTools/nano_protocol_tester.py --port COM8 pump_suck --listen 5
python PythonTools/nano_protocol_tester.py --port COM8 pump_release --listen 2
```

急停和恢复：

```bash
python PythonTools/nano_protocol_tester.py --port COM8 estop
python PythonTools/nano_protocol_tester.py --port COM8 reset --listen 5
```

## 14. 图形化上位机调试界面

图形界面脚本位置：

```text
PythonTools/nano_protocol_ui.py
```

依赖：

```bash
pip install pyserial
```

启动方式：

```bash
python PythonTools/nano_protocol_ui.py
```

如果使用当前工程调试环境中的虚拟环境，也可以直接运行：

```bash
G:\baidu\CommunicationTemp\.venv\Scripts\python.exe PythonTools\nano_protocol_ui.py
```

界面分为四块：

- 顶部串口栏：选择 `COM` 口和波特率，默认 `115200`，点击“连接”后开始收包解析。
- 左侧控制区：系统、底盘、机械臂、气泵命令都可以直接点击发送。
- 中间可视化区：显示机械臂逻辑位姿和三维坐标轴，红色虚线框为禁止静止到达区域 `-13 < x < 13, 0 < y < 30, 0 < z < 20`。
- 右侧状态与日志区：状态区按协议解析 `STATUS / ARM_POSE / CHASSIS_STATE / PUMP_STATE`，日志区显示完整 TX、RX 原始帧和解析结果。

状态刷新按钮：

| 按钮 | 发送命令 |
| --- | --- |
| `系统` | `SYSTEM / GET_STATUS` |
| `底盘` | `CHASSIS / GET_STATE` |
| `机械臂` | `ARM / GET_POSE` |
| `气泵` | `PUMP / GET_STATE` |

底盘键盘控制：

| 按键 | 效果 |
| --- | --- |
| `↑` | `vy` 为正，向前移动 |
| `↓` | `vy` 为负，向后移动 |
| `→` | `vx` 为正，向右平移 |
| `←` | `vx` 为负，向左平移 |
| `Space` | 发送 `CHASSIS_STOP` |

方向键会以约 `100ms` 周期发送 `CHASSIS_RUN`，松开全部方向键后自动发送 `CHASSIS_STOP`。在文本输入框中编辑参数时，方向键不会控制底盘，避免输入数字时误动车。

机械臂可视化逻辑：

- 点击“读取位姿”后，上位机会发送 `ARM_GET_POSE`，收到 `RESP_ARM_POSE` 后更新机械臂三维姿态。
- 机械臂目标到位或收敛到位后，下位机会发送 `EVENT_ARM_DONE` 或 `EVENT_ARM_FOLD_DONE`；界面收到机械臂事件后会自动再发送一次 `ARM_GET_POSE`，用于刷新最终姿态。
- 黄色点表示吸盘末端逻辑位置，白色折线表示按当前逻辑关节角计算出的机械臂姿态。

底盘常用操作：

```text
1. 连接串口。
2. 点击 HELLO，确认右侧收到 ACK 和 STATUS。
3. 方向键控制底盘速度运动，松开方向键后停止。
4. 需要定角旋转时，填写“旋转角”和“最大角速”，点击“发送旋转”。
5. 需要相对平移时，填写 dx、dy、速度，点击“发送平移”。
6. 点击“底盘”状态按钮可读取当前底盘状态。
```

机械臂常用操作：

```text
1. 点击“机械臂”状态按钮或“读取位姿”，查看当前逻辑位姿。
2. 填写 X、Y、Z、Dir，点击“发送目标”。
3. 若目标在禁止区域或不可达，下位机会返回 NACK 或 ARM_REJECTED 事件。
4. 点击“收敛”可回默认收敛姿态。
5. 点击“锁持”可停止并锁持当前机械臂。
```

气泵常用操作：

```text
1. 点击“吸附”发送 PUMP_SUCK。
2. 吸附成功后等待下位机返回 PUMP_SUCK_DONE 事件。
3. 点击“放下”发送 PUMP_RELEASE。
4. 放下完成后等待下位机返回 PUMP_RELEASE_DONE 事件。
```

## 15. 编译验证

已单独使用 ARMCC 编译以下文件：

- `Code/ChaseNum1_NanoProtocol.c`
- `Code/ChaseNum1_COM.c`
- `Code/ChaseNum1_MecArm.c`
- `Code/ChaseNum1_Movement.c`
- `Code/ChaseNum1_Uart.c`
- `USER/main.c`

结果均为：

```text
0 errors
```

剩余警告为工程原有 `#pragma region` 不被 ARMCC 识别，以及 UART 文件中已有的字符串指针类型警告，均不是本次新增协议导致的编译错误。

Python 测试脚本已通过语法检查：

```text
python -m py_compile PythonTools/nano_protocol_tester.py
```

图形界面脚本已通过语法检查：

```text
python -m py_compile PythonTools/nano_protocol_ui.py
```
