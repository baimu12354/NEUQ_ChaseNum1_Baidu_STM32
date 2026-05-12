# 底盘测试通信协议

本协议只用于当前阶段的底盘测试。下位机只执行 `Movement_Breaking` 和 `Movement_Running`，机械臂相关功能不在本协议内。

## 串口参数

- UART：USART3，PB10 TX / PB11 RX
- 波特率：115200
- 数据位：8
- 校验：None
- 停止位：1
- 流控：None

## 帧格式

所有多字节数据均为 little-endian，浮点数为 IEEE-754 float32。

| 字段 | 字节数 | 说明 |
| --- | ---: | --- |
| HEAD | 2 | 固定 `AA 55` |
| VERSION | 1 | 当前固定 `01` |
| SEQ | 1 | 发送端递增序号，下位机回包会使用自己的递增序号 |
| CMD | 1 | 命令字或响应字 |
| LEN | 1 | payload 字节数，最大 64 |
| PAYLOAD | LEN | 命令载荷 |
| CHECK | 1 | `VERSION + SEQ + CMD + LEN + PAYLOAD所有字节` 的低 8 位 |
| TAIL | 2 | 固定 `0D 0A` |

## 上位机命令

| CMD | 名称 | PAYLOAD | 说明 |
| ---: | --- | --- | --- |
| `0x01` | HELLO | 无 | 通信握手，下位机返回 ACK 和 STATUS |
| `0x02` | BRAKE | 无 | 调用 `Movement_Breaking`，立即刹车 |
| `0x03` | RUN | `float vx, float vy, float wz_deg_s` | 调用 `Movement_Running`，对应 `Car_X_vel / Car_Y_vel / Car_angvel` |
| `0x04` | GET_STATUS | 无 | 请求一次状态反馈 |
| `0x05` | SET_YAW | `float yaw_deg` | 设置逻辑航向角零偏 |
| `0x06` | SET_FEEDBACK | `uint16 interval_ms` | 设置状态周期反馈；0 表示关闭 |

`RUN` 命令的坐标含义沿用 `RC_control()`：`vx` 为车体横向速度，右为正；`vy` 为车体纵向速度，前为正；`wz_deg_s` 为航向角速度，正方向与工程内 `Car_angvel` 保持一致。

## 下位机响应

| RESP | 名称 | PAYLOAD | 说明 |
| ---: | --- | --- | --- |
| `0x81` | ACK | `uint8 original_cmd, uint8 err=0` | 命令已接收并执行 |
| `0x82` | ERR | `uint8 original_cmd, uint8 err` | 命令错误或运行超时 |
| `0x83` | STATUS | `uint8 mode, float yaw_deg, float vx, float vy, float wz_deg_s, uint32 timestamp_ms` | 底盘状态与航向角反馈 |

错误码：

| ERR | 含义 |
| ---: | --- |
| `0x00` | 无错误 |
| `0x01` | 未知命令 |
| `0x02` | payload 长度错误 |
| `0x03` | 校验错误，当前下位机解析阶段直接丢包，不主动回包 |
| `0x04` | 设备忙，预留 |
| `0x05` | 上位机控制超时，已自动刹车 |

## 运行保护

下位机收到 `RUN` 后会记录最近一次运动命令时间。如果超过 `MAIN_COM_CMD_TIMEOUT_MS` 没有新的 `RUN` 命令，会自动调用 `Movement_Breaking` 并发送错误响应。默认值在下位机和 `communication.yml` 中均为 500 ms。

## 上位机调用示例

```python
from chassis_controller import ChassisController

with ChassisController("communication.yml") as chassis:
    print(chassis.hello())
    chassis.set_feedback_interval(100)
    chassis.run(vx=0.0, vy=10.0, wz_deg_s=0.0)
    status = chassis.get_status()
    print(status)
    chassis.brake()
```

连续运动建议使用 `run_for()` 或自行以 10-20 Hz 周期发送 `run()`，因为下位机有 500 ms 控制超时保护。

`communication.yml` 中列出了串口参数、帧头帧尾、命令字、速度限幅、反馈周期和 `RC_control()` 的通道映射比例。测试稳定后，可以把不需要调整的项硬编码进上位机代码。

## 下位机代码修改说明

本次下位机侧只接入底盘测试协议，没有接入机械臂协议。

| 文件 | 修改说明 |
| --- | --- |
| `Code/ChaseNum1_COM.h` | 新增测试协议帧头、命令字、响应字、错误码、载荷长度、速度限幅、反馈周期和超时参数；声明 `Main_COM_send_frame()`、`Main_COM_send_status()`、`Main_COM_RX_CALLBACK()`。 |
| `Code/ChaseNum1_COM.c` | 将 UART3 接收回调改为字节状态机拼包；主循环中的 `Main_COM_Response_CALLBACK()` 负责解析命令并调用 `CN_set_movement()`；实现 `HELLO/BRAKE/RUN/GET_STATUS/SET_YAW/SET_FEEDBACK`；实现 ACK、ERR、STATUS 回包；保留 UART1 的 VOFA 电机调参接收逻辑。 |
| `Code/ChaseNum1_Uart.c` | 在 `CN_UART3_init()` 调用 `HAL_UART_Init()` 前将 UART3 参数覆盖为 115200/8N1，用于上位机普通串口通信。 |
| `USER/main.c` | 主循环改为调用 `Main_COM_Response_CALLBACK()`，由上位机协议驱动底盘；1 ms PIT 中断继续调用 `CN_Movement_ctrl_CALLBACK(1)`，保持底盘闭环控制节奏。 |

`RUN` 最终只填充 `Car_Movement_struct.Car_X_vel`、`Car_Y_vel`、`Car_angvel`，并以 `Movement_Running` 传入 `CN_set_movement()`；`BRAKE` 以全零运动结构调用 `Movement_Breaking`。这与 `RC_control()` 中遥控器控制底盘的底层入口保持一致。
