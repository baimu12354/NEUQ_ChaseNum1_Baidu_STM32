# IMU CAN 功能扩展报告

## 1. 结论

当前工程原来已经能通过 CAN 读取 DM-IMU 的加速度、角速度、欧拉角和四元数。根据随包官方例程 `IMU_can/MC02_CAN收发例程/DM-IMU/dm_imu.h`、`dm_imu.c`，厂家还提供了一组 CAN 寄存器读写命令，可以把驱动扩展成更完整的模块控制接口。

本次已把这些接口合入工程的 `Code/ChaseNum1_IMUcan.c` 和 `Code/ChaseNum1_IMUcan.h`：

- 通用寄存器读写：`imu_write_reg()`、`imu_read_reg()`
- 数据请求：加速度、角速度、欧拉角、四元数
- 姿态/传感器控制：重启、置零、加速度校准、陀螺仪校准、磁力计校准
- 通讯配置：切换通讯口、设置 CAN 波特率、设置 IMU CAN ID、设置主机 ID
- 工作模式配置：切换主动模式/请求模式、设置主动模式发送间隔、设置数据输出选择
- 参数管理：保存参数、恢复出厂设置
- 调试增强：记录最近一次 TX 命令帧和加速度帧温度

说明：本机当前没有可靠的 PDF 文本抽取工具，因此说明书 PDF 未能直接全文解析。本次确定实现依据来自工程随附的厂家 CAN 例程；加速度帧温度字段依据公开 DM-IMU-L1 CAN 帧格式，`DATA[1]` 为温度值。

## 2. 协议依据

官方例程中的命令帧格式为 8 字节经典 CAN 数据帧：

```c
uint8_t buf[8] = {0xCC, reg_id, ac, 0xDD, 0, 0, 0, 0};
memcpy(buf + 4, &data, 4);
tx_header.Identifier = imu.can_id;
```

字段含义：

| 字段 | 含义 |
| --- | --- |
| `0xCC` | 命令帧头 |
| `reg_id` | 寄存器/命令 ID |
| `ac` | `0` 读，`1` 写 |
| `0xDD` | 命令标记 |
| `data[4]` | 小端 32 位参数 |
| CAN 标准 ID | 发送到 IMU 设备 ID，默认 `0x01` |

接收数据帧仍沿用原来的 4 类数据：

| `DATA[0]` | 数据类型 | 当前解析结果 |
| --- | --- | --- |
| `1` | 加速度 | `imu.accel[3]`，同时更新 `imu.cur_temp` |
| `2` | 角速度 | `imu.gyro[3]` |
| `3` | 欧拉角 | `imu.pitch`、`imu.yaw`、`imu.roll` |
| `4` | 四元数 | `imu.q[4]` |

## 3. 新增接口说明

### 3.1 通用寄存器访问

| 函数 | 作用 | 说明 |
| --- | --- | --- |
| `imu_write_reg(uint8_t reg_id, uint32_t data)` | 写任意寄存器 | 用于厂家后续新增命令，也方便临时调试 |
| `imu_read_reg(uint8_t reg_id)` | 读任意寄存器 | 四类数据请求函数都改为调用它 |

### 3.2 数据读取

| 函数 | 等价寄存器 | 作用 |
| --- | --- | --- |
| `imu_request_accel()` | `ACCEL_DATA` | 请求加速度帧；本次新增温度解析 `imu.cur_temp` |
| `imu_request_gyro()` | `GYRO_DATA` | 请求角速度帧 |
| `imu_request_euler()` | `EULER_DATA` | 请求欧拉角帧 |
| `imu_request_quat()` | `QUAT_DATA` | 请求四元数帧 |

### 3.3 校准和姿态控制

| 函数 | 写入寄存器 | 写入值 | 作用 |
| --- | --- | --- | --- |
| `imu_reboot()` | `REBOOT_IMU` | `0` | 远程重启 IMU |
| `imu_set_zero()` | `SET_ZERO` | `0` | 设置当前姿态为零点 |
| `imu_accel_calibration()` | `ACCEL_CALI` | `0` | 加速度计校准 |
| `imu_gyro_calibration()` | `GYRO_CALI` | `0` | 陀螺仪校准 |
| `imu_mag_calibration()` | `MAG_CALI` | `0` | 磁力计校准，若当前 L1 模块无磁力计则命令可能无效果 |

使用注意：

- 校准命令应在模块静止、安装姿态明确时调用。
- `imu_set_zero()` 会改变后续姿态零点，建议只在初始化或人工确认后调用。
- `imu_reboot()` 会中断当前数据流，重启后需要等待模块重新上线。

### 3.4 通讯和 ID 配置

| 函数 | 写入寄存器 | 参数 |
| --- | --- | --- |
| `imu_change_com_port(imu_com_port_e port)` | `CHANGE_COM` | `COM_USB`、`COM_RS485`、`COM_CAN`、`COM_VOFA` |
| `imu_set_baud(imu_baudrate_e baud)` | `SET_BAUD` | `CAN_BAUD_1M` 到 `CAN_BAUD_25K` |
| `imu_set_can_id(uint8_t can_id)` | `SET_CAN_ID` | 新 IMU 设备 CAN ID |
| `imu_set_mst_id(uint8_t mst_id)` | `SET_MST_ID` | 新主机 CAN ID |
| `imu_set_local_ids(uint8_t can_id, uint8_t mst_id)` | 仅修改本地变量 | 用于设备 ID 已改变后，让工程继续发到正确 ID |

使用注意：

- `imu_set_can_id()` 只是向旧 ID 发送修改命令，发送后本地 `imu.can_id` 不会自动改变。确认设备已经改 ID 后，再调用 `imu_set_local_ids(new_id, imu.mst_id)` 或重启后修改 `IMU_CAN_DEFAULT_DEVICE_ID`。
- `imu_set_baud()` 会改变 IMU CAN 波特率。若保存并重启后，MCU 端 FDCAN 位时序也必须同步修改，否则会立刻失联。
- `imu_change_com_port(COM_CAN)` 可用于确认模块处于 CAN 模式。切到 USB/RS485/VOFA 后，CAN 可能不再回应。

### 3.5 工作模式和参数保存

| 函数 | 写入寄存器 | 写入值 | 作用 |
| --- | --- | --- | --- |
| `imu_set_active_mode_delay(uint32_t delay)` | `SET_DELAY` | 发送周期/延时参数 | 设置主动模式输出间隔 |
| `imu_change_to_active()` | `CHANGE_ACTIVE` | `1` | 切换到主动上报模式 |
| `imu_change_to_request()` | `CHANGE_ACTIVE` | `0` | 切换到请求应答模式 |
| `imu_set_data_output_selection(uint32_t selection)` | `DATA_OUTPUT_SELECTION` | 输出选择掩码/参数 | 配置主动模式输出哪些数据 |
| `imu_save_parameters()` | `SAVE_PARAM` | `0` | 保存当前配置 |
| `imu_restore_settings()` | `RESTORE_SETTING` | `0` | 恢复出厂设置 |

使用注意：

- 主动模式下，模块会自己周期性发数据，不需要在主循环里一直 `imu_request_euler()`。
- 请求模式下，仍然需要主控主动调用 `imu_request_*()`。
- `DATA_OUTPUT_SELECTION` 的具体 bit 定义没有在随包例程里展开，当前提供通用写接口，后续若从说明书确认 bit 位，可再封装成更直观的函数。
- `imu_save_parameters()` 用于持久化 ID、波特率、模式等配置。没有保存时，重启后可能恢复旧配置。
- `imu_restore_settings()` 可能改变 ID、波特率和通信模式，调用后应准备重新匹配默认参数。

## 4. 调试增强

本次扩展了 `imu_can_status_t`：

| 字段 | 作用 |
| --- | --- |
| `last_tx_id` | 最近一次命令发送目标 CAN ID |
| `last_tx_reg` | 最近一次发送的寄存器 ID |
| `last_tx_action` | 最近一次读/写标志 |
| `last_tx_data` | 最近一次写入参数 |
| `last_tx_payload[8]` | 最近一次完整命令帧 |

这样调试时可以直接在 Watch 窗口确认当前是否真的发出了例如：

```text
CC 03 00 DD 00 00 00 00
```

这类请求欧拉角命令。

## 5. 建议使用方式

请求模式示例：

```c
IMU_CAN_Init();

while(1)
{
    imu_request_euler();
    HAL_Delay(10);
    // read imu.pitch / imu.roll / imu.yaw
}
```

主动模式示例：

```c
IMU_CAN_Init();
imu_set_active_mode_delay(10);
imu_change_to_active();
imu_save_parameters();
```

修改 ID 示例：

```c
IMU_CAN_Init();
imu_set_can_id(0x02);
imu_save_parameters();
// After the module accepts the new ID:
imu_set_local_ids(0x02, IMU_CAN_DEFAULT_MASTER_ID);
```

## 6. 本次改动清单

| 文件 | 改动 |
| --- | --- |
| `Code/ChaseNum1_IMUcan.h` | 增加通讯口/波特率枚举、温度字段、完整寄存器控制 API、TX 调试字段 |
| `Code/ChaseNum1_IMUcan.c` | 增加寄存器读写封装、控制/配置函数、温度解析、最近 TX 帧记录；同时清理旧注释导致的代码粘连风险 |
| `IMU_can/IMU_CAN_Function_Extension_Report.md` | 新增本报告 |

