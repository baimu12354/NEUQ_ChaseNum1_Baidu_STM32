# IMU CAN 驱动修改报告

## 1. 问题结论

这次 CAN 通讯没有回调，主要不是 IMU 解包函数本身的问题，而是 FDCAN 底层链路没有完整接起来：

- 工程里缺少 `FDCAN1_IT0_IRQHandler()`，即使 FIFO0 收到报文，HAL 的 `HAL_FDCAN_IRQHandler()` 也不会被调用，`HAL_FDCAN_RxFifo0Callback()` 自然没有反应。
- 原驱动发送命令时使用了 `imu.mst_id` 作为 CAN 标准 ID；对照 `IMU_can/MC02_CAN收发例程/DM-IMU/dm_imu.c`，命令应该发给 IMU 设备 ID `imu.can_id`。
- 原驱动的 FDCAN 初始化字段不完整，部分 Message RAM 配置字段依赖未清零的句柄状态，存在隐性风险。
- 原驱动注释写的是 PH13/PH14，但例程和 `.ioc` 使用的是 FDCAN1 的 PD0/PD1。如果实际接线按例程走，原引脚配置会导致硬件层完全收不到。
- 原接收回调只取一帧，没有记录错误状态；出现波特率、引脚、ID、Bus-Off 等问题时不容易定位。

## 2. 参考依据

参考文件：

- `IMU_can/MC02_CAN收发例程/Core/Src/fdcan.c`
- `IMU_can/MC02_CAN收发例程/User/can_bsp.c`
- `IMU_can/MC02_CAN收发例程/DM-IMU/dm_imu.c`
- `IMU_can/MC02_CAN收发例程/CtrBoard-H7_FDCAN.ioc`

关键对比：

| 项目 | 例程 | 原工程问题 | 当前修改 |
| --- | --- | --- | --- |
| 命令发送 ID | `tx_header.Identifier = imu.can_id` | 发给了 `imu.mst_id` | 改为发给 `imu.can_id` |
| FDCAN1 引脚 | PD0 RX / PD1 TX | 注释和配置为 PH13/PH14 | 默认改为 PD0/PD1，并保留宏切换到 PH14/PH13 |
| 接收中断 | 启动 FDCAN 后激活 FIFO0 新消息通知 | 缺少 `FDCAN1_IT0_IRQHandler()` | 补齐 IT0/IT1 handler |
| Message RAM | FIFO、Buffer、TxEvent 字段都配置 | 原工程只配置了一部分 | 补齐未配置字段 |
| 回调处理 | FIFO0 收到后取报文并解包 | 只取一帧，缺少状态记录 | 循环清空 FIFO0，并记录收发/错误状态 |

说明：本地没有可用的 PDF 文本解析工具，说明书 `达妙科技DM-IMU-L1六轴IMU模块使用说明书V1.2.pdf` 未能可靠抽取文字；本报告中的确定结论来自随仓库提供的官方例程和 `.ioc` 配置。

## 3. 代码改动说明

### 3.1 `Code/ChaseNum1_IMUcan.c`

1. 增加 `<string.h>`。
   - 原因：驱动中使用了 `memset()` 和 `memcpy()`，显式包含头文件可以避免编译器隐式声明问题。

2. 增加 `volatile imu_can_status_t imu_can_status`。
   - 原因：CAN 通讯失败时需要能在调试器里直接看到初始化步骤、收发计数、最后一次接收 ID、DLC、数据和 HAL 错误状态。

3. `HAL_FDCAN_MspInit()` 默认改为 PD0/PD1。
   - 原因：例程 `.ioc` 中 FDCAN1 为 `PD0=RX`、`PD1=TX`。
   - 兼容：如果实际硬件接在 `PH14=RX`、`PH13=TX`，把 `Code/ChaseNum1_IMUcan.h` 里的 `IMU_CAN_USE_FDCAN1_PD01` 改为 `0` 即可。

4. 同时开启 `FDCAN1_IT0_IRQn` 和 `FDCAN1_IT1_IRQn`。
   - 原因：FIFO0 新消息当前映射到 interrupt line 0，IT0 是必要的；IT1 保留给后续错误/状态中断扩展。

5. 新增 `IMU_CAN_UpdateStatus()`。
   - 原因：统一读取 `HAL_FDCAN_GetError()` 和 `HAL_FDCAN_GetProtocolStatus()`，方便判断是否 Bus-Off、Error Passive、Warning 或协议错误。

6. `IMU_CAN_Init()` 开头清零 `hfdcan1`、`imu` 和状态结构。
   - 原因：避免上一次运行或未初始化字段影响 HAL 初始化，特别是 FDCAN Message RAM 相关字段。

7. 补齐 FDCAN 初始化字段。
   - 新增 `RxFifo1ElmtsNbr`、`RxBuffersNbr`、`TxEventsNbr`、`TxBuffersNbr` 等字段。
   - 原因：对齐例程 `fdcan.c`，让 Message RAM 布局完整明确。

8. 检查每一步 HAL 初始化返回值，并写入 `init_error_step`。
   - `1`：`HAL_FDCAN_Init()` 失败
   - `2`：`HAL_FDCAN_ConfigFilter()` 失败
   - `3`：`HAL_FDCAN_ConfigGlobalFilter()` 失败
   - `4`：`HAL_FDCAN_ConfigFifoWatermark()` 失败
   - `5`：`HAL_FDCAN_ConfigInterruptLines()` 失败
   - `6`：`HAL_FDCAN_Start()` 失败
   - `7`：`HAL_FDCAN_ActivateNotification()` 失败

9. 接收回调改为循环读取 FIFO0。
   - 原因：中断触发时 FIFO 中可能已有多帧，只取一帧可能造成积压。
   - 同时校验标准帧、数据帧、8 字节 DLC，不符合就计入 `rx_bad_len_count`。

10. 发送命令的标准 ID 改为 `imu.can_id`。
    - 原因：例程 `dm_imu.c` 明确使用 IMU 设备 ID 发送命令。`mst_id` 是主机 ID，不应该作为命令目标 ID。

11. 发送路径增加计数。
    - `tx_success_count`：成功压入 TX FIFO
    - `tx_fail_count`：HAL 发送失败
    - `tx_no_fifo_count`：TX FIFO 无空位

### 3.2 `Code/ChaseNum1_IMUcan.h`

1. 增加默认配置宏：
   - `IMU_CAN_USE_FDCAN1_PD01`
   - `IMU_CAN_DEFAULT_DEVICE_ID`
   - `IMU_CAN_DEFAULT_MASTER_ID`

2. 增加 `imu_can_status_t`。
   - 原因：把通讯状态暴露给调试器，定位“无回调、无响应、总线错误、ID 不对、DLC 不对”等问题。

3. 导出 `extern volatile imu_can_status_t imu_can_status`。
   - 原因：主循环或调试器可以直接读取状态。

### 3.3 `USER/stm32h7xx_it.c`

1. 增加 `#include "../Code/ChaseNum1_IMUcan.h"`。
   - 原因：中断函数需要访问 `hfdcan1`。

2. 增加：
   - `FDCAN1_IT0_IRQHandler()`
   - `FDCAN1_IT1_IRQHandler()`

   原因：启动文件中已有这两个弱符号，必须在用户中断文件里实现并调用 `HAL_FDCAN_IRQHandler(&hfdcan1)`，HAL 才能进一步进入 `HAL_FDCAN_RxFifo0Callback()`。

### 3.4 `USER/stm32h7xx_it.h`

增加 FDCAN1 IT0/IT1 中断函数声明，保持头文件声明完整。

## 4. 使用和排查建议

调试时重点观察 `imu_can_status`：

| 字段 | 含义 | 判断 |
| --- | --- | --- |
| `init_error_step` | 初始化失败步骤 | 为 0 表示初始化流程通过 |
| `tx_success_count` | 成功写入 TX FIFO 次数 | 调用 `imu_request_euler()` 后应增加 |
| `rx_callback_count` | FIFO0 中断回调次数 | 一直为 0 说明中断/引脚/波特率/收发器链路有问题 |
| `rx_frame_count` | 实际取出的 CAN 帧数 | 大于 0 说明已经收到报文 |
| `last_rx_id` | 最近一次接收 ID | 用于确认 IMU 回包 ID 是否落在滤波范围内 |
| `last_rx_dlc` | 最近一次 DLC | 正常应为 `FDCAN_DLC_BYTES_8` |
| `last_rx_data[8]` | 最近一帧原始数据 | 第 0 字节为 1/2/3/4 时会进入对应解包 |
| `bus_off` | CAN Bus-Off | 为 1 通常是波特率、终端电阻、接线或 ACK 问题 |
| `error_passive` / `warning` | CAN 错误状态 | 非 0 时优先检查物理层和波特率 |

硬件侧请确认：

- IMU CANH/CANL 没接反。
- 总线上有 120 欧终端电阻。
- MCU 到外部 CAN 收发器连接正确，FDCAN_TX/FDCAN_RX 不是直接接 CANH/CANL。
- 当前代码默认使用 PD0/PD1；如果板子实际接 PH14/PH13，需要修改 `IMU_CAN_USE_FDCAN1_PD01`。
- 当前工程按 `Stm32_Clock_Init(160,5,2,8)` 推算 FDCAN 时钟为 PLL1Q=100 MHz，代码使用 `NominalPrescaler=20`、`TimeSeg1=2`、`TimeSeg2=2`，即 1 Mbps。例程 `.ioc` 的 FDCAN 时钟为 120 MHz、Prescaler=24，也同样得到 1 Mbps。
- 如果 IMU 实际被配置成其他 CAN 波特率或 ID，需要同步修改 `IMU_CAN_DEFAULT_DEVICE_ID` 和 FDCAN 位时序。

## 5. 当前仍需现场确认的点

- 实际板子 FDCAN1 引脚到底是 PD0/PD1 还是 PH14/PH13。
- IMU 当前保存的 CAN ID 是否仍为默认 `0x01`。
- IMU 当前 CAN 波特率是否为 1 Mbps。
- IMU 是否处于 CAN 通讯模式；若之前被切到 UART 或其他模式，需要用官方工具或对应命令切回。

