# FIFO 越界修复与运行状态诊断屏报告

## 修改目的

本次修改解决两个问题：

1. 修复串口发送 FIFO 在多次读写后可能出现的 head/tail 指针越界问题。
2. 利用现有 TFT 屏幕显示 MCU 运行状态和 UART2/NANO 通信诊断信息，方便下次偶发卡死时判断是主循环卡死、串口硬件错误、协议解析异常，还是发送 FIFO 堵塞。

本次没有修改现有 NANO 协议帧格式、上位机命令格式、下位机回包格式，也没有改变原有命令处理逻辑。

## FIFO 越界修复

修改文件：

- `Code/ChaseNum1_FIFO.c`

原问题：

- `CN_FIFO_read_Byte()` 和 `CN_FIFO_write_Byte()` 原来使用 `if(FIFO->head++ >= FIFO->temp_len)` 这种后自增判断。
- 当 `head` 或 `tail` 等于 `temp_len - 1` 时，自增后会变成 `temp_len`，但判断用的是自增前的值，导致下一次访问 `temp[temp_len]`，发生数组越界。
- 这种越界会破坏相邻变量，表现可能是偶发卡死、串口异常、协议状态异常。

修复方式：

- 先执行 `head++` 或 `tail++`。
- 再判断是否 `>= temp_len`。
- 若达到边界，立即回绕到 0。

修复后 head/tail 不会再等于 `temp_len`，只会保持在 `0 ~ temp_len - 1`。

## UART2 诊断信息

修改文件：

- `Code/ChaseNum1_Uart.c`
- `Code/ChaseNum1_Uart.h`

新增结构体：

- `CN_UART_Diag_t`

屏幕主要使用 UART2 的这些信息：

- `uart_error_count`：UART 硬件错误总次数。
- `uart_overrun_count`：接收溢出错误次数，常见于上位机发送过快或中断处理不及时。
- `uart_frame_error_count`：帧错误次数，可能和波特率、线缆、干扰有关。
- `uart_noise_error_count`：噪声错误次数，可能和电气干扰有关。
- `tx_fifo_occupied`：UART2 发送 FIFO 当前占用字节数。
- `tx_fifo_write_fail_count`：发送 FIFO 写入失败次数，说明回包太密或串口发送不出去。
- `hal_error_code`：HAL 最近一次 UART 错误码。

注意：

- UART 错误计数对所有串口都记录。
- 目前只对 UART2 保持原有错误恢复动作，即清错误标志、AbortReceive、重新开启 1 字节接收。
- 这样不会影响其他外设串口行为。

## NANO 协议诊断信息

修改文件：

- `Code/ChaseNum1_NanoProtocol.c`
- `Code/ChaseNum1_NanoProtocol.h`

新增结构体：

- `Nano_COM_Diag_t`

新增接口：

- `Nano_COM_Get_Diag()`

统计内容：

- `task_loop_count`：NANO 主循环执行次数，用于判断主循环是否还活着。
- `last_loop_ms`：最近一次主循环执行时的 `HAL_GetTick()`。
- `rx_byte_count`：UART2 收到的字节总数。
- `rx_frame_count`：成功解析出的完整协议帧数量。
- `rx_process_count`：已经进入业务处理的协议帧数量。
- `rx_drop_count`：UART2 软件接收环形缓冲满时丢弃旧字节的次数。
- `rx_busy_drop_count`：上一帧还没处理完，又收到完整新帧导致丢弃的次数。
- `rx_bad_check_count`：校验错误次数。
- `rx_bad_tail_count`：帧尾错误次数。
- `rx_bad_len_count`：载荷长度超过协议最大值的次数。
- `tx_frame_count`：NANO 协议发送回包帧数量。
- `rx_ring_occupied`：UART2 软件接收环形缓冲当前占用字节数。
- `package_ready`：当前是否有待处理完整包。
- `rx_state`：当前协议解析状态机状态。
- `system_state/chassis_state/arm_state/pump_state`：系统、底盘、机械臂、气泵状态。

## 屏幕状态页

修改文件：

- `Code/ChaseNum1_Menu.c`
- `Code/ChaseNum1_Menu.h`
- `USER/main.c`

屏幕初始化：

- 在 `main()` 初始化阶段调用 `Menu_init()`。
- `Menu_init()` 内部调用 `TS_TFT180_init()` 初始化 TFT。

屏幕刷新：

- 在主循环中通过原有 `PrintFlag` 节拍调用 `Menu_poll()`。
- `Menu_poll()` 内部限制约 250 ms 刷新一次，避免 TFT 刷新占用过多主循环时间。

屏幕字段含义：

- `ALIVE`：NANO 主循环累计运行次数，持续变化说明主循环仍在运行。
- `HB`：心跳翻转标志，0/1 交替变化说明屏幕刷新和主循环都正常。
- `SYS/C/ARM/P`：系统、底盘、机械臂、气泵状态。
- `RXB/FR`：UART2 收到字节数 / 成功解析完整帧数。
- `PROC/TX`：已处理命令帧数 / 已发送回包帧数。
- `RING/RD`：接收环形缓冲占用 / 接收环形缓冲丢字节次数。
- `CHK/TAIL`：校验错误 / 帧尾错误。
- `LEN/BUSY`：长度错误 / 业务未处理完导致的新帧丢弃。
- `UERR/ORE`：UART 硬件错误 / 接收溢出错误。
- `FE/NE`：帧错误 / 噪声错误。
- `TXF/F`：UART2 发送 FIFO 占用 / 发送 FIFO 写失败次数。
- `READY/RS`：是否有待处理完整包 / 当前协议解析状态。
- `LOOPMS/HE`：最近主循环时间戳 / HAL 最近一次 UART 错误码。

## 现象判断方法

1. `ALIVE` 和 `HB` 不变化：
   - 主循环可能卡死，或者系统已经进硬故障。

2. `ALIVE` 正常变化，但 `RXB` 不增加：
   - 上位机可能没发数据，线缆/串口连接可能有问题，或 UART2 接收没进中断。

3. `RXB` 增加，但 `FR` 不增加：
   - 收到了字节，但协议帧没解析成功，重点看 `CHK/TAIL/LEN`。

4. `ORE` 增加：
   - UART2 接收过载，上位机发送频率过高或 MCU 中断/主循环处理不及时。

5. `RD` 增加：
   - UART2 软件接收环形缓冲满了，说明上位机发包速度超过下位机消费速度。

6. `BUSY` 增加：
   - 下位机一次只能暂存一个完整待处理包，上位机连续发包太快，上一帧未处理完时新帧到达。

7. `TXF` 持续接近 256 或 `F` 增加：
   - UART2 回包发送拥塞，可能是上位机读取慢、回包太密、串口发送异常。

## 编译验证

已单独编译以下文件，均为 `0 errors`：

- `Code/ChaseNum1_FIFO.c`
- `Code/ChaseNum1_Uart.c`
- `Code/ChaseNum1_NanoProtocol.c`
- `Code/ChaseNum1_Menu.c`
- `USER/main.c`

现有 ARMCC 警告主要为工程原有 `#pragma region` 不识别，未由本次修改引入。
