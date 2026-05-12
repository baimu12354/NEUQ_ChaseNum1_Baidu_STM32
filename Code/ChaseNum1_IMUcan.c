#include "ChaseNum1_Include.h"
#include <string.h>

FDCAN_HandleTypeDef hfdcan1;
imu_t imu;
volatile imu_can_status_t imu_can_status;

static void IMU_CAN_UpdateStatus(void);
static void IMU_UpdateAccel(uint8_t *pData);
static void IMU_UpdateGyro(uint8_t *pData);
static void IMU_UpdateEuler(uint8_t *pData);
static void IMU_UpdateQuaternion(uint8_t *pData);

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcanHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    if(fdcanHandle->Instance == FDCAN1)
    {
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
        PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
        if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        {
            return;
        }

        __HAL_RCC_FDCAN_CLK_ENABLE();

#if IMU_CAN_USE_FDCAN1_PD01
        __HAL_RCC_GPIOD_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
#else
        __HAL_RCC_GPIOH_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
#endif
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;

#if IMU_CAN_USE_FDCAN1_PD01
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
#else
        HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
#endif

        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
        HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
    }
}

void IMU_CAN_Init(void)
{
    FDCAN_FilterTypeDef fdcan_filter;

    memset(&hfdcan1, 0, sizeof(hfdcan1));
    memset((void *)&imu_can_status, 0, sizeof(imu_can_status));
    memset(&imu, 0, sizeof(imu));

    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = DISABLE;
    hfdcan1.Init.TransmitPause = DISABLE;
    hfdcan1.Init.ProtocolException = DISABLE;

    // PLL1Q = 100MHz, bitrate = 100MHz / (20 * (1 + 2 + 2)) = 1Mbps.
    hfdcan1.Init.NominalPrescaler = 20;
    hfdcan1.Init.NominalSyncJumpWidth = 1;
    hfdcan1.Init.NominalTimeSeg1 = 2;
    hfdcan1.Init.NominalTimeSeg2 = 2;

    hfdcan1.Init.DataPrescaler = 1;
    hfdcan1.Init.DataSyncJumpWidth = 1;
    hfdcan1.Init.DataTimeSeg1 = 1;
    hfdcan1.Init.DataTimeSeg2 = 1;

    hfdcan1.Init.MessageRAMOffset = 0;
    hfdcan1.Init.StdFiltersNbr = 1;
    hfdcan1.Init.ExtFiltersNbr = 0;
    hfdcan1.Init.RxFifo0ElmtsNbr = 32;
    hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.RxFifo1ElmtsNbr = 0;
    hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.RxBuffersNbr = 0;
    hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.TxEventsNbr = 0;
    hfdcan1.Init.TxBuffersNbr = 0;
    hfdcan1.Init.TxFifoQueueElmtsNbr = 32;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;

    if(HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
    {
        imu_can_status.init_error_step = 1u;
        IMU_CAN_UpdateStatus();
        return;
    }

    fdcan_filter.IdType = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_RANGE;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x01;
    fdcan_filter.FilterID2 = 0xFF;
    if(HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter) != HAL_OK)
    {
        imu_can_status.init_error_step = 2u;
        IMU_CAN_UpdateStatus();
        return;
    }

    if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, ENABLE, ENABLE) != HAL_OK)
    {
        imu_can_status.init_error_step = 3u;
        IMU_CAN_UpdateStatus();
        return;
    }

    if(HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1) != HAL_OK)
    {
        imu_can_status.init_error_step = 4u;
        IMU_CAN_UpdateStatus();
        return;
    }

    if(HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0) != HAL_OK)
    {
        imu_can_status.init_error_step = 5u;
        IMU_CAN_UpdateStatus();
        return;
    }

    if(HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        imu_can_status.init_error_step = 6u;
        IMU_CAN_UpdateStatus();
        return;
    }

    if(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        imu_can_status.init_error_step = 7u;
        IMU_CAN_UpdateStatus();
        return;
    }

    imu.can_id = IMU_CAN_DEFAULT_DEVICE_ID;
    imu.mst_id = IMU_CAN_DEFAULT_MASTER_ID;
    imu.can_handle = &hfdcan1;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if(hfdcan->Instance == FDCAN1)
        {
            FDCAN_RxHeaderTypeDef rx_header;
            uint8_t rx_data[8] = {0};
            uint32_t guard = 0;

            imu_can_status.rx_callback_count++;
            while((HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u) && (guard < 32u))
            {
                guard++;
                if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
                {
                    IMU_CAN_UpdateStatus();
                    break;
                }

                imu_can_status.rx_frame_count++;
                imu_can_status.last_rx_id = rx_header.Identifier;
                imu_can_status.last_rx_dlc = rx_header.DataLength;
                for(uint32_t i = 0; i < 8u; i++)
                {
                    imu_can_status.last_rx_data[i] = rx_data[i];
                }

                if((rx_header.IdType != FDCAN_STANDARD_ID) ||
                   (rx_header.RxFrameType != FDCAN_DATA_FRAME) ||
                   (rx_header.DataLength != FDCAN_DLC_BYTES_8))
                {
                    imu_can_status.rx_bad_len_count++;
                    continue;
                }

                switch(rx_data[0])
                {
                    case 1: IMU_UpdateAccel(rx_data); break;
                    case 2: IMU_UpdateGyro(rx_data); break;
                    case 3: IMU_UpdateEuler(rx_data); break;
                    case 4: IMU_UpdateQuaternion(rx_data); break;
                    default: break;
                }
            }

            if(HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
            {
                IMU_CAN_UpdateStatus();
            }
        }
    }
}

static void IMU_CAN_UpdateStatus(void)
{
    FDCAN_ProtocolStatusTypeDef protocol_status;

    imu_can_status.last_hal_error = HAL_FDCAN_GetError(&hfdcan1);
    if(HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol_status) == HAL_OK)
    {
        imu_can_status.last_protocol_error = protocol_status.LastErrorCode;
        imu_can_status.bus_off = protocol_status.BusOff;
        imu_can_status.error_passive = protocol_status.ErrorPassive;
        imu_can_status.warning = protocol_status.Warning;
    }
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;

    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void imu_send_cmd(uint8_t reg_id, uint8_t ac, uint32_t data)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t buf[8] = {0xCC, reg_id, ac, 0xDD, 0, 0, 0, 0};

    if(imu.can_handle == NULL)
    {
        return;
    }

    memcpy(buf + 4, &data, 4);

    imu_can_status.last_tx_id = imu.can_id;
    imu_can_status.last_tx_reg = reg_id;
    imu_can_status.last_tx_action = ac;
    imu_can_status.last_tx_data = data;
    for(uint32_t i = 0; i < 8u; i++)
    {
        imu_can_status.last_tx_payload[i] = buf[i];
    }

    tx_header.Identifier = imu.can_id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0x00;

    if(HAL_FDCAN_GetTxFifoFreeLevel(imu.can_handle) > 0u)
    {
        if(HAL_FDCAN_AddMessageToTxFifoQ(imu.can_handle, &tx_header, buf) == HAL_OK)
        {
            imu_can_status.tx_success_count++;
        }
        else
        {
            imu_can_status.tx_fail_count++;
            IMU_CAN_UpdateStatus();
        }
    }
    else
    {
        imu_can_status.tx_no_fifo_count++;
    }
}

void imu_write_reg(uint8_t reg_id, uint32_t data)
{
    imu_send_cmd(reg_id, CMD_WRITE, data);
}

void imu_read_reg(uint8_t reg_id)
{
    imu_send_cmd(reg_id, CMD_READ, 0);
}

void imu_reboot(void)
{
    imu_write_reg(REBOOT_IMU, 0);
}

void imu_set_zero(void)
{
    imu_write_reg(SET_ZERO, 0);
}

void imu_accel_calibration(void)
{
    imu_write_reg(ACCEL_CALI, 0);
}

void imu_gyro_calibration(void)
{
    imu_write_reg(GYRO_CALI, 0);
}

void imu_mag_calibration(void)
{
    imu_write_reg(MAG_CALI, 0);
}

void imu_change_com_port(imu_com_port_e port)
{
    imu_write_reg(CHANGE_COM, (uint8_t)port);
}

void imu_set_active_mode_delay(uint32_t delay)
{
    imu_write_reg(SET_DELAY, delay);
}

void imu_change_to_active(void)
{
    imu_write_reg(CHANGE_ACTIVE, 1);
}

void imu_change_to_request(void)
{
    imu_write_reg(CHANGE_ACTIVE, 0);
}

void imu_set_baud(imu_baudrate_e baud)
{
    imu_write_reg(SET_BAUD, (uint8_t)baud);
}

void imu_set_can_id(uint8_t can_id)
{
    imu_write_reg(SET_CAN_ID, can_id);
}

void imu_set_mst_id(uint8_t mst_id)
{
    imu_write_reg(SET_MST_ID, mst_id);
}

void imu_set_data_output_selection(uint32_t selection)
{
    imu_write_reg(DATA_OUTPUT_SELECTION, selection);
}

void imu_save_parameters(void)
{
    imu_write_reg(SAVE_PARAM, 0);
}

void imu_restore_settings(void)
{
    imu_write_reg(RESTORE_SETTING, 0);
}

void imu_set_local_ids(uint8_t can_id, uint8_t mst_id)
{
    imu.can_id = can_id;
    imu.mst_id = mst_id;
}

void imu_request_accel(void)
{
    imu_read_reg(ACCEL_DATA);
}

void imu_request_gyro(void)
{
    imu_read_reg(GYRO_DATA);
}

void imu_request_euler(void)
{
    imu_read_reg(EULER_DATA);
}

void imu_request_quat(void)
{
    imu_read_reg(QUAT_DATA);
}

static void IMU_UpdateAccel(uint8_t *pData)
{
    uint16_t accel[3];
    uint16_t temp;

    temp = pData[1];
    accel[0] = pData[3] << 8 | pData[2];
    accel[1] = pData[5] << 8 | pData[4];
    accel[2] = pData[7] << 8 | pData[6];

    imu.cur_temp = (float)temp;
    imu.accel[0] = uint_to_float(accel[0], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
    imu.accel[1] = uint_to_float(accel[1], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
    imu.accel[2] = uint_to_float(accel[2], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
}

static void IMU_UpdateGyro(uint8_t *pData)
{
    uint16_t gyro[3];

    gyro[0] = pData[3] << 8 | pData[2];
    gyro[1] = pData[5] << 8 | pData[4];
    gyro[2] = pData[7] << 8 | pData[6];

    imu.gyro[0] = uint_to_float(gyro[0], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
    imu.gyro[1] = uint_to_float(gyro[1], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
    imu.gyro[2] = uint_to_float(gyro[2], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
}

static void IMU_UpdateEuler(uint8_t *pData)
{
    uint16_t euler[3];

    euler[0] = pData[3] << 8 | pData[2];
    euler[1] = pData[5] << 8 | pData[4];
    euler[2] = pData[7] << 8 | pData[6];

    imu.pitch = uint_to_float(euler[0], PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
    imu.yaw   = uint_to_float(euler[1], YAW_CAN_MIN, YAW_CAN_MAX, 16);
    imu.roll  = uint_to_float(euler[2], ROLL_CAN_MIN, ROLL_CAN_MAX, 16);
}

static void IMU_UpdateQuaternion(uint8_t *pData)
{
    int w = pData[1] << 6 | ((pData[2] & 0xF8) >> 2);
    int x = (pData[2] & 0x03) << 12 | (pData[3] << 4) | ((pData[4] & 0xF0) >> 4);
    int y = (pData[4] & 0x0F) << 10 | (pData[5] << 2) | ((pData[6] & 0xC0) >> 6);
    int z = (pData[6] & 0x3F) << 8 | pData[7];

    imu.q[0] = uint_to_float(w, Quaternion_MIN, Quaternion_MAX, 14);
    imu.q[1] = uint_to_float(x, Quaternion_MIN, Quaternion_MAX, 14);
    imu.q[2] = uint_to_float(y, Quaternion_MIN, Quaternion_MAX, 14);
    imu.q[3] = uint_to_float(z, Quaternion_MIN, Quaternion_MAX, 14);
}
