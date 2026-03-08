#include "TopSpeed_Include.h"

TOF_Handle_struct TOF_Handle={0};

#if TOF_USE_VL53L1X

I2C_HandleTypeDef Hard_IIC_2_Handle={0};

void Tof_init(void){
    __HAL_RCC_I2C2_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_initure={0};

    GPIO_initure.Pin=GPIO_PIN_4;
    GPIO_initure.Mode=GPIO_MODE_AF_PP;
    GPIO_initure.Pull=GPIO_NOPULL;
    GPIO_initure.Speed=GPIO_SPEED_FREQ_HIGH;
    GPIO_initure.Alternate=GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOH,&GPIO_initure);
    GPIO_initure.Pin=GPIO_PIN_5;
    GPIO_initure.Mode=GPIO_MODE_AF_OD;
    HAL_GPIO_Init(GPIOH,&GPIO_initure);

    Hard_IIC_2_Handle.Instance=I2C2;

    Hard_IIC_2_Handle.Init.Timing=0x10C0ECFF;
    Hard_IIC_2_Handle.Init.OwnAddress1=0;
    Hard_IIC_2_Handle.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT;
    Hard_IIC_2_Handle.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE;
    Hard_IIC_2_Handle.Init.OwnAddress2=0;
    Hard_IIC_2_Handle.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE;
    Hard_IIC_2_Handle.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;

    printf("iic init res:%d\n",HAL_I2C_Init(&Hard_IIC_2_Handle));

}


#endif

#if TOF_USE_WT_VL53

#define WT_VL53_MAX_DIS (1000)


#define TS_TOF_RX_BUF_SIZE 128
static char rx_buf[TS_TOF_RX_BUF_SIZE];
static uint8_t rx_index = 0;

void Tof_init(void){
    TS_UART7_init();
    TOF_Handle.Status=TOF_Status_Valid;
}

void TS_UART7_RX_CALLBACK(uint8_t dat){
    if (rx_index >= TS_TOF_RX_BUF_SIZE - 1) {
        rx_index = 0; // overflow protection
    }

    rx_buf[rx_index++] = dat;
    rx_buf[rx_index] = '\0';

    // 检测行结束（\r\n）
    if (rx_index >= 2 && rx_buf[rx_index - 2] == '\r' && rx_buf[rx_index - 1] == '\n') {
        // 处理完整一行
        if (strncmp(rx_buf, "State;", 6) == 0) {
            int state_val = -1;
            if (sscanf(rx_buf, "State;%d", &state_val) == 1) {
                if(state_val==0){
                    TOF_Handle.Status=TOF_Status_Valid;
                }
                else if(state_val==3 || state_val==4){
                    TOF_Handle.Status=TOF_Status_OverRange;
                }
                else{
                    TOF_Handle.Status=TOF_Status_ERR;
                }
                // printf("state:%d,",state_val);
            }
        }
        else if (strncmp(rx_buf, "d:", 2) == 0) {
            int dist_val = -1;
            if (sscanf(rx_buf, "d:%d", &dist_val) == 1) {
                if(dist_val>0 && dist_val<WT_VL53_MAX_DIS && TOF_Handle.Status==TOF_Status_Valid){
                    TOF_Handle.Distance_mm=dist_val;
                    // printf("distance:%d\n",dist_val);
                }
            }
        }

        // 清空接收缓冲
        rx_index = 0;
        rx_buf[0] = '\0';
    }
}

uint32_t TS_get_Tof(void){
    return TOF_Handle.Distance_mm;
}   

#endif