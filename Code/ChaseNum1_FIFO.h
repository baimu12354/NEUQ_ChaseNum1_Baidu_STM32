#ifndef __CHASE_NUM1_FIFO_H_
#define __CHASE_NUM1_FIFO_H_

#include "stm32h7xx_hal.h"
#include "ChaseNum1_Typedef.h"

void CN_FIFO_init(CN_FIFO_struct *FIFO,uint8_t *pdata,uint16_t templen);
uint8_t CN_FIFO_read(CN_FIFO_struct *FIFO,uint8_t *pdata,uint16_t readlen);
uint8_t CN_FIFO_write(CN_FIFO_struct *FIFO,uint8_t *pdata,uint16_t writelen);
uint8_t CN_FIFO_read_Byte(CN_FIFO_struct *FIFO,uint8_t *pdata);
uint8_t CN_FIFO_write_Byte(CN_FIFO_struct *FIFO,uint8_t data);
uint16_t CN_get_FIFO_space(CN_FIFO_struct *FIFO);
uint16_t CN_get_FIFO_occupied(CN_FIFO_struct *FIFO);

#endif // !__CHASE_NUM1_FIFO_H_