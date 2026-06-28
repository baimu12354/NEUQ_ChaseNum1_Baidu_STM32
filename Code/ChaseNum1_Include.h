#include "stm32h7xx_hal.h"
#include <math.h>

#include "ChaseNum1_Typedef.h"

#include "ChaseNum1_calcustom.h"
#include "ChaseNum1_COM.h"
#include "ChaseNum1_ENC.h"
#include "ChaseNum1_IMU.h"
#include "ChaseNum1_Motor.h"
#include "ChaseNum1_Movement.h"
#include "ChaseNum1_PID.h"
#include "ChaseNum1_PIT.h"
#include "ChaseNum1_TFT.h"
#include "ChaseNum1_Uart.h"
#include "ChaseNum1_FIFO.h"
#include "ChaseNum1_BusServo.h"
#include "ChaseNum1_MecArm.h"
#include "ChaseNum1_Pump.h"
#include "ChaseNum1_Shooter.h"
#include "ChaseNum1_NanoProtocol.h"

#include "ChaseNum1_Menu.h"

#include "3rd_Party_SDK/wit_c_sdk/wit_c_sdk.h"
#include "3rd_Party_SDK/wit_c_sdk/REG.h"
#include "ChaseNum1_TFT180/lcd_init.h"
#include "ChaseNum1_TFT180/lcd.h"

extern SYS_STATUS_STRUCT SYS_STATUS;
