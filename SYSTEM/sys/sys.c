#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32H7开发板
//系统时钟初始化	
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2017/6/8
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//********************************************************************************
//修改说明
//无
////////////////////////////////////////////////////////////////////////////////// 

//使能CPU的L1-Cache
void Cache_Enable(void)
{
    SCB_EnableICache();//使能I-Cache(指令缓存)
    SCB_EnableDCache();//使能D-Cache(数据缓存)   
	SCB->CACR|=1<<2;   //强制D-Cache透写,如不开启,实际使用中可能遇到各种问题	
}

//时钟设置函数
//Fvco=Fs*(plln/pllm);
//Fsys=Fvco/pllp=Fs*(plln/(pllm*pllp));
//Fq=Fvco/pllq=Fs*(plln/(pllm*pllq));

//Fvco:VCO频率
//Fsys:系统时钟频率,也是PLL1的p分频输出时钟频率
//Fq:PLL1的q分频输出时钟频率
//Fs:PLL输入时钟频率,可以是HSI,CSI,HSE等. 

//plln:PLL1倍频系数(PLL倍频),取值范围:4~512.
//pllm:PLL1预分频系数(进PLL之前的分频),取值范围:2~63.
//pllp:PLL1的p分频系数(PLL之后的分频),分频后作为系统时钟,取值范围:2~128.(且必须是2的倍数)
//pllq:PLL1的q分频系数(PLL之后的分频),取值范围:1~128.

//CPU频率(rcc_c_ck)=sys_d1cpre_ck=400Mhz 
//rcc_aclk=rcc_hclk3=200Mhz
//AHB1/2/3/4(rcc_hclk1/2/3/4)=200Mhz  
//APB1/2/3/4(rcc_pclk1/2/3/4)=100Mhz  
//FMC时钟频率=pll2_r_ck=((25/25)*512/2)=256Mhz

//外部晶振为25M的时候,推荐值:plln=160,pllm=5,pllp=2,pllq=2.
//得到:Fvco=25*(160/5)=800Mhz
//     Fsys=800/2=400Mhz
//     Fq=800/2=400Mhz
//返回值:0,成功;1,失败。

//spi使用pllq=8,即100Mhz的时钟频率
void Stm32_Clock_Init(u32 plln,u32 pllm,u32 pllp,u32 pllq)
{
	HAL_StatusTypeDef ret=HAL_OK;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
  
	// 核心电压配置
	MODIFY_REG(PWR->CR3,PWR_CR3_SCUEN, 0);     // 清除电源控制寄存器的某些位
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);    // 将芯片核心电压级别设置为VOS1
	while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY) {}   // 等待电压稳定到VOS1级别
  
	// 配置外部晶振和主锁相环(PLL1)
	RCC_OscInitStruct.OscillatorType=RCC_OSCILLATORTYPE_HSE;    // 配置 HSE外部高速晶振
	RCC_OscInitStruct.HSEState=RCC_HSE_ON;    // 开启 HSE外部高速晶振
	RCC_OscInitStruct.HSIState=RCC_HSI_OFF;    // 关闭内部高速晶振（精度不够）
	RCC_OscInitStruct.CSIState=RCC_CSI_OFF;    // 关闭内部低功耗晶振
	RCC_OscInitStruct.PLL.PLLState=RCC_PLL_ON;    // 开启PLL1（主锁相环 用来把25MHZ放大到400MHZ）
	RCC_OscInitStruct.PLL.PLLSource=RCC_PLLSOURCE_HSE;    // 向PLL1输入HSE外部高速晶振

	// 填入PLL1 的分频和倍频器
	RCC_OscInitStruct.PLL.PLLN=plln;    // plln:PLL1倍频系数(PLL倍频),取值范围:4~512.
	RCC_OscInitStruct.PLL.PLLM=pllm;    // pllm:PLL1预分频系数(进PLL之前的分频),取值范围:2~63.
	RCC_OscInitStruct.PLL.PLLP=pllp;    // pllp:PLL1的p分频系数(PLL之后的分频),分频后作为系统时钟,取值范围:2~128.(且必须是2的倍数)
	RCC_OscInitStruct.PLL.PLLQ=pllq;    // pllq:PLL1的q分频系数(PLL之后的分频),分频后作为SPI等特定外设的时钟,取值范围:1~128.
   
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;    // 宽泛的高频模式
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;    // 设置PLL输入频率范围
	ret=HAL_RCC_OscConfig(&RCC_OscInitStruct);    // 提交配置
	if(ret!=HAL_OK) while(1);   // 等待配置成功
  
	// 配置 SYCLK、HCLK以及各个APB/PCLK 总线
	RCC_ClkInitStruct.ClockType=(RCC_CLOCKTYPE_SYSCLK|\
                                RCC_CLOCKTYPE_HCLK |\
                                RCC_CLOCKTYPE_D1PCLK1 |\
                                RCC_CLOCKTYPE_PCLK1 |\
                                RCC_CLOCKTYPE_PCLK2 |\
                                RCC_CLOCKTYPE_D3PCLK1);

	RCC_ClkInitStruct.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;   // 系统时钟源配置为PLL1
	RCC_ClkInitStruct.SYSCLKDivider=RCC_SYSCLK_DIV1;  // 系统时钟不分频
	RCC_ClkInitStruct.AHBCLKDivider=RCC_HCLK_DIV2;    // AHB 高速总线分频为 2 (400/2 = 200MHz)
	RCC_ClkInitStruct.APB1CLKDivider=RCC_APB1_DIV2;     // APB1 低速总线分频为 2 (200/2 = 100MHz)
	RCC_ClkInitStruct.APB2CLKDivider=RCC_APB2_DIV2;     // APB2 总线分频为 2 (100MHz)
	RCC_ClkInitStruct.APB3CLKDivider=RCC_APB3_DIV2;     // APB3 总线分频为 2 (100MHz)
	RCC_ClkInitStruct.APB4CLKDivider=RCC_APB4_DIV4;     // APB4 总线分频为 4 (50MHz)
	ret=HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);    // 提交总线配置
	if(ret!=HAL_OK) while(1);  // 等待配置成功

	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};    // 单独配置ADC时钟

    PeriphClkInitStruct.PLL2.PLL2M = 4;      // 预分频系数
    PeriphClkInitStruct.PLL2.PLL2N = 160;    // 倍频系数
    PeriphClkInitStruct.PLL2.PLL2P = 7;
    PeriphClkInitStruct.PLL2.PLL2Q = 8;
    PeriphClkInitStruct.PLL2.PLL2R = 4;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;    
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 0;

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;    // 给ADC时钟源接上PLL2
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);    //提交配置

	// 读取当前外设时钟配置（推荐，避免影响其他外设）
	HAL_RCCEx_GetPeriphCLKConfig(&PeriphClkInitStruct);

	// 设置 SPI2 时钟源为 PLL1Q
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
	PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;  // SPI1/2/3 共用

	// 提交配置
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		// Error_Handler();  // 你自己的错误处理函数
	}

	__HAL_RCC_CSI_ENABLE() ;
	__HAL_RCC_SYSCFG_CLK_ENABLE() ;  
	HAL_EnableCompensationCell();    // 开启GPIO补偿单元
}


#ifdef  USE_FULL_ASSERT
//当编译提示出错的时候此函数用来报告错误的文件和所在行
//file：指向源文件
//line：指向在文件中的行数
void assert_failed(uint8_t* file, uint32_t line)
{ 
	while (1)
	{
	}
}
#endif

//判断I_Cache是否打开
//返回值:0 关闭，1 打开
u8 Get_ICahceSta(void)
{
    u8 sta;
    sta=((SCB->CCR)>>17)&0X01;
    return sta;
}

//判断I_Dache是否打开
//返回值:0 关闭，1 打开
u8 Get_DCahceSta(void)
{
    u8 sta;
    sta=((SCB->CCR)>>16)&0X01;
    return sta;
}

#if defined(__clang__) //使用V6编译器(clang)
//THUMB指令不支持汇编内联
//采用如下方法实现执行汇编指令WFI  
void __attribute__((noinline)) WFI_SET(void)
{
    __asm__("wfi");
}

//关闭所有中断(但是不包括fault和NMI中断)   
void __attribute__((noinline)) INTX_DISABLE(void)
{
    __asm__("cpsid i \t\n"
            "bx lr");
}

//开启所有中断
void __attribute__((noinline)) INTX_ENABLE(void)
{
    __asm__("cpsie i \t\n"
            "bx lr");
}

//设置栈顶地址
//addr:栈顶地址
void __attribute__((noinline)) MSR_MSP(u32 addr) 
{
    __asm__("msr msp, r0 \t\n"
            "bx r14");
}
#elif defined (__CC_ARM)    //使用V5编译器(ARMCC)

//THUMB指令不支持汇编内联
//采用如下方法实现执行汇编指令WFI  
__asm void WFI_SET(void)
{
	WFI;		  
}
//关闭所有中断(但是不包括fault和NMI中断)
__asm void INTX_DISABLE(void)
{
	CPSID   I
	BX      LR	  
}
//开启所有中断
__asm void INTX_ENABLE(void)
{
	CPSIE   I
	BX      LR  
}
//设置栈顶地址
//addr:栈顶地址
__asm void MSR_MSP(u32 addr) 
{
	MSR MSP, r0 			//set Main Stack value
	BX r14
}
#endif
