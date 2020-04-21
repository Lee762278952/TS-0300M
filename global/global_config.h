#ifndef _GLOBAL_CONFIG_H_
#define _GLOBAL_CONFIG_H_

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/***********************************************************************************************
 * 									设备版本相关信息
 **********************************************************************************************/
/* 本机型号 */
#define DEVICE_MODEL													"TS_0300M"

/*  软件版本  */
#define APP_VERSION														"V1.0"

/* 打印设备信息 */
#define APP_PRINT_DEV_MSG()												APP_PrintDeviceMsg()

/* 软件编译时间 */
#define APP_BUILD_TIME													APP_BuildTime()


extern char *APP_BuildTime(void);
extern void APP_PrintDeviceMsg(void);
extern void APP_GetBuildDate(uint16_t *year,uint16_t *mon,uint16_t *day);


/***********************************************************************************************
 * 									全局定义
 **********************************************************************************************/
#define ENABLE															(true)
#define DISABLE															(false)

/* 最大数值 */
#define MAX_NUM  														(0xffffffffUL)

/* 微秒与系统Tick转换 */
#define MsToTick(ms) 													((1000L + ((uint32_t)configTICK_RATE_HZ * (uint32_t)(ms - 1U))) / 1000L)
#define TickToMs(tick) 													((tick)*1000uL / (uint32_t)configTICK_RATE_HZ)

/* OS延时函数 */
#define DELAY(ms)														vTaskDelay(MsToTick(ms))

/* null值 */
#ifndef null
#define null 															(0)
#endif

/* 错误条件判断 */
#define ERR_CHECK(condition, implement) 								do { if (!(condition)) {implement;}} while(0)
/* 带debug错误条件判断 */
#define ERR_CHECK_DBG(condition, dbg, implement) 						do { if (!(condition)) {Log.e(dbg); implement;}} while(0)


/*********************************************************************************************
 * 							    功能开关定义
 ********************************************************************************************/
/* 关闭以下功能编译和下载代码会快很多，一般测试用 */
/* WEB功能开关 */
#define WEB_FUNCTION_ENABLE												(false)

/* DSP功能开关 */
#define DSP_FUNCTION_ENABLE  											(false)

/*********************************************************************************************
 * 							    调试信息相关定义
 ********************************************************************************************/
#define LOG_SERIAL_PORT 												(LPUART1)
#define LOG_BAUDRATE													(115200U)

/*********************************************************************************************
 * 							   网络及相关默认配置
 ********************************************************************************************/
#define EX_CTRL_LOCAL_IP_DEF											{172,16,14,65}
#define EX_CTRL_GATEWAY_DEF												{172,16,14,254}
#define EX_CTRL_NETMASK_DEF												{255,255,255,0}
#define EX_CTRL_PORT_DEF												(50000)

/*********************************************************************************************
 * 							   主机及会议相关默认配置
 ********************************************************************************************/
/* 最大有线单元数 */
#define WIRED_UNIT_MAX_ONLINE_NUM										(4096)

/* 最大WIFI单元数 */
#define WIFI_UNIT_MAX_ONLINE_NUM										(300)

/* 默认最大允许开话筒数（有线） */
#define WIRED_UNIT_MAX_ALLWO_OPEN_DEF									(8)

/* 默认最大允许开话筒数（WIFI） */
#define WIFI_UNIT_MAX_ALLWO_OPEN_DEF									(6)

/* 默认语言 */
#define CONFERENCE_LANGUAGE_DEF											(Chinese)


/*******************************************************************************
 * 								外部接口/硬件配置
 ******************************************************************************/

/******************************** I2C 1 ~ 3 ************************************/

#define LPI2C1_ENABLE													ENABLE

#define LPI2C2_ENABLE													ENABLE

#define LPI2C3_ENABLE													ENABLE


#if (defined(LPI2C1_ENABLE) && LPI2C1_ENABLE) || (defined(LPI2C2_ENABLE) && LPI2C2_ENABLE) || (defined(LPI2C3_ENABLE) && LPI2C3_ENABLE)

#define I2C_CLOCK_SOURCE_DIVIDER 										(5U)
#define I2C_CLOCK_FREQ 													((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (I2C_CLOCK_SOURCE_DIVIDER + 1U))

#if defined(LPI2C1_ENABLE) && LPI2C1_ENABLE
#define LPI2C1_BAUDRATE 												(100000U)
#endif

#if defined(LPI2C2_ENABLE) && LPI2C2_ENABLE
#define LPI2C2_BAUDRATE 												(100000U)
#endif

#if defined(LPI2C3_ENABLE) && LPI2C3_ENABLE
#define LPI2C3_BAUDRATE 												(200000U)
#endif

#endif

/******************************** SPI 1 ~ 3 ************************************/

#define LPSPI1_ENABLE												ENABLE

#define LPSPI2_ENABLE												DISABLE

#define LPSPI3_ENABLE												ENABLE

#if (defined(LPSPI1_ENABLE) && LPSPI1_ENABLE) || (defined(LPSPI2_ENABLE) && LPSPI2_ENABLE) || (defined(LPSPI3_ENABLE) && LPSPI3_ENABLE)

/* Select USB1 PLL PFD0 (720 MHz) as lpspi clock source */
#define LPSPI_CLOCK_SOURCE_SELECT 									(1U)
/* Clock divider for master lpspi clock source */
#define LPSPI_CLOCK_SOURCE_DIVIDER 									(7U)

#define LPSPI_MASTER_CLK_FREQ 										(CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / (LPSPI_CLOCK_SOURCE_DIVIDER + 1U))

#define LPSPI_USE_IRQ 												DISABLE

#if defined(LPSPI1_ENABLE) && LPSPI1_ENABLE
#define LPSPI1_BAUDRATE 											(40*1000*1000U)				//40MHz
#define LPSPI1_DELAY_NANO_SEC										(0)
#define LPSPI1_MASTER_PCS											(kLPSPI_Pcs0)
#endif

#if defined(LPSPI2_ENABLE) && LPSPI2_ENABLE
#define LPSPI2_BAUDRATE 											(500000U)
#define LPSPI2_DELAY_NANO_SEC										(0)
#define LPSPI2_MASTER_PCS											(kLPSPI_Pcs0)
#endif

#if defined(LPSPI3_ENABLE) && LPSPI3_ENABLE
#define LPSPI3_BAUDRATE 											(40*1000*1000U)				//40MHz
#define LPSPI3_DELAY_NANO_SEC										(0)
#define LPSPI3_MASTER_PCS											(kLPSPI_Pcs0)
#endif

#endif

/******************************** ENET / ENET2 ************************************/
/* ENET 接口使能  */
#define ENET_ENABLE													ENABLE

/* SMI 接口使能  */
#define SMI_ENABLE													ENABLE

/* ENET2 接口使能  */
#define ENET2_ENABLE												ENABLE

/* SMI2 接口使能  */
#define SMI2_ENABLE													ENABLE


/* SMI时钟. */
#define SMI_CLOCK_NAME 												kCLOCK_CoreSysClk

/******************************** UART 1 ~ 8 ************************************/
/* UART1 接口使能  */
#define UART1_ENABLE												ENABLE

/* UART2 接口使能  */
#define UART2_ENABLE												ENABLE

/* UART3 接口使能  */
#define UART3_ENABLE												ENABLE

/* UART4 接口使能  */
#define UART4_ENABLE												DISABLE

/* UART5 接口使能  */
#define UART5_ENABLE												DISABLE

/* UART6 接口使能  */
#define UART6_ENABLE												ENABLE

/* UART7 接口使能  */
#define UART7_ENABLE												DISABLE

/* UART8 接口使能  */
#define UART8_ENABLE												ENABLE


/* 串口默认中断优先级 */
#define UART_DEF_ISR_PRIORITY										(3U)


/* UARTx_USE_IN_DRIVER宏定义用与区分接口用与HAL库调用还是驱动直接调用
   驱动调用在hal_init中初始化 */
#define UART6_USE_IN_DRIVER											ENABLE

#define UART6_BAUDRATE												(9600U)


/******************************** SAI 1 ~ 3 ************************************/

/* SAI1 接口使能  */
#define SAI1_ENABLE													DISABLE

/* SAI2 接口使能   */
#define SAI2_ENABLE													DISABLE

/* SAI3 接口使能   */	
#define SAI3_ENABLE													DISABLE

#if SAI1_ENABLE || SAI2_ENABLE || SAI3_ENABLE
/* SAI接口DMA */
#ifndef	SAI_DMA
#define SAI_DMA 													DMA0
#endif

#ifndef SAI_DMAMUX
#define SAI_DMAMUX 													DMAMUX
#endif

#endif


/*
 * AUDIO PLL config: Frequency = Fref * (DIV_SELECT + NUM / DENOM)
 *								= 24 * (32 + 77/100)
 *								= 786.48 MHz
 */
#define AUDIO_PLL_LOOP_DIVIDER										(32)
#define AUDIO_PLL_POST_DIVIDER										(1)
#define AUDIO_PLL_NUMERATOR											(77)
#define AUDIO_PLL_DENOMINATOR										(100)

#endif
