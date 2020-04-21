#ifndef _GLOBAL_CONFIG_H_
#define _GLOBAL_CONFIG_H_

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/***********************************************************************************************
 * 									�豸�汾�����Ϣ
 **********************************************************************************************/
/* �����ͺ� */
#define DEVICE_MODEL													"TS_0300M"

/*  ����汾  */
#define APP_VERSION														"V1.0"

/* ��ӡ�豸��Ϣ */
#define APP_PRINT_DEV_MSG()												APP_PrintDeviceMsg()

/* �������ʱ�� */
#define APP_BUILD_TIME													APP_BuildTime()


extern char *APP_BuildTime(void);
extern void APP_PrintDeviceMsg(void);
extern void APP_GetBuildDate(uint16_t *year,uint16_t *mon,uint16_t *day);


/***********************************************************************************************
 * 									ȫ�ֶ���
 **********************************************************************************************/
#define ENABLE															(true)
#define DISABLE															(false)

/* �����ֵ */
#define MAX_NUM  														(0xffffffffUL)

/* ΢����ϵͳTickת�� */
#define MsToTick(ms) 													((1000L + ((uint32_t)configTICK_RATE_HZ * (uint32_t)(ms - 1U))) / 1000L)
#define TickToMs(tick) 													((tick)*1000uL / (uint32_t)configTICK_RATE_HZ)

/* OS��ʱ���� */
#define DELAY(ms)														vTaskDelay(MsToTick(ms))

/* nullֵ */
#ifndef null
#define null 															(0)
#endif

/* ���������ж� */
#define ERR_CHECK(condition, implement) 								do { if (!(condition)) {implement;}} while(0)
/* ��debug���������ж� */
#define ERR_CHECK_DBG(condition, dbg, implement) 						do { if (!(condition)) {Log.e(dbg); implement;}} while(0)


/*********************************************************************************************
 * 							    ���ܿ��ض���
 ********************************************************************************************/
/* �ر����¹��ܱ�������ش�����ܶ࣬һ������� */
/* WEB���ܿ��� */
#define WEB_FUNCTION_ENABLE												(false)

/* DSP���ܿ��� */
#define DSP_FUNCTION_ENABLE  											(false)

/*********************************************************************************************
 * 							    ������Ϣ��ض���
 ********************************************************************************************/
#define LOG_SERIAL_PORT 												(LPUART1)
#define LOG_BAUDRATE													(115200U)

/*********************************************************************************************
 * 							   ���缰���Ĭ������
 ********************************************************************************************/
#define EX_CTRL_LOCAL_IP_DEF											{172,16,14,65}
#define EX_CTRL_GATEWAY_DEF												{172,16,14,254}
#define EX_CTRL_NETMASK_DEF												{255,255,255,0}
#define EX_CTRL_PORT_DEF												(50000)

/*********************************************************************************************
 * 							   �������������Ĭ������
 ********************************************************************************************/
/* ������ߵ�Ԫ�� */
#define WIRED_UNIT_MAX_ONLINE_NUM										(4096)

/* ���WIFI��Ԫ�� */
#define WIFI_UNIT_MAX_ONLINE_NUM										(300)

/* Ĭ�����������Ͳ�������ߣ� */
#define WIRED_UNIT_MAX_ALLWO_OPEN_DEF									(8)

/* Ĭ�����������Ͳ����WIFI�� */
#define WIFI_UNIT_MAX_ALLWO_OPEN_DEF									(6)

/* Ĭ������ */
#define CONFERENCE_LANGUAGE_DEF											(Chinese)


/*******************************************************************************
 * 								�ⲿ�ӿ�/Ӳ������
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
/* ENET �ӿ�ʹ��  */
#define ENET_ENABLE													ENABLE

/* SMI �ӿ�ʹ��  */
#define SMI_ENABLE													ENABLE

/* ENET2 �ӿ�ʹ��  */
#define ENET2_ENABLE												ENABLE

/* SMI2 �ӿ�ʹ��  */
#define SMI2_ENABLE													ENABLE


/* SMIʱ��. */
#define SMI_CLOCK_NAME 												kCLOCK_CoreSysClk

/******************************** UART 1 ~ 8 ************************************/
/* UART1 �ӿ�ʹ��  */
#define UART1_ENABLE												ENABLE

/* UART2 �ӿ�ʹ��  */
#define UART2_ENABLE												ENABLE

/* UART3 �ӿ�ʹ��  */
#define UART3_ENABLE												ENABLE

/* UART4 �ӿ�ʹ��  */
#define UART4_ENABLE												DISABLE

/* UART5 �ӿ�ʹ��  */
#define UART5_ENABLE												DISABLE

/* UART6 �ӿ�ʹ��  */
#define UART6_ENABLE												ENABLE

/* UART7 �ӿ�ʹ��  */
#define UART7_ENABLE												DISABLE

/* UART8 �ӿ�ʹ��  */
#define UART8_ENABLE												ENABLE


/* ����Ĭ���ж����ȼ� */
#define UART_DEF_ISR_PRIORITY										(3U)


/* UARTx_USE_IN_DRIVER�궨���������ֽӿ�����HAL����û�������ֱ�ӵ���
   ����������hal_init�г�ʼ�� */
#define UART6_USE_IN_DRIVER											ENABLE

#define UART6_BAUDRATE												(9600U)


/******************************** SAI 1 ~ 3 ************************************/

/* SAI1 �ӿ�ʹ��  */
#define SAI1_ENABLE													DISABLE

/* SAI2 �ӿ�ʹ��   */
#define SAI2_ENABLE													DISABLE

/* SAI3 �ӿ�ʹ��   */	
#define SAI3_ENABLE													DISABLE

#if SAI1_ENABLE || SAI2_ENABLE || SAI3_ENABLE
/* SAI�ӿ�DMA */
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
