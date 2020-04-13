/*
 *	 @FileName					 hal_uart.c
 *	 @Author					 KT
 *	 @CreationTime				 2019/11/15
 *	 @Description
 *
 *	 @Include					 hal_uart.h
 *
 *
 **/

#include "hal_uart.h"
#include "hal.h"
#include "board.h"
#include "fsl_lpuart.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define UART_HAL_INSTANCE_NUM				(8)

#define UART_INTERRUPT_PRIORITY				(3)

typedef struct {
	LPUART_Type *base;
	UartIrqCallback callback;
	uint8_t *rxData;
	uint32_t maxSize;
	uint32_t rxCount;
	void *para;
}UartHandler_S;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*******************************************************************************
 * Variables
 ******************************************************************************/
static UartHandler_S *uartInstance[UART_HAL_INSTANCE_NUM + 1] = {null};

/*******************************************************************************
 * Code
 ******************************************************************************/
status_t HAL_UartInit(HAL_UartHandler_S handle, HAL_UartConfig_S *config)
{
    UartHandler_S *uartHandle;
    lpuart_config_t lpuartConfig;
    status_t status;
	IRQn_Type uartIrq[] = LPUART_RX_TX_IRQS;

	uint32_t srcClock_Hz;
	uint32_t instance;

	ERR_CHECK(handle != null , return kStatus_Fail);
	ERR_CHECK(config != null , return kStatus_Fail);

	instance = LPUART_GetInstance(config->base);

    LPUART_GetDefaultConfig(&lpuartConfig);
	
	srcClock_Hz = BOARD_DebugConsoleSrcFreq();

	lpuartConfig.baudRate_Bps = config->baudRate;
    lpuartConfig.enableRx = config->enableRx;
    lpuartConfig.enableTx = config->enableTx;

    status = LPUART_Init(config->base, &lpuartConfig, srcClock_Hz);
	if(status != kStatus_Success){
		return kStatus_Fail;
	}

    uartHandle = (UartHandler_S *)handle;
	uartHandle->base = config->base;
	
	uartInstance[instance] = uartHandle;

    /* Enable interrupt in NVIC. */
	if(config->rxIrq | config->txIrq){
		
		if(config->rxIrq)
			LPUART_EnableInterrupts(uartHandle->base, kLPUART_RxDataRegFullInterruptEnable);
		
		if(config->txIrq)
			LPUART_EnableInterrupts(uartHandle->base, kLPUART_TxDataRegEmptyInterruptEnable);

		NVIC_SetPriority(uartIrq[instance], UART_INTERRUPT_PRIORITY);
	    EnableIRQ(uartIrq[instance]);
	}

    return kStatus_Success;
}

status_t HAL_UartDeinit(HAL_UartHandler_S handle)
{
	UartHandler_S *uartHandle;
	uint32_t instance;

	ERR_CHECK(handle != null , return kStatus_Fail);
	
	uartHandle = (UartHandler_S *)handle;
	instance = LPUART_GetInstance(uartHandle->base);
	
	LPUART_Deinit(uartHandle->base);
	uartInstance[instance] = null;

    return kStatus_Success;
}

status_t HAL_UartSetCallback(HAL_UartHandler_S handle, uint8_t *data,uint32_t maxSize, UartIrqCallback callback,void *para)
{
	UartHandler_S *uartHandle;
	
	ERR_CHECK(handle != null , return kStatus_Fail);
	ERR_CHECK(callback != null && data != null , return kStatus_Fail);

	uartHandle = (UartHandler_S *)handle;

	uartHandle->callback = callback;
	uartHandle->rxData = data;
	uartHandle->maxSize = maxSize;
	uartHandle->para = para;
	uartHandle->rxCount = 0;
	
	return kStatus_Success;
}

status_t HAL_UartClearCount(HAL_UartHandler_S handle)
{
	UartHandler_S *uartHandle;
	ERR_CHECK(handle != null , return kStatus_Fail);
	uartHandle = (UartHandler_S *)handle;
	uartHandle->rxCount = 0;
	return kStatus_Success;
}

status_t HAL_UartReceive(HAL_UartHandler_S handle, uint8_t *data, size_t length)
{
    UartHandler_S *uartHandle;
    status_t status;

	ERR_CHECK(handle != null , return kStatus_Fail);
	ERR_CHECK(data != null && length != 0 , return kStatus_Fail);

    uartHandle = (UartHandler_S *)handle;
	
    status = LPUART_ReadBlocking(uartHandle->base, data, length);

    return status;
}

status_t HAL_UartSend(HAL_UartHandler_S handle, const uint8_t *data, size_t length)
{
    UartHandler_S *uartHandle;

	ERR_CHECK(handle != null , return kStatus_Fail);
	ERR_CHECK(data != null && length != 0 , return kStatus_Fail);
	
    uartHandle = (UartHandler_S *)handle;
//	printf("uart send(len = %d) : ",length);
//	for(uint8_t i = 0;i < length;i++){
//		printf("%02X ",data[i]);
//	}
//	printf("\r\n");
	LPUART_WriteBlocking(uartHandle->base, data, length);

    return kStatus_Success;
}

/*******  IRQ  ********/

static void HAL_UartIrqCommon(uint32_t instance,LPUART_Type *base){
//	uint8_t data;
	uint32_t status = LPUART_GetStatusFlags(base);
	UartHandler_S *uartHandler = uartInstance[instance];

	if(uartHandler == null)
		return;
	
    /* If new data arrived. */
    if (kLPUART_RxDataRegFullFlag & status)
    {
    	if(uartHandler->rxCount < uartHandler->maxSize)
	        uartHandler->rxData[uartHandler->rxCount++] = LPUART_ReadByte(base);
		
		if(uartHandler->callback != null){
			uartHandler->callback(uartHandler->rxCount,uartHandler->para);
		}
    }

	    /* If RX overrun. */
    if (kLPUART_RxOverrunFlag & status)
    {
        /* Clear overrun flag, otherwise the RX does not work. */
        base->STAT = ((base->STAT & 0x3FE00000U) | LPUART_STAT_OR_MASK);
    }
}

#ifndef UART1_USE_IN_DRIVER
void LPUART1_IRQHandler(void)
{
	HAL_UartIrqCommon(1,LPUART1);
}
#endif

#ifndef UART2_USE_IN_DRIVER
void LPUART2_IRQHandler(void)
{
	HAL_UartIrqCommon(2,LPUART2);
}
#endif

#ifndef UART3_USE_IN_DRIVER
void LPUART3_IRQHandler(void)
{
	HAL_UartIrqCommon(3,LPUART3);
}
#endif

#ifndef UART4_USE_IN_DRIVER
void LPUART4_IRQHandler(void)
{
	HAL_UartIrqCommon(4,LPUART4);
}
#endif

#ifndef UART5_USE_IN_DRIVER
void LPUART5_IRQHandler(void)
{
	HAL_UartIrqCommon(5,LPUART5);
}
#endif

#ifndef UART6_USE_IN_DRIVER
void LPUART6_IRQHandler(void)
{
	HAL_UartIrqCommon(6,LPUART6);
}
#endif

#ifndef UART7_USE_IN_DRIVER
void LPUART7_IRQHandler(void)
{
	HAL_UartIrqCommon(7,LPUART7);
}
#endif

#ifndef UART8_USE_IN_DRIVER
void LPUART8_IRQHandler(void)
{
	HAL_UartIrqCommon(8,LPUART8);
}
#endif







