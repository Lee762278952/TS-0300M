/**
 *
 *	@name				slave_mcu.c
 *	@author 			KT
 *	@date 				2020/02/20
 *	@brief
 *  @include			slave_mcu.h
 *
 *  @API
 *
 *  @description
 *
 **/

/*******************************************************************************
 * includes
 ******************************************************************************/
/* HAL */
#include "hal_uart.h"

/* OS */
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"
#include "queue.h"

/* API */
#include "ram.h"

/* APP */
#include "slave_mcu.h"
#include "conference.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SLAVE_MCU_UART_BASE									(LPUART2)		

#define SLAVE_MCU_UART_BAUDRATE								(115200U)

/* 任务堆栈大小及优先级 */
#define SLAVE_MCU_TASK_STACK_SIZE							(1024)
#define SLAVE_MCU_TASK_PRIORITY								(11)

/* 接收静态缓存BUF大小*/
#define UART_RECV_BUF_SIZE									(32)

/* 全数字会议协议数据包最小长度 */
#define SLAVE_MCU_CMD_MIN_LEN								(13)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* API */
static void SlaveMcu_Init(void);
static void SlaveMcu_Launch(void);
static void SlaveMcu_TransmitWithExData(ConfProtocol_S *prot,uint16_t exLen, uint8_t *exData);
static void SlaveMcu_Transmit(ConfProtocol_S *prot);


/* Internal */
static void SlaveMcu_UartDataProcessTask(void *pvParameters);
static void SlaveMcu_CtrlUartCallback(uint8_t data,void *para);
static void SlaveMcu_NotifyConferenceWithExData(ConfProtocol_S *prot, uint16_t exLen, uint8_t *exData);
static void SlaveMcu_NotifyConference(ConfProtocol_S *prot);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* 串口控制句柄 */
static HAL_UartHandler_S SlaveUartHandler;

static uint8_t RecvCount = 0;
static uint8_t RecvBuf[UART_RECV_BUF_SIZE];

/* API */
SlaveMcu_S SlaveMcu = {
	.init = SlaveMcu_Init,
	.launch = SlaveMcu_Launch,
	.transmit = SlaveMcu_Transmit,
	.transWithExData = SlaveMcu_TransmitWithExData,
};
/*******************************************************************************
 * Code
 ******************************************************************************/
/**
* @Name 		SlaveMcu_Init
* @Author		KT
* @Description
* @para
*
*
* @return
*/
static void SlaveMcu_Init(void){
	HAL_UartConfig_S *config;
	
	config = MALLOC(sizeof(HAL_UartConfig_S));
	SlaveUartHandler = MALLOC(HAL_UART_HANDLER_SIZE);

	config->base = SLAVE_MCU_UART_BASE;
    config->baudRate = SLAVE_MCU_UART_BAUDRATE;                  
    config->enableRx = true;                       
    config->enableTx = true;  
	config->rxIrq = true;
	config->txIrq = false;
	
	HAL_UartInit(SlaveUartHandler, config);
	HAL_UartSetCallback(SlaveUartHandler, RecvBuf , UART_RECV_BUF_SIZE, SlaveMcu_CtrlUartCallback, null);
}


/**
* @Name  		SlaveMcu_Launch
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void SlaveMcu_Launch(void){
	if (xTaskCreate(SlaveMcu_UartDataProcessTask, "SlaveMcu_UartDataProcessTask", SLAVE_MCU_TASK_STACK_SIZE,null, SLAVE_MCU_TASK_PRIORITY, NULL) != pdPASS){
        Log.e("create SlaveMcu task error\r\n");
    }
}


/**
* @Name  		SlaveMcu_UartDataProcessTask
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void SlaveMcu_UartDataProcessTask(void *pvParameters)
{
	ConfProtocol_S prot;
	uint8_t exData[15],exLen,i;

	Log.d("Slave mcu process task start...\r\n");
	while(1){
		DELAY(100);
		if(RecvCount < SLAVE_MCU_CMD_MIN_LEN)
			continue;

		for(i = 0; i <= RecvCount - SLAVE_MCU_CMD_MIN_LEN; i++) {
	        if(RecvBuf[i] == 0xAA && RecvBuf[i+1] == 0xEE && \
	           RecvBuf[i + 3 + RecvBuf[i+2]] == 0xEE && \
	           RecvBuf[i + 4 + RecvBuf[i+2]] == 0xFC) {

				memcpy(&prot,&RecvBuf[i + 3],sizeof(ConfProtocol_S));
	            SlaveMcu_NotifyConference(&prot);
	        }
   		}

		RecvCount = 0;
		HAL_UartClearCount(SlaveUartHandler);
	}
}

/**
* @Name  		SlaveMcu_NotifyConferenceWithExData
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void SlaveMcu_NotifyConferenceWithExData(ConfProtocol_S *prot, uint16_t exLen, uint8_t *exData)
{
    Notify_S *notify;

    if(prot == null || (exLen != 0 && exData == null))
        return;

    notify = MALLOC(sizeof(Notify_S) + exLen);

    notify->nSrc = kType_NotiSrc_SlaveMcu;

    memcpy(&notify->prot.conference,prot,sizeof(ConfProtocol_S));
    notify->exLen = exLen;
    if(exLen != 0) {
        memcpy(&notify->exDataHead,exData,exLen);
    }

    Conference.notify(notify);
}

/**
* @Name  		SlaveMcu_NotifyConference
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void SlaveMcu_NotifyConference(ConfProtocol_S *prot)
{
    SlaveMcu_NotifyConferenceWithExData(prot, null, null);
}

/**
* @Name  		SlaveMcu_TransmitWithExData
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void SlaveMcu_TransmitWithExData(ConfProtocol_S *prot,uint16_t exLen, uint8_t *exData)
{
	uint8_t *data,dataLen;

	ERR_CHECK(prot != null,return);
	ERR_CHECK(!(exLen != 0 && exData == null),return);


	dataLen = sizeof(ConfProtocol_S) + exLen;

	data = MALLOC(dataLen);

	memcpy(&data[0],prot,sizeof(ConfProtocol_S));
	if(exLen != 0)
		memcpy(&data[sizeof(ConfProtocol_S)],exData,exLen);
	
	HAL_UartSend(SlaveUartHandler, data, dataLen);

	FREE(data);
}


static void SlaveMcu_Transmit(ConfProtocol_S *prot){
	SlaveMcu_TransmitWithExData(prot,null,null);
}

/**
* @Name  		SlaveMcu_CtrlUartCallback
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void SlaveMcu_CtrlUartCallback(uint8_t count,void *para)
{
	RecvCount = count;
}



