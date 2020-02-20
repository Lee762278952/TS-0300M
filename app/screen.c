/**
 *
 *	@name				screen.c
 *	@author 			KT
 *	@date 				2019/11/18
 *	@brief
 *  @include			screen.h
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
#include "screen.h"
#include "conference.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CTRL_UART_BASE									(LPUART8)		

#define CTRL_UART_BAUDRATE								(115200U)

/* 任务堆栈大小及优先级 */
#define SCREEN_TASK_STACK_SIZE							(1024)
#define SCREEN_TASK_PRIORITY							(12)

/* 接收静态缓存BUF大小*/
#define UART_RECV_BUF_SIZE								(32)

/* 数据头长度 */
#define DATA_HEAD_LENGTH								(3)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* API */
static void Screen_Init(void);
static void Screen_Launch(void);
static void Screen_TransmitWithExData(ScreenProtocol_S *prot,uint16_t exLen, uint8_t *exData);
static void Screen_Transmit(ScreenProtocol_S *prot);
static void Screen_SetBacklight(uint8_t brightness);


/* Internal */
static void Screen_UartDataProcessTask(void *pvParameters);
static void Screen_CtrlUartCallback(uint8_t data,void *para);
static void Screen_NotifyConferenceWithExData(ScreenProtocol_S *prot, uint16_t exLen, uint8_t *exData);
static void Screen_NotifyConference(ScreenProtocol_S *prot);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* 串口控制句柄 */
static HAL_UartHandler_S CtrlUartHandler;

static uint8_t RecvCount = 0;
static uint8_t RecvBuf[UART_RECV_BUF_SIZE];

/* 数据头(2字节数据头+1字节长度) */
static const uint8_t DataHead[DATA_HEAD_LENGTH] = {0x5A,0xA5,0};

/* API */
Screen_S Screen = {
	.init = Screen_Init,
	.launch = Screen_Launch,
	.transmit = Screen_Transmit,
	.transWithExData = Screen_TransmitWithExData,
	.backlight = Screen_SetBacklight,
};
/*******************************************************************************
 * Code
 ******************************************************************************/
/**
* @Name 		Screen_Init
* @Author		KT
* @Description
* @para
*
*
* @return
*/
static void Screen_Init(void){
	HAL_UartConfig_S *config;
	
	config = MALLOC(sizeof(HAL_UartConfig_S));
	CtrlUartHandler = MALLOC(HAL_UART_HANDLER_SIZE);

	config->base = CTRL_UART_BASE;
    config->baudRate = CTRL_UART_BAUDRATE;                  
    config->enableRx = true;                       
    config->enableTx = true;  
	config->rxIrq = true;
	config->txIrq = false;
	
	HAL_UartInit(CtrlUartHandler, config);
	HAL_UartSetCallback(CtrlUartHandler, RecvBuf , UART_RECV_BUF_SIZE, Screen_CtrlUartCallback, null);
}


/**
* @Name  		Screen_Launch
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Screen_Launch(void){
	if (xTaskCreate(Screen_UartDataProcessTask, "Screen_UartDataProcessTask", SCREEN_TASK_STACK_SIZE,null, SCREEN_TASK_PRIORITY, NULL) != pdPASS){
        Log.e("create screen task error\r\n");
    }
}


/**
* @Name  		Screen_UartDataProcessTask
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Screen_UartDataProcessTask(void *pvParameters)
{
	ScreenProtocol_S prot;
	uint8_t exData[15],exLen;

	Log.d("Screen process task start...\r\n");
	while(1){
		DELAY(100);
		if(RecvCount < 7)
			continue;

		if(RecvBuf[0] == 0x5A && RecvBuf[1] == 0xA5 && RecvBuf[2] > 4 && RecvBuf[2] <= 6){
			prot.type = (ScreenProtType_EN)RecvBuf[3];
			memcpy(&prot.para[0],&RecvBuf[4],RecvBuf[2] - 1);
			Screen_NotifyConference(&prot);
		}
		else if(RecvBuf[0] == 0x5A && RecvBuf[1] == 0xA5 && RecvBuf[2] > 6){
			prot.type = (ScreenProtType_EN)RecvBuf[3];
			memcpy(&prot.para[0],&RecvBuf[4],5);

			exLen = RecvBuf[2] - 6;
			memcpy(exData,&RecvBuf[9],exLen);
			Screen_NotifyConferenceWithExData(&prot,exLen,exData);
		}
		
		RecvCount = 0;
		HAL_UartClearCount(CtrlUartHandler);
	}
}

/**
* @Name  		Screen_NotifyConferenceWithExData
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Screen_NotifyConferenceWithExData(ScreenProtocol_S *prot, uint16_t exLen, uint8_t *exData)
{
    Notify_S *notify;

    if(prot == null || (exLen != 0 && exData == null))
        return;

    notify = MALLOC(sizeof(Notify_S) + exLen);

    notify->nSrc = kType_NotiSrc_ScreenCtrl;

    memcpy(&notify->prot.screen,prot,sizeof(ScreenProtocol_S));
    notify->exLen = exLen;
    if(exLen != 0) {
        memcpy(&notify->exDataHead,exData,exLen);
    }

    Conference.notify(notify);
}

/**
* @Name  		Screen_NotifyConference
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Screen_NotifyConference(ScreenProtocol_S *prot)
{
    Screen_NotifyConferenceWithExData(prot, null, null);
}

/**
* @Name  		Screen_TransmitWithExData
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Screen_TransmitWithExData(ScreenProtocol_S *prot,uint16_t exLen, uint8_t *exData)
{
	uint8_t *data,dataLen,protLen;

	ERR_CHECK(prot != null,return);
	ERR_CHECK(!(exLen != 0 && exData == null),return);

	protLen = (prot->type == tType_Screen_Page) ? 4 : 3;

	dataLen = DATA_HEAD_LENGTH + protLen + exLen ;

	data = MALLOC(dataLen);

	memcpy(&data[0],DataHead,DATA_HEAD_LENGTH);
	memcpy(&data[DATA_HEAD_LENGTH],prot,protLen);
	if(exLen != 0)
		memcpy(&data[DATA_HEAD_LENGTH + protLen],exData,exLen);
	
	data[2] = dataLen - DATA_HEAD_LENGTH;

	HAL_UartSend(CtrlUartHandler, data, dataLen);

	FREE(data);
}


static void Screen_Transmit(ScreenProtocol_S *prot){
	Screen_TransmitWithExData(prot,null,null);
}

/**
* @Name  		Screen_SetBacklight
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Screen_SetBacklight(uint8_t brightness)
{
	uint8_t cmd[10] = {0x5A, 0xA5, 0x07, 0x80, 0x70, 0x02, 0, 30, 100, 0};
	cmd[8] = brightness;
	
	HAL_UartSend(CtrlUartHandler, cmd, 10);
}



/**
* @Name  		Screen_CtrlUartCallback
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Screen_CtrlUartCallback(uint8_t count,void *para)
{
	RecvCount = count;
}


