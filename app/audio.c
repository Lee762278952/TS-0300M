/**
 *
 *	@FileName							audio.c
 *	@Author								KT
 *	@CreationTime						2019/05/15
 *	@Description						音频录放功能
 *
 * 	@Include							audio.h
 *
 *	@API
 *
 *  @Modify								2019/12/05 by KT
 **/
/*******************************************************************************
 * includes
 ******************************************************************************/
/* DRIVER */
#include "mp3/wt2000a.h"

/* HAL */
#include "hal_gpio.h"
#include "hal_aud_codec.h"

/* OS */
#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"

/* GLOBAL */
#include "log.h"

/* APP */
#include "ram.h"
#include "audio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* 继电器控制 */
#define RELAY_CTRL_GPIO									(GPIO1)
#define RELAY_CTRL_PIN									(11)				//V4版本硬件
//#define RELAY_CTRL_PIN									(1)						//V3版本硬件

/* USB电源控制 */
#define USB_POWER_CTRL_GPIO								(GPIO1)
#define USB_POWER_CTRL_PIN								(25)


/* 继电器控制外部USB接口连接到MCU或WT2000 */
#define MCU												(0)
#define WT2000											(1)
#define USB_CONNET_TO(_target)							HAL_SetGpioLevel(SwitchCtrlIo, _target);
#define USB_POWER_ENABLE(_bool)							HAL_SetGpioLevel(UsbPowerCtrlIo, _bool);

/* 开启/关闭声卡(ADC 88C22)输入 */
#define CLOSE											(0x30)
#define OPEN											(0x33)
#if defined(AUD_CODEC_NAU88C22_W_ENABLE) && AUD_CODEC_NAU88C22_W_ENABLE
#define SET_ADC_INPUT(_opt)								(HAL_AudCodeWriteReg(tNau88c22_W,0x2C, _opt))
#else
#define SET_ADC_INPUT(_opt)								
#endif


/* 音频录放功能命令队列长度 */
#define AUDIO_CMD_QUEUE_LEN      	 					(15)
#define AUDIO_CMD_ITEM_SIZE    							(sizeof(void *))

/* 串口接收队列长度 */
#define UART_RECV_QUEUE_LEN      	 					(15)
#define UART_RECV_ITEM_SIZE    							(sizeof(void *))


/* 音频录放功能线程堆栈大小、优先级 */
#define AUDIO_TASK_STACK								(1024)
#define AUDIO_TASK_PRIORITY								(9)

/* 运行定时器计时间隔 */
#define AUDIO_RUN_TIME_MS								(1000)



/* 默认等待USB连接时间(秒) */
#define AUDIO_WAIT_USB_CONNECT_TIME						(5)

/* 音频录放控制命令类型 */
typedef enum {
    tNone = 0,
    tRecord,
    tStopRec,
    tPlayPause,
    tPlayNext,
    tPlayPrevious,
    tStopPlay,
} AudCmdType_EN;

/* 串口接收结构 */
typedef struct {
    uint8_t data[32];
    uint8_t len;
} AudUartRecv_S;


/* 音频录放控制命令结构体 */
typedef struct {
    AudCmdType_EN type;
    char *fileName;
} AudCmd_S;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* LAUNCHER */
static void Audio_ConfigTask(void *pvParameters);
static void Audio_CmdProcecssTask(void *pvParameters);


/* API */
static void Audio_RecordStart(const char *fileName);
static void Audio_RecordStop(void);
static void Audio_PlayPause(void);
static void Audio_PlayNext(void);
static void Audio_PlayPrevious(void);
static void Audio_SetListener(Audio_StaListener listener);



/* INTERNAL */

static void Audio_Wt2000Callback(uint8_t data);
static void Audio_NoticeTask(AudCmdType_EN type, char *fileName);
static void Audio_NoticeListener(AudEvent_EN event);
static bool Audio_WaitUsbConnect(uint8_t waitTime);
static void Audio_RunningTimer(TimerHandle_t xTimer);
static AudUartRecv_S *  Audio_SendCmdToWt2000(bool isWaitAck,WtCmdType_EN cmd,const char *dir,const char *file,uint8_t *data,uint8_t dLen);
static bool Audio_CheckUsbConnect(void);
static void Audio_UsbConnectMcu(void);
static bool Audio_UsbConnectWt2000(void);


/*******************************************************************************
 * Variables
 ******************************************************************************/
/* 继电器控制、电源控制GPIO */
static HAL_GpioHandler SwitchCtrlIo;
static HAL_GpioHandler UsbPowerCtrlIo;

/* 接收发送互斥信号量 */
static SemaphoreHandle_t UartCmdSendSem;

/* WT2000操作互斥信号量 */
static SemaphoreHandle_t Wt2kOperatSem;

/* USB控制任务队列 */
static QueueHandle_t AudioCmdQueue;

/* 运行定时器 */
static TimerHandle_t AudRunTimer;

/* 串口接收缓存、计数 */
static AudUartRecv_S UartRecv;

/* 状态监听函数 */
static Audio_StaListener AudStaListener = null;

/* 音频状态 */
static AudInfo_S AudInfo;

/* U盘挂载后第一次录音或播放 */
static bool isFirstPlayOrRecord = true;


/*******************************************************************************
 * Task & API
 ******************************************************************************/

static AppTask_S ConfigTask = {
	.task = Audio_ConfigTask,
	.name = "App.Audio.Config",	
	.stack = AUDIO_TASK_STACK,
	.para = null,
	.prio = AUDIO_TASK_PRIORITY,
	.handle = null,
};

static AppTask_S CmdProcess = {
	.task = Audio_CmdProcecssTask,
	.name = "App.Audio.CmdProcess",	
	.stack = AUDIO_TASK_STACK,
	.para = null,
	.prio = AUDIO_TASK_PRIORITY,
	.handle = null
};


static AppTask_S *FuncTask[] = {&CmdProcess};

static AppLauncher_S Launcher = {
	.init = null,
	.configTask = &ConfigTask,
	.funcNum  = 1,
	.funcTask = FuncTask,
};


/* API */
Audio_S Audio = {
	.launcher = &Launcher,
	
    .playPause = Audio_PlayPause,
    .next = Audio_PlayNext,
    .previous = Audio_PlayPrevious,
    .record = Audio_RecordStart,
    .stopRec = Audio_RecordStop,
//    .stopPlay = Audio_PlayStop,
    .setListener = Audio_SetListener,

};
/*******************************************************************************
 * Code
 ******************************************************************************/
/**
* @Name  		Audio_ConfigTask
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Audio_ConfigTask(void *pvParameters)
{
    const WtCmdType_EN cfgItem[5] = {kType_WtCmd_NeedReturn,kType_WtCmd_Disk,kType_WtCmd_RecordRate,kType_WtCmd_SetVolume,kType_WtCmd_SetPlayMode};
    const uint8_t cfgPara[5] = {0x01,0x01,r128KBPS,0x1F,0x02};
    uint8_t i;

    /* 初始化GPIO */
    SwitchCtrlIo = HAL_GpioInit(RELAY_CTRL_GPIO, RELAY_CTRL_PIN, kGPIO_DigitalOutput, null, (gpio_interrupt_mode_t)null);
    UsbPowerCtrlIo = HAL_GpioInit(USB_POWER_CTRL_GPIO, USB_POWER_CTRL_PIN, kGPIO_DigitalOutput, null, (gpio_interrupt_mode_t)null);

    /* 默认连接 */
    USB_CONNET_TO(MCU);

	/* 默认关闭声卡输入 */
	SET_ADC_INPUT(CLOSE);

    /* 打开U盘电源 */
    USB_POWER_ENABLE(true);

    /* 初始化音频编解码器 (Dante ,WT2000) */
    HAL_AudCodecInit(tNau88c22_D);
    HAL_AudCodecInit(tNau88c22_W);

    /* 初始化信号量 */
    UartCmdSendSem = xSemaphoreCreateMutex();
	Wt2kOperatSem  = xSemaphoreCreateMutex();

    /* 任务队列初始化 */
    AudioCmdQueue = xQueueCreate(AUDIO_CMD_QUEUE_LEN,AUDIO_CMD_ITEM_SIZE);


    /* 关闭串口接收 */
    Wt2000_SetRecvEnable(false);

    /* 初始化配置WT2000 */
    for(i = 0; i < 5; i++) {
        Wt2000_SendConfig(cfgItem[i],cfgPara[i]);
        DELAY(200);
    }

    /* 设置回调函数 */
    Wt2000_SetAckCallback(Audio_Wt2000Callback);

    /* 初始化运行定时器 */
    AudRunTimer = xTimerCreate("AudRuning",AUDIO_RUN_TIME_MS,pdTRUE,null,Audio_RunningTimer);

	Log.i("Audio configuration finish ... \r\n");

    vTaskDelete(null);

}


/**
* @Name  		Audio_SetListener
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Audio_SetListener(Audio_StaListener listener)
{
    AudStaListener = listener;
}


/**
* @Name  		Audio_RecordStart
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Audio_RecordStart(const char *fileName)
{
    char *fname;
    ERR_CHECK(fileName != 0, return);

    fname = MALLOC(strlen(fileName) + 1);
    strcpy(fname,fileName);
    Audio_NoticeTask(tRecord,fname);
}
/**
* @Name  		Audio_RecordStop
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Audio_RecordStop(void)
{
    Audio_NoticeTask(tStopRec,null);
}


/**
* @Name  		Audio_PlayPause
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Audio_PlayPause(void)
{
    Audio_NoticeTask(tPlayPause,null);
}


/**
* @Name  		Audio_PlayNext
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Audio_PlayNext(void)
{
    Audio_NoticeTask(tPlayNext,null);
}


/**
* @Name  		Audio_PlayPrevious
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Audio_PlayPrevious(void)
{
    Audio_NoticeTask(tPlayPrevious,null);
}



/**
* @Name  		Audio_CmdProcecssTask
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Audio_CmdProcecssTask(void *pvParameters)
{
    AudCmd_S *cmd;
    AudUartRecv_S *uartRecv;

    /* 音频录放状态句柄 */
    AudInfo.state = kStatus_Aud_Idle;

	Log.i("Audio command process task start!!\r\n");

    while(1) {
        /* 等待任务队列 */
        xQueueReceive(AudioCmdQueue, &cmd, MAX_NUM);

		/* 获取WT2000操作信号量 */
		xSemaphoreTake(Wt2kOperatSem, MAX_NUM);
		
        switch(cmd->type) {
        /* 开始录音 */
        case tRecord: {
            uint8_t strLen;

            if(cmd->fileName == null)
                break;

            strLen = strlen(cmd->fileName);
            memset(AudInfo.recFile,0,25);
            if(strLen > 22) {
                memcpy(AudInfo.recFile,cmd->fileName,22);
            } else {
                memcpy(AudInfo.recFile,cmd->fileName,strLen);
            }
            FREE(cmd->fileName);

            /* 将U盘挂载到WT2000 */
            if(!Audio_UsbConnectWt2000())
                break;

			if(isFirstPlayOrRecord){
				isFirstPlayOrRecord = false;
				DELAY(3000);
			}

            uartRecv = Audio_SendCmdToWt2000(true, kType_WtCmd_RecStart, AUDIO_DEFAULT_DIR, (const char *)AudInfo.recFile,null,null);

            if(!(uartRecv != null && uartRecv->len != 0 && uartRecv->data[0] == 0)) {
                Audio_NoticeListener(kEvent_Aud_RecordErr);
				AudInfo.state = kStatus_Aud_Idle;
                Audio_UsbConnectMcu();
            } 
			
            FREE(uartRecv);
        }
        break;
        /* 停止录音 */
        case tStopRec: {
            if(AudInfo.state != kStatus_Aud_Recording)
                break;

            Audio_SendCmdToWt2000(false, kType_WtCmd_RecStop, null, null,null,null);
			xTimerStop(AudRunTimer,0);
			DELAY(500);
			
			/* 此时USB要切换到MCU更新音频列表 */
			USB_CONNET_TO(MCU);
			SET_ADC_INPUT(CLOSE);
            Audio_NoticeListener(kEvent_Aud_UsbFileUpdata);
			
			DELAY(3000);
			AudInfo.usbConnect = false;
			isFirstPlayOrRecord = true;
			AudInfo.state = kStatus_Aud_Idle;
    		Audio_NoticeListener(kEvent_Aud_RecordStop);
        }
        break;

        /* 播放 */
        case tPlayPause: 

		/* 播放下一首 */
        case tPlayNext:

		/* 播放上一首 */
        case tPlayPrevious:{
			const char dir[6] = AUDIO_DEFAULT_DIR;
			uint8_t *para;

            /* 将U盘挂载到WT2000 */
            if(!Audio_UsbConnectWt2000())
                break;

			/* 连接U盘后首次播放 */
			if(isFirstPlayOrRecord){
				isFirstPlayOrRecord = false;
				DELAY(2000);

				para = MALLOC(7);

				memcpy(para,dir,5);
				para[5] = 0;
				para[6] = 1;

				uartRecv = Audio_SendCmdToWt2000(true, kType_WtCmd_PlayDirIndex, null, null,para,7);

				if(!(uartRecv != null && uartRecv->len != 0 && uartRecv->data[0] == 0)) {
	                Audio_NoticeListener(kEvent_Aud_PlayErr);
					AudInfo.state = kStatus_Aud_Idle;
	                Audio_UsbConnectMcu();
	            } 

				FREE(uartRecv);
				FREE(para);
			}
			else{
				/* 播放或暂停 */
				if(cmd->type == tPlayPause){
					uartRecv = Audio_SendCmdToWt2000(true, kType_WtCmd_PlayOrPause, null, null,null,null);

					if(!(uartRecv != null && uartRecv->len != 0 && uartRecv->data[0] == 0)) {
		                Audio_NoticeListener(kEvent_Aud_PlayErr);
						AudInfo.state = kStatus_Aud_Idle;
		                Audio_UsbConnectMcu();
		            } 

					FREE(uartRecv);
				}
				/* 下一首 */
				else if(cmd->type == tPlayNext){
					Audio_SendCmdToWt2000(false, kType_WtCmd_Next, null, null,null,null);
            		AudInfo.runTime = 0;
				}
				/* 上一首 */
				else if(cmd->type == tPlayPrevious){
					Audio_SendCmdToWt2000(false, kType_WtCmd_Previous, null, null,null,null);
            		AudInfo.runTime = 0;
				}
			}

        }
        break;

        default:
            break;
        }

        FREE(cmd);
		
		/* 释放WT2000操作信号量 */
		xSemaphoreGive(Wt2kOperatSem);

    }
}



/**
* @Name  		Audio_RunningTimer
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Audio_RunningTimer(TimerHandle_t xTimer)
{
//    uint8_t ack[5];
    AudUartRecv_S *uartRecv;
    WtCmdType_EN cmdType;
    WtPlaySta_EN sta;
	
	/* 获取WT2000操作信号量 */
    if(!xSemaphoreTake(Wt2kOperatSem, AUDIO_RUN_TIME_MS - 100)){
		return;
	}


    /* 检查USB连接 */
    if(!Audio_CheckUsbConnect()) {
        Audio_SendCmdToWt2000(false, kType_WtCmd_Stop, null, null,null,null);
		AudInfo.state = kStatus_Aud_Idle;
        Audio_UsbConnectMcu();
        goto aud_run_time_end;
    }

    /* 检查音频录放状态 */
    uartRecv = Audio_SendCmdToWt2000(true, kType_WtCmd_State, null, null,null,null);
    if(uartRecv == null){
        goto aud_run_time_end;
	}

    cmdType = (WtCmdType_EN)uartRecv->data[0];
    sta = (WtPlaySta_EN)uartRecv->data[1];
    FREE(uartRecv);

    if(cmdType == kType_WtCmd_State) {
        switch(sta) {
	        case kStatus_WtPlay_Playing:{
				/* 获取当前播放索引 */
	            uartRecv = Audio_SendCmdToWt2000(true, kType_WtCmd_CurrPlayFile, null, null,null,null);
	            if(uartRecv != null) {
					/* 如果发现索引变化，证明已经切歌，重新计时 */
	                if(uartRecv->data[0] == kType_WtCmd_CurrPlayFile && AudInfo.playIndex != uartRecv->data[2]) {
	                    AudInfo.playIndex = uartRecv->data[2];
	                    AudInfo.runTime = 0;
	                } else
	                    AudInfo.runTime++;
					
	                FREE(uartRecv);
	            }
		
				if(AudInfo.state != kStatus_Aud_Playing){
	            	AudInfo.state = kStatus_Aud_Playing;
					Audio_NoticeListener(kEvent_Aud_PlayStar);
				}
				else
					Audio_NoticeListener(kEvent_Aud_None);
	        }break;
	        case kStatus_WtPlay_Stop:
	        case kStatus_WtPlay_Pause:{
				AudStaType_EN staTmp = AudInfo.state;

			 	AudInfo.state = kStatus_Aud_Idle;
				AudInfo.runTime = 0;
				
				if(staTmp == kStatus_Aud_Playing)
					Audio_NoticeListener(kEvent_Aud_PlayStop);

				else if(staTmp == kStatus_Aud_Recording)
					Audio_NoticeListener(kEvent_Aud_RecordStop);
					
	        }break;
	        case kStatus_WtPlay_Recording:{
				AudInfo.runTime++;
				if(AudInfo.state != kStatus_Aud_Recording){
	            	AudInfo.state = kStatus_Aud_Recording;
					Audio_NoticeListener(kEvent_Aud_RecordStar);
				}
	            else
					Audio_NoticeListener(kEvent_Aud_None);
	        }break;
        }
    }

aud_run_time_end:
	/* 释放WT2000操作信号量 */
    xSemaphoreGive(Wt2kOperatSem);
    
}


/**
* @Name  		Audio_NoticeTask
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Audio_NoticeTask(AudCmdType_EN type, char *fileName)
{
    AudCmd_S *cmd;

    cmd = MALLOC(sizeof(AudCmd_S));
    cmd->type = type;
    cmd->fileName = fileName;
    xQueueSend(AudioCmdQueue, &cmd, MAX_NUM);
}


/**
* @Name  		Audio_SendCmdToWt2000
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static AudUartRecv_S * Audio_SendCmdToWt2000(bool isWaitAck,WtCmdType_EN cmd,const char *dir,const char *file,uint8_t *data,uint8_t dLen)
{
//    uint8_t i;
    AudUartRecv_S *uartRecv = null;

    /* 获取串口发送信号量 */
    xSemaphoreTake(UartCmdSendSem, MAX_NUM);

    UartRecv.len = 0;

    Wt2000_SetRecvEnable(true);
    Wt2000_SendCommand(cmd, dir, file ,data ,dLen);

    if(isWaitAck) {
        uint8_t waitCnt = 250;

        while(UartRecv.len == 0 && waitCnt-- > 0)
            DELAY(30);

        if(UartRecv.len > 0) {
            uartRecv = MALLOC(sizeof(AudUartRecv_S));
            uartRecv->len = UartRecv.len;
            memcpy(uartRecv->data,UartRecv.data,UartRecv.len);
        }
    } else
        DELAY(50);
#if 0
    printf("WT2k recv : ");
    for(i = 0; i < UartRecv.len; i++)
        printf("%X ",UartRecv.data[i]);
    printf("\r\n");
#endif


    Wt2000_SetRecvEnable(false);
    /* 释放信号量 */
    xSemaphoreGive(UartCmdSendSem);

    return uartRecv;
}


/**
* @Name  		Audio_NoticeListener
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Audio_NoticeListener(AudEvent_EN event)
{
    if(AudStaListener != null)
        AudStaListener(event,&AudInfo);
}



/**
* @Name  		Audio_Wt2000Callback
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Audio_Wt2000Callback(uint8_t data)
{
    UartRecv.data[UartRecv.len++] = data;
#if 0
	printf(" *%02X * \r\n",data);
#endif
}


/**
* @Name  		Audio_UsbConnectMcu
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Audio_UsbConnectMcu(void)
{
//    AudInfo.state = kStatus_Aud_Idle;
    USB_CONNET_TO(MCU);
	SET_ADC_INPUT(CLOSE);

    AudInfo.usbConnect = false;
    xTimerStop(AudRunTimer,0);
	isFirstPlayOrRecord = true;
    Audio_NoticeListener(kEvent_Aud_UsbDisconnect);
}


/**
* @Name  		Audio_UsbConnectWt2000
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static bool Audio_UsbConnectWt2000(void)
{
    /* 等待USB挂载 */
    USB_CONNET_TO(WT2000);
	SET_ADC_INPUT(OPEN);
    if(!Audio_WaitUsbConnect(AUDIO_WAIT_USB_CONNECT_TIME))	{
		AudInfo.state = kStatus_Aud_Idle;
        Audio_UsbConnectMcu();
        return false;
    }

    AudInfo.usbConnect = true;
    xTimerStart(AudRunTimer,0);
    Audio_NoticeListener(kEvent_Aud_UsbConnect);
    return true;
}



/**
* @Name  		Audio_WaitUsbConnect
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static bool Audio_WaitUsbConnect(uint8_t waitTime)
{
    AudUartRecv_S *uartRecv;

    do {
        uartRecv = Audio_SendCmdToWt2000(true, kType_WtCmd_LinkSta, null, null,null,null);
        if(uartRecv != null) {
            if(uartRecv->len > 0 && uartRecv->data[0] == kType_WtCmd_LinkSta && uartRecv->data[1] == 2) {
				FREE(uartRecv);
                return true;
            }
            FREE(uartRecv);
        }
        DELAY(200);
    } while(waitTime--);

    return false;
}


/**
* @Name  		Audio_CheckUsbConnect
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static bool Audio_CheckUsbConnect(void)
{
    return Audio_WaitUsbConnect(0);
}





