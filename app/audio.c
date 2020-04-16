/**
 *
 *	@FileName							audio.c
 *	@Author								KT
 *	@CreationTime						2019/05/15
 *	@Description						��Ƶ¼�Ź���
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
/* �̵������� */
#define RELAY_CTRL_GPIO									(GPIO1)
#define RELAY_CTRL_PIN									(11)				//V4�汾Ӳ��
//#define RELAY_CTRL_PIN									(1)						//V3�汾Ӳ��

/* USB��Դ���� */
#define USB_POWER_CTRL_GPIO								(GPIO1)
#define USB_POWER_CTRL_PIN								(25)


/* �̵��������ⲿUSB�ӿ����ӵ�MCU��WT2000 */
#define MCU												(0)
#define WT2000											(1)
#define USB_CONNET_TO(_target)							HAL_SetGpioLevel(SwitchCtrlIo, _target);
#define USB_POWER_ENABLE(_bool)							HAL_SetGpioLevel(UsbPowerCtrlIo, _bool);

/* ����/�ر�����(ADC 88C22)���� */
#define CLOSE											(0x30)
#define OPEN											(0x33)
#if defined(AUD_CODEC_NAU88C22_W_ENABLE) && AUD_CODEC_NAU88C22_W_ENABLE
#define SET_ADC_INPUT(_opt)								(HAL_AudCodeWriteReg(tNau88c22_W,0x2C, _opt))
#else
#define SET_ADC_INPUT(_opt)								
#endif


/* ��Ƶ¼�Ź���������г��� */
#define AUDIO_CMD_QUEUE_LEN      	 					(15)
#define AUDIO_CMD_ITEM_SIZE    							(sizeof(void *))

/* ���ڽ��ն��г��� */
#define UART_RECV_QUEUE_LEN      	 					(15)
#define UART_RECV_ITEM_SIZE    							(sizeof(void *))


/* ��Ƶ¼�Ź����̶߳�ջ��С�����ȼ� */
#define AUDIO_TASK_STACK								(1024)
#define AUDIO_TASK_PRIORITY								(9)

/* ���ж�ʱ����ʱ��� */
#define AUDIO_RUN_TIME_MS								(1000)



/* Ĭ�ϵȴ�USB����ʱ��(��) */
#define AUDIO_WAIT_USB_CONNECT_TIME						(5)

/* ��Ƶ¼�ſ����������� */
typedef enum {
    tNone = 0,
    tRecord,
    tStopRec,
    tPlayPause,
    tPlayNext,
    tPlayPrevious,
    tStopPlay,
} AudCmdType_EN;

/* ���ڽ��սṹ */
typedef struct {
    uint8_t data[32];
    uint8_t len;
} AudUartRecv_S;


/* ��Ƶ¼�ſ�������ṹ�� */
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
/* �̵������ơ���Դ����GPIO */
static HAL_GpioHandler SwitchCtrlIo;
static HAL_GpioHandler UsbPowerCtrlIo;

/* ���շ��ͻ����ź��� */
static SemaphoreHandle_t UartCmdSendSem;

/* WT2000���������ź��� */
static SemaphoreHandle_t Wt2kOperatSem;

/* USB����������� */
static QueueHandle_t AudioCmdQueue;

/* ���ж�ʱ�� */
static TimerHandle_t AudRunTimer;

/* ���ڽ��ջ��桢���� */
static AudUartRecv_S UartRecv;

/* ״̬�������� */
static Audio_StaListener AudStaListener = null;

/* ��Ƶ״̬ */
static AudInfo_S AudInfo;

/* U�̹��غ��һ��¼���򲥷� */
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

    /* ��ʼ��GPIO */
    SwitchCtrlIo = HAL_GpioInit(RELAY_CTRL_GPIO, RELAY_CTRL_PIN, kGPIO_DigitalOutput, null, (gpio_interrupt_mode_t)null);
    UsbPowerCtrlIo = HAL_GpioInit(USB_POWER_CTRL_GPIO, USB_POWER_CTRL_PIN, kGPIO_DigitalOutput, null, (gpio_interrupt_mode_t)null);

    /* Ĭ������ */
    USB_CONNET_TO(MCU);

	/* Ĭ�Ϲر��������� */
	SET_ADC_INPUT(CLOSE);

    /* ��U�̵�Դ */
    USB_POWER_ENABLE(true);

    /* ��ʼ����Ƶ������� (Dante ,WT2000) */
    HAL_AudCodecInit(tNau88c22_D);
    HAL_AudCodecInit(tNau88c22_W);

    /* ��ʼ���ź��� */
    UartCmdSendSem = xSemaphoreCreateMutex();
	Wt2kOperatSem  = xSemaphoreCreateMutex();

    /* ������г�ʼ�� */
    AudioCmdQueue = xQueueCreate(AUDIO_CMD_QUEUE_LEN,AUDIO_CMD_ITEM_SIZE);


    /* �رմ��ڽ��� */
    Wt2000_SetRecvEnable(false);

    /* ��ʼ������WT2000 */
    for(i = 0; i < 5; i++) {
        Wt2000_SendConfig(cfgItem[i],cfgPara[i]);
        DELAY(200);
    }

    /* ���ûص����� */
    Wt2000_SetAckCallback(Audio_Wt2000Callback);

    /* ��ʼ�����ж�ʱ�� */
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

    /* ��Ƶ¼��״̬��� */
    AudInfo.state = kStatus_Aud_Idle;

	Log.i("Audio command process task start!!\r\n");

    while(1) {
        /* �ȴ�������� */
        xQueueReceive(AudioCmdQueue, &cmd, MAX_NUM);

		/* ��ȡWT2000�����ź��� */
		xSemaphoreTake(Wt2kOperatSem, MAX_NUM);
		
        switch(cmd->type) {
        /* ��ʼ¼�� */
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

            /* ��U�̹��ص�WT2000 */
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
        /* ֹͣ¼�� */
        case tStopRec: {
            if(AudInfo.state != kStatus_Aud_Recording)
                break;

            Audio_SendCmdToWt2000(false, kType_WtCmd_RecStop, null, null,null,null);
			xTimerStop(AudRunTimer,0);
			DELAY(500);
			
			/* ��ʱUSBҪ�л���MCU������Ƶ�б� */
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

        /* ���� */
        case tPlayPause: 

		/* ������һ�� */
        case tPlayNext:

		/* ������һ�� */
        case tPlayPrevious:{
			const char dir[6] = AUDIO_DEFAULT_DIR;
			uint8_t *para;

            /* ��U�̹��ص�WT2000 */
            if(!Audio_UsbConnectWt2000())
                break;

			/* ����U�̺��״β��� */
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
				/* ���Ż���ͣ */
				if(cmd->type == tPlayPause){
					uartRecv = Audio_SendCmdToWt2000(true, kType_WtCmd_PlayOrPause, null, null,null,null);

					if(!(uartRecv != null && uartRecv->len != 0 && uartRecv->data[0] == 0)) {
		                Audio_NoticeListener(kEvent_Aud_PlayErr);
						AudInfo.state = kStatus_Aud_Idle;
		                Audio_UsbConnectMcu();
		            } 

					FREE(uartRecv);
				}
				/* ��һ�� */
				else if(cmd->type == tPlayNext){
					Audio_SendCmdToWt2000(false, kType_WtCmd_Next, null, null,null,null);
            		AudInfo.runTime = 0;
				}
				/* ��һ�� */
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
		
		/* �ͷ�WT2000�����ź��� */
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
	
	/* ��ȡWT2000�����ź��� */
    if(!xSemaphoreTake(Wt2kOperatSem, AUDIO_RUN_TIME_MS - 100)){
		return;
	}


    /* ���USB���� */
    if(!Audio_CheckUsbConnect()) {
        Audio_SendCmdToWt2000(false, kType_WtCmd_Stop, null, null,null,null);
		AudInfo.state = kStatus_Aud_Idle;
        Audio_UsbConnectMcu();
        goto aud_run_time_end;
    }

    /* �����Ƶ¼��״̬ */
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
				/* ��ȡ��ǰ�������� */
	            uartRecv = Audio_SendCmdToWt2000(true, kType_WtCmd_CurrPlayFile, null, null,null,null);
	            if(uartRecv != null) {
					/* ������������仯��֤���Ѿ��и裬���¼�ʱ */
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
	/* �ͷ�WT2000�����ź��� */
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

    /* ��ȡ���ڷ����ź��� */
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
    /* �ͷ��ź��� */
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
    /* �ȴ�USB���� */
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





