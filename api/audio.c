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

/* OS */
#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"

/* API */
#include "ram.h"
#include "audio.h"

/* HAL */
#include "hal_gpio.h"
#include "hal_aud_codec.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* �̵������� */
#define RELAY_CTRL_GPIO									(GPIO1)
#define RELAY_CTRL_PIN									(1)

/* USB��Դ���� */
#define USB_POWER_CTRL_GPIO								(GPIO1)
#define USB_POWER_CTRL_PIN								(25)


/* �̵��������ⲿUSB�ӿ����ӵ�MCU��WT2000 */
#define MCU												(0)
#define WT2000											(1)

#define USB_CONNET_TO(_target)							HAL_SetGpioLevel(SwitchCtrlIo, _target);

#define USB_POWER_ENABLE(_bool)							HAL_SetGpioLevel(UsbPowerCtrlIo, _bool);

/* ��Ƶ¼�Ź���������г��� */
#define AUDIO_CMD_QUEUE_LEN      	 					(10)
#define AUDIO_CMD_ITEM_SIZE    							(sizeof(AudCmd_S))

/* ��Ƶ¼�Ź����̶߳�ջ��С�����ȼ� */
#define AUDIO_TASK_STACK								(1024)
#define AUDIO_TASK_PRIORITY								(9)

/* ���м�ʱ����ʱ��� */
#define AUDIO_RUN_TIME_MS								(1000 - 100)

/* ¼���ļ�ָ��Ŀ¼�ļ��� */
#define AUDIO_RECORD_DIR								"AUREC"

/* Ĭ�ϵȴ�USB����ʱ��(��) */
#define AUDIO_WAIT_USB_CONNECT_TIME						(10)

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



/* ��Ƶ¼�ſ�������ṹ�� */
typedef struct {
    AudCmdType_EN type;
    char *fileName;
} AudCmd_S;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* API */
static void Audio_Launch(void);
static void Audio_RecordStart(const char *fileName);
static void Audio_RecordStop(void);
static void Audio_PlayPause(void);
static void Audio_PlayNext(void);
static void Audio_PlayPrevious(void);
static void Audio_PlayStop(void);
static void Audio_SetListener(Audio_StaListener listener);



/* INTERNAL */
static void Audio_LaunchTask(void *pvParameters);
static void Audio_CmdProcecssTask(void *pvParameters);
static void Audio_Wt2000Callback(uint8_t data);
static uint8_t Audio_WaitAck(uint8_t *data);
static bool Audio_WaitUsbConnect(uint8_t waitTime);
static void Audio_RunningTimer(TimerHandle_t xTimer);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* �̵������ơ���Դ����GPIO */
static HAL_GpioIndex SwitchCtrlIo;
static HAL_GpioIndex UsbPowerCtrlIo;

/* ���շ��Ͷ�ֵ�ź��� */
static SemaphoreHandle_t UartCmdRecvSem;
static SemaphoreHandle_t UartCmdSendSem;

/* USB����������� */
static QueueHandle_t AudioCmdQueue;

/* ���ж�ʱ�� */
static TimerHandle_t AudRunTimer;

/* ���ڽ��ջ��桢���� */
static uint8_t DriveRetData[32];
static uint8_t DriveRetCont = 0;

/* ״̬�������� */
static Audio_StaListener Listener = null;

/* API */
Audio_S Audio = {
    .launch = Audio_Launch,
    .playPause = Audio_PlayPause,
    .next = Audio_PlayNext,
    .previous = Audio_PlayPrevious,
    .record = Audio_RecordStart,
    .stopRec = Audio_RecordStop,
    .stopPlay = Audio_PlayStop,
    .setListener = Audio_SetListener,

};
/*******************************************************************************
 * Code
 ******************************************************************************/
static void Audio_Launch(void)
{
    if (xTaskCreate(Audio_LaunchTask, "Audio_LaunchTask", AUDIO_TASK_STACK,null, AUDIO_TASK_PRIORITY, NULL) != pdPASS) {
        debug("create audio task error\r\n");
    }
}

//static void Audio_UsbSwitch(uint8_t target){
//	if(target == MCU){
//	
//	}else if(target == )
//}

static void Audio_LaunchTask(void *pvParameters)
{
    /* ��ʼ��GPIO */
    SwitchCtrlIo = HAL_GpioInit(RELAY_CTRL_GPIO, RELAY_CTRL_PIN, kGPIO_DigitalOutput, null, null);
    UsbPowerCtrlIo = HAL_GpioInit(USB_POWER_CTRL_GPIO, USB_POWER_CTRL_PIN, kGPIO_DigitalOutput, null, null);

    /* Ĭ������ */
    USB_CONNET_TO(MCU);
	
	/* ��U�̵�Դ */
//    HAL_SetGpioLevel(UsbPowerCtrlIo, true);
	USB_POWER_ENABLE(true);

    /* ��ʼ����Ƶ������� */
    HAL_AudCodecInit(tNau88c22);

    /* ��ʼ���ź��� */
    UartCmdRecvSem = xSemaphoreCreateBinary();
    UartCmdSendSem = xSemaphoreCreateBinary();
	 xSemaphoreGive(UartCmdSendSem);

    /* ������г�ʼ�� */
    AudioCmdQueue = xQueueCreate(AUDIO_CMD_QUEUE_LEN,AUDIO_CMD_ITEM_SIZE);

    /* �رմ��ڽ��� */
    Wt2000_AckRecvEnable(false);

    /* ��ʼ������WT2000 */
    Wt2000_CfgReturnNecessary();
    DELAY(200);
    Wt2000_CfgUsbDisk();
    DELAY(200);
    Wt2000_CfgMp3Rate();
    DELAY(200);
    Wt2000_CfgDefVolume();

    /* ���ûص����� */
    Wt2000_SetAckCallback(Audio_Wt2000Callback);

    /* ������ʼ���߳� */
    if (xTaskCreate(Audio_CmdProcecssTask, "Audio_ProcecssTask", AUDIO_TASK_STACK,null, AUDIO_TASK_PRIORITY, NULL) != pdPASS) {
        debug("create audio process task error\r\n");
    }

    vTaskDelete(null);

    while(1);
}

static void Audio_CmdProcecssTask(void *pvParameters)
{
	AudState_S *audState;
    AudCmd_S cmd;
    uint8_t *ack;

    /* ACK���ջ�������ڴ�         */
    ack = MALLOC(30);
	
	/* ��Ƶ¼��״̬��� */
	audState = MALLOC(sizeof(AudState_S));
	audState->state = kStatus_Aud_Idle;

	/* ��ʼ����ʱ�� */
    AudRunTimer = xTimerCreate("AudRunTimer",AUDIO_RUN_TIME_MS,pdTRUE,audState,Audio_RunningTimer);

    while(1) {
        /* �ȴ�������� */
        xQueueReceive(AudioCmdQueue, &cmd, MAX_NUM);
        /* ��ȡ���ڷ����ź��� */
        xSemaphoreTake(UartCmdSendSem, MAX_NUM);

        switch(cmd.type) {
        case tRecord: {
            if(cmd.fileName == null)
                break;

			memset(audState->recFile,0,25);
			if(strlen(cmd.fileName) > 22){
				memcpy(audState->recFile,cmd.fileName,22);
			}else{
				memcpy(audState->recFile,cmd.fileName,strlen(cmd.fileName));
			}
			FREE(cmd.fileName);

            USB_CONNET_TO(WT2000);
			
            if(!Audio_WaitUsbConnect(AUDIO_WAIT_USB_CONNECT_TIME)){
				audState->state = kStatus_Aud_UsbTimeout;
				if(Listener != null) 
                        Listener(audState);
				audState->state = kStatus_Aud_Idle;
				USB_CONNET_TO(MCU);
				break;
			}

            if(audState->state != kStatus_Aud_Idle && audState->state != kStatus_Aud_Pause)
                break;
			

            Wt2000_AudioRecord(AUDIO_RECORD_DIR,(const char *)audState->recFile);
            Audio_WaitAck(ack);
            if(ack[0] == 0) {
				audState->runTime = 0;
                audState->state = kStatus_Aud_Recording;
                xTimerStart(AudRunTimer,0);
            } else {
				audState->state = kStatus_Aud_StarRecErr;
                if(Listener != null) {
                    Listener(audState);
                }
                audState->state = kStatus_Aud_Idle;
				USB_CONNET_TO(MCU);
            }
			
			
        }
        break;
		
		
        case tStopRec: {
            Wt2000_RecordStop();
            Audio_WaitAck(ack);
            xTimerStop(AudRunTimer,0);
            audState->runTime = 0;

			DELAY(3000);

			audState->state = kStatus_Aud_RecStoped;
            if(Listener != null) {
                Listener(audState);
            }
			audState->state = kStatus_Aud_Idle;
			
			USB_CONNET_TO(MCU);
        }
        break;

        /* ���� */
        case tPlayPause: {
            USB_CONNET_TO(WT2000);

            /* �ȴ�USB���� */
            if(!Audio_WaitUsbConnect(AUDIO_WAIT_USB_CONNECT_TIME))	{
				audState->state = kStatus_Aud_UsbTimeout;
				if(Listener != null) 
                        Listener(audState);
				audState->state = kStatus_Aud_Idle;
				USB_CONNET_TO(MCU);
				break;
			}

            if(audState->state == kStatus_Aud_Idle || audState->state == kStatus_Aud_Pause) {
                Wt2000_AudioPlayPause();
                Audio_WaitAck(ack);
                if(ack[0] == 0) {
                    audState->state = kStatus_Aud_Playing;
                    xTimerStart(AudRunTimer,0);
                } else {
					audState->state = kStatus_Aud_StarPlayErr;
                    if(Listener != null) 
                        Listener(audState);
                    audState->state = kStatus_Aud_Idle;
                }
            }

            else if(audState->state == kStatus_Aud_Playing) {
                Wt2000_AudioPlayPause();
                Audio_WaitAck(ack);
                audState->state = kStatus_Aud_Pause;
                if(Listener != null) {
                    Listener(audState);
                }
            }
        }
        break;

        /* ������һ�� */
        case tPlayNext: {
            USB_CONNET_TO(WT2000);

            if(audState->state != kStatus_Aud_Playing && audState->state != kStatus_Aud_Pause)	break;

            /* �ȴ�USB���� */
            if(!Audio_WaitUsbConnect(AUDIO_WAIT_USB_CONNECT_TIME)){
				audState->state = kStatus_Aud_UsbTimeout;
				if(Listener != null) 
                        Listener(audState);
				audState->state = kStatus_Aud_Idle;
				break;
			}

            Wt2000_PlayNext();
            Audio_WaitAck(ack);
            audState->runTime = 0;
        }
        break;

        /* ������һ�� */
        case tPlayPrevious: {
            USB_CONNET_TO(WT2000);

            if(audState->state != kStatus_Aud_Playing && audState->state != kStatus_Aud_Pause)	break;

            /* �ȴ�USB���� */
            if(!Audio_WaitUsbConnect(AUDIO_WAIT_USB_CONNECT_TIME)){
				audState->state = kStatus_Aud_UsbTimeout;
				if(Listener != null) 
                        Listener(audState);
				audState->state = kStatus_Aud_Idle;
				break;
			}

            Wt2000_PlayPrevious();
            Audio_WaitAck(ack);
            audState->runTime = 0;
        }
        break;

        /* ֹͣ */
        case tStopPlay: {
            Wt2000_PlayStop();
            Audio_WaitAck(ack);

            xTimerStop(AudRunTimer,0);
            audState->runTime = 0;
            audState->state = kStatus_Aud_Idle;

			USB_CONNET_TO(MCU);
        }
        break;

        default:
            break;
        }

        /* �ͷ��ź��� */
        xSemaphoreGive(UartCmdSendSem);
    }
}

static void Audio_SetListener(Audio_StaListener listener){
	Listener = listener;
}


static void Audio_SendCmdToTask(AudCmdType_EN type, char *fileName)
{
    AudCmd_S cmd;

    cmd.type = type;
    cmd.fileName = fileName;
    xQueueSend(AudioCmdQueue, &cmd, MAX_NUM);
}

static void Audio_RecordStart(const char *fileName)
{
    char *fname;
    ERR_CHECK(fileName != 0, return);

    fname = MALLOC(strlen(fileName) + 1);
    strcpy(fname,fileName);
    Audio_SendCmdToTask(tRecord,fname);
}

static void Audio_RecordStop(void)
{
    Audio_SendCmdToTask(tStopRec,null);
}

static void Audio_PlayPause(void)
{
    Audio_SendCmdToTask(tPlayPause,null);
}

static void Audio_PlayNext(void)
{
    Audio_SendCmdToTask(tPlayNext,null);
}

static void Audio_PlayPrevious(void)
{
    Audio_SendCmdToTask(tPlayPrevious,null);
}

static void Audio_PlayStop(void)
{
    Audio_SendCmdToTask(tStopPlay,null);
}





static uint8_t Audio_WaitAckByTime(uint8_t *data,uint32_t time)
{
    uint8_t i,count = 0;
    ERR_CHECK(data != 0, return 0);

    Wt2000_AckRecvEnable(true);

    /* ��ȡ���ź�������ʱ��Ȼ���ٻ�ȡһ���ź��������ȴ�����
    	��������жϽ��ն��ֽ�ʱ���ظ��ź��� */
    xSemaphoreTake(UartCmdRecvSem, MAX_NUM);
    DELAY(time);
    xSemaphoreTake(UartCmdRecvSem, 1);

    if(DriveRetCont == 0)
        return 0;

    count = DriveRetCont;
    memcpy(data,DriveRetData,count);
    DriveRetCont = 0;
#if 0
    for(i = 0; i < count; i++)
        debug("%X ",data[i]);
    debug("\r\n");
#endif
    Wt2000_AckRecvEnable(false);
    return count;
}

static uint8_t Audio_WaitAck(uint8_t *data)
{
    return Audio_WaitAckByTime(data,30);
}


static void Audio_Wt2000Callback(uint8_t data)
{
    portBASE_TYPE taskToWake = pdFALSE;

    DriveRetData[DriveRetCont++] = data;
    xSemaphoreGiveFromISR(UartCmdRecvSem, &taskToWake);
}

static bool Audio_WaitUsbConnect(uint8_t waitTime)
{
    uint8_t ack[5],cont;

    do {
        Wt2000_QueryState(kType_WtCmd_LinkSta);
        cont = Audio_WaitAck(ack);
        if(cont > 0 && ack[0] == kType_WtCmd_LinkSta && ack[1] == 2) {
            return true;
        } else {
//            if(Listener != null)
//                Listener(kStatus_Aud_UsbTimeout,null,null);
            debug("Usb connect timeout\r\n");
            return false;
        }
    } while(waitTime--);
}

static bool Audio_CheckUsbConnect(void){
	return Audio_WaitUsbConnect(0);
}

static void Audio_RunningTimer(TimerHandle_t xTimer)
{
	AudState_S *audState;
    uint8_t ack[5];
	bool usbConnect;
	WtPlaySta_EN playSta;

	audState = pvTimerGetTimerID(xTimer);

    xSemaphoreTake(UartCmdSendSem, MAX_NUM);

	/* ���USB���� */
	usbConnect = Audio_CheckUsbConnect();
	/* USB�Ͽ� */
	if(!usbConnect){
		Audio_PlayStop();
		xSemaphoreGive(UartCmdSendSem);
		audState->state = kStatus_Aud_UsbBroken;
		if(Listener != null)
        	Listener(audState);
		debug("Usb was broken!! \r\n");
		return;
	}

	/* �����Ƶ¼��״̬ */
	Wt2000_QueryState(kType_WtCmd_State);
	Audio_WaitAck(ack);
	if(ack[0] == kType_WtCmd_State){
        playSta = ack[1];
	}	

    switch(audState->state) {
    case kStatus_Aud_Playing: {
		if(playSta == kStatus_WtPlay_Playing){
	        audState->runTime++;
	        /* ��ȡ��ǰ�������� */
	        Wt2000_QueryState(kType_WtCmd_CurrPlayFile);
	        Audio_WaitAck(ack);
	        if(ack[0] == kType_WtCmd_CurrPlayFile && audState->playIndex != ack[2]){
	            audState->playIndex = ack[2];
				audState->runTime = 0;
			}	
		}
		else if(playSta == kStatus_WtPlay_Stop){
			Audio_PlayNext();
		}
    }
    break;
    case kStatus_Aud_Recording: {
        audState->runTime++;
    }
    break;

    default:
        break;

    }

    xSemaphoreGive(UartCmdSendSem);

//    debug("AudioState = %X, audSta = %X, time = %d ,index = %d, usbConnect = %d\r\n",audState->state,playSta,audState->runTime,audState->playIndex,usbConnect);

    if(Listener != null)
        Listener(audState);
}



