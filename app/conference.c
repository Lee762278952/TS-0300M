/**
 *
 *	@name				conference.c
 *	@author 			KT
 *	@date 				2019/07/26
 *	@brief
 *  @include			conference.h
 *
 *  @API
 *
 *  @description
 *
 **/

/*******************************************************************************
 * includes
 ******************************************************************************/
#include "stdarg.h"
#include "stdio.h"
#include "string.h"

/* HAL */
#include "hal_gpio.h"

/* OS */
#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"

/* API */
#include "app.h"
#include "data_queue.h"
#include "ram.h"
#include "usb_disk.h"
#include "audio.h"

/* APP */
#include "conference.h"
#include "wired_unit.h"
#include "wifi_unit.h"
#include "external_ctrl.h"
#include "screen.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* �����ջ��С�����ȼ� */
#define CONFERENCE_TASK_STACK_SIZE						(2048)
#define CONFERENCE_TASK_PRIORITY						(17)

#define NOTICE_QUEUE_LENGTH								(128)
#define NOTICE_QUEUE_SIZE								(sizeof(void *))

/* WIFI��ԪIDת��   ��WIFI��ԪID��0x3000��ʼ */
#define WIFI_ID(_id)									(_id + 0x3000)

/* ��Ԫָ��(type �� cmd)�����(ph,pl)�ϲ���չ��32λ�������ϱ�־λ��������ͬʱ�����������͵ĵ�Ԫ��ָ��ʱ���ֻ���������ظ�
	��һ���ֽ�Ϊ�豸���ͱ�ʾ �ڶ����ֽ�Ϊָ��(type��cmd) �������ĸ��ֽ��ǲ���(ph,pl)*/
#define WIRED_CMD(_type,_ph,_pl)						( 0x80000000 | (_type << 16) | (_ph << 8) | _pl)
#define WIFI_CMD(_cmd,_ph,_pl)							( 0x40000000 | (_cmd << 16) | (_ph << 8) | _pl)

#if 0
/* �ж�ͶƱģʽ�Ƿ���Ҫǩ�� */
#define IS_VOTE_NEED_SIGN(_mode)						(_mode == Key3First_Sign_vote 		 || _mode == Key3Last_Sign_vote 		||   	\
														 _mode == Key5First_Sign_Select 	 || _mode == Key5Last_Sign_Select  		||  	\
														 _mode == Key5First_Sign_Rate		 || _mode == Key5Last_Sign_Rate			||  	\
														 _mode == Key2First_Sign_CustomTerm  || _mode == Key2Last_Sign_CustomTerm	||  	\
														 _mode == Key3First_Sign_CustomTerm  || _mode == Key3Last_Sign_CustomTerm	||  	\
														 _mode == Key4First_Sign_CustomTerm  || _mode == Key4Last_Sign_CustomTerm	||  	\
														 _mode == Key5First_Sign_CustomTerm  || _mode == Key5Last_Sign_CustomTerm	||  	\
														 _mode == Key3First_Sign_Satisfaction|| _mode == Key3Last_Sign_Satisfaction)

/* �ж�ͶƱģʽ�Ƿ��һ����Ч */
#define IS_VOTE_FIRST_VALID(_mode)						(_mode == Key3First_Sign_vote 		 || _mode == Key3First_NoSign_vote		||		\
														 _mode == Key5First_Sign_Select 	 || _mode == Key5First_NoSign_Select   	||  	\
														 _mode == Key5First_Sign_Rate		 || _mode == Key5First_NoSign_Rate		||  	\
														 _mode == Key2First_Sign_CustomTerm  || _mode == Key2First_NoSign_CustomTerm||  	\
														 _mode == Key3First_Sign_CustomTerm  || _mode == Key3First_NoSign_CustomTerm||  	\
														 _mode == Key4First_Sign_CustomTerm  || _mode == Key4First_NoSign_CustomTerm||  	\
														 _mode == Key5First_Sign_CustomTerm  || _mode == Key5First_NoSign_CustomTerm||  	\
														 _mode == Key3First_Sign_Satisfaction|| _mode == Key3First_NoSign_Satisfaction)

#endif


/* �����б���󳤶� */
#define MUSIC_FILE_LIST_MAX_LEN						(64)

/*************************************  Instructions  *******************************************/


typedef enum {
    tOpenMic,
    tCloseMic,
    tWaiting,
    tDisWait,
    tMicFull,
    tMicApply,
    tRevokeApply,
} MicControlType_EN;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* API */
static void Conference_Launch(void);
static void Conference_Notify(Notify_S *notify);
static void Conference_UnitOnlineNum(UnitOnlineNum *onlineNum);
static SysMode_EN Conference_GetSysMode(void);
static uint16_t Conference_GetCurrentEditedID(void);
static UnitInfo_S *Conference_WiredUnitInfo(void);
static UnitInfo_S *Conference_WifiUnitInfo(void);
static ConfSysInfo_S Conference_GetConfSysInfo(void);


/* �ڲ����� */
static void Conference_NoticeProcessTask(void *pvParameters);
static void Conference_MessageProcess(Notify_S *notify);
static void Conference_WifiUnitMessageProcess(Notify_S *notify);

static void Conference_ScreenPageSwitch(uint8_t page);
static void Conference_ScreenMessageProcess(Notify_S *notify);

static void Conference_ResetAudioChannel(UnitType_EN type);
static uint8_t Conference_GetAudioChannel(UnitType_EN type);
static void Conference_GiveAudioChannel(UnitType_EN type,uint8_t ch);

static void Conference_SignInstruction(uint16_t id,UnitType_EN type,uint32_t cmd,uint16_t totalNum,uint16_t curNum);
static void Conference_MicCtrlInstruction(uint16_t id, UnitType_EN type,UnitAttr_EN attr, MicControlType_EN ctrlType,uint8_t channel);
static void Conference_MicControl(uint16_t id, UnitType_EN type, uint32_t cmd);


static void Conference_CloseAllMic(UnitType_EN type,UnitAttr_EN attr);
static void Conference_OfflineAllUnit(void);
static void Conference_ClearApplyMic(UnitType_EN type);
static void Conference_ChangeSysMode(SysMode_EN mode);
static void Conference_ChairmanPriority(UnitType_EN type,uint16_t id);

static void Conference_UsbStateListener(status_t sta);
static void Conference_AudioStateListener(AudState_S *sta);


static void Conference_OpenMicListDebug(void);

#ifdef	CONFIRM_UNIT_NUM
static void Conference_ConfirmUnitNum(TimerHandle_t xTimer);
#endif
/*******************************************************************************
 * Variables
 ******************************************************************************/
/* ����֪ͨ���� */
static QueueHandle_t noticeQueue;

///* ����ϵͳͨѶЭ�� */
//static ConfProtocol_S confProt;
//
///* WIFI��ԪͨѶЭ�� */
//static WifiUnitProtocol_S wifiProt;

/* ��Ԫ��Ϣ�����ջ
�ܴ�СΪ���Ԫ��+1 ��Ϊ��Ԫû��0��,�������� */
static struct {
    UnitInfo_S wired[WIRED_UNIT_MAX_ONLINE_NUM + 1];
    UnitInfo_S wifi[WIFI_UNIT_MAX_ONLINE_NUM + 1];
} UnitInfo __attribute__((section("OcramHeap")));

/* ���߻�Ͳ�� */
static UnitOnlineNum OnlineNum;

/* ����ϵͳ��Ϣ */
static ConfSysInfo_S SysInfo;

/* ȫ���֣����ߣ���ϯ������ */
static DataQueueHandler_S WiredChmMicQueue;

/* ȫ���֣����ߣ���������� */
static DataQueueHandler_S WiredRpsMicQueue;

/* ȫ���֣����ߣ��ȴ�(����)���� */
static DataQueueHandler_S WiredWaitQueue;

/* ȫ���֣����ߣ���Ƶͨ������ */
static DataQueueHandler_S WiredChannelQueue;

/* WIFI ��ϯ������ */
static DataQueueHandler_S WifiChmMicQueue;

/* WIFI ��������� */
static DataQueueHandler_S WifiRpsMicQueue;

/* WIFI �ȴ�(����)���� */
static DataQueueHandler_S WifiWaitQueue;

/* WIFI ��Ƶͨ������ */
static DataQueueHandler_S WifiChannelQueue;


#ifdef	CONFIRM_UNIT_NUM
/* ȷ�ϻ�Ͳ������ʱ�� */
static TimerHandle_t ConfirmUnitNumTimer;
#endif


/* ͶƱģʽ�루ȫ����Э�飩 */
static const uint8_t VoteModeCode[] = {
    KEY3_LAST_NOSIGN_VOTE, KEY3_LAST_SIGN_VOTE, KEY3_FIRST_NOSIGN_VOTE, KEY3_FIRST_SIGN_VOTE,
    KEY5_LAST_NOSIGN_SELECT, KEY5_LAST_SIGN_SELECT, KEY5_FIRST_NOSIGN_SELECT, KEY5_FIRST_SIGN_SELECT,
    KEY5_LAST_NOSIGN_RATE, KEY5_LAST_SIGN_RATE, KEY5_FIRST_NOSIGN_RATE, KEY5_FIRST_SIGN_RATE,
    KEY2_FIRST_SIGN_CUSTOM, KEY2_FIRST_NOSIGN_CUSTOM, KEY2_LAST_SIGN_CUSTOM, KEY2_LAST_NOSIGN_CUSTOM,
    KEY3_FIRST_SIGN_CUSTOM, KEY3_FIRST_NOSIGN_CUSTOM, KEY3_LAST_SIGN_CUSTOM, KEY3_LAST_NOSIGN_CUSTOM,
    KEY4_FIRST_SIGN_CUSTOM, KEY4_FIRST_NOSIGN_CUSTOM, KEY4_LAST_SIGN_CUSTOM, KEY4_LAST_NOSIGN_CUSTOM,
    KEY5_FIRST_SIGN_CUSTOM, KEY5_FIRST_NOSIGN_CUSTOM, KEY5_LAST_SIGN_CUSTOM, KEY5_LAST_NOSIGN_CUSTOM,
    KEY3_LAST_NOSIGN_SATISFACTION, KEY3_LAST_SIGN_SATISFACTION, KEY3_FIRST_NOSIGN_SATISFACTION, KEY3_FIRST_SIGN_SATISFACTION
};

/* ��¼��ǰ��ʾҳ�� */
static uint8_t ScreenCurrentPage = 0;


/* USB��Ƶ¼�ſ��ƾ�� */
static struct {
    /* MCU������U�� */
    bool mcuConnectUsb;
    /* ��Ƶ¼��״̬ */
    AudStaType_EN audSta;
	/* ¼�������ʱ�� */
	uint32_t runtime;
	/* ¼���ļ� */
	char recFile[25];
    /* ��ǰ������������ */
    uint8_t playIndex;
    /* �����ļ����� */
    uint8_t musicFileNum;
    /* �����ļ��б� */
    FILINFO *musicFile[MUSIC_FILE_LIST_MAX_LEN];
} UsbAudio;



/* API  */
Conference_S Conference = {
    .launch = Conference_Launch,
    .notify = Conference_Notify,
    .getOnlineNum = Conference_UnitOnlineNum,
    .getSysMode = Conference_GetSysMode,
    .getCurrentEditId = Conference_GetCurrentEditedID,
    .wiredUnitInfo = Conference_WiredUnitInfo,
    .wifiUnitInfo = Conference_WifiUnitInfo,
    .getConfSysInfo = Conference_GetConfSysInfo,
};
/*******************************************************************************
 * Code
 ******************************************************************************/


/**
* @Name  		Conference_Launch
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_Launch(void)
{
    noticeQueue = xQueueCreate(NOTICE_QUEUE_LENGTH,NOTICE_QUEUE_SIZE);

    memset(&UnitInfo,0,sizeof(UnitInfo));

    memset(&UsbAudio,0,sizeof(UsbAudio));

    SysInfo.micMode = kMode_Mic_Fifo;

    SysInfo.wiredAllowOpen = WIRED_UNIT_MAX_ALLWO_OPEN;
    SysInfo.wiredAllowWait = WIRED_UNIT_MAX_ALLWO_OPEN;
    WiredChmMicQueue = DataQueue.creat(WIRED_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WiredRpsMicQueue = DataQueue.creat(WIRED_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WiredWaitQueue = DataQueue.creat(WIRED_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WiredChannelQueue = DataQueue.creat(WIRED_UNIT_MAX_ALLWO_OPEN,sizeof(uint8_t));

    SysInfo.wifiAllowOpen = WIFI_UNIT_MAX_ALLWO_OPEN;
    SysInfo.wifiAllowWait = WIFI_UNIT_MAX_ALLWO_OPEN;
    WifiChmMicQueue = DataQueue.creat(WIFI_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WifiRpsMicQueue = DataQueue.creat(WIFI_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WifiWaitQueue = DataQueue.creat(WIFI_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WifiChannelQueue = DataQueue.creat(WIFI_UNIT_MAX_ALLWO_OPEN,sizeof(uint8_t));

    /* ��λ��Ԫ��Ƶͨ������ */
    Conference_ResetAudioChannel(tWired);
    Conference_ResetAudioChannel(tWifi);

    /* ����USB״̬���� */
    UsbDisk.setListener(Conference_UsbStateListener);

    /* ��Ƶ¼��״̬���� */
    Audio.setListener(Conference_AudioStateListener);

    if (xTaskCreate(Conference_NoticeProcessTask, "NoticeProcessTask", CONFERENCE_TASK_STACK_SIZE, null, CONFERENCE_TASK_PRIORITY, null) != pdPASS) {
        debug("create host task error\r\n");
    }

//	DevLaunchMute = xTimerCreate("DevLaunchMute", const TickType_t xTimerPeriodInTicks, const UBaseType_t uxAutoReload, void * const pvTimerID, TimerCallbackFunction_t pxCallbackFunction)
}


/**
* @Name  		Conference_Notify
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_Notify(Notify_S *notify)
{
    if(notify == null)
        return;

    xQueueSend(noticeQueue,&notify,0);
}

/**
* @Name  		Conference_NoticeProcessTask
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_NoticeProcessTask(void *pvParameters)
{
    Notify_S *notify;

    debug("Conference notice process task launch!\r\n");
#ifdef	CONFIRM_UNIT_NUM
    /* ��ʼ��ȷ�ϻ�Ͳ������ʱ�� */
    ConfirmUnitNumTimer = xTimerCreate("ConfirmUnitNum",3000,pdFALSE,null,Conference_ConfirmUnitNum);
#endif
    while(1) {
        xQueueReceive(noticeQueue, &notify, MAX_NUM);

        /* �ж�֪ͨ��Դ */
        if(notify->nSrc & (EX_CTRL_DEST | kType_NotiSrc_WiredUnit)) {
            /* ֪ͨ��ԴΪ�ⲿ���Ƽ����ߵ�Ԫ */
            Conference_MessageProcess(notify);
        }

        else if(notify->nSrc & kType_NotiSrc_WifiUnit) {
            /* ֪ͨ��ԴΪWIFI��Ԫ */
            Conference_WifiUnitMessageProcess(notify);
        }

        else if(notify->nSrc & kType_NotiSrc_ScreenCtrl) {
            /* ֪ͨ��ԴΪ��Ļ���� */
            Conference_ScreenMessageProcess(notify);
        }
        FREE(notify);
    }
}



#ifdef	CONFIRM_UNIT_NUM
/**
* @Name  		Conference_ConfirmUnitNum
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_ConfirmUnitNum(TimerHandle_t xTimer)
{
    uint16_t chmNum = 0, rpsNum = 0, i;

    for(i = 0; i < WIRED_UNIT_MAX_ONLINE_NUM; i++) {
        if(UnitInfo.wired[i].online) {
            if(UnitInfo.wired[i].attr == aChairman)	chmNum++;
            else if(UnitInfo.wired[i].attr == aRepresentative) rpsNum++;
        }
    }

    if(OnlineNum.wiredChm != chmNum || OnlineNum.wiredRps != rpsNum) {
        OnlineNum.wiredChm = chmNum;
        OnlineNum.wiredRps = rpsNum;
        debug("Online num is update: wiredUnit(CHM) = %d , wiredUnit(RPS) = %d\r\n",OnlineNum.wiredChm,OnlineNum.wiredRps);
    }
}
#endif

/**
* @Name  		Conference_MessageProcess
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_MessageProcess(Notify_S *notify)
{
    uint16_t id;
    uint8_t type, *para;
    ConfProtocol_S confProt;
    WifiUnitProtocol_S wifiProt;

    ERR_CHECK(notify != null, return);

    id = notify->prot.conference.id;
    type = notify->prot.conference.type;
    para = &notify->prot.conference.ph;

#if 1
    if(true) {
        char srcStr[10];
        sprintf(srcStr,"%s",notify->nSrc == kType_NotiSrc_WiredUnit ? "WiredUnit" :  \
                notify->nSrc == kType_NotiSrc_PC ? "PC" : \
                notify->nSrc == kType_NotiSrc_Web ? "WEB" : \
                notify->nSrc == kType_NotiSrc_UartCtrl ? "UART" : "unknow");
        debug("Conference notify from %s: id = %X, type = %X,para{ %X , %X , %X , %X , %X }",srcStr,id,type,para[0],para[1],para[2],para[3],para[4]);
        if(notify->exLen) {
            uint8_t i;

            debug(" exData{ ");
            for(i=0; i<notify->exLen; i++)
                debug(" %X ,",(&notify->exDataHead)[i]);
            debug(" } ");
        }
        debug("\r\n");
    }
#endif

    switch(type) {
    /* ������Ϣ 0x80 */
    case BASIC_MSG: {
        uint8_t mode = para[0], cmd = para[1];

        switch(mode) {
        /* ����ģʽ 0x00 */
        case CONFERENCE_MODE: {
            switch(cmd) {
            case START_CONFERENCE_MODE: {
                Conference_ChangeSysMode(kMode_Conference);
            }
            break;
            /*��ϯ���뿪��Ͳ*/
            case CHM_OPEN_MIC:
            /*�������뿪��Ͳ*/
            case RPS_OPEN_MIC:
            /* ��Ͳ�����ȴ� */
            case MIC_DISWAIT:
            /*��ϯ����ػ�Ͳ*/
            case CHM_CLOSE_MIC:
            /*��������ػ�Ͳ*/
            case RPS_CLOSE_MIC: {
                if(id > 0x3000 && (notify->nSrc & EX_CTRL_DEST))
                    Conference_MicControl(id - WIFI_UNIT_START_ID, tWifi, WIRED_CMD(cmd,0,0));
                else
                    Conference_MicControl(id, tWired, WIRED_CMD(cmd,0,0));
            }
            break;

            /* ͬ�⿪��Ͳ */
            case AGREE_OPEN_MIC:
            /* ��ͬ�⿪��Ͳ */
            case DISAGREE_OPEN_MIC: {
                if(notify->nSrc & EX_CTRL_DEST && id > 0x3000 )
                    Conference_MicControl(id - WIFI_UNIT_START_ID, tWifi, WIRED_CMD(cmd,0,0));
                else if(notify->nSrc & EX_CTRL_DEST && id > 0 && id < 4096 )
                    Conference_MicControl(id, tWired, WIRED_CMD(cmd,0,0));
                else if(notify->nSrc == kType_NotiSrc_WifiUnit || notify->nSrc == kType_NotiSrc_WiredUnit)
                    Conference_MicControl(null, (UnitType_EN)null, WIRED_CMD(cmd,0,0));
            }
            break;

            /* ��ϯ����Ȩ */
            case CHM_PRIORITY: {
                Conference_ChairmanPriority(tWired,id);
            }
            break;

            /* ��Ͳ���� */
            case UNIT_ACCESS_SYS: {
                if(UnitInfo.wired[id].attr == aChairman)
                    OnlineNum.wiredChm++;
                else if(UnitInfo.wired[id].attr == aRepresentative)
                    OnlineNum.wiredRps++;
#ifdef	CONFIRM_UNIT_NUM
                /* �򿪶�ʱ����3S����һ�»�Ͳ�б������Բ��ԣ���ֹ��Ͳ�ظ����� */
                xTimerStart(ConfirmUnitNumTimer,0);
#endif
                debug("WiredUnit(%s) id = %d online ( Chm num = %d , Rps num = %d )\r\n",UnitInfo.wired[id].attr == aChairman ? "CHM":"RPS",id,OnlineNum.wiredChm,OnlineNum.wiredRps);
            }
            break;

            /*��Ͳ����*/
            case UNIT_OFFLINE: {
                if(UnitInfo.wired[id].attr == aChairman) {
                    Conference_MicControl(id, tWired, WIRED_CMD(CHM_CLOSE_MIC,0,0));

                    OnlineNum.wiredChm--;
                } else if(UnitInfo.wired[id].attr == aRepresentative) {
                    Conference_MicControl(id, tWired, WIRED_CMD(RPS_CLOSE_MIC,0,0));
                    OnlineNum.wiredRps--;
                }
#ifdef	CONFIRM_UNIT_NUM
                xTimerStart(ConfirmUnitNumTimer,0);
#endif
                debug("WiredUnit(%s) id = %d offline ( Chm num = %d , Rps num = %d )\r\n",UnitInfo.wired[id].attr == aChairman ? "CHM":"RPS",id,OnlineNum.wiredChm,OnlineNum.wiredRps);
            }
            break;
            }
        }
        break;

        /* ǩ��ģʽ 0x01 */
        case SIGN_MODE: {
            switch(cmd) {
            /* ����ǩ��ģʽ */
            case START_SIGN_MODE: {
                SysInfo.totalSignNum = (uint16_t)((para[3] << 8) | para[4]);
                SysInfo.currentSignNum = 0;

                Conference_ChangeSysMode(kMode_Sign);
            }
            break;
            /* ��Ԫǩ�� */
            case UNIT_SIGN_IN: {
                /* ��Ԫǩ�� */
                if((notify->nSrc & kType_NotiSrc_WiredUnit) && !UnitInfo.wired[id].sign) {
                    UnitInfo.wired[id].sign = true;
                    SysInfo.currentSignNum++;
                    Conference_SignInstruction(id,tWired,WIRED_CMD(UNIT_SIGN_IN,0,0),null,null);
                    Conference_SignInstruction(null,(UnitType_EN)null,WIRED_CMD(START_SIGN_MODE,0,0),SysInfo.totalSignNum,SysInfo.currentSignNum);
                }
                /* �����Ԫǩ�������ⲿ���ƣ���Ϊ����ǩ�� */
                else if(notify->nSrc & (kType_NotiSrc_PC | kType_NotiSrc_Web | kType_NotiSrc_UartCtrl)) {
                    /* �������ߵ�Ԫǩ�� */
                    if(id > 0 && id <= WIRED_UNIT_MAX_ONLINE_NUM && !UnitInfo.wired[id].sign) {
                        Conference_SignInstruction(id,tWired,WIRED_CMD(CONTROL_UNIT_SIGN,0,0),null,null);
                    }
                    /* ����WIFI��Ԫǩ�� */
                    else if(id > WIFI_UNIT_START_ID && id <= WIFI_UNIT_START_ID + WIFI_UNIT_MAX_ONLINE_NUM  \
                            && !UnitInfo.wifi[id - WIFI_UNIT_START_ID].sign) {
                        Conference_SignInstruction(id - WIFI_UNIT_START_ID,tWifi,WIFI_CMD(ControlSign_MtoU_D,0,0),null,null);
                    }
                }
            }
            break;
            /* ����ǩ�� */
            case END_SIGN_MODE: {
                WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,SIGN_MODE,END_SIGN_MODE,null,null));
                WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,EnterSignMode_MtoU_G,1,null));
                Conference_ChangeSysMode(kMode_Conference);
            }
            break;

            /* ����ǩ�� */
            case SUPPLEMENT_SIGN: {
                /* ���ߵ�Ԫ����ǩ�� */
                if(id > 0 && id <= WIRED_UNIT_MAX_ONLINE_NUM && !UnitInfo.wired[id].sign) {
                    Conference_SignInstruction(id,tWired,WIRED_CMD(SUPPLEMENT_SIGN,0,0),null,null);
                }
                /* WIFI��Ԫ����ǩ��*/
                else if(id > WIFI_UNIT_START_ID && id <= WIFI_UNIT_START_ID + WIFI_UNIT_MAX_ONLINE_NUM  \
                        && !UnitInfo.wifi[id - WIFI_UNIT_START_ID].sign) {
                    Conference_SignInstruction(id - WIFI_UNIT_START_ID,tWifi,WIFI_CMD(SupplementSign_MtoU_D,0,0),null,null);
                }
            }
            break;

            /* ��ԪӦ�𲹳�ǩ�� */
            case UNIT_SUPPLEMENT_SIGN: {
                if((notify->nSrc & kType_NotiSrc_WiredUnit) && !UnitInfo.wired[id].sign) {
                    UnitInfo.wired[id].sign = true;
                    SysInfo.currentSignNum++;
                    Conference_SignInstruction(id,tWired,WIRED_CMD(UNIT_SUPPLEMENT_SIGN,0,0),null,null);
                }
            }
            break;
            }
        }
        break;

        /* ͶƱģʽ 0x02 */
        case VOTE_MODE: {
            uint8_t voteResult;

            switch(cmd) {
            /* PC����ĸ�����ͶƱ������������������ */
            case KEY3_FIRST_SIGN_VOTE:
                SysInfo.voteMode = Key3First_Sign_vote;
                goto startVote;
            case KEY3_FIRST_NOSIGN_VOTE:
                SysInfo.voteMode = Key3First_NoSign_vote;
                goto startVote;
            case KEY3_LAST_SIGN_VOTE:
                SysInfo.voteMode = Key3Last_Sign_vote;
                goto startVote;
            case KEY3_LAST_NOSIGN_VOTE:
                SysInfo.voteMode = Key3Last_NoSign_vote;
                goto startVote;
            case KEY5_FIRST_SIGN_SELECT:
                SysInfo.voteMode = Key5First_Sign_Select;
                goto startVote;
            case KEY5_FIRST_NOSIGN_SELECT:
                SysInfo.voteMode = Key5First_NoSign_Select;
                goto startVote;
            case KEY5_LAST_SIGN_SELECT:
                SysInfo.voteMode = Key5Last_Sign_Select;
                goto startVote;
            case KEY5_LAST_NOSIGN_SELECT:
                SysInfo.voteMode = Key5Last_NoSign_Select;
                goto startVote;
            case KEY5_FIRST_SIGN_RATE:
                SysInfo.voteMode = Key5First_Sign_Rate;
                goto startVote;
            case KEY5_FIRST_NOSIGN_RATE:
                SysInfo.voteMode = Key5First_NoSign_Rate;
                goto startVote;
            case KEY5_LAST_SIGN_RATE:
                SysInfo.voteMode = Key5Last_Sign_Rate;
                goto startVote;
            case KEY5_LAST_NOSIGN_RATE:
                SysInfo.voteMode = Key5Last_NoSign_Rate;
                goto startVote;
            case KEY2_FIRST_SIGN_CUSTOM:
                SysInfo.voteMode = Key2First_Sign_CustomTerm;
                goto startVote;
            case KEY2_FIRST_NOSIGN_CUSTOM:
                SysInfo.voteMode = Key2First_NoSign_CustomTerm;
                goto startVote;
            case KEY2_LAST_SIGN_CUSTOM:
                SysInfo.voteMode = Key2Last_Sign_CustomTerm;
                goto startVote;
            case KEY2_LAST_NOSIGN_CUSTOM:
                SysInfo.voteMode = Key2Last_NoSign_CustomTerm;
                goto startVote;
            case KEY3_FIRST_SIGN_CUSTOM:
                SysInfo.voteMode = Key3First_Sign_CustomTerm;
                goto startVote;
            case KEY3_FIRST_NOSIGN_CUSTOM:
                SysInfo.voteMode = Key3First_NoSign_CustomTerm;
                goto startVote;
            case KEY3_LAST_SIGN_CUSTOM:
                SysInfo.voteMode = Key3Last_Sign_CustomTerm;
                goto startVote;
            case KEY3_LAST_NOSIGN_CUSTOM:
                SysInfo.voteMode = Key3Last_NoSign_CustomTerm;
                goto startVote;
            case KEY4_FIRST_SIGN_CUSTOM:
                SysInfo.voteMode = Key4First_Sign_CustomTerm;
                goto startVote;
            case KEY4_FIRST_NOSIGN_CUSTOM:
                SysInfo.voteMode = Key4First_NoSign_CustomTerm;
                goto startVote;
            case KEY4_LAST_SIGN_CUSTOM:
                SysInfo.voteMode = Key4Last_Sign_CustomTerm;
                goto startVote;
            case KEY4_LAST_NOSIGN_CUSTOM:
                SysInfo.voteMode = Key4Last_NoSign_CustomTerm;
                goto startVote;
            case KEY5_FIRST_SIGN_CUSTOM:
                SysInfo.voteMode = Key5First_Sign_CustomTerm;
                goto startVote;
            case KEY5_FIRST_NOSIGN_CUSTOM:
                SysInfo.voteMode = Key5First_NoSign_CustomTerm;
                goto startVote;
            case KEY5_LAST_SIGN_CUSTOM:
                SysInfo.voteMode = Key5Last_Sign_CustomTerm;
                goto startVote;
            case KEY5_LAST_NOSIGN_CUSTOM:
                SysInfo.voteMode = Key5Last_NoSign_CustomTerm;
                goto startVote;
            case KEY3_FIRST_SIGN_SATISFACTION:
                SysInfo.voteMode = Key3First_Sign_Satisfaction;
                goto startVote;
            case KEY3_FIRST_NOSIGN_SATISFACTION:
                SysInfo.voteMode = Key3First_NoSign_Satisfaction;
                goto startVote;
            case KEY3_LAST_SIGN_SATISFACTION:
                SysInfo.voteMode = Key3Last_Sign_Satisfaction;
                goto startVote;
            case KEY3_LAST_NOSIGN_SATISFACTION:
                SysInfo.voteMode = Key3Last_NoSign_Satisfaction;
                goto startVote;
                {
startVote:
                    Conference_ChangeSysMode(kMode_Vote);
                }
                break;

            /* ��ϯ���� */
            case VOTE_LAUNCH_BY_CHAIRMAN: {
                if(notify->nSrc & kType_NotiSrc_WiredUnit || notify->nSrc & kType_NotiSrc_WifiUnit) {

                }
            }
            break;

            /* ������� */
            case FINISH_VOTE: {
                if(SysInfo.sysMode == kMode_Vote) {
                    Conference_ChangeSysMode(kMode_Conference);
                }
            }
            break;

            /* ��ԪͶƱ�������ѡ�ٽ�� */
            case 0x25:
                voteResult = 1;//�޳�
                goto unitVoted;
            case 0x26:
                voteResult = 2;//��Ȩ
                goto unitVoted;
            case 0x27:
                voteResult = 3;//����
                goto unitVoted;
            case 0x45:
                voteResult = 1;//--
                goto unitVoted;
            case 0x46:
                voteResult = 2;//-
                goto unitVoted;
            case 0x47:
                voteResult = 3;//0
                goto unitVoted;
            case 0x48:
                voteResult = 4;//+
                goto unitVoted;
            case 0x49:
                voteResult = 5;//++
                goto unitVoted;
            case 0x4A:
                voteResult = 1;//�ܲ�����(���)/������(����)
                goto unitVoted;
            case 0x4B:
                voteResult = 2;//������(���)/һ��(����)
                goto unitVoted;
            case 0x4C:
                voteResult = 3;//һ��(���)/����(����)
                goto unitVoted;
            case 0x4D:
                voteResult = 4;//����(���)
                goto unitVoted;
            case 0x4E:
                voteResult = 5;//������(���)
                goto unitVoted;
            case 0x65:
                voteResult = 1;//�Զ�����ѡ��1
                goto unitVoted;
            case 0x66:
                voteResult = 2;//�Զ�����ѡ��2
                goto unitVoted;
            case 0x67:
                voteResult = 3;//�Զ�����ѡ��3
                goto unitVoted;
            case 0x68:
                voteResult = 4;//�Զ�����ѡ��4
                goto unitVoted;
            case 0x69:
                voteResult = 5;//�Զ�����ѡ��5
                {
unitVoted:
                    if(notify->nSrc & kType_NotiSrc_WiredUnit && SysInfo.sysMode == kMode_Vote) {
                        /* ���ͶƱ����Ϊ��Ҫ��ǩ��������Ԫδ����ǩ�� */
//                        if(IS_VOTE_NEED_SIGN(SysInfo.voteMode) && UnitInfo.wired[id].sign != true)
//                            break;
//                        if(IS_VOTE_FIRST_VALID(SysInfo.voteMode) && UnitInfo.wired[id].vote != null)
//                            break;

                        UnitInfo.wired[id].vote = voteResult;
                        ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id,BASIC_MSG,VOTE_MODE,voteResult,null,null));
                    }
                }
                break;

            }
        }
        break;

        /* ��IDģʽ 0x03 */
        case EDIT_ID_MODE: {
            switch(cmd) {

            /* ��Ԫȷ�ϵ�ǰIDΪ����ID */
            case UNIT_CONFIRM_ID: {
                Network_Mac_S *devMac = (Network_Mac_S *)&notify->exDataHead;

                debug("Wired confirm id = %d , dev mac = %X:%X:%X:%X:%X:%X \r\n",id,devMac->mac0,devMac->mac1,devMac->mac2,devMac->mac3,devMac->mac4,devMac->mac5);
                if(id == SysInfo.wiredCurEditID) {
                    /* �ظ���Ԫȷ�� */
                    WiredUnit.transWithExData(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,POLLING_MSG,EDIT_ID_POLLING,CONFIRM_ID,null,id),NETWORK_MAC_SIZE,(uint8_t *)devMac);
                    SysInfo.wiredCurEditID = (SysInfo.wiredCurEditID + 1) > WIRED_UNIT_MAX_ONLINE_NUM ? 1 : SysInfo.wiredCurEditID + 1;
                    WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,POLLING_MSG,EDIT_ID_POLLING,CURRENT_ID,null,SysInfo.wiredCurEditID));
                }
            }
            break;
            }
        }
        break;
        }
    }
    break;
    /* ״̬��Ϣ 0x82*/
    case STATE_MSG: {
        switch(id) {
        case MODE_BROADCAST_ID:
        case WIFI_MODE_BROADCAST_ID: {
            uint8_t mode = para[0], num = para[1];

            if(mode >= kMode_Mic_Fifo && mode <= kMode_Mic_Apply)
                SysInfo.micMode = (MicMode_EN)mode;

            /* �������ߵ�Ԫ */
            if(id == MODE_BROADCAST_ID && num >= 1 && num <= WIRED_UNIT_MAX_ALLWO_OPEN) {
                SysInfo.wiredAllowOpen = num;
                SysInfo.wiredAllowWait = num;
            }

            /* ����WIFI��Ԫ */
            else if(id == WIFI_MODE_BROADCAST_ID && num >= 1 && num <= WIFI_UNIT_MAX_ALLWO_OPEN) {
                SysInfo.wifiAllowOpen = num;
                SysInfo.wifiAllowWait = num;
            }

            /* �ر����е�Ԫ */
            Conference_CloseAllMic(tWired,aChairman);
            Conference_CloseAllMic(tWired,aRepresentative);
            Conference_CloseAllMic(tWifi,aChairman);
            Conference_CloseAllMic(tWifi,aRepresentative);

            /* ���������� */
            Conference_ClearApplyMic(tWired);
            Conference_ClearApplyMic(tWifi);


            /* �㲥ģʽ������ */
            /* ���͵���Ԫ */
            WiredUnit.transmit(Protocol.conference(&confProt,MODE_BROADCAST_ID,STATE_MSG,SysInfo.micMode,SysInfo.wiredAllowOpen,null,null));
            WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,ChangeMicManage_MtoU_G,SysInfo.micMode,SysInfo.wifiAllowOpen));
            /* �ظ��ⲿ�����豸ȷ��״̬ */
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,MODE_BROADCAST_ID,STATE_MSG,SysInfo.micMode,SysInfo.wiredAllowOpen,null,null));
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_MODE_BROADCAST_ID,STATE_MSG,SysInfo.micMode,SysInfo.wifiAllowOpen,null,null));
        }
        break;

        }
    }
    break;
    /* ͨѶ��Ϣ 0x86*/
    case PC_MSG: {
        uint8_t cmd = para[0];

        switch(cmd) {
        /* 0x01 ~ 0x05 */
        case CFG_UART_1:
        case CFG_UART_2:
        case CFG_UART_3:
        case CFG_UART_4:
        case CFG_UART_5: {

        } break;
        /* 0x06 */
        case CFG_MAC:
            break;
        /* 0x07 */
        case CFG_IP:
            break;
        /* 0x08 */
        case CFG_MASK:
            break;
        /* 0x09 */
        case CFG_GW:
            break;
        /* 0x0F */
        case QUERY_HOST: {
            /* �ظ���ǰ����ģʽ�����ߵ�Ԫ���Ͳ�� */
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,MODE_BROADCAST_ID,STATE_MSG,SysInfo.micMode,SysInfo.wiredAllowOpen,null,null));
            /* �ظ���ǰ����ģʽ��WIFI��Ԫ���Ͳ�� */
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WIFI_MODE_BROADCAST_ID,STATE_MSG,SysInfo.micMode,SysInfo.wifiAllowOpen,null,null));
        }
        break;
        /* �·��Զ������� 0x24 */
        case CUSTOM_VOTE_ITEM: {
            uint8_t item = para[1];

            if(item >= 1 && item <= 5) {
                uint8_t *customItem;

                customItem = MALLOC(3 + notify->exLen);
                memcpy(&customItem[0],&para[2],3);
                if(notify->exLen > 0)
                    memcpy(&customItem[3],&notify->exDataHead,notify->exLen);

                /* Ŀǰֻ��WIFI��Ԫ���Զ����� */
                WifiUnit.transWithExData(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,PCUpdateCustomTerm_MtoU_G,item,0),3 + notify->exLen,customItem);
                FREE(customItem);
            }
        }
        break;

        /* �����ID   	0x2F*/
        case PC_EDIT_ID: {
            if(para[1] == PC_START_EDIT_ID) {
                /* ��ʼ��ID */
                uint8_t startID = (notify->prot.conference.sec == 0 ? 1 : notify->prot.conference.sec);

                SysInfo.wiredCurEditID = startID;
                SysInfo.wifiCurEditID = WIFI_ID(startID);
                Conference_ChangeSysMode(kMode_EditID);
            }

            else if(para[1] == PC_STOP_EDIT_ID) {
                Conference_ChangeSysMode(kMode_Conference);
                WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,MasterStarUp_MtoU_G,0,0));
            }
        }
        break;
        /* ɨ�����ߵ�ԪID 0x30 */
        case SCAN_ONLINE_UNIT: {
            /* �ϴ��������ߵ�Ԫ */
            for(id = 1; id <= WIRED_UNIT_MAX_ONLINE_NUM; id++) {
                if(UnitInfo.wired[id].online) {
                    ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,id,PC_MSG,ONLINE_UNIT,UnitInfo.wired[id].attr,null,null));
                }
            }
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,SCAN_ONLINE_UNIT,SCAN_UNIT_END,tWired, \
                                  (OnlineNum.wiredChm + OnlineNum.wiredRps)));

            /* �ϴ�����WIFI��Ԫ */
            for(id = 1; id <= WIFI_UNIT_MAX_ONLINE_NUM; id++) {
                if(UnitInfo.wifi[id].online) {
                    ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WIFI_ID(id),PC_MSG,ONLINE_UNIT,UnitInfo.wifi[id].attr,null,null));
                }
            }
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,SCAN_ONLINE_UNIT,SCAN_UNIT_END,tWifi, \
                                  (OnlineNum.wifiChm + OnlineNum.wifiRps)));
        }
        break;

        case QUERY_PRIOR_SIGN:
        case QUERY_PRIOR_VOTE:
        case QUERY_PRIOR_SCAN: {
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,cmd,0x01,SysInfo.sysMode,null));
        }
        break;
        }
    }
    break;
    }
}

/**
* @Name  		Conference_WifiUnitMessageProcess
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_WifiUnitMessageProcess(Notify_S *notify)
{
    uint16_t id;
    uint8_t cmd, ph,pl;
    WifiUnitProtocol_S wifiProt;
    ConfProtocol_S confProt;

    ERR_CHECK(notify != null, return);

    id = notify->prot.wifi.id;
    cmd = notify->prot.wifi.cmd;
    ph = notify->prot.wifi.ph;
    pl = notify->prot.wifi.pl;
#if 1
    debug("Wifi Unit Msg: id = %d, cmd = %d,ph = %d , pl = %d",id,cmd,ph,pl);
    if(notify->exLen) {
        uint8_t i;

        debug(" exData{ ");
        for(i=0; i<notify->exLen; i++)
            debug(" %X,",(&notify->exDataHead)[i]);
        debug(" } ");
    }
    debug("\r\n");
#endif

    switch(cmd) {

    case RepPollUnitl_UtoM_D: {
        switch(SysInfo.sysMode) {
        case kMode_Sign: {
            if((ph & 0x01) && !UnitInfo.wifi[id].sign) {
                UnitInfo.wifi[id].sign = true;
                SysInfo.currentSignNum++;
                Conference_SignInstruction(id,tWifi,WIFI_CMD(RepPollUnitl_UtoM_D,kMode_Sign,0),null,null);
                Conference_SignInstruction(null,(UnitType_EN)null,WIFI_CMD(EnterSignMode_MtoU_G,0,0),SysInfo.totalSignNum,SysInfo.currentSignNum);
            }
        }
        break;

        case kMode_Vote: {
            uint8_t voteResult = pl;

            /* ���ͶƱ����Ϊ��Ҫ��ǩ��������Ԫδ����ǩ�� */
//            if(IS_VOTE_NEED_SIGN(SysInfo.voteMode) && UnitInfo.wifi[id].sign != true)
//                break;
//            if(IS_VOTE_FIRST_VALID(SysInfo.voteMode) && UnitInfo.wifi[id].vote != null)
//                break;

            UnitInfo.wifi[id].vote = voteResult;
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),BASIC_MSG,VOTE_MODE,voteResult,null,null));
        }
        break;

        default:
            break;
        }
    }
    break;

    /* ��Ԫ���� */
    case RepApplyEnterSystm_MtoU_D: {
        if(UnitInfo.wifi[id].attr == aChairman)
            OnlineNum.wifiChm++;
        else if(UnitInfo.wifi[id].attr == aRepresentative)
            OnlineNum.wifiRps++;
#ifdef	CONFIRM_UNIT_NUM
        /* �򿪶�ʱ����3S����һ�»�Ͳ�б������Բ��ԣ���ֹ��Ͳ�ظ����� */
        xTimerStart(ConfirmUnitNumTimer,0);
#endif
        debug("WifiUnit(%s) id = %d online ( Chm num = %d , Rps num = %d )\r\n",UnitInfo.wired[id].attr == aChairman ? "CHM":"RPS",id,OnlineNum.wiredChm,OnlineNum.wiredRps);
    }
    break;

    /*��Ԫ�����뿪��Ͳ*/
    case ApplyOpenMic_UtoM_D:
        Conference_MicControl(id, tWifi, WIFI_CMD(cmd,0,0));
        break;

    /*��Ԫ���ؿ���Ͳ*/
    case UnitCloseMic_UtoM_D:
        Conference_MicControl(id, tWifi, WIFI_CMD(cmd,0,0));
        break;

    /*��Ԫ���˳��ȴ�����*/
    case UnitGetoutWaitQuenen_UtoM_D:
        Conference_MicControl(id, tWifi, WIFI_CMD(cmd,0,0));
        break;

    /*��ϯ��ִ������Ȩ*/
    case ChairExecutePriority_UtoM_D:
        Conference_ChairmanPriority(tWifi,id);
        break;

    /*��ϯ��������ͬ����������Ͳ*/
    case ChairAgreeOpenOrNot_UtoM_D:
        Conference_MicControl(0, tWifi, WIFI_CMD(ChairAgreeOpenOrNot_UtoM_D,ph,0));
        break;

    /*��Ԫ����ˮ����    */
    case UnitApplyWater_UtoM_D:
        break;


    /*�ظ������ﵥԪ����ǩ��   */
    case RepSupplementSign_UtoM_D:
        Conference_SignInstruction(id,tWifi,WIFI_CMD(RepSupplementSign_UtoM_D,0,0), null, null);
        break;

    /*�ظ���ϯ������ǩ��   */
    case RepChairStarupOrEndSign_MtoU_D:
        break;

    /*�ظ���ϯ��������   */
    case RepChairStarupOrEndVote_MtoU_D:

        break;

    /*��Ԫ���Ե�ǰ�㲥��ID��Ϊ����ID   */
    case UnitGraspIDUtoM_D: {
        uint8_t *ip = &(&notify->exDataHead)[6],*mac = &(&notify->exDataHead)[0];
        uint16_t confirmID = (uint16_t)(ph << 8) | pl;

        debug("Wifi confirm ID = %d ip : %d.%d.%d.%d  mac : %X-%X-%X-%X-%X-%X \r\n",confirmID - 0x3000,ip[0],ip[1],ip[2],ip[3],mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
        if(confirmID == SysInfo.wifiCurEditID) {
            memcpy(&UnitInfo.wifi[id].ip,ip,NETWORK_IP_SIZE);
            memcpy(&UnitInfo.wifi[id].mac,mac,NETWORK_MAC_SIZE);
            /* �ظ���Ԫȷ�� */
            WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,ConfirmUnitIDMtoU_D,(confirmID >> 8),(confirmID & 0xFF)));
            SysInfo.wifiCurEditID = (SysInfo.wifiCurEditID + 1) > WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM) ? WIFI_ID(1) : SysInfo.wifiCurEditID + 1;
            WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,EnterEditingIDMtoU_G,(SysInfo.wifiCurEditID >> 8),(SysInfo.wifiCurEditID & 0xFF)));
        }
    }
    break;

    case UnitCapacityStrChangeUtoM_D:
        break;

    case RepReadUnitMICSensitivity_UtoM_D:
        break;

//			case RepReadUnitMICEQ_UtoM_D:
//			break;


    default:
        break;
    }


}

/**
* @Name  		Conference_ScreenPageSwitch
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_ScreenPageSwitch(uint8_t page){
	ScreenProtocol_S screenProt;
	ScreenCurrentPage = page;
	Screen.transmit(Protocol.screen(&screenProt,tType_Screen_Page,ScreenCurrentPage));
}



/**
* @Name  		Conference_ScreenMessageProcess
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_ScreenMessageProcess(Notify_S *notify)
{
    WifiUnitProtocol_S wifiProt;
    ConfProtocol_S confProt;
    ScreenProtocol_S screenProt;
    ScreenProtType_EN type;
    uint8_t *para, exData[25];

    static UnitType_EN cfgWitchTypeUnit;

    ERR_CHECK(notify != null, return);

    type = notify->prot.screen.type;
    para = notify->prot.screen.para;

#if 1
    debug("Screen Msg: type = %X, para{%X %X %X %X %X}",type,para[0],para[1],para[2],para[3],para[4]);
    if(notify->exLen) {
        uint8_t i;

        debug(" exData{ ");
        for(i=0; i<notify->exLen; i++)
            debug(" %X,",(&notify->exDataHead)[i]);
        debug(" } ");
    }
    debug("\r\n");
#endif

    switch(type) {
    case tType_Screen_InterfaceRet: {
        uint8_t reg = para[1];
        uint8_t cmd = para[4];

        switch(reg) {
        case 0://�����ع���
            if(para[0]==0x01) {
                switch(cmd) {
                case 0x01://IP
                    break;
                case 0x02://����
                    break;
                case 0x03://��������
                    break;
                case 0x04://�˿�����
                    break;
                case 0x05:
                    break;
                default:
                    break;
                }
            }
            break;
        case 0x10://�������У�������������ָ��
            switch(cmd) {
            /* ����ģʽ */
            case 0x01: {
                Conference_ScreenPageSwitch(System_Type_Page);
            }
            break;

            /* ¼���ڽ��� */
            case 0x02: {
                if(UsbAudio.mcuConnectUsb || UsbAudio.audSta == kStatus_Aud_Playing ||   \
                   UsbAudio.audSta == kStatus_Aud_Pause || UsbAudio.audSta == kStatus_Aud_Recording) {
                    switch(UsbAudio.audSta) {
                    /* �л���ͣҳ�� */
                    case kStatus_Aud_Idle:
                    case kStatus_Aud_Pause: {
                        Conference_ScreenPageSwitch(RECORD_CARD_PAUSE_Page);
                    }
                    break;
                    /* �л�����ҳ�� */
                    case kStatus_Aud_Playing: {
                        char *name = UsbAudio.musicFile[UsbAudio.playIndex - 1]->fname;
                        Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x03A0),strlen(name)+1,(uint8_t *)name);
                        Conference_ScreenPageSwitch(RECORD_CARD_PLAY_Page);
                    }
                    break;
                    /* �л�¼��ҳ�� */
                    case kStatus_Aud_Recording: {
                        Conference_ScreenPageSwitch(RECORD_CARD_RECOEDING_Page);
                    }
                    break;

                    default:
                        break;
                    }
                } 
				else {
                    Conference_ScreenPageSwitch(RECORD_CARD_INITING_Page);
                }

            }
            break;

            /* ϵͳ״̬ */
            case 0x03: {
                uint16_t num,year,mon,day;

                /* �л�ҳ�� */
                Conference_ScreenPageSwitch(SysState_Page);

                /* �������ߵ�Ԫ�������� */
                num = OnlineNum.wiredChm + OnlineNum.wiredRps;
                exData[0]= (uint8_t)(num >> 8);
                exData[1]= (uint8_t)(num & 0xFF);
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0034),2,exData);

                /* ����������ϯ���������� */
                exData[0]= (uint8_t)(OnlineNum.wiredChm >> 8);
                exData[1]= (uint8_t)(OnlineNum.wiredChm & 0xFF);
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0035),2,exData);

                /* �������ߴ������������  */
                exData[0]= (uint8_t)(OnlineNum.wiredRps >> 8);
                exData[1]= (uint8_t)(OnlineNum.wiredRps & 0xFF);
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0036),2,exData);


                /* �������ߵ�Ԫ�������� */
                num = OnlineNum.wifiChm + OnlineNum.wifiRps;
                exData[0]= (uint8_t)(num >> 8);
                exData[1]= (uint8_t)(num & 0xFF);
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0001),2,exData);

                /* ����������ϯ���������� */
                exData[0]= (uint8_t)(OnlineNum.wifiChm >> 8);
                exData[1]= (uint8_t)(OnlineNum.wifiChm & 0xFF);
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0002),2,exData);

                /* �������ߴ������������ */
                exData[0]= (uint8_t)(OnlineNum.wifiRps >> 8);
                exData[1]= (uint8_t)(OnlineNum.wifiRps & 0xFF);
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0003),2,exData);


                /* ������Ա���������� */
                exData[0]= (uint8_t)(OnlineNum.interpreter >> 8);
                exData[1]= (uint8_t)(OnlineNum.interpreter & 0xFF);
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0037),2,exData);

                APP_GetBuildDate(&year, &mon, &day);

                /* ������ʾ��ǰϵͳ�汾�� */
                sprintf((char *)&exData[0],"%s",APP_VERSION);
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0039),sizeof(APP_VERSION),exData);

                /* ������ʾ����������� */
                sprintf((char *)&exData[0],"%d%d%d",year,mon,day);
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0080),8,exData);
            }
            break;

            /* ��ID���� */
            case 0x04: {
                /* �л�ҳ�� */
                Conference_ScreenPageSwitch(SetID_Page);
                sprintf((char *)&exData[0],"%04d  ",1);
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0022),6,exData);
            }
            break;

            /* �з�Ӣ�汾 */
            case 0x05: {
                /* �л�ҳ�� */
                Conference_ScreenPageSwitch(SystemSet_Page);
            }
            break;

            /*��Ա��*/
            case 0x06: {
                /* �л�ҳ�� */
                Conference_ScreenPageSwitch(StartTranSetID_Page);
            }
            break;
            /* ID ģʽ */
            case 0x07: {
                /* �л�ҳ�� */
                Conference_ScreenPageSwitch(SystemSet_Page);
            }
            break;
            default:
                break;
            }
            break;
        /* �л���Ͳģʽ(0x12)������(0x14) */
        case 0x12:
        case 0x14: {
            uint8_t mode;

            if(reg == 0x12) {
                mode = cmd + 1;
                if(mode >= kMode_Mic_Fifo && mode <= kMode_Mic_Apply)
                    SysInfo.micMode = (MicMode_EN)mode;
            } else if(reg == 0x14) {
                /* �������ߵ�Ԫ */
                if(cfgWitchTypeUnit == tWired && cmd >= 0 && cmd <= 3) {
                    static const uint8_t Num[4] = {1,2,4,8};

                    SysInfo.wiredAllowOpen = Num[cmd];
                    SysInfo.wiredAllowWait = Num[cmd];
                }

                /* ����WIFI��Ԫ */
                else if(cfgWitchTypeUnit == tWifi && cmd >= 0 && cmd <= 3) {
                    static const uint8_t Num[4] = {1,2,4,6};

                    SysInfo.wifiAllowOpen = Num[cmd];
                    SysInfo.wifiAllowWait = Num[cmd];
                }
            }

            /* �ر����е�Ԫ */
            Conference_CloseAllMic(tWired,aChairman);
            Conference_CloseAllMic(tWired,aRepresentative);
            Conference_CloseAllMic(tWifi,aChairman);
            Conference_CloseAllMic(tWifi,aRepresentative);

            /* ���������� */
            Conference_ClearApplyMic(tWired);
            Conference_ClearApplyMic(tWifi);


            /* �㲥ģʽ������ */
            /* ���͵���Ԫ */
            WiredUnit.transmit(Protocol.conference(&confProt,MODE_BROADCAST_ID,STATE_MSG,SysInfo.micMode,SysInfo.wiredAllowOpen,null,null));
            WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,ChangeMicManage_MtoU_G,SysInfo.micMode,SysInfo.wifiAllowOpen));
            /* �ظ��ⲿ�����豸ȷ��״̬ */
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,MODE_BROADCAST_ID,STATE_MSG,SysInfo.micMode,SysInfo.wiredAllowOpen,null,null));
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_MODE_BROADCAST_ID,STATE_MSG,SysInfo.micMode,SysInfo.wifiAllowOpen,null,null));
        }
        break;
        case 0x2B:  //0 1 2 3���߻���ģʽ�У��л���Ͳģʽ
            break;

        case 0x22:  //�Ƿ�������������
            break;
        case 0x24:  //��Ա��ģʽ����
            break;
        case 0x26:  //��Ա����ID
            break;
        case 0x2A:  //������߽�����ID
            break;
        case 0x16:  //����������
            break;
        case 0x18:  //����������
            break;
        case 0x20:  //������������������
            break;
        case 0x3A: { //ϵͳ����
            switch(cmd) {
            /* ����ѡ�� */
            case 0x01: {
                /* �л�ҳ�� */
                Conference_ScreenPageSwitch(Language_Page);
                exData[1] = SysInfo.language;
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0040),2,exData);
            }
            break;

            /* ������IP���ý��棬��ʱ��Ҫ����������ʾ */
            case 0x02: {
                /* �л�ҳ�� */
                Conference_ScreenPageSwitch(IP_Page);

                sprintf((char *)&exData[0],"192.168.1.123");
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0048),15,exData);

                sprintf((char *)&exData[0],"192.168.12.123");
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0058),15,exData);

                sprintf((char *)&exData[0],"192.168.123.123");
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0068),15,exData);

            }
            break;

            /* �ָ��������� */
            case 0x03: {
            } break;

            /* ��ʾ����-���ȵ��� */
            case 0x04: {
                /* �л�ҳ�� */
                Conference_ScreenPageSwitch(Display_Page);
            }
            break;

            /* ��IDģʽ */
//            case 0x05:{
//            	/* �л�ҳ�� */
//				Screen.transmit(Protocol.screen(&screenProt,tType_Screen_Page,ID_MODE_Page));
//			} break;

            /* �������� */
            case 0x06: {
                /* �л�ҳ�� */
                Conference_ScreenPageSwitch(Volume_Page);
            }
            break;

            /* �´����� */
            case 0x07: {
                /* �л�ҳ�� */
                Conference_ScreenPageSwitch(Sender_Sound_SW_Page);
            }
            break;

            default:
                break;
            }
        }
        break;

        /* ������¼�������� */
        case 0x3B:
            switch(cmd) {
            case 0x00://��һ��
                Audio.previous();
                break;

            case 0x01://����
                Audio.playPause();
                break;

            case 0x02://��һ��
                Audio.next();
                break;

            case 0x03://��ʼ/ֹͣ¼��
            	if(UsbAudio.audSta == kStatus_Aud_Recording){
					Audio.stopRec();
					/* �л�����Ƶ¼����ҳ */
//					Conference_ScreenPageSwitch(RECORD_CARD_PAUSE_Page);
					Conference_ScreenPageSwitch(RECORD_CARD_WAIT_RECORD_Page);
				}
				else if(UsbAudio.audSta == kStatus_Aud_Idle || UsbAudio.audSta == kStatus_Aud_Pause){
					static uint16_t testNum = 1719;
					char *name;
					
					Conference_ScreenPageSwitch(RECORD_CARD_WAIT_RECORD_Page);
            		sprintf(UsbAudio.recFile,"%s%d","RE",testNum++);
                	Audio.record(UsbAudio.recFile);

					name = MALLOC(72);
					strcpy(name,UsbAudio.recFile);
                	Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x03A0),72,(uint8_t *)name);
					FREE(name);
				}
                break;

            case 0x04://����
                /* �л�ҳ�� */
                Conference_ScreenPageSwitch(Main_Munu_Page);
                break;

            case 0x05://��ͣ
                Audio.playPause();
                break;

            }
            break;

        case 0x3C:
            switch(cmd) {
            case 0x01:
                break;

            case 0x02:
                break;

            default:
                break;
            }
            break;

        case 0x40: //������������ѡ��
            break;
        case 0x41://���������´���������
            break;
        case 0x44://�����������������Ļ��ǰ����ֵ��
            break;
        case 0x45://��������������µ���ֵ��
            break;
        case 0x50: //��������IDģʽ����
            break;
        case 0x51: //���������±�ID
            break;
        case 0x52:  //�������������±�ID
            break;
        case 0x54://������������������ѡ�񷵻�
            switch(cmd) {
            /* ���� */
            case 0x00: {
                static const uint8_t Reg[7] = {null,0,1,null,2,null,3};

                cfgWitchTypeUnit = tWifi;

                exData[0] = 0;
                exData[1] = 1;
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0005),2,exData);

                /* �л�ҳ�� */
                Conference_ScreenPageSwitch(ModeNum_Page);

                exData[1] = SysInfo.micMode - 1;
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0012),2,exData);

                exData[1] = Reg[SysInfo.wifiAllowOpen];
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0014),2,exData);
            }
            break;

            /* ���� */
            case 0x01: {
                static const uint8_t Reg[9] = {null,0,1,null,2,null,null,null,3};

                cfgWitchTypeUnit = tWired;

                exData[0] = 0;
                exData[1] = 0;
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0005),2,exData);

                /* �л�ҳ�� */
                Conference_ScreenPageSwitch(ModeNum_Page);

                exData[1] = SysInfo.micMode - 1;
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0012),2,exData);

                exData[1] = Reg[SysInfo.wiredAllowOpen];
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0014),2,exData);
            }
            break;
            }
            break;
        case 0x55://������������Ͳ���� 20190819
            switch(cmd) {
            case 0x00://����
                break;
            case 0x01://�½�
                break;
            }
            break;
        default:
            break;
        }
    }
    break;

    default:
        break;

    }

}

/**
* @Name  		Conference_SignInstruction
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_SignInstruction(uint16_t id,UnitType_EN type,uint32_t cmd,uint16_t totalNum,uint16_t curNum)
{
    ConfProtocol_S confProt;
    WifiUnitProtocol_S wifiProt;

    switch(cmd) {
    /* ��ʼǩ���Լ��·�ǩ��������ǩ����ǰ���� */
    case WIRED_CMD(START_SIGN_MODE,0,0):
    case WIFI_CMD(EnterSignMode_MtoU_G,0,0): {
        uint8_t data[4] = {0},len = 0;

        data[0] = (uint8_t)(totalNum >> 8);
        data[1] = (uint8_t)(totalNum & 0xFF);
        data[2] = (uint8_t)(curNum >> 8);
        data[3] = (uint8_t)(curNum & 0xFF);
        len = 4;
        WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,SIGN_MODE,START_SIGN_MODE,null,null));
        WiredUnit.transmit(Protocol.conference(&confProt,RESULT_MSG_ID(RES_SIGN_NUM),RESULT_MSG,data[0],data[1], data[2],data[3]));
        WifiUnit.transWithExData(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,EnterSignMode_MtoU_G,0,null),len,data);
    }
    break;

    /* ��Ԫǩ�� */
    case WIRED_CMD(UNIT_SIGN_IN,0,0):
    case WIFI_CMD(RepPollUnitl_UtoM_D,kMode_Sign,0): {
        if(id != null) {
            id = type == tWired ? id : WIFI_ID(id);
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id,BASIC_MSG,SIGN_MODE,UNIT_SIGN_IN,null,null));
        }
    }
    break;

    /* ���Ƶ�Ԫǩ�� */
    case WIRED_CMD(CONTROL_UNIT_SIGN,0,0):
    case WIFI_CMD(ControlSign_MtoU_D,0,0): {
        if(type == tWired)
            WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,SIGN_MODE,CONTROL_UNIT_SIGN,null,null));
        else if(type == tWifi)
            WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,ControlSign_MtoU_D,null,null));
    }
    break;

    /* ����ǩ�� */
    case WIRED_CMD(SUPPLEMENT_SIGN,0,0):
    case WIFI_CMD(SupplementSign_MtoU_D,0,0): {
        if(type == tWired)
            WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,SIGN_MODE,SUPPLEMENT_SIGN,null,null));
        else if(type == tWifi)
            WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,SupplementSign_MtoU_D,null,null));
    }
    break;

    /* �ظ�����ǩ�� */
    case WIRED_CMD(UNIT_SUPPLEMENT_SIGN,0,0):
    case WIFI_CMD(RepSupplementSign_UtoM_D,0,0): {
        if(id != null) {
            id = type == tWired ? id : WIFI_ID(id);
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id,BASIC_MSG,SIGN_MODE,UNIT_SUPPLEMENT_SIGN,null,null));
        }
    }
    break;
    }

}


/**
* @Name  		Conference_MicCtrlInstruction
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_MicCtrlInstruction(uint16_t id, UnitType_EN type,UnitAttr_EN attr, MicControlType_EN ctrlType,uint8_t channel)
{
    ConfProtocol_S confProt;
    WifiUnitProtocol_S wifiProt;
    uint8_t data[5] = {0},len =0,arg = 0;


    switch(ctrlType) {
    /* ����Ͳ */
    case tOpenMic: {
        arg = (attr == aChairman) ? CHM_OPEN_MIC : RPS_OPEN_MIC;
        if(type == tWired) {
            Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,arg,channel,null);
            WiredUnit.transmit(&confProt);
            ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
        } else if(type == tWifi) {
            data[0] = channel;
            len = 1;
            WifiUnit.transWithExData(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,MasterOpenOrCloseMic_MtoU_D,0,null),len,data);
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),BASIC_MSG,CONFERENCE_MODE,arg,channel,null));
        }

    }
    break;

    /* �ػ�Ͳ */
    case tCloseMic: {
        arg = (attr == aChairman) ? CHM_CLOSE_MIC : RPS_CLOSE_MIC;
        if(type == tWired) {
            Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,arg,null,null);
            WiredUnit.transmit(&confProt);
            ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
        } else if(type == tWifi) {
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),BASIC_MSG,CONFERENCE_MODE,arg,null,null));
            WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,MasterOpenOrCloseMic_MtoU_D,1,null));
        }
    }
    break;

    /* ��Ͳ�ȴ� */
    case tWaiting: {
        if(type == tWired) {
            Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,MIC_WAIT,null,null);
            WiredUnit.transmit(&confProt);
            ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
        } else if(type == tWifi) {
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),BASIC_MSG,CONFERENCE_MODE,MIC_WAIT,null,null));
            WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,RepApplyOpenMic_MtoU_D,3,null));
        }
    }
    break;

    /* ��Ͳȡ���ȴ� */
    case tDisWait: {
        if(type == tWired) {
            Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,MIC_DISWAIT,null,null);
            ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
            WiredUnit.transmit(&confProt);
        } else if(type == tWifi) {
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),BASIC_MSG,CONFERENCE_MODE,MIC_DISWAIT,null,null));
            WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,MasterOpenOrCloseMic_MtoU_D,1,null));
        }
    }
    break;

    /* ��Ͳ���� */
    case tMicFull: {
        if(type == tWired) {
            Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,MIC_CHANNEL_FULL,null,null);
            ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
            WiredUnit.transmit(&confProt);
        } else if(type == tWifi) {
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),BASIC_MSG,CONFERENCE_MODE,MIC_CHANNEL_FULL,null,null));
            WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,RepApplyOpenMic_MtoU_D,1,null));
        }

    }
    break;

    /* ��Ͳ���� */
    case tMicApply: {
        uint16_t applyId = (type == tWifi) ? WIFI_ID(id) : id;

        data[0] = (uint8_t)(applyId >> 8);
        data[1] = (uint8_t)(applyId& 0xFF);
        len = 2;
        Protocol.conference(&confProt,APPLY_OPEN_MIC_ID,APPLY_MSG,data[0],data[1],null,null);
        Protocol.wifiUnit(&wifiProt,0,RepresentApplyOpenMic_MtoU_G,0,null);

        ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
        WiredUnit.transmit(&confProt);
        WifiUnit.transWithExData(kMode_Wifi_Multicast,&wifiProt,len,data);
    }
    break;

    /* �������� */
    case tRevokeApply: {
        uint16_t applyId = (type == tWifi) ? WIFI_ID(id) : id;

        data[0] = (uint8_t)(applyId >> 8);
        data[1] = (uint8_t)(applyId& 0xFF);
        len = 2;
        Protocol.conference(&confProt,REVOKE_APPLY_ID,APPLY_MSG,data[0],data[1],null,null);
        Protocol.wifiUnit(&wifiProt,0,RepresentApplyOpenMic_MtoU_G,1,null);

        ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
        WiredUnit.transmit(&confProt);
        WifiUnit.transWithExData(kMode_Wifi_Multicast,&wifiProt,len,data);

    }
    break;
    }
}


/**
* @Name  		Conference_MicControl
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_MicControl(uint16_t id, UnitType_EN type, uint32_t cmd)
{
    uint8_t channel, chmMicNum, rpsMicNum, waitNum, index,allowOpen,allowWait;
    DataQueueHandler_S chmMicQueue,rpsMicQueue,waitQueue;
    UnitInfo_S *unitInfo;


    ERR_CHECK(!(type == tWired && (id < 0 || id > WIRED_UNIT_MAX_ONLINE_NUM)),return);
//    ERR_CHECK(!(type == tWired && UnitInfo.wired[id].online != true),return);
    ERR_CHECK(!(type == tWifi && (id < 0 || id > WIFI_UNIT_MAX_ONLINE_NUM)),return);
//    ERR_CHECK(!(type == tWifi && UnitInfo.wifi[id].online != true),return);

    if(type == tWired) {
        chmMicQueue = WiredChmMicQueue;
        rpsMicQueue = WiredRpsMicQueue;
        waitQueue = WiredWaitQueue;
        unitInfo = UnitInfo.wired;
        allowOpen = SysInfo.wiredAllowOpen;
        allowWait = SysInfo.wiredAllowWait;
    }

    else if(type == tWifi) {
        chmMicQueue = WifiChmMicQueue;
        rpsMicQueue = WifiRpsMicQueue;
        waitQueue = WifiWaitQueue;
        unitInfo = UnitInfo.wifi;
        allowOpen = SysInfo.wifiAllowOpen;
        allowWait = SysInfo.wifiAllowWait;
    } else
        return;

    /* ��ȡ�����е�ǰ�򿪻�Ͳ���� */
    chmMicNum = DataQueue.getSize(chmMicQueue);
    rpsMicNum = DataQueue.getSize(rpsMicQueue);
    waitNum = DataQueue.getSize(waitQueue);

    switch(cmd) {

    /* ����Ͳ */
    case WIRED_CMD(CHM_OPEN_MIC,0,0):
    case WIRED_CMD(RPS_OPEN_MIC,0,0):
    case WIFI_CMD(ApplyOpenMic_UtoM_D,0,0): {

        /* ��ϯ���뿪��Ͳ */
        if(unitInfo[id].attr == aChairman) {
            switch(SysInfo.micMode) {
            /*** �Ƚ��ȳ�ģʽ ***/
            case kMode_Mic_Fifo:
            /*** ����ģʽ ***/
            case kMode_Mic_Normal:
            /*** ����ģʽ ***/
            case kMode_Mic_VoiceCtrl:
            /*** ����ģʽ ***/
            case kMode_Mic_Apply: {
                /* ���һ�Ͳ�����Ƿ��Ѿ����ڶ�ӦID�Ļ�Ͳ */
                if(DataQueue.search(chmMicQueue,&id)) {
                    channel = unitInfo[id].channel;
                    Conference_MicCtrlInstruction(id,type,aChairman,tOpenMic,channel);
                    break;
                }

                /* ��ϯ+������Ͳ����δ����������Ͳ�� */
                if(chmMicNum + rpsMicNum < allowOpen) {
                    DataQueue.enter(chmMicQueue,&id);
                    channel = Conference_GetAudioChannel(type);
                    unitInfo[id].channel = channel;

                    Conference_MicCtrlInstruction(id,type,aChairman,tOpenMic,channel);
                }
                /* ��ϯ+������Ͳ��������������Ͳ�� */
                else {
                    if(rpsMicNum > 0) {
                        uint16_t closeId;

                        /* ����ȡ������Ԫ���ر� */
                        DataQueue.exit(rpsMicQueue,&closeId);
                        channel = unitInfo[closeId].channel;
                        unitInfo[closeId].channel = null;
                        Conference_MicCtrlInstruction(closeId,type,aRepresentative,tCloseMic,null);

                        /* ����ϯ */
                        unitInfo[id].channel = channel;
                        DataQueue.enter(chmMicQueue,&id);
                        Conference_MicCtrlInstruction(id,type,aChairman,tOpenMic,channel);
                    } else {
                        /* ��Ͳ���� */
                        Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tMicFull,null);
                    }
                }
            }
            break;

            default:
                break;
            }
        }

        else if(unitInfo[id].attr == aRepresentative) {
            switch(SysInfo.micMode) {
            /*** �Ƚ��ȳ�ģʽ ***/
            case kMode_Mic_Fifo: {
                /* ���һ�Ͳ�����Ƿ��Ѿ����ڶ�ӦID�Ļ�Ͳ */
                if(DataQueue.search(rpsMicQueue,&id)) {
                    channel = unitInfo[id].channel;
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tOpenMic,channel);
                    break;
                }

                /* ��ϯ+������Ͳ����δ����������Ͳ�� */
                if(chmMicNum + rpsMicNum < allowOpen) {
                    channel = Conference_GetAudioChannel(type);
                    unitInfo[id].channel = channel;
                    DataQueue.enter(rpsMicQueue,&id);
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tOpenMic,channel);
                }
                /* ��ϯ+������Ͳ��������������Ͳ�� */
                else {
                    if(rpsMicNum > 0) {
                        uint16_t closeId;

                        /* ����ȡ������Ԫ���ر� */
                        DataQueue.exit(rpsMicQueue,&closeId);
                        channel = unitInfo[closeId].channel;
                        unitInfo[closeId].channel = null;
                        Conference_MicCtrlInstruction(closeId,type,aRepresentative,tCloseMic,null);
                        /* �򿪴��� */
                        unitInfo[id].channel = channel;
                        DataQueue.enter(rpsMicQueue,&id);
                        Conference_MicCtrlInstruction(id,type,aRepresentative,tOpenMic,channel);
                    } else {
                        /* ��Ͳ���� */
                        Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tMicFull,null);
                    }
                }
            }
            break;
            /*** ����ģʽ ***/
            case kMode_Mic_Normal:
            /*** ����ģʽ ***/
            case kMode_Mic_VoiceCtrl: {
                /* ���һ�Ͳ�����Ƿ��Ѿ����ڶ�ӦID�Ļ�Ͳ */
                if(DataQueue.search(rpsMicQueue,&id)) {
                    /* ���·��ʹ򿪻�Ͳָ�����Ҫ�������� */
                    channel = unitInfo[id].channel;
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tOpenMic,channel);
                    break;
                }

                /* ���ҵȴ������Ƿ���ڶ�ӦID��Ͳ */
                if(waitNum > 0 ) {
                    /* ȡ���û�Ͳ�ȴ� */
                    index = DataQueue.search(waitQueue,&id);
                    if(index != 0) {
                        DataQueue.deleted(waitQueue,index);
                        Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tDisWait,null);
                    }
                    break;
                }

                /* ��ϯ+������Ͳ����δ����������Ͳ�� */
                if(chmMicNum + rpsMicNum < allowOpen) {
                    channel = Conference_GetAudioChannel(type);
                    unitInfo[id].channel = channel;
                    DataQueue.enter(rpsMicQueue,&id);
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tOpenMic,channel);
                }
                /* ��ϯ+������Ͳ��������������Ͳ�� */
                else {
                    /* �ȴ�����δ�� */
                    if(waitNum < allowWait) {
                        /* ����ȴ����� */
                        DataQueue.enter(waitQueue,&id);
                        Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tWaiting,null);
                    } else {
                        /* ��Ͳ���� */
//                        EXE_MIC_FULL(id,kType_NotiSrc_PC | kType_NotiSrc_WiredUnit);
                        Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tMicFull,null);
                    }
                }
            }
            break;
            /*** ����ģʽ ***/
            case kMode_Mic_Apply: {
                /* ���һ�Ͳ�����Ƿ��Ѿ����ڶ�ӦID�Ļ�Ͳ */
                if(DataQueue.search(rpsMicQueue,&id)) {
                    channel = unitInfo[id].channel;
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tOpenMic,channel);
                    break;
                }

                /* ���ҵȴ������Ƿ���ڶ�ӦID��Ͳ */
                if(waitNum > 0 ) {
                    index = DataQueue.search(waitQueue,&id);
                    if(index != 0) {
                        Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tWaiting,null);
                        break;
                    }
                }

                /* ��ϯ+������Ͳ����δ����������Ͳ�� */
                if(chmMicNum + rpsMicNum < allowOpen && waitNum < allowWait) {
                    /* ����ȴ����� */
                    DataQueue.enter(waitQueue,&id);
                    Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tWaiting,null);
                    Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tMicApply,null);
                }
                /* ��ϯ+������Ͳ��������������Ͳ�� */
                else {
                    /* ��Ͳ���� */
                    Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tMicFull,null);
                }
            }
            break;
            default:
                break;
            }
        }
    }
    break;

    /* �ػ�Ͳ */
    case WIRED_CMD(CHM_CLOSE_MIC,0,0):
    case WIRED_CMD(RPS_CLOSE_MIC,0,0):
    case WIFI_CMD(UnitCloseMic_UtoM_D,0,0): {
        switch(SysInfo.micMode) {
        /*** �Ƚ��ȳ�ģʽ ***/
        case kMode_Mic_Fifo: {
            /* ��ϯ��Ԫ */
            if(unitInfo[id].attr == aChairman) {
                /* ���һ�Ͳ�����Ƿ��Ѿ����ڶ�ӦID�Ļ�Ͳ */
                index = DataQueue.search(chmMicQueue,&id);
                if(index) {
                    Conference_GiveAudioChannel(type,unitInfo[id].channel);
                    DataQueue.deleted(chmMicQueue,index);
                    Conference_MicCtrlInstruction(id,type,aChairman,tCloseMic,null);
                }
            }

            /* ����Ԫ */
            else if(unitInfo[id].attr == aRepresentative) {
                /* ���һ�Ͳ�����Ƿ��Ѿ����ڶ�ӦID�Ļ�Ͳ */
                index = DataQueue.search(rpsMicQueue,&id);
                if(index) {
                    Conference_GiveAudioChannel(type,unitInfo[id].channel);
                    unitInfo[id].channel = null;
                    DataQueue.deleted(rpsMicQueue,index);
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tCloseMic,null);
                }
            }
        }
        break;

        /*** ����ģʽ ***/
        case kMode_Mic_Normal:
        /*** ����ģʽ ***/
        case kMode_Mic_VoiceCtrl: {
            /* ��ϯ��Ԫ */
            if(unitInfo[id].attr == aChairman) {
                /* ���һ�Ͳ�����Ƿ��Ѿ����ڶ�ӦID�Ļ�Ͳ */
                index = DataQueue.search(chmMicQueue,&id);
                if(index) {
                    Conference_GiveAudioChannel(type,unitInfo[id].channel);
                    DataQueue.deleted(chmMicQueue,index);
                    Conference_MicCtrlInstruction(id,type,aChairman,tCloseMic,null);
                } else
                    break;

                if(waitNum > 0) {
                    /* �ȴ�����ȡ����Ԫ���� */
                    uint16_t waitId;
                    DataQueue.exit(waitQueue,&waitId);
                    Conference_MicControl(waitId,type,WIRED_CMD(RPS_OPEN_MIC,0,0));
                }
            }

            /* ����Ԫ */
            else if(unitInfo[id].attr == aRepresentative) {
                /* ���һ�Ͳ�����Ƿ��Ѿ����ڶ�ӦID�Ļ�Ͳ */
                index = DataQueue.search(rpsMicQueue,&id);
                if(index) {
                    Conference_GiveAudioChannel(type,unitInfo[id].channel);
                    unitInfo[id].channel = null;
                    DataQueue.deleted(rpsMicQueue,index);
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tCloseMic,null);
                }

                if(waitNum > 0) {
                    /* �ȴ�����ȡ����Ԫ���� */
                    uint16_t waitId;
                    DataQueue.exit(waitQueue,&waitId);
                    Conference_MicControl(waitId,type,WIRED_CMD(RPS_OPEN_MIC,0,0));
                }
            }
//            break;
        }
        break;

        /*** ����ģʽ ***/
        case kMode_Mic_Apply: {
            /* ��ϯ��Ԫ */
            if(unitInfo[id].attr == aChairman) {
                /* ���һ�Ͳ�����Ƿ��Ѿ����ڶ�ӦID�Ļ�Ͳ */
                index = DataQueue.search(chmMicQueue,&id);
                if(index) {
                    Conference_GiveAudioChannel(type,unitInfo[id].channel);
                    DataQueue.deleted(chmMicQueue,index);
                    Conference_MicCtrlInstruction(id,type,aChairman,tCloseMic,null);
                }
            }

            /* ����Ԫ */
            else if(unitInfo[id].attr == aRepresentative) {
                /* ���һ�Ͳ�����Ƿ��Ѿ����ڶ�ӦID�Ļ�Ͳ */
                index = DataQueue.search(rpsMicQueue,&id);
                if(index) {
                    Conference_GiveAudioChannel(type,unitInfo[id].channel);
                    unitInfo[id].channel = null;
                    DataQueue.deleted(rpsMicQueue,index);
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tCloseMic,null);
                }
            }
        }
        break;
        }
    }
    break;

    /* ��ϯͬ�⿪��Ͳ */
    case WIRED_CMD(AGREE_OPEN_MIC,0,0):
    case WIFI_CMD(ChairAgreeOpenOrNot_UtoM_D,0,0): {
        uint8_t waitId;

        /* ���ⲿ����ͬ�⣬��ID */
        if(id != 0) {
            /* �򿪻�Ͳ����ɾ���ȴ������ж�ӦID�Ļ�Ͳ */
            index = DataQueue.search(waitQueue,&id);
            if(index != 0) {
                DataQueue.deleted(waitQueue,index);

                DataQueue.enter(rpsMicQueue,&id);
                channel = Conference_GetAudioChannel(type);
                unitInfo[id].channel = channel;
                Conference_MicCtrlInstruction(id,type,aRepresentative,tOpenMic,channel);
            }

        }

        /* ��ϯ��ͬ�� */
        /* @brief ����ϯ��ͬ��/�ܾ�����Ͳ�����·�ͬ���ID��ֻ���·�һ��ͬ��/�ܾ�ָ�
                    ������Ҫ�ڶ���������ң����ն���˳��ͬ��/�ܾ���Ͳ�򿪣��ҵ���������
                    ��Ҫ�����ߺ�WIFI������ϵͳ�߼��ϲ��������ͬ��/�ܾ�����Ͳ������Ҫ
                    ����������Ķ���   */
        else {

            if(DataQueue.getSize(WiredWaitQueue) > 0) {
                /* �Ȳ������ߵ�Ԫ�ȴ����� */
                if(DataQueue.exit(WiredWaitQueue,&waitId) && \
                   (DataQueue.getSize(WiredRpsMicQueue) + DataQueue.getSize(WiredChmMicQueue) < SysInfo.wiredAllowOpen)) {
                    DataQueue.enter(WiredRpsMicQueue,&waitId);
                    channel = Conference_GetAudioChannel(tWired);
                    UnitInfo.wired[waitId].channel = channel;
                    Conference_MicCtrlInstruction(waitId,tWired,aRepresentative,tOpenMic,channel);
                }

            }

            else if(DataQueue.getSize(WifiWaitQueue) > 0 ) {
                /* �ٲ���WIFI��Ԫ�ȴ����� */
                if(DataQueue.exit(WifiWaitQueue,&waitId) && \
                   (DataQueue.getSize(WifiRpsMicQueue) + DataQueue.getSize(WifiChmMicQueue) < SysInfo.wifiAllowOpen)) {
                    DataQueue.enter(WifiRpsMicQueue,&waitId);
                    channel = Conference_GetAudioChannel(tWifi);
                    UnitInfo.wifi[waitId].channel = channel;
                    Conference_MicCtrlInstruction(waitId,tWifi,aRepresentative,tOpenMic,channel);
                }
            }
        }

        if(DataQueue.getSize(WifiWaitQueue) == 0 && DataQueue.getSize(WiredWaitQueue) == 0) {
            Conference_MicCtrlInstruction(null,(UnitType_EN)null,(UnitAttr_EN)null,tRevokeApply,null);
        }
    }
    break;

    /* ��ϯ��ͬ�⿪��Ͳ */
    case WIRED_CMD(DISAGREE_OPEN_MIC,0,0):
    case WIFI_CMD(ChairAgreeOpenOrNot_UtoM_D,1,0): {
        /* ���ⲿ���ƾܾ�����ID */
        if(id != 0) {
            /* �򿪻�Ͳ����ɾ���ȴ������ж�ӦID�Ļ�Ͳ */
            index = DataQueue.search(waitQueue,&id);
            if(index != 0) {
                DataQueue.deleted(waitQueue,index);
                Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tDisWait,null);
            }
        }

        /* ��ϯ���ܾ� */
        else {
            uint8_t waitId;

            if(DataQueue.getSize(WiredWaitQueue) > 0) {
                /* �Ȳ������ߵ�Ԫ�ȴ����� */
                if(DataQueue.exit(WiredWaitQueue,&waitId)) {
                    Conference_MicCtrlInstruction(waitId,tWired,(UnitAttr_EN)null,tDisWait,null);
                }

            }

            else if(DataQueue.getSize(WifiWaitQueue) > 0 ) {
                /* �ٲ���WIFI��Ԫ�ȴ����� */
                if(DataQueue.exit(WifiWaitQueue,&waitId)) {
                    Conference_MicCtrlInstruction(waitId,tWifi,(UnitAttr_EN)null,tDisWait,null);
                }
            }
        }


        if(DataQueue.getSize(WifiWaitQueue) == 0 && DataQueue.getSize(WiredWaitQueue) == 0) {
            Conference_MicCtrlInstruction(null,(UnitType_EN)null,(UnitAttr_EN)null,tRevokeApply,null);
        }
    }
    break;

    /* ��Ԫȡ���ȴ� */
    case WIRED_CMD(MIC_DISWAIT,0,0):
    case WIFI_CMD(UnitGetoutWaitQuenen_UtoM_D,0,0): {
        switch(SysInfo.micMode) {
        /*** ����ģʽ ***/
        case kMode_Mic_Normal:
        /*** ����ģʽ ***/
        case kMode_Mic_VoiceCtrl: {
            if(waitNum > 0 && unitInfo[id].attr == aRepresentative) {
                /* ���ҵȴ������Ƿ��Ѿ����ڶ�ӦID�Ļ�Ͳ */
                index = DataQueue.search(waitQueue,&id);
                if(index) {
                    DataQueue.deleted(waitQueue,index);
                    Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tDisWait,null);
                }
            }
        }
        break;
        /*** ����ģʽ ***/
        case kMode_Mic_Apply: {
            if(waitNum > 0 && unitInfo[id].attr == aRepresentative) {
                /* ���ҵȴ������Ƿ��Ѿ����ڶ�ӦID�Ļ�Ͳ */
                index = DataQueue.search(waitQueue,&id);
                if(index) {
                    DataQueue.deleted(waitQueue,index);
                    Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tDisWait,null);
                }
            }

            if(DataQueue.getSize(WifiWaitQueue) == 0 && DataQueue.getSize(WiredWaitQueue) == 0) {
                Conference_MicCtrlInstruction(null,(UnitType_EN)null,(UnitAttr_EN)null,tRevokeApply,null);
            }
        }
        break;
        default:
            break;

        }
    }
    break;
    }


    Conference_OpenMicListDebug();
}



/**
* @Name  		Conference_CloseAllMic
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_CloseAllMic(UnitType_EN type,UnitAttr_EN attr)
{
    DataQueueHandler_S *micQueue;
    uint8_t micNum,i;
    uint16_t *idArr;
    UnitInfo_S *unitInfo;

    if(type == tWired) {
        unitInfo = UnitInfo.wired;

        if(attr == aChairman)
            micQueue = WiredChmMicQueue;
        else if(attr == aRepresentative)
            micQueue = WiredRpsMicQueue;
        else
            return;
    } else if(type == tWifi) {
        unitInfo = UnitInfo.wifi;

        if(attr == aChairman)
            micQueue = WifiChmMicQueue;
        else if(attr == aRepresentative)
            micQueue = WifiRpsMicQueue;
        else
            return;
    } else
        return;

    micNum = DataQueue.getSize(micQueue);
    if(micNum > 0) {
        idArr = MALLOC(micNum * sizeof(uint16_t));
        DataQueue.toArray(micQueue,idArr);
        for(i = 0; i<micNum; i++) {
            if(attr == aChairman)
                Conference_MicCtrlInstruction(idArr[i], type, aChairman, tCloseMic, null);
            else
                Conference_MicCtrlInstruction(idArr[i], type, aRepresentative, tCloseMic, null);

            Conference_GiveAudioChannel(type,unitInfo[idArr[i]].channel);
            unitInfo[idArr[i]].channel = null;
        }
        DataQueue.empty(micQueue);
        FREE(idArr);
    }
}


/**
* @Name  		Conference_ClearApplyMic
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_ClearApplyMic(UnitType_EN type)
{
    uint8_t waitNum,i;
    uint16_t *idArr;
    DataQueueHandler_S *micQueue;
//	UnitInfo_S *unitInfo;

    if(type == tWired) {
//		unitInfo = UnitInfo.wired;
        micQueue = WiredWaitQueue;
    } else if(type == tWifi) {
//		unitInfo = UnitInfo.wifi;
        micQueue = WifiWaitQueue;
    } else
        return;

    waitNum = DataQueue.getSize(micQueue);

    if(waitNum > 0) {
        idArr = MALLOC(waitNum * sizeof(uint16_t));
        DataQueue.toArray(micQueue,idArr);
        for(i = 0; i<waitNum; i++) {
            Conference_MicCtrlInstruction(idArr[i], type, (UnitAttr_EN)null, tDisWait, null);
            if(SysInfo.micMode == kMode_Mic_Apply)
                Conference_MicCtrlInstruction(idArr[i], type, (UnitAttr_EN)null, tRevokeApply, null);
        }
        DataQueue.empty(micQueue);
        FREE(idArr);
    }
}



/**
* @Name  		Conference_OfflineAllUnit
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_OfflineAllUnit(void)
{
    Conference_CloseAllMic(tWired,aChairman);
    Conference_CloseAllMic(tWired,aRepresentative);
    Conference_CloseAllMic(tWifi,aChairman);
    Conference_CloseAllMic(tWifi,aRepresentative);
    Conference_ClearApplyMic(tWired);
    Conference_ClearApplyMic(tWifi);

    memset(&OnlineNum,0,sizeof(UnitOnlineNum));
    memset(&UnitInfo,0,sizeof(UnitInfo));

    /* ������Ƶͨ������ */
    Conference_ResetAudioChannel(tWired);
    Conference_ResetAudioChannel(tWifi);

}





/**
* @Name  		Conference_ChangeSysMode
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_ChangeSysMode(SysMode_EN mode)
{
    ConfProtocol_S confProt;
    WifiUnitProtocol_S wifiProt;

    /* ��鵱ǰģʽ��Ŀ��ģʽ�������ǰ�ǡ�����ģʽ���£�Ŀ��ģʽ������л���
    	�����ǰ�ǡ��ǻ���ģʽ���£�����˻���ģʽ���ģʽ�������л�
    (���仰˵�ڷǻ���ģʽ[EditID,Sign,Vote]�£������Ƚ�����ģʽ���л���
    ����ģʽ[conference]���ſ������л��ɱ�ķǻ���ģʽ) */
    ERR_CHECK(!(SysInfo.sysMode != kMode_Conference && mode != kMode_Conference), return);

    switch(mode) {
    case kMode_Conference: {
        Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,CONFERENCE_MODE,CONFERENCE_MODE,null,null);
        Protocol.wifiUnit(&wifiProt,0,EnterMeetingMode_MtoU_G,SysInfo.sysMode,SysInfo.wifiAllowOpen);

        ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
        WiredUnit.transmit(&confProt);
        WifiUnit.transmit(kMode_Wifi_Multicast,&wifiProt);

        SysInfo.sysMode = kMode_Conference;
    }
    break;

    case kMode_EditID: {
        Conference_OfflineAllUnit();

        Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,EDIT_ID_MODE,START_EDIT_ID_MODE,null,null);
        Protocol.wifiUnit(&wifiProt,0,EnterEditingIDMtoU_G,(SysInfo.wifiCurEditID >> 8),(SysInfo.wifiCurEditID & 0xFF));

        ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
        WiredUnit.transmit(&confProt);
        WifiUnit.transmit(kMode_Wifi_Multicast,&wifiProt);

        SysInfo.sysMode = kMode_EditID;

    }
    break;

    case kMode_Sign: {
        uint16_t id;

        for(id = 0; id < WIRED_UNIT_MAX_ONLINE_NUM; id++)
            UnitInfo.wired[id].sign = false;

        for(id = 0; id < WIFI_UNIT_MAX_ONLINE_NUM; id++)
            UnitInfo.wifi[id].sign = false;


        Conference_SignInstruction(null,(UnitType_EN)null,WIRED_CMD(START_SIGN_MODE,0,0),SysInfo.totalSignNum,SysInfo.currentSignNum);

        SysInfo.sysMode = kMode_Sign;
    }
    break;

    case kMode_Vote: {
        uint16_t id;

        for(id = 0; id < WIRED_UNIT_MAX_ONLINE_NUM; id++)
            UnitInfo.wired[id].vote = null;

        for(id = 0; id < WIFI_UNIT_MAX_ONLINE_NUM; id++)
            UnitInfo.wifi[id].vote = null;

        Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,VOTE_MODE,VoteModeCode[SysInfo.voteMode],null,null);
        Protocol.wifiUnit(&wifiProt,0,EnterVoteMode_MtoU_G,0,SysInfo.voteMode);

        ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
        WiredUnit.transmit(&confProt);
        WifiUnit.transmit(kMode_Wifi_Multicast,&wifiProt);

        SysInfo.sysMode = kMode_Vote;
    }
    break;

    default:
        break;
    }
}


/**
* @Name  		Conference_ChairmanPriority
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_ChairmanPriority(UnitType_EN type,uint16_t id)
{

    ERR_CHECK(!(type == tWired && (id <= 0 || id > WIRED_UNIT_MAX_ONLINE_NUM || UnitInfo.wired[id].attr != aChairman)),return);
    ERR_CHECK(!(type == tWifi && (id <= 0 || id > WIFI_UNIT_MAX_ONLINE_NUM || UnitInfo.wifi[id].attr != aChairman)),return);

    /* �ر����д���Ԫ */
    Conference_CloseAllMic(tWired,aRepresentative);
    Conference_CloseAllMic(tWifi,aRepresentative);

    /* ȡ�����л�Ͳ���� */
    Conference_ClearApplyMic(tWired);
    Conference_ClearApplyMic(tWifi);

    /* ����ϯ��Ԫ */
    Conference_MicControl(id,type,type == tWired ? WIRED_CMD(CHM_OPEN_MIC,0,0) : WIFI_CMD(ApplyOpenMic_UtoM_D,0,0));
}



/**
* @Name  		Conference_ResetAudioChannel
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_ResetAudioChannel(UnitType_EN type)
{
    uint8_t ch;

    if(type == tWired) {
        DataQueue.empty(WiredChannelQueue);
        for(ch = 1; ch <= WIRED_UNIT_MAX_ALLWO_OPEN; ch++) {
            DataQueue.enter(WiredChannelQueue,&ch);
        }
    } else if(type == tWifi) {
        DataQueue.empty(WifiChannelQueue);
        for(ch = 1; ch <= WIFI_UNIT_MAX_ALLWO_OPEN; ch++) {
            DataQueue.enter(WifiChannelQueue,&ch);
        }
    }
}

/**
* @Name  		Conference_GetAudioChannel
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static uint8_t Conference_GetAudioChannel(UnitType_EN type)
{
    uint8_t ch;

    if(type == tWired)
        DataQueue.exit(WiredChannelQueue,&ch);
    else if(type == tWifi)
        DataQueue.exit(WifiChannelQueue,&ch);

    ch += 0x80;
    return ch;
}

/**
* @Name  		Conference_GiveAudioChannel
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_GiveAudioChannel(UnitType_EN type,uint8_t ch)
{
    if(ch < 0x81 || ch > WIRED_UNIT_MAX_ALLWO_OPEN + 0x80)
        return;

    ch -= 0x80;

    if(type == tWired)
        DataQueue.enter(WiredChannelQueue,&ch);
    else if(type == tWifi)
        DataQueue.enter(WifiChannelQueue,&ch);
}




/**
* @Name  		Conference_UnitOnlineNum
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_UnitOnlineNum(UnitOnlineNum *onlineNum)
{
    ERR_CHECK(onlineNum != null,return);
    memcpy(onlineNum,&OnlineNum,sizeof(UnitOnlineNum));
}

/**
* @Name  		Conference_GetSysMode
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static SysMode_EN Conference_GetSysMode(void)
{
    return SysInfo.sysMode;
}

/**
* @Name  		Conference_GetConfSysInfo
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static ConfSysInfo_S Conference_GetConfSysInfo(void)
{
    return SysInfo;
}


/**
* @Name  		Conference_GetCurrentEditedID
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static uint16_t Conference_GetCurrentEditedID(void)
{
    return SysInfo.wiredCurEditID;
}

/**
* @Name  		Conference_WiredUnitInfo
* @Author  		KT
* @Description	�Ե�Ԫ���������·���Ԫ��Ϣ�ṹָ��
*
*
* @para
*
* @return
*/
static UnitInfo_S *Conference_WiredUnitInfo(void)
{
    return UnitInfo.wired;
}

/**
* @Name  		Conference_WiredUnitInfo
* @Author  		KT
* @Description	�Ե�Ԫ���������·���Ԫ��Ϣ�ṹָ��
*
*
* @para
*
* @return
*/
static UnitInfo_S *Conference_WifiUnitInfo(void)
{
    return UnitInfo.wifi;
}


/**
* @Name  		Conference_GetMusicFile
* @Author  		KT
* @Description
* @para
*
*
* @return
*/

static void Conference_GetMusicFile(const char *path)
{
    FILINFO *filinfo;
    uint8_t fileNum, i;
    char *suffix = null;

    UsbAudio.musicFileNum = 0;

    fileNum = UsbDisk.getFileList(path,&filinfo);

    for(i = 0; i < fileNum; i++) {
        suffix = strrchr(filinfo[i].fname, '.');
        if(suffix != null) {
            if (strcmp(suffix, ".mp3") == 0) {
                debug("%s\r\n",filinfo[i].fname);
                UsbAudio.musicFile[UsbAudio.musicFileNum++] = &filinfo[i];
            } else if (strcmp(suffix, ".wav") == 0) {
                debug("%s\r\n",filinfo[i].fname);
                UsbAudio.musicFile[UsbAudio.musicFileNum++] = &filinfo[i];
            }
            if(UsbAudio.musicFileNum >= MUSIC_FILE_LIST_MAX_LEN)
                break;
        }
        suffix = null;
    }
}

/**
* @Name  		Conference_UsbStateListener
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_UsbStateListener(status_t sta)
{


    switch(sta) {
    case kStatus_DEV_Attached: {
        debug("Usb disk is attached!!\r\n");
    }
    break;
    case kStatus_DEV_Detached: {
        UsbAudio.mcuConnectUsb = false;
		
//		/* ��MCU��USB�Ͽ�ʱ���жϵ�ǰ��Ƶ¼�Ź��ܵ�״̬������ǿ��У�����ҳ��ͣ����¼�Ź������棬���л�ҳ�� */
//		if(UsbAudio.audSta == kStatus_Aud_Idle && (ScreenCurrentPage == RECORD_CARD_PAUSE_Page || ScreenCurrentPage == RECORD_CARD_PLAY_Page ||   \
//			ScreenCurrentPage == RECORD_CARD_RECOEDING_Page || 	ScreenCurrentPage ==RECORD_CARD_WAIT_RECORD_Page)){
//			Conference_ScreenPageSwitch(RECORD_CARD_INITING_Page);
//		}
        debug("Usb disk is detached!!\r\n");
    }
    break;
    case kStatus_FS_Mounted: {
//			uint8_t i;
        UsbAudio.mcuConnectUsb = true;

        debug("Usb file system is mounted!!\r\n");
        debug("Usb disk free size : %dMB\r\n",(UsbDisk.freeSize()/1024));

        Conference_GetMusicFile(USBDISK_ROOT);

//			debug("Music file num = %d \r\n",UsbAudHandler.num);
//			for(i = 0;i < UsbAudHandler.num;i++){
//				debug("%s \r\n",UsbAudHandler.fMusic[i]->fname);
//			}

    }
    break;
    case kStatus_DEV_Idle: {
        debug("Usb idle!!\r\n");
    }
    break;
    }
}


/**
* @Name  		Conference_AudioStateListener
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_AudioStateListener(AudState_S *sta)
{
    ScreenProtocol_S screenProt;
	uint8_t exdata[5];
	char *name;

    switch(sta->state) {
    case kStatus_Aud_Playing: {
		if(UsbAudio.audSta != sta->state) {
            UsbAudio.audSta = sta->state;
			Conference_ScreenPageSwitch(RECORD_CARD_PLAY_Page);
            debug("Music playing .. \r\n");
        }
		
        if(UsbAudio.playIndex != sta->playIndex) {
            UsbAudio.playIndex = sta->playIndex;
            if(UsbAudio.playIndex > 0 && UsbAudio.playIndex <= UsbAudio.musicFileNum) {
				name = MALLOC(72);
				strcpy(name,UsbAudio.musicFile[UsbAudio.playIndex - 1]->fname);
                debug("Music name : \'%s\'  ... \r\n",name);
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x03A0),72,(uint8_t *)name);
				FREE(name);
            }
        }
    }
    break;
    case kStatus_Aud_Pause: {
        if(UsbAudio.audSta != sta->state) {
            UsbAudio.audSta = sta->state;
			Conference_ScreenPageSwitch(RECORD_CARD_PAUSE_Page);
            debug("Music pause .. \r\n");
        }
        
    }
    break;
    case kStatus_Aud_Recording: {
		char time[10] = {0};
	
		if(UsbAudio.audSta != sta->state) {
            UsbAudio.audSta = sta->state;
        	Conference_ScreenPageSwitch(RECORD_CARD_RECOEDING_Page);
			/* ��ʾֹͣ¼���İ�ť */
			exdata[1] = 0x01;
			Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0505),2,exdata);
        }
//		debug("Audio recording sec = %d \r\n",sta->runTime);
		sprintf(time,"%02d:%02d:%02d",sta->runTime / 3600,sta->runTime / 60,sta->runTime % 60);
        Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x06A0),9,(uint8_t *)time);
    }
    break;

	case kStatus_Aud_RecStoped:{
		/* ����¼���İ�ť */
		exdata[1] = 0x00;
		Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0505),2,exdata);
		UsbAudio.audSta = kStatus_Aud_Idle;
		Conference_ScreenPageSwitch(RECORD_CARD_PAUSE_Page);
	}
	break;
    case kStatus_Aud_UsbBroken: 
	case kStatus_Aud_StarPlayErr:
	case kStatus_Aud_StarRecErr:
	case kStatus_Aud_UsbTimeout:{
        Conference_ScreenPageSwitch(RECORD_CARD_INITING_Page);
		memset(&UsbAudio,0,sizeof(UsbAudio));
		UsbAudio.audSta = kStatus_Aud_Idle;
    }
    break;

    default:
        break;
    }
}




/**
* @Name  		Conference_OpenMicListDebug
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_OpenMicListDebug(void)
{
    uint32_t i;
    uint16_t *idArr;
    uint8_t len;


    len = DataQueue.getSize(WiredChmMicQueue);
    idArr = MALLOC(len * sizeof(uint16_t));
    DataQueue.toArray(WiredChmMicQueue,idArr);

    debug("======================== Wired ===========================\r\n");
    debug("==      Attr      ==      ID      ==      Channel       ==\r\n");
    for(i=0; i<len; i++)
        debug("==    Chairman    ==      %02d      ==        %02X          ==\r\n",idArr[i],UnitInfo.wired[idArr[i]].channel);
    FREE(idArr);

    len = DataQueue.getSize(WiredRpsMicQueue);
    idArr = MALLOC(len * sizeof(uint16_t));
    DataQueue.toArray(WiredRpsMicQueue,idArr);

    for(i=0; i<len; i++)
        debug("== Representative ==      %02d      ==        %02X          ==\r\n",idArr[i],UnitInfo.wired[idArr[i]].channel);
    debug("==========================================================\r\n");
    FREE(idArr);

    len = DataQueue.getSize(WifiChmMicQueue);
    idArr = MALLOC(len * sizeof(uint16_t));
    DataQueue.toArray(WifiChmMicQueue,idArr);

    debug("======================== Wifi ============================\r\n");
    debug("==      Attr      ==      ID      ==      Channel       ==\r\n");
    for(i=0; i<len; i++)
        debug("==    Chairman    ==      %02d      ==        %02X          ==\r\n",idArr[i],UnitInfo.wifi[idArr[i]].channel);
    FREE(idArr);

    len = DataQueue.getSize(WifiRpsMicQueue);
    idArr = MALLOC(len * sizeof(uint16_t));
    DataQueue.toArray(WifiRpsMicQueue,idArr);

    for(i=0; i<len; i++)
        debug("== Representative ==      %02d      ==        %02X          ==\r\n",idArr[i],UnitInfo.wifi[idArr[i]].channel);
    debug("==========================================================\r\n");
    FREE(idArr);
}

