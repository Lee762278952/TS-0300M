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
#include "global_config.h"
#include "data_queue.h"
#include "ram.h"
#include "usb_disk.h"
#include "audio.h"
#include "time.h"

/* APP */
#include "conference.h"
#include "wired_unit.h"
#include "wifi_unit.h"
#include "external_ctrl.h"
#include "screen.h"
#include "camera.h"
#include "slave_mcu.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* �����ջ��С�����ȼ� */
#define CONFERENCE_TASK_STACK_SIZE						(2048)
#define CONFERENCE_TASK_PRIORITY						(configMAX_PRIORITIES - 1)

#define NOTICE_QUEUE_LENGTH								(128)
#define NOTICE_QUEUE_SIZE								(sizeof(void *))

/* WIFI��ԪIDת��   ��WIFI��ԪID��0x3000��ʼ */
#define WIFI_ID(_id)									(_id + WIFI_UNIT_START_ID)

/* ��Ԫָ��(type �� cmd)�����(ph,pl)�ϲ���չ��32λ�������ϱ�־λ��������ͬʱ�����������͵ĵ�Ԫ��ָ��ʱ���ֻ���������ظ�
	��һ���ֽ�Ϊ�豸���ͱ�ʾ �ڶ����ֽ�Ϊָ��(type��cmd) �������ĸ��ֽ��ǲ���(ph,pl)*/
#define WIRED_CMD(_type,_ph,_pl)						( 0x80000000 | (_type << 16) | (_ph << 8) | _pl)
#define WIFI_CMD(_cmd,_ph,_pl)							( 0x40000000 | (_cmd << 16) | (_ph << 8) | _pl)

/* ��Ԫ�������� */
#define WIRED_ONLINE_NUM								(OnlineNum.wiredChm + OnlineNum.wiredRps)
#define WIFI_ONLINE_NUM									(OnlineNum.wifiChm + OnlineNum.wifiRps)
#define UNIT_ONLINE_NUM 								(WIRED_ONLINE_NUM + WIFI_ONLINE_NUM)


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

/* �ж���Ƶ�ļ����Ƿ�Ϊ�ض�����¼���ļ� */
#define IS_SPECIFIC_AUD_REC_FILE(_fname)				(strlen(_fname) == 8 && _fname[0] == 'R' && (_fname[1] >= '0' && _fname[1] <= '9') \
														 && (_fname[2] >= '0' && _fname[1] <= '9') && (_fname[3] >= '0' && _fname[1] <= '9'))

/* ���ض�����¼���ļ�������ȡ���� */
#define GET_SPECIFIC_AUD_REC_INDEX(_fname)				((_fname[1] - '0') * 100 + (_fname[2] - '0') * 10 + (_fname[3] - '0'))


/* �����б���󳤶� */
#define MUSIC_FILE_LIST_MAX_LEN						(64)

/* ��ʱ�����ʱ�����м��(ms) */
#define TIMING_TASK_INTERVAL						(1000)

/* ʱ����������ü��(S) */
#define TIME_COUNT_RESET_INTERVAL					(36000)
/* ��ʱ�·�Real Time Clock���(S) */
#define BROADCAST_RTC_INTERVAL						(900)

/* ID�ظ��ȴ�ʱ��(S) */
#define ID_DUPICATE_WAIT_TIME						(3)


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
/* LAUNCHER */
static void Conference_ConfigTask(void *pvParameters);
static void Conference_NoticeProcessTask(void *pvParameters);

/* API */
static void Conference_Notify(Notify_S *notify);
static void Conference_UnitOnlineNum(UnitOnlineNum *onlineNum);
static SysMode_EN Conference_GetSysMode(void);
static uint16_t Conference_GetCurrentEditedID(void);
static UnitInfo_S *Conference_WiredUnitInfo(void);
static UnitInfo_S *Conference_WifiUnitInfo(void);
static ConfSysInfo_S *Conference_GetConfSysInfo(void);


/* �ڲ����� */
static void Conference_TimingTask(TimerHandle_t xTimer);
static void Conference_MessageProcess(Notify_S *notify);
static void Conference_WifiUnitMessageProcess(Notify_S *notify);

static void Conference_ScreenUpdataUnitNum(void);
static void Conference_ScreenMessageProcess(Notify_S *notify);

static void Conference_ResetAudioChannel(UnitType_EN type);
static uint8_t Conference_GetAudioChannel(UnitType_EN type);
static void Conference_GiveAudioChannel(UnitType_EN type,uint8_t ch);

static void Conference_SignInstruction(uint16_t id,UnitType_EN type,uint32_t cmd,uint16_t totalNum,uint16_t curNum);
static void Conference_VoteResult(void);
static void Conference_MicCtrlInstruction(uint16_t id, UnitType_EN type,UnitAttr_EN attr, MicControlType_EN ctrlType,uint8_t channel);
static void Conference_MicControl(uint16_t id, UnitType_EN type, uint32_t cmd);
static void Conference_DspUnitCtrl(uint16_t id, UnitType_EN type, MicControlType_EN ctrlType,uint8_t channel);

static void Conference_CloseAllMic(UnitType_EN type,UnitAttr_EN attr);
static void Conference_OfflineAllUnit(void);
static void Conference_ClearApplyMic(void);
static void Conference_ChangeSysMode(SysMode_EN mode);
static void Conference_ChairmanPriority(UnitType_EN type,uint16_t id);
static void Conference_UpdataUnitTime(TimePara_S *time);

static void Conference_UsbStateListener(status_t sta);
static void Conference_AudioStateListener(AudEvent_EN event,AudInfo_S *info);
static void Conference_FireAlarmSignalCallback(void *param);


static void Conference_OpenMicListDebug(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* ͶƱģʽ�루ȫ����Э�飩 */
static const uint8_t VoteModeCode[] = {
    KEY3_FIRST_SIGN_VOTE,KEY3_FIRST_NOSIGN_VOTE,KEY3_LAST_SIGN_VOTE,KEY3_LAST_NOSIGN_VOTE,
    KEY5_FIRST_SIGN_SELECT, KEY5_FIRST_NOSIGN_SELECT, KEY5_LAST_SIGN_SELECT,KEY5_LAST_NOSIGN_SELECT,
    KEY5_FIRST_SIGN_RATE,KEY5_FIRST_NOSIGN_RATE, KEY5_LAST_SIGN_RATE, KEY5_LAST_NOSIGN_RATE,
    KEY2_LAST_NOSIGN_CUSTOM,KEY2_LAST_SIGN_CUSTOM,KEY2_FIRST_NOSIGN_CUSTOM, KEY2_FIRST_SIGN_CUSTOM,
    KEY3_LAST_NOSIGN_CUSTOM,KEY3_LAST_SIGN_CUSTOM, KEY3_FIRST_NOSIGN_CUSTOM, KEY3_FIRST_SIGN_CUSTOM,
    KEY4_LAST_NOSIGN_CUSTOM,KEY4_LAST_SIGN_CUSTOM, KEY4_FIRST_NOSIGN_CUSTOM, KEY4_FIRST_SIGN_CUSTOM,
    KEY5_LAST_NOSIGN_CUSTOM,KEY5_LAST_SIGN_CUSTOM, KEY5_FIRST_NOSIGN_CUSTOM, KEY5_FIRST_SIGN_CUSTOM,
    KEY3_FIRST_SIGN_SATISFACTION,KEY3_FIRST_NOSIGN_SATISFACTION, KEY3_LAST_SIGN_SATISFACTION, KEY3_LAST_NOSIGN_SATISFACTION,
};


/* ����֪ͨ���� */
static QueueHandle_t noticeQueue;

/* ���ж�ʱ����ʱ�� */
static TimerHandle_t TimingTask;

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

/* ���߼����ߵ�Ԫ���뿪��Ͳ����(ֻ��������ģʽ) */
static DataQueueHandler_S ApplyQueue;

/* AFC����GPIO   (D14 GPIO2_29) */
static HAL_GpioHandler AfcCtrl;

/* LineOut�����������GPIO (H13 GPIO1_24) */
static HAL_GpioHandler LineOutCtrl;

/* ����ͨ�� Out1 ~ Out16 �����������GPIO (L10 GPIO1_15) */
static HAL_GpioHandler PartOutCtrl;

/* �豸�����ź� (L12 GPIO1_20) */
static HAL_GpioHandler RunSignal;

/* ���ź����� (H14 GPIO1_14) */
static HAL_GpioHandler FireAlarmSignal;

/* ���ź� */
static bool FireAlarm = false;

/* ���µ�Ԫ���� (��־λ���ڼ�¼�Ƿ�����
��Ԫ���ߣ�Ȼ���ٶ�ʱ�������·��㲥�����Ϣ)*/
static struct {
    bool wiredUnit;
    bool wifiUnit;
} NewUpline;

/* USB��Ƶ¼�ſ��ƾ�� */
static struct {
    /* MCU�Ƿ�������U�� */
    bool isMcuConnectUsb;
    /* wt2000�Ƿ�����U�� */
    bool isWt2000ConnectUsb;
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
    /* ����¼���ļ�������� */
    uint16_t recFileMaxIndex;
    /* �����ļ��б� */
    FILINFO *musicFile[MUSIC_FILE_LIST_MAX_LEN];
} UsbAudio;



/*******************************************************************************
 * Task & API
 ******************************************************************************/

static AppTask_S ConfigTask = {
    .task = Conference_ConfigTask,
    .name = "Conference.Config",
    .stack = CONFERENCE_TASK_STACK_SIZE,
    .para = null,
    .prio = CONFERENCE_TASK_PRIORITY,
    .handle = null
};

static AppTask_S NoticeProcess = {
    .task = Conference_NoticeProcessTask,
    .name = "Conference.NoticeProcess",
    .stack = CONFERENCE_TASK_STACK_SIZE,
    .para = null,
    .prio = CONFERENCE_TASK_PRIORITY,
    .handle = null
};


static AppTask_S *FuncTask[] = {&NoticeProcess};

static AppLauncher_S Launcher = {
    .init = null,
    .configTask = &ConfigTask,
    .funcNum  = 1,
    .funcTask = FuncTask,
};


/* API  */
Conference_S Conference = {
    .launcher = &Launcher,

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

/*******************************************************************************
 * @Section:         Conference
 ******************************************************************************/

/**
* @Name  		Conference_ConfigTask
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_ConfigTask(void *pvParameters)
{
    uint16_t id;
    UnitCfg_S *unitCfg;

    noticeQueue = xQueueCreate(NOTICE_QUEUE_LENGTH,NOTICE_QUEUE_SIZE);

    memset(&UnitInfo,0,sizeof(UnitInfo));
    memset(&UsbAudio,0,sizeof(UsbAudio));

    /* GPIO��ʼ�� */
    AfcCtrl = HAL_GpioInit(GPIO2, 29, kGPIO_DigitalOutput, 0, (gpio_interrupt_mode_t)null);
    LineOutCtrl = HAL_GpioInit(GPIO1, 24, kGPIO_DigitalOutput, 0, (gpio_interrupt_mode_t)null);
    PartOutCtrl = HAL_GpioInit(GPIO1, 15, kGPIO_DigitalOutput, 1, (gpio_interrupt_mode_t)null);
    RunSignal = HAL_GpioInit(GPIO1, 20, kGPIO_DigitalOutput, 0, (gpio_interrupt_mode_t)null);
    FireAlarmSignal = HAL_GpioInit(GPIO1, 14, kGPIO_DigitalInput, null, kGPIO_IntRisingOrFallingEdge);


    /* ��ʼ����Ͳ���� */
    WiredChmMicQueue = DataQueue.creat(WIRED_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WiredRpsMicQueue = DataQueue.creat(WIRED_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WiredWaitQueue = DataQueue.creat(WIRED_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WiredChannelQueue = DataQueue.creat(WIRED_UNIT_MAX_ALLWO_OPEN,sizeof(uint8_t));

    WifiChmMicQueue = DataQueue.creat(WIFI_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WifiRpsMicQueue = DataQueue.creat(WIFI_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WifiWaitQueue = DataQueue.creat(WIFI_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WifiChannelQueue = DataQueue.creat(WIFI_UNIT_MAX_ALLWO_OPEN,sizeof(uint8_t));

    ApplyQueue = DataQueue.creat(WIFI_UNIT_MAX_ALLWO_OPEN + WIRED_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));

    /* ��λ��Ԫ��Ƶͨ������ */
    Conference_ResetAudioChannel(kType_Unit_Wired);
    Conference_ResetAudioChannel(kType_Unit_Wifi);

    /* ����USB״̬���� */
    UsbDisk.setListener(Conference_UsbStateListener);

    /* ��Ƶ¼��״̬���� */
    Audio.setListener(Conference_AudioStateListener);

    /* �����ݿ��ȡ����ʼ��������� */
    SysInfo.config = (SysCfg_S *)Database.getInstance(kType_Database_SysCfg);

    /* ���ߵ�Ԫ�������ӵ����ݿ� */
    unitCfg = (UnitCfg_S *)Database.getInstance(kType_Database_WiredCfg);
    for(id = 1; id <= WIRED_UNIT_MAX_ONLINE_NUM; id++) {
        UnitInfo.wired[id].config = &unitCfg[id];
    }

    /* WIFI��Ԫ�������ӵ����ݿ� */
    unitCfg = (UnitCfg_S *)Database.getInstance(kType_Database_WifiCfg);
    for(id = 1; id <= WIFI_UNIT_MAX_ONLINE_NUM; id++) {
        UnitInfo.wifi[id].config = &unitCfg[id];
    }

    /* ��ʼ����ʱ�����ʱ�� */
    TimingTask = xTimerCreate("TimingTask",TIMING_TASK_INTERVAL,pdTRUE,null,Conference_TimingTask);


    vTaskDelete(null);


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

    Log.i("Conference notice process task start!!\r\n");

    HAL_SetGpioLevel(PartOutCtrl, 0);
    HAL_SetGpioLevel(LineOutCtrl, 1);
    HAL_SetGpioLevel(AfcCtrl, 0);

    HAL_SetIrqCallback(FireAlarmSignal, Conference_FireAlarmSignalCallback, null);


    /* ���RTC��ؼ��л���ҳ�� */
    if(!Time.getRst())
        Screen.togglePage(SP_BATTERY_ERR);
    else
        Screen.togglePage(SP_WELCOME);

    xTimerStart(TimingTask, 0);

    while(1) {
        xQueueReceive(noticeQueue, &notify, MAX_NUM);

        /* �ж�֪ͨ��Դ */

        /* ֪ͨ��ԴΪ�ⲿ���Ƽ����ߵ�ԪͨѶ���� */
        if(notify->nSrc & (EX_CTRL_DEST | kType_NotiSrc_WiredUnit)) {
            Conference_MessageProcess(notify);
        }

        /* ֪ͨ��ԴΪWIFI��ԪͨѶ���� */
        else if(notify->nSrc & kType_NotiSrc_WifiUnit) {
            Conference_WifiUnitMessageProcess(notify);
        }

        /* ֪ͨ��ԴΪ��Ļ����ͨѶ���� */
        else if(notify->nSrc & kType_NotiSrc_ScreenCtrl) {
            Conference_ScreenMessageProcess(notify);
        }

        /* ֪ͨ��ԴΪ�ӵ�Ƭ��ͨѶ���� */
        else if(notify->nSrc & kType_NotiSrc_SlaveMcu) {

        }

        FREE(notify);

    }
}



/**
* @Name  		Conference_TimingTask
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_TimingTask(TimerHandle_t xTimer)
{
    static bool runSignal;
    static bool alarmTmp = false;
    static uint16_t timeCnt = 1;

    TimePara_S time;
    ConfProtocol_S confProt;
    WifiUnitProtocol_S wifiProt;
    uint8_t i;


    /*** @TimingTask0 �� ���е� ***/
    runSignal = !runSignal;
    HAL_SetGpioLevel(RunSignal, runSignal);


    /*** @TimingTask1 �� ��������,����ⲿ��������״̬,������Ļ�л�����Ӧ״̬ ***/
    /* ���ź����� */
    if(FireAlarm) {
        Screen.lock(SP_FIRE_WARNING);
    }

    /* ID�ظ� */
    else if(SysInfo.state.idDupicate) {
        Screen.lock(SP_ID_PEPEAT);
    }

    /* PC�������� */
    else if(ExternalCtrl.connectSta(kType_NotiSrc_PC) || ExternalCtrl.connectSta(kType_NotiSrc_Web)) {
        Screen.lock(SP_EX_CTRL_CONNECT);
    }

    /* ͶƱ��ǩ������������ */
    else if(SysInfo.state.sysMode != kMode_Conference) {
        if(SysInfo.state.sysMode == kMode_DevSign)
            Screen.lock(SP_SIGNING_IN);

        else if(SysInfo.state.sysMode == kMode_DevVote)
            Screen.lock(SP_VOTING);
    }

    /* ���������� */
    else {
        if(Screen.isLock())
            Screen.unlock();
    }


    /*** @TimingTask2 �� ����״̬,���㲥״̬ ***/
    /* �������� */
    if(FireAlarm) {
        alarmTmp = true;

        Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,CONFERENCE_MODE,FIRE_ALARM_SIGNAL,null,null);
        WiredUnit.transmit(&confProt);
        ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);

        WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,AlarmOrNot_MtoU_G,0,0));
    }
    /* ������� */
    else if(alarmTmp == true && FireAlarm == false) {
        alarmTmp = false;

        Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,CONFERENCE_MODE,FIRE_ALARM_SIGNAL_CANCEL,null,null);
        WiredUnit.transmit(&confProt);
        ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);

        WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,AlarmOrNot_MtoU_G,1,0));
    }


    /*** @TimingTask3 �� ��ʱ�·���ʵʱ�䵽��Ԫ ***/
    if(Time.getRst() && (timeCnt % BROADCAST_RTC_INTERVAL == 0)) {
        Time.getNow(&time);
        Conference_UpdataUnitTime(&time);
    }


    /*** @TimingTask4 �� ����µ�Ԫ���߱�־���㲥�Զ�������  ***/
    if(NewUpline.wiredUnit || NewUpline.wifiUnit) {


        /* �·��Զ������� */
        if(SysInfo.state.sysMode == kMode_Vote &&  \
           SysInfo.state.voteMode >= Key2First_Sign_CustomTerm && \
           SysInfo.state.voteMode <= Key5Last_NoSign_CustomTerm) {

            for(i = 0; i < 5; i++) {
                uint8_t *item = SysInfo.state.voteCustomItem[i];
                uint8_t len = strlen((char *)item);

                Log.d("len = %d content = %s\r\n",len,item);

                /* ���ߵ�Ԫ */
                if(NewUpline.wiredUnit) {
                    if(len <= 3) {
                        WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,CUSTOM_VOTE_ITEM,i+1, item[0],(item[1] << 8) | item[2]));
                    } else {
                        WiredUnit.transWithExData(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,CUSTOM_VOTE_ITEM,i+1,item[0],(item[1] << 8) | item[2]),len - 3,&item[3]);
                    }
                }

                /* WIFI��Ԫ */
                if(NewUpline.wifiUnit)
                    WifiUnit.transWithExData(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,PCUpdateCustomTerm_MtoU_G,i+1,0), len,item);

            }

        }
    }

    /*** @TimingTask5 �� ���ID�ظ���־ ***/
    if(SysInfo.state.idDupicate) {
#if 0
        if(SysInfo.state.idDupCnt-- > 0) {
            /* �㲥֪ͨȫ���ֻ���ϵͳ */
            WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID, BASIC_MSG, CONFERENCE_MODE, ID_DUPICATE,null,SysInfo.state.dupId));
            /* �㲥֪ͨȫWIFI����ϵͳ */
            WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0, IDRepeatingMtoU_G, SysInfo.state.dupId >> 8, SysInfo.state.dupId & 0xFF));
        }
        /* ����ID_DUPICATE_WAIT_TIME���û�������յ�ID�ظ������״̬ */
        else {
            SysInfo.state.idDupicate = false;

            /* �㲥֪ͨȫ���ֻ���ϵͳ */
            WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID, BASIC_MSG, CONFERENCE_MODE, ID_UNDUPICATE,null,null));
            /* �㲥֪ͨȫWIFI����ϵͳ */

            /**@brief WIFIϵͳû��ȡ��ID�ظ�Э�飬�������� */
        }
#else
        /* �㲥֪ͨȫ���ֻ���ϵͳ */
        WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID, BASIC_MSG, CONFERENCE_MODE, ID_DUPICATE,null,SysInfo.state.dupId));
        /* �㲥֪ͨȫWIFI����ϵͳ */
        WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0, IDRepeatingMtoU_G, SysInfo.state.dupId >> 8, SysInfo.state.dupId & 0xFF));
#endif
    }


#ifdef	CONFIRM_UNIT_NUM
    for(i = 0; i < WIRED_UNIT_MAX_ONLINE_NUM; i++) {
        if(UnitInfo.wired[i].online) {
            if(UnitInfo.wired[i].attr == aChairman) chmNum++;
            else if(UnitInfo.wired[i].attr == aRepresentative) rpsNum++;
        }
    }

    if(OnlineNum.wiredChm != chmNum || OnlineNum.wiredRps != rpsNum) {
        OnlineNum.wiredChm = chmNum;
        OnlineNum.wiredRps = rpsNum;
        Log.d("Online num is update: wiredUnit(CHM) = %d , wiredUnit(RPS) = %d\r\n",OnlineNum.wiredChm,OnlineNum.wiredRps);
    }
#endif

    /* ��λ�����߱�־ */
    NewUpline.wiredUnit = false;
    NewUpline.wifiUnit = false;


    /* ��ʱ��ÿ36000�루10Сʱ������һ�� */
    if(timeCnt++ >= TIME_COUNT_RESET_INTERVAL)
        timeCnt = 0;
}

/*******************************************************************************
 * @Section:         Process
 ******************************************************************************/

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
    uint8_t protType, *protPara;
    uint16_t id;
    ConfProtocol_S confProt;
    WifiUnitProtocol_S wifiProt;

    /* ֪ͨԴ */
    NotifySrcType_EN noticeSrc;
    /* ID��Ӧ�ĵ�Ԫ���� */
    UnitType_EN idUnitType = (UnitType_EN)null;

    ERR_CHECK(notify != null, return);

    id = notify->prot.conference.id;
    protType = notify->prot.conference.type;
    protPara = &notify->prot.conference.ph;
    noticeSrc = notify->nSrc;

    if(id > WIFI_UNIT_START_ID && id <= WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM)) {
        idUnitType = kType_Unit_Wifi;
        id -= WIFI_UNIT_START_ID;
    } else if(id >= 1 && id <= WIRED_UNIT_MAX_ONLINE_NUM) {
        idUnitType = kType_Unit_Wired;
    }

#if 1
    if(true) {
        char srcStr[10];
        sprintf(srcStr,"%s",noticeSrc == kType_NotiSrc_WiredUnit ? "Wired Unit" :  noticeSrc == kType_NotiSrc_PC ? "PC" : \
                noticeSrc == kType_NotiSrc_Web ? "WEB" : noticeSrc == kType_NotiSrc_UartCtrl ? "UART" : "unknow");
        Log.d("%s Msg: id = 0x%X, type = 0x%X,para{ %X , %X , %X , %X , %X }",srcStr,id,protType,protPara[0],protPara[1],protPara[2],protPara[3],protPara[4]);
        if(notify->exLen) {
            uint8_t i;

            printf(" exData{ ");
            for(i=0; i<notify->exLen; i++)
                printf(" %X ,",(&notify->exDataHead)[i]);
            printf(" } ");
        }
        printf("\r\n");
    }
#endif

    switch(protType) {
    /* ������Ϣ 0x80 */
    case BASIC_MSG: {
        uint8_t mode = protPara[0], cmd = protPara[1];

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
                /* id��Χ��WIFI��Ԫ���ҿ���ָ����Դ���ⲿ���� */
//                if(idUnitType == kType_Unit_Wifi && (notify->nSrc & EX_CTRL_DEST))
//                    Conference_MicControl(id - WIFI_UNIT_START_ID, kType_Unit_Wifi, WIRED_CMD(cmd,0,0));
//                else if(idUnitType == kType_Unit_Wired)
//                    Conference_MicControl(id, kType_Unit_Wired, WIRED_CMD(cmd,0,0));
                Conference_MicControl(id, idUnitType, WIRED_CMD(cmd,0,0));
            }
            break;

            /* ͬ�⿪��Ͳ */
            case AGREE_OPEN_MIC:
            /* ��ͬ�⿪��Ͳ */
            case DISAGREE_OPEN_MIC: {
                /* PC�·�ͬ���ͬ�� */
//                if(idUnitType == kType_Unit_Wifi && (notify->nSrc & EX_CTRL_DEST))
//                    Conference_MicControl(id - WIFI_UNIT_START_ID, kType_Unit_Wifi, WIRED_CMD(cmd,0,0));
//                else if(idUnitType == kType_Unit_Wired && (notify->nSrc & EX_CTRL_DEST))
//                    Conference_MicControl(id, kType_Unit_Wired, WIRED_CMD(cmd,0,0));
                if(notify->nSrc & EX_CTRL_DEST)
                    Conference_MicControl(id, idUnitType, WIRED_CMD(cmd,0,0));

                /* ������ϯ��Ԫ�·�ͬ���ͬ�� */
                else if(notify->nSrc == kType_NotiSrc_WiredUnit)
                    Conference_MicControl(0, (UnitType_EN)null, WIRED_CMD(cmd,0,0));
            }
            break;

            /* ��ϯ����Ȩ */
            case CHM_PRIORITY: {
                if(idUnitType == kType_Unit_Wired)
                    Conference_ChairmanPriority(kType_Unit_Wired,id);
            }
            break;

            /* ��Ͳ���� */
            case UNIT_ACCESS_SYS: {
                /* ��Ͳ�����Ѿ����� */
                if(UnitInfo.wired[id].online) {
                    /* �·�MIC״̬ */
                    switch(UnitInfo.wired[id].micSta) {
                    case kStatus_UnitMic_Open: {
                        uint8_t pl = (UnitInfo.wired[id].attr == aChairman ? CHM_OPEN_MIC : RPS_OPEN_MIC);

                        WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,pl,UnitInfo.wired[id].channel,null));
                    }
                    break;
                    case kStatus_UnitMic_Apply:
                    case kStatus_UnitMic_Wait: {
                        WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,MIC_WAIT,null,null));
                    }
                    break;

                    default:
                        break;
                    }

                    Log.d("WiredUnit(%s) id = %d reonline\r\n",UnitInfo.wired[id].attr == aChairman ? "CHM":"RPS",id);
                } else {
                    if(UnitInfo.wired[id].attr == aChairman)
                        OnlineNum.wiredChm++;
                    else if(UnitInfo.wired[id].attr == aRepresentative)
                        OnlineNum.wiredRps++;

                    /* ������Ļ��ʾ���� */
                    Conference_ScreenUpdataUnitNum();

                    UnitInfo.wired[id].online = true;

                    Log.d("WiredUnit(%s) id = %d online ( Chm num = %d , Rps num = %d )\r\n",UnitInfo.wired[id].attr == aChairman ? "CHM":"RPS",id,OnlineNum.wiredChm,OnlineNum.wiredRps);
                }

                /* ֪ͨ�ⲿ���ƻ�Ͳ���� */
                ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id,PC_MSG,UNIT_LOGIN,0x01,null,null));

                NewUpline.wiredUnit = true;

                /* ͬ��EQ �����Ȳ��� */
            }
            break;

            /*��Ͳ����*/
            case UNIT_OFFLINE: {
                /* �رջ�Ͳ��ȡ���ȴ� */
                if(UnitInfo.wired[id].attr == aChairman) {
                    Conference_MicControl(id, kType_Unit_Wired, WIRED_CMD(CHM_CLOSE_MIC,0,0));
                    OnlineNum.wiredChm--;
                } else if(UnitInfo.wired[id].attr == aRepresentative) {
                    Conference_MicControl(id, kType_Unit_Wired, WIRED_CMD(RPS_CLOSE_MIC,0,0));
                    /* ����Ǵ����������ȴ� */
                    Conference_MicControl(id, kType_Unit_Wired, WIRED_CMD(MIC_DISWAIT,0,0));
                    OnlineNum.wiredRps--;
                }

                /* ֪ͨ�ⲿ���ƻ�Ͳ���� */
                ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id,PC_MSG,UNIT_LOGOUT,0x01,null,null));

                /* ������Ļ��ʾ���� */
                Conference_ScreenUpdataUnitNum();

                /* ������߻�ͲΪ����ǩ������Ļ�Ͳ����ϵͳģʽ�ָ�������ģʽ */
                if(SysInfo.state.initSignVoteId == id && (SysInfo.state.sysMode == kMode_DevSign || SysInfo.state.sysMode == kMode_DevVote)) {
                    Conference_ChangeSysMode(kMode_Conference);
                }

                Log.d("WiredUnit(%s) id = %d offline ( Chm num = %d , Rps num = %d )\r\n",UnitInfo.wired[id].attr == aChairman ? "CHM":"RPS",id,OnlineNum.wiredChm,OnlineNum.wiredRps);
            }
            break;

            /* ID�ظ� */
            case ID_DUPICATE: {
                SysInfo.state.dupId = protPara[3] << 8 | protPara[4] ;

                if(SysInfo.state.sysMode != kMode_EditID) {
                    Log.d("Wired unit id repead !! ID = %d\r\n",SysInfo.state.dupId);

                    /* �㲥֪ͨȫ���ֻ���ϵͳ */
                    WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID, BASIC_MSG, CONFERENCE_MODE, ID_DUPICATE,null,SysInfo.state.dupId));
                    /* �㲥֪ͨȫWIFI����ϵͳ */
                    WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0, IDRepeatingMtoU_G, SysInfo.state.dupId >> 8, SysInfo.state.dupId & 0xFF));

                    SysInfo.state.idDupicate = true;
//					SysInfo.state.idDupCnt = ID_DUPICATE_WAIT_TIME;

//                    Screen.togglePage(SP_ID_PEPEAT);
                }
            }
            break;
            /* ���Լ�ʱ */
            case RECKON_TIME_ENABLE:
            case RECKON_TIME_DISABLE: {
                if(id == WHOLE_BROADCAST_ID) {
                    WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,CONFERENCE_MODE,cmd,null,null));
                    WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,EnterSpeakWithTimeOrNot_MtoU,cmd == RECKON_TIME_DISABLE,null));
                } else if(idUnitType == kType_Unit_Wifi) {
                    WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,EnterSpeakWithTimeOrNot_MtoU, cmd == RECKON_TIME_DISABLE,null));
                } else if(idUnitType == kType_Unit_Wired) {
                    WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,CONFERENCE_MODE,cmd,null,null));
                }
            }
            break;
            /* ��ʱ���� */
            case TIME_LIMIT_ENABLE:
            case TIME_LIMIT_DISABLE: {
                if(id == WHOLE_BROADCAST_ID) {
                    WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,CONFERENCE_MODE,cmd,null,protPara[3] << 8 | protPara[4]));
                    WifiUnit.transWithExData(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,TimeLimitOrNot_MtoU,cmd == TIME_LIMIT_DISABLE,null),2,&protPara[3]);
                } else if(idUnitType == kType_Unit_Wifi) {
                    WifiUnit.transWithExData(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,TimeLimitOrNot_MtoU,cmd == TIME_LIMIT_DISABLE,null),2,&protPara[3]);
                } else if(idUnitType == kType_Unit_Wired) {
                    WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,cmd,null,protPara[3] << 8 | protPara[4]));
                }

            }
            break;

            /* ��Ԫ�����з��� */
            case SERVICE_TEA:
            case SERVICE_PAPER_PEN:
            case SERVICE_ARTIFICIAL:
            case SERVICE_COFFEE:
            case SERVICE_FLOWER:
            case SERVICE_MICPHONE:
            case SERVICE_RESERVED_1:
            case SERVICE_RESERVED_2: {
                ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,cmd,null,null));
            }
            break;

            /* ���ź� */
            case FIRE_ALARM_SIGNAL:
            case FIRE_ALARM_SIGNAL_CANCEL: {
                Log.d("Fire signal = %X\r\n",cmd);
            }
            break;

            }
        }
        break;
        /* ǩ��ģʽ 0x01 */
        case SIGN_MODE: {
            switch(cmd) {
            /* ����ǩ��ģʽ(���ƶ˷���) */
            case START_SIGN_MODE: {
                SysInfo.state.totalSignNum = (uint16_t)((protPara[3] << 8) | protPara[4]);
                SysInfo.state.currentSignNum = 0;

                Conference_ChangeSysMode(kMode_Sign);
            }
            break;
            /* ����ǩ��ģʽ(��ϯ��Ԫ����) */
            case DEV_START_SIGN_MODE: {
                /* ϵͳ���ڻ���ģʽ�������л� */
                if(SysInfo.state.sysMode != kMode_Conference)
                    break;

                SysInfo.state.totalSignNum = (uint16_t)(UNIT_ONLINE_NUM);
                SysInfo.state.currentSignNum = 0;
                /* ���淢��ID */
                SysInfo.state.initSignVoteId = id;

                /* ��Ӧ��ϯ�� */
                WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,SIGN_MODE,DEV_START_SIGN_MODE,null,null));

                /* ��ģʽ */
                Conference_ChangeSysMode(kMode_DevSign);
            }
            break;
            /* ����ǩ��(���ƶ˷���) */
            case END_SIGN_MODE:
            /* ����ǩ��(��ϯ��Ԫ����) */
            case DEV_END_SIGN_MODE: {
                Conference_ChangeSysMode(kMode_Conference);
            }
            break;
            /* ��Ԫǩ�� */
            case UNIT_SIGN_IN: {
                /* ��Ԫǩ�� */
                if((notify->nSrc & kType_NotiSrc_WiredUnit) && !UnitInfo.wired[id].sign) {
                    UnitInfo.wired[id].sign = true;
                    SysInfo.state.currentSignNum++;
                    Conference_SignInstruction(id,kType_Unit_Wired,WIRED_CMD(UNIT_SIGN_IN,0,0),null,null);
                    /* �·�ǩ������������ǩ��������� */
                    Conference_SignInstruction(null,(UnitType_EN)null,WIRED_CMD(START_SIGN_MODE,0,1),SysInfo.state.totalSignNum,SysInfo.state.currentSignNum);
                }
                /* �����Ԫǩ�������ⲿ���ƣ���Ϊ����ǩ�� */
                else if(notify->nSrc & (kType_NotiSrc_PC | kType_NotiSrc_Web | kType_NotiSrc_UartCtrl)) {
                    /* �������ߵ�Ԫǩ�� */
                    if(idUnitType == kType_Unit_Wired && !UnitInfo.wired[id].sign) {
                        Conference_SignInstruction(id,kType_Unit_Wired,WIRED_CMD(CONTROL_UNIT_SIGN,0,0),null,null);
                    }
                    /* ����WIFI��Ԫǩ�� */
                    else if(idUnitType == kType_Unit_Wifi  && !UnitInfo.wifi[id].sign) {
                        Conference_SignInstruction(id,kType_Unit_Wifi,WIFI_CMD(ControlSign_MtoU_D,0,0),null,null);
                    }
                }
            }
            break;
            /* ����/��ֹǩ�� */
            case SIGN_ENABLE:
            case SIGN_DISABLE: {

                if(id == UNIT_BROADCAST_ID) {
                    SysInfo.config->signEn = (cmd == SIGN_ENABLE);
                    WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,EnableSignOrNot_MtoU_D,!SysInfo.config->signEn,null));
                    WiredUnit.transmit(Protocol.conference(&confProt,UNIT_BROADCAST_ID,BASIC_MSG,SIGN_MODE,cmd,null,null));
                } else {
                    /* �ж��豸���� */
                    if(idUnitType == kType_Unit_Wifi)
                        WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,EnableSignOrNot_MtoU_D,!(cmd == SIGN_ENABLE),null));
                    else if(idUnitType == kType_Unit_Wired)
                        WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,SIGN_MODE,cmd,null,null));
                }
            }
            break;
//            /* ��ֹǩ�� */
//            case SIGN_DISABLE: {
//
//            }
//            break;
            /* ����ǩ�� */
            case SUPPLEMENT_SIGN: {
                /* ���ߵ�Ԫ����ǩ�� */
                if(idUnitType == kType_Unit_Wired && !UnitInfo.wired[id].sign) {
                    Conference_SignInstruction(id,kType_Unit_Wired,WIRED_CMD(SUPPLEMENT_SIGN,0,0),null,null);
                }
                /* WIFI��Ԫ����ǩ��*/
                else if(idUnitType == kType_Unit_Wifi && !UnitInfo.wifi[id].sign) {
                    Conference_SignInstruction(id,kType_Unit_Wifi,WIFI_CMD(SupplementSign_MtoU_D,0,0),null,null);
                }
            }
            break;
            /* ��ԪӦ�𲹳�ǩ�� */
            case UNIT_SUPPLEMENT_SIGN: {
                if((notify->nSrc & kType_NotiSrc_WiredUnit) && !UnitInfo.wired[id].sign) {
                    UnitInfo.wired[id].sign = true;
                    SysInfo.state.currentSignNum++;
                    Conference_SignInstruction(id,kType_Unit_Wired,WIRED_CMD(UNIT_SUPPLEMENT_SIGN,0,0),null,null);
                }
            }
            break;
            }
        }
        break;
        /* ͶƱģʽ 0x02 */
        case VOTE_MODE: {
            uint8_t unitVote;

            switch(cmd) {
            /* PC����ĸ�����ͶƱ������������������ */
            case KEY3_FIRST_SIGN_VOTE:
                SysInfo.state.voteMode = Key3First_Sign_vote;
                goto startVote;
            case KEY3_FIRST_NOSIGN_VOTE:
                SysInfo.state.voteMode = Key3First_NoSign_vote;
                goto startVote;
            case KEY3_LAST_SIGN_VOTE:
                SysInfo.state.voteMode = Key3Last_Sign_vote;
                goto startVote;
            case KEY3_LAST_NOSIGN_VOTE:
                SysInfo.state.voteMode = Key3Last_NoSign_vote;
                goto startVote;
            case KEY5_FIRST_SIGN_SELECT:
                SysInfo.state.voteMode = Key5First_Sign_Select;
                goto startVote;
            case KEY5_FIRST_NOSIGN_SELECT:
                SysInfo.state.voteMode = Key5First_NoSign_Select;
                goto startVote;
            case KEY5_LAST_SIGN_SELECT:
                SysInfo.state.voteMode = Key5Last_Sign_Select;
                goto startVote;
            case KEY5_LAST_NOSIGN_SELECT:
                SysInfo.state.voteMode = Key5Last_NoSign_Select;
                goto startVote;
            case KEY5_FIRST_SIGN_RATE:
                SysInfo.state.voteMode = Key5First_Sign_Rate;
                goto startVote;
            case KEY5_FIRST_NOSIGN_RATE:
                SysInfo.state.voteMode = Key5First_NoSign_Rate;
                goto startVote;
            case KEY5_LAST_SIGN_RATE:
                SysInfo.state.voteMode = Key5Last_Sign_Rate;
                goto startVote;
            case KEY5_LAST_NOSIGN_RATE:
                SysInfo.state.voteMode = Key5Last_NoSign_Rate;
                goto startVote;
            case KEY2_FIRST_SIGN_CUSTOM:
                SysInfo.state.voteMode = Key2First_Sign_CustomTerm;
                goto startVote;
            case KEY2_FIRST_NOSIGN_CUSTOM:
                SysInfo.state.voteMode = Key2First_NoSign_CustomTerm;
                goto startVote;
            case KEY2_LAST_SIGN_CUSTOM:
                SysInfo.state.voteMode = Key2Last_Sign_CustomTerm;
                goto startVote;
            case KEY2_LAST_NOSIGN_CUSTOM:
                SysInfo.state.voteMode = Key2Last_NoSign_CustomTerm;
                goto startVote;
            case KEY3_FIRST_SIGN_CUSTOM:
                SysInfo.state.voteMode = Key3First_Sign_CustomTerm;
                goto startVote;
            case KEY3_FIRST_NOSIGN_CUSTOM:
                SysInfo.state.voteMode = Key3First_NoSign_CustomTerm;
                goto startVote;
            case KEY3_LAST_SIGN_CUSTOM:
                SysInfo.state.voteMode = Key3Last_Sign_CustomTerm;
                goto startVote;
            case KEY3_LAST_NOSIGN_CUSTOM:
                SysInfo.state.voteMode = Key3Last_NoSign_CustomTerm;
                goto startVote;
            case KEY4_FIRST_SIGN_CUSTOM:
                SysInfo.state.voteMode = Key4First_Sign_CustomTerm;
                goto startVote;
            case KEY4_FIRST_NOSIGN_CUSTOM:
                SysInfo.state.voteMode = Key4First_NoSign_CustomTerm;
                goto startVote;
            case KEY4_LAST_SIGN_CUSTOM:
                SysInfo.state.voteMode = Key4Last_Sign_CustomTerm;
                goto startVote;
            case KEY4_LAST_NOSIGN_CUSTOM:
                SysInfo.state.voteMode = Key4Last_NoSign_CustomTerm;
                goto startVote;
            case KEY5_FIRST_SIGN_CUSTOM:
                SysInfo.state.voteMode = Key5First_Sign_CustomTerm;
                goto startVote;
            case KEY5_FIRST_NOSIGN_CUSTOM:
                SysInfo.state.voteMode = Key5First_NoSign_CustomTerm;
                goto startVote;
            case KEY5_LAST_SIGN_CUSTOM:
                SysInfo.state.voteMode = Key5Last_Sign_CustomTerm;
                goto startVote;
            case KEY5_LAST_NOSIGN_CUSTOM:
                SysInfo.state.voteMode = Key5Last_NoSign_CustomTerm;
                goto startVote;
            case KEY3_FIRST_SIGN_SATISFACTION:
                SysInfo.state.voteMode = Key3First_Sign_Satisfaction;
                goto startVote;
            case KEY3_FIRST_NOSIGN_SATISFACTION:
                SysInfo.state.voteMode = Key3First_NoSign_Satisfaction;
                goto startVote;
            case KEY3_LAST_SIGN_SATISFACTION:
                SysInfo.state.voteMode = Key3Last_Sign_Satisfaction;
                goto startVote;
            case KEY3_LAST_NOSIGN_SATISFACTION:
                SysInfo.state.voteMode = Key3Last_NoSign_Satisfaction;
                goto startVote;
                {
startVote:
                    Conference_ChangeSysMode(kMode_Vote);
                }
                break;

            /* ��ϯ���� */
            case DEV_LAUNCH_VOTE: {
                if(!(notify->nSrc & kType_NotiSrc_WiredUnit) || SysInfo.state.sysMode != kMode_Conference)
                    break;

                SysInfo.state.voteMode = Key3Last_NoSign_vote;
                /* ���淢��ID */
                SysInfo.state.initSignVoteId = id;

                /* ��Ӧ��ϯ�� */
                WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,VOTE_MODE,DEV_LAUNCH_VOTE,null,null));

                /* ��ģʽ */
                Conference_ChangeSysMode(kMode_DevVote);

            }
            break;

            /* ��ͣ������·������ */
            case PAUSE_VOTE: {
                /* ���ߵ�Ԫ�㲥��ͣ���(WIFI��Ԫ������ͬһ��ָ��) */
                WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,VOTE_MODE,PAUSE_VOTE,null,null));
                /* �·�ͶƱ��� */
                Conference_VoteResult();
                SysInfo.state.voteMode = VotePause;
            }
            break;

            /* ������� */
            case FINISH_VOTE:
            case DEV_FINISH_VOTE: {
                if(SysInfo.state.sysMode == kMode_Vote || SysInfo.state.sysMode == kMode_DevVote) {
                    SysInfo.state.voteMode = VotePause;
                    Conference_ChangeSysMode(kMode_Conference);
                }
            }
            break;

            /* ����/��ֹ��� */
            case VOTE_ENABLE:
            case VOTE_DISABLE: {
                if(id == UNIT_BROADCAST_ID) {
                    SysInfo.config->voteEn = ENABLE;

                    WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,EnableVoteOrNot_MtoU_D,!SysInfo.config->voteEn,null));
                    WiredUnit.transmit(Protocol.conference(&confProt,UNIT_BROADCAST_ID,BASIC_MSG,VOTE_MODE,cmd,null,null));
                } else {
                    /* �ж��豸���� */
                    if(idUnitType == kType_Unit_Wifi)
                        WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,EnableVoteOrNot_MtoU_D,!(cmd == VOTE_ENABLE),null));
                    else if(idUnitType == kType_Unit_Wired)
                        WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,VOTE_MODE,cmd,null,null));

                }
            }
            break;


            /* ��ԪͶƱ�������ѡ�� */
            case 0x25:
                unitVote = 1;//�޳�
                goto unitVoted;
            case 0x26:
                unitVote = 2;//��Ȩ
                goto unitVoted;
            case 0x27:
                unitVote = 3;//����
                goto unitVoted;
            case 0x45:
                unitVote = 1;//--
                goto unitVoted;
            case 0x46:
                unitVote = 2;//-
                goto unitVoted;
            case 0x47:
                unitVote = 3;//0
                goto unitVoted;
            case 0x48:
                unitVote = 4;//+
                goto unitVoted;
            case 0x49:
                unitVote = 5;//++
                goto unitVoted;
            case 0x4A:
                unitVote = 1;//�ܲ�����(���)/������(����)
                goto unitVoted;
            case 0x4B:
                unitVote = 2;//������(���)/һ��(����)
                goto unitVoted;
            case 0x4C:
                unitVote = 3;//һ��(���)/����(����)
                goto unitVoted;
            case 0x4D:
                unitVote = 4;//����(���)
                goto unitVoted;
            case 0x4E:
                unitVote = 5;//������(���)
                goto unitVoted;
            case 0x65:
                unitVote = 1;//�Զ�����ѡ��1
                goto unitVoted;
            case 0x66:
                unitVote = 2;//�Զ�����ѡ��2
                goto unitVoted;
            case 0x67:
                unitVote = 3;//�Զ�����ѡ��3
                goto unitVoted;
            case 0x68:
                unitVote = 4;//�Զ�����ѡ��4
                goto unitVoted;
            case 0x69:
                unitVote = 5;//�Զ�����ѡ��5
                {
unitVoted:
                    if(notify->nSrc & kType_NotiSrc_WiredUnit && (SysInfo.state.sysMode == kMode_Vote || SysInfo.state.sysMode == kMode_DevVote)   \
                       && SysInfo.state.voteMode != VotePause) {
//                        /* ���ͶƱ����Ϊ��Ҫ��ǩ��������Ԫδ����ǩ�� */
//                        if(IS_VOTE_NEED_SIGN(SysInfo.voteMode) && UnitInfo.wired[id].sign != true)
//                            break;
                        if(IS_VOTE_FIRST_VALID(SysInfo.state.voteMode) && UnitInfo.wired[id].vote != null)
                            break;

                        if(UnitInfo.wired[id].vote != unitVote) {
                            UnitInfo.wired[id].vote = unitVote;
                            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id,BASIC_MSG,VOTE_MODE,unitVote,null,null));
                        }
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

                Log.d("Wired confirm id = %d , dev mac = %X:%X:%X:%X:%X:%X \r\n",id,devMac->mac0,devMac->mac1,devMac->mac2,devMac->mac3,devMac->mac4,devMac->mac5);
                if(id == SysInfo.state.wiredCurEditID) {
                    /* �ظ���Ԫȷ�� */
                    WiredUnit.transWithExData(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,POLLING_MSG,EDIT_ID_POLLING,CONFIRM_ID,null,id),NETWORK_MAC_SIZE,(uint8_t *)devMac);

                    SysInfo.state.wiredCurEditID = (SysInfo.state.wiredCurEditID + 1) > WIRED_UNIT_MAX_ONLINE_NUM ? 1 : SysInfo.state.wiredCurEditID + 1;

                    WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,POLLING_MSG,EDIT_ID_POLLING,CURRENT_ID,null,SysInfo.state.wiredCurEditID));
                }
            }
            break;
            }
        }
        break;

        /* ��Ԫ���� 0x04 */
        case UNIT_CTRL: {
            /** @brief: �����EQ��������������AUDIO_MATRIX��AUDIO_MATRIX_INQUIRE(0xB0,0xB1)
            			�ֶΣ�����ʹ�õ�Э���Ǽ���PC�����2������ϵͳ��Э�顣���Э��������
            			��λ���㣬WEB�˲��ô����������Ƶ�������������˹���EQ�������ȵ�
            			Э�� */

            switch(cmd) {
            /* EQ */
            case SET_UNTI_EQ: {
                uint8_t eq,freq,vol;

                eq = (protPara[3] >> 4) - 1;
                freq = protPara[4] >> 5;
                vol = (protPara[4] & 0x1F);

                if(!(eq >= 0 && eq <= 3))
                    break;

                freq = (freq >= 0 && freq <= 3) ? freq : 0;
                vol = (vol >= 0 && vol <= 24) ? vol : 24;

                /* �������ߵ�Ԫ��EQ �����ȱ����ڵ�Ԫ��������PC��Ҫ���������ȡ�� */
                if(notify->nSrc & kType_NotiSrc_WiredUnit) {
                    UnitInfo.wired[id].config->eqFreq[eq] = freq;
                    UnitInfo.wired[id].config->eqVol[eq] = 24 - vol;
                    ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id,BASIC_MSG,UNIT_CTRL,SET_UNTI_EQ,null,protPara[3] << 8 | protPara[4]));
                }
                /* �����ⲿ���� */
                else {
                    if(idUnitType == kType_Unit_Wifi) {
                        UnitInfo.wifi[id].config->eqFreq[eq] = freq;
                        UnitInfo.wifi[id].config->eqVol[eq] = 24 - vol;
                        WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,SetUnitEQ_MtoU_G,protPara[3],protPara[4]));
                    } else if(idUnitType == kType_Unit_Wired) {
                        UnitInfo.wired[id].config->eqFreq[eq] = freq;
                        UnitInfo.wired[id].config->eqVol[eq] = 24 - vol;
                        WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,UNIT_CTRL,SET_UNTI_EQ,null,protPara[3] << 8 | protPara[4]));
                    }
                }
            }
            break;

            /* ������ */
            case SET_UNIT_SENSITIVITY: {
                /* �������ߵ�Ԫ��EQ �����ȱ����ڵ�Ԫ��������PC��Ҫ���������ȡ�� */
                if(notify->nSrc & kType_NotiSrc_WiredUnit) {
                    UnitInfo.wired[id].config->sensitivity = protPara[4];
                    ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id, BASIC_MSG, UNIT_CTRL, SET_UNIT_SENSITIVITY,null,protPara[4]));
                }
                /* �����ⲿ���� */
                else {
                    if(idUnitType == kType_Unit_Wifi) {
                        UnitInfo.wifi[id].config->sensitivity = protPara[4];
                        WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id, SetUnitMICSensitivity_MtoU_D, protPara[4], null));
                    } else if(idUnitType == kType_Unit_Wired) {
                        UnitInfo.wired[id].config->sensitivity = protPara[4];
                        WiredUnit.transmit(Protocol.conference(&confProt,id, BASIC_MSG, UNIT_CTRL, SET_UNIT_SENSITIVITY,null,protPara[4]));
                    }
                }
            }
            break;

            /* ����������&����ʱ�� */
            case SET_VOICE_CTRL_SENSITIVITY:
            case SET_VOICE_CTRL_CLOSE_TIME: {
                uint8_t wifiCmd = (cmd == SET_VOICE_CTRL_SENSITIVITY ? SetVoiceSensitivity_MtoU_D : SetVoiceCloseTime_MtoU_D);

                if(id == WHOLE_BROADCAST_ID) {
                    /* �㲥֪ͨȫ���ֻ���ϵͳ */
                    WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID, BASIC_MSG, UNIT_CTRL, cmd,null,protPara[4]));
                    /* �㲥֪ͨȫWIFI����ϵͳ */
                    WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0, wifiCmd, protPara[4], null));
                } else {
                    if(idUnitType == kType_Unit_Wifi)
                        WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id, wifiCmd, protPara[4], null));
                    else if(idUnitType == kType_Unit_Wired)
                        WiredUnit.transmit(Protocol.conference(&confProt,id, BASIC_MSG, UNIT_CTRL, cmd,null,protPara[4]));
                }
            }
            break;

            /* �������� */
            case SET_LANGUAGE(Chinese):
            case SET_LANGUAGE(English):
            case SET_LANGUAGE(Russian):
            case SET_LANGUAGE(French): {
                Protocol.conference(&confProt,UNIT_BROADCAST_ID,BASIC_MSG,UNIT_CTRL,cmd,null,null);

                if(id == UNIT_BROADCAST_ID) {
                    SysInfo.config->language = (Language_EN)(cmd - 0xF0);
                    /* �ظ���������ȷ�� */
                    ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
                    /* �㲥����Ԫ */
                    WiredUnit.transmit(&confProt);
                    WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,null, SetLanguage_MtoU_G, SysInfo.config->language, null));
                    /* ������Ļ���� */
                    Screen.setLanguage(SysInfo.config->language);
                }
                /* ���������õ�Ԫ���� */
                else {
                    if(idUnitType == kType_Unit_Wifi)
                        WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id, SetLanguage_MtoU_G, (cmd - 0xF0), null));
                    else if(idUnitType == kType_Unit_Wired)
                        WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,UNIT_CTRL,cmd,null,null));
                }


            }
            break;

            }
        }
        break;

        default:
            break;
        }


    }
    break;
    /* ״̬��Ϣ 0x82*/
    case STATE_MSG: {
        switch(id) {
        case MODE_BROADCAST_ID:
        case WIFI_MODE_BROADCAST_ID: {
            uint8_t mode = protPara[0], num = protPara[1];

            if(mode >= kMode_Mic_Fifo && mode <= kMode_Mic_Apply)
                SysInfo.config->micMode = (MicMode_EN)mode;

            /* �������ߵ�Ԫ */
            if(id == MODE_BROADCAST_ID && num >= 1 && num <= WIRED_UNIT_MAX_ALLWO_OPEN) {
                SysInfo.config->wiredAllowOpen = num;
                SysInfo.config->wiredAllowWait = num;
            }

            /* ����WIFI��Ԫ */
            else if(id == WIFI_MODE_BROADCAST_ID && num >= 1 && num <= WIFI_UNIT_MAX_ALLWO_OPEN) {
                SysInfo.config->wifiAllowOpen = num;
                SysInfo.config->wifiAllowWait = num;
            }

            Database.saveSpecify(kType_Database_SysCfg,null);


            /* �ر����е�Ԫ */
            Conference_CloseAllMic(kType_Unit_Wired,aChairman);
            Conference_CloseAllMic(kType_Unit_Wired,aRepresentative);
            Conference_CloseAllMic(kType_Unit_Wifi,aChairman);
            Conference_CloseAllMic(kType_Unit_Wifi,aRepresentative);

            /* ���������� */
            Conference_ClearApplyMic();

            /* �㲥ģʽ������ */
            /* ���͵���Ԫ */
            WiredUnit.transmit(Protocol.conference(&confProt,MODE_BROADCAST_ID,STATE_MSG,SysInfo.config->micMode,SysInfo.config->wiredAllowOpen,null,null));
            WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,ChangeMicManage_MtoU_G,SysInfo.config->micMode,SysInfo.config->wifiAllowOpen));
            /* �ظ��ⲿ�����豸ȷ��״̬ */
            ExternalCtrl.transmit(EX_CTRL_DEST,  \
                                  Protocol.conference(&confProt,MODE_BROADCAST_ID,STATE_MSG,SysInfo.config->micMode,SysInfo.config->wiredAllowOpen,null,null));
            ExternalCtrl.transmit(EX_CTRL_DEST,  \
                                  Protocol.conference(&confProt,WIFI_MODE_BROADCAST_ID,STATE_MSG,SysInfo.config->micMode,SysInfo.config->wifiAllowOpen,null,null));
        }
        break;

        case NOW_DATE_TIME_ID: {
            TimePara_S *time;
            time = MALLOC(sizeof(TimePara_S));

            time->year = protPara[0];
            time->month = protPara[1];
            time->day = protPara[2];
            time->hour = protPara[3];
            time->min = protPara[4];
            time->sec = (&notify->exDataHead)[0];
            /* ��������ʱ�� */
            Time.setNow(time);

            /* �·�ʱ�䵽��Ԫ */
            Conference_UpdataUnitTime(time);

            FREE(time);
        }
        break;

        }
    }
    break;
    /* ͨѶ��Ϣ 0x86*/
    case PC_MSG: {
        uint8_t cmd = protPara[0];

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
        case CFG_IP: {
            SysInfo.config->ip[0] = protPara[2];
            SysInfo.config->ip[1] = protPara[3];
            SysInfo.config->ip[2] = protPara[4];
            SysInfo.config->ip[3] = (&notify->exDataHead)[0];

            Database.saveSpecify(kType_Database_SysCfg,null);
        }
        break;
        /* 0x08 */
        case CFG_MASK: {
            SysInfo.config->mask[0] = protPara[2];
            SysInfo.config->mask[1] = protPara[3];
            SysInfo.config->mask[2] = protPara[4];
            SysInfo.config->mask[3] = (&notify->exDataHead)[0];

            Database.saveSpecify(kType_Database_SysCfg,null);
        }
        break;
        /* 0x09 */
        case CFG_GW: {
            SysInfo.config->gateWay[0] = protPara[2];
            SysInfo.config->gateWay[1] = protPara[3];
            SysInfo.config->gateWay[2] = protPara[4];
            SysInfo.config->gateWay[3] = (&notify->exDataHead)[0];

            Database.saveSpecify(kType_Database_SysCfg,null);
        }
        break;
        /* 0x0F */
        case QUERY_HOST: {
            uint8_t data[4];

            /* �ظ���ǰ����ģʽ�����ߵ�Ԫ���Ͳ�� */
            ExternalCtrl.transmit(notify->nSrc,  \
                                  Protocol.conference(&confProt,MODE_BROADCAST_ID,STATE_MSG,SysInfo.config->micMode,SysInfo.config->wiredAllowOpen,null,null));
            /* �ظ���ǰ����ģʽ��WIFI��Ԫ���Ͳ�� */
            ExternalCtrl.transmit(notify->nSrc,  \
                                  Protocol.conference(&confProt,WIFI_MODE_BROADCAST_ID,STATE_MSG,SysInfo.config->micMode,SysInfo.config->wifiAllowOpen,null,null));

            /* �ظ���ǰIP��ַ�����롢���� */
            memcpy(data,SysInfo.config->ip,4);
            ExternalCtrl.transWithExData(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,CFG_IP,0x04,data[0],data[1] << 8 | data[2]),1,&data[3]);
            memcpy(data,SysInfo.config->gateWay,4);
            ExternalCtrl.transWithExData(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,CFG_GW,0x04,data[0],data[1] << 8 | data[2]),1,&data[3]);
            memcpy(data,SysInfo.config->mask,4);
            ExternalCtrl.transWithExData(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,CFG_MASK,0x04,data[0],data[1] << 8 | data[2]),1,&data[3]);


            /* �ظ���ǰ���� */
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,UNIT_BROADCAST_ID,BASIC_MSG,UNIT_CTRL,SET_LANGUAGE(SysInfo.config->language),null,null));
        }
        break;

        /* ����ͷԤ��λ���� */
        case CAMERA_POSITION: {
            if(id == 0) {
                SysInfo.config->panorPos = protPara[2];
                SysInfo.config->panorCh = protPara[3];
                Database.saveSpecify(kType_Database_SysCfg,null);
            } else if(idUnitType == kType_Unit_Wired) {
                UnitInfo.wired[id].config->camPos = protPara[2];
                UnitInfo.wired[id].config->camCh = protPara[3];
                Database.saveSpecify(kType_Database_WiredCfg,id);
            } else if(idUnitType == kType_Unit_Wifi) {
                UnitInfo.wifi[id].config->camPos = protPara[2];
                UnitInfo.wifi[id].config->camCh = protPara[3];
                Database.saveSpecify(kType_Database_WifiCfg,null);
            }
        }
        break;

        /* ����ͷ���� 0x14 */
        case CAMERA_CONTROL: {
            uint8_t *data,len = protPara[1];

            data = MALLOC(len);
            memcpy(&data[0],&protPara[2],3);
            memcpy(&data[3],&notify->exDataHead,len - 3);

            Camera.cmdSend(data,len);

            FREE(data);
        }
        break;

        /* �·��Զ������� 0x24 */
        case CUSTOM_VOTE_ITEM: {
            uint8_t item = protPara[1];

            if(item >= 1 && item <= 5) {

                memset(SysInfo.state.voteCustomItem[item - 1],0,VOTE_CUSTOM_ITEM_LEN);

                memcpy(&SysInfo.state.voteCustomItem[item - 1][0],&protPara[2],3);
                if(notify->exLen > 0)
                    memcpy(&SysInfo.state.voteCustomItem[item - 1][3],&notify->exDataHead,notify->exLen);

                /* WIFI��Ԫ���Զ����� */
                WifiUnit.transWithExData(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,PCUpdateCustomTerm_MtoU_G,item,0), \
                                         3 + notify->exLen,SysInfo.state.voteCustomItem[item - 1]);

                /* ֱ��ת�������ߵ�Ԫ */
                WiredUnit.transWithExData(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,CUSTOM_VOTE_ITEM,item, \
                                          protPara[2],(protPara[3] << 8) | protPara[4]),notify->exLen,&notify->exDataHead);
            }
        }
        break;

        /* �����ID   	0x2F */
        case PC_EDIT_ID: {
            if(protPara[1] == PC_START_EDIT_ID) {
                /* ��ʼ��ID */
                uint8_t startID = (notify->prot.conference.sec == 0 ? 1 : notify->prot.conference.sec);

                SysInfo.state.wiredCurEditID = startID;
                SysInfo.state.wifiCurEditID = WIFI_ID(startID);
                Conference_ChangeSysMode(kMode_EditID);
            }

            else if(protPara[1] == PC_STOP_EDIT_ID) {
                Conference_ChangeSysMode(kMode_Conference);
            }
        }
        break;

        /* ɨ�����ߵ�ԪID 0x30 */
        case SCAN_ONLINE_UNIT: {
            uint8_t cmd,ch;
            /* �ϴ��������ߵ�Ԫ */
            for(id = 1; id <= WIRED_UNIT_MAX_ONLINE_NUM; id++) {
                if(UnitInfo.wired[id].online) {
                    ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,id,PC_MSG,ONLINE_UNIT,UnitInfo.wired[id].attr,null,null));
                }
            }
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,SCAN_ONLINE_UNIT,SCAN_UNIT_END,kType_Unit_Wired, \
                                  (OnlineNum.wiredChm + OnlineNum.wiredRps)));

            /* �ϴ�����WIFI��Ԫ */
            for(id = 1; id <= WIFI_UNIT_MAX_ONLINE_NUM; id++) {
                if(UnitInfo.wifi[id].online) {
                    ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WIFI_ID(id),PC_MSG,ONLINE_UNIT,UnitInfo.wifi[id].attr,null,null));
                }
            }
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,SCAN_ONLINE_UNIT,SCAN_UNIT_END,kType_Unit_Wifi, \
                                  (OnlineNum.wifiChm + OnlineNum.wifiRps)));

            /* �ϴ��������ߵ�Ԫ��Ͳ״̬ */
            for(id = 1; id <= WIRED_UNIT_MAX_ONLINE_NUM; id++) {
                if(UnitInfo.wired[id].online) {
                    if(UnitInfo.wired[id].micSta == kStatus_UnitMic_Open) {
                        cmd = (UnitInfo.wired[id].attr == aChairman) ? CHM_OPEN_MIC : RPS_OPEN_MIC;
                        ch = UnitInfo.wired[id].channel;
                        ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,cmd,ch,null));
                    } else if(UnitInfo.wired[id].micSta == kStatus_UnitMic_Wait) {
                        ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,MIC_WAIT,null,null));
                    } else if(UnitInfo.wired[id].micSta == kStatus_UnitMic_Apply) {
                        ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,APPLY_OPEN_MIC_ID,APPLY_MSG,id << 8,id & 0xFF,null,null));
                    }
                }
            }

            /* �ϴ�����WIFI��Ԫ��Ͳ״̬ */
            for(id = 1; id <= WIFI_UNIT_MAX_ONLINE_NUM; id++) {
                if(UnitInfo.wifi[id].online) {
                    if(UnitInfo.wifi[id].micSta == kStatus_UnitMic_Open) {
                        cmd = (UnitInfo.wifi[id].attr == aChairman) ? CHM_OPEN_MIC : RPS_OPEN_MIC;
                        ch = UnitInfo.wifi[id].channel;
                        ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),BASIC_MSG,CONFERENCE_MODE,cmd,ch,null));
                    } else if(UnitInfo.wifi[id].micSta == kStatus_UnitMic_Wait) {
                        ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),BASIC_MSG,CONFERENCE_MODE,MIC_WAIT,null,null));
                    } else if(UnitInfo.wifi[id].micSta == kStatus_UnitMic_Apply) {
                        uint16_t applyId = WIFI_ID(id);
                        ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,APPLY_OPEN_MIC_ID,APPLY_MSG,applyId << 8,applyId & 0xFF,null,null));
                    }
                }
            }

        }
        break;

        /* �ⲿ���Ʋ�ѯ��ǰ״̬ */
        case QUERY_PRIOR_SIGN:
        case QUERY_PRIOR_VOTE:
        case QUERY_PRIOR_SCAN: {
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,cmd,0x01,SysInfo.state.sysMode,null));
        }
        break;
        }
    }
    break;

    /* �������� */
    case NAMEPLATE_CONTENT:
    case NAMEPLATE_COMP:
    case NAMEPLATE_POS:
    case NAMEPLATE_NAME:
    case NAMEPLATE_UPDATE:
    case CONSTANT_NOTIFY: {
        uint8_t *data,len,isSave = 1;

        if(protType == NAMEPLATE_CONTENT)
            /* ��Ϣ������ */
            isSave = 2;

        if(id == WHOLE_BROADCAST_ID) {
            len = protPara[0];
            data = MALLOC(len);

            memcpy(data,&protPara[1],len);
            WifiUnit.transWithExData(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,SetUnitPrintMsg_MtoU_G,isSave,null),len,data);
            WiredUnit.transWithExData(Protocol.conference(&confProt,id,protType,protPara[0],protPara[1],protPara[2],(protPara[3] << 8) | protPara[4]),notify->exLen,&notify->exDataHead);

            FREE(data);
        } else {
            if(idUnitType == kType_Unit_Wifi) {
                len = protPara[0];
                data = MALLOC(len);

                memcpy(data,&protPara[1],len);
                WifiUnit.transWithExData(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,SetUnitPrintMsg_MtoU_G,isSave,null),len,data);

                FREE(data);
            } else if(idUnitType == kType_Unit_Wired) {
                WiredUnit.transWithExData(Protocol.conference(&confProt,id,protType,protPara[0],protPara[1],protPara[2],(protPara[3] << 8) | protPara[4]),notify->exLen,&notify->exDataHead);
            }
        }
    }
    break;

    /* ��Ƶ���� 0xB0*/
    case AUDIO_MATRIX: {
        uint8_t ph = protPara[0],pl = protPara[1], *exdata, i;

        switch(ph) {
        /* DSPģʽ */
        case DSP_MODE: {
            uint8_t output,input;

            switch(pl) {
            /* ���߼�����ģʽ */
            case DSP_MODE_WIRE:
            case DSP_MODE_WIFI: {
                /* ����DSP */
                SysInfo.config->dspMode = pl;
                Dsp.setMode((DspSysMode_E)SysInfo.config->dspMode);
            }
            break;

            /* ����ģʽ */
            case DSP_MODE_PARTITION: {
                /* ����DSP */
                SysInfo.config->dspMode = pl;
                Dsp.setMode((DspSysMode_E)SysInfo.config->dspMode);

                for(output = DSP_OUTPUT_CH1; output <= DSP_OUTPUT_CH16 ; output++) {
                    /* �ָ���Ͳ��������Ϊ��� */
                    SysInfo.config->dsp[output].inputVol[DSP_ALL_MIC_MIX] = 0x1F;
                    /* ���ö�Ӧ����ͨ������ */
                    for(input = 0; input < 7; input++)
                        Dsp.chInputSrc((DspOutput_E)output,(DspInputSrc_E)input,(DspVolume_E)(31 - SysInfo.config->dsp[output].inputVol[input]));
                }

            }
            break;

            /* ͬ��ģʽ  */
            case DSP_MODE_SI: {
                /* ����DSP */
                SysInfo.config->dspMode = pl;
                Dsp.setMode((DspSysMode_E)SysInfo.config->dspMode);

                /* ����Out1ͨ������ */
                for(input = 0; input < 7; input++)
                    Dsp.chInputSrc((DspOutput_E)DSP_OUTPUT_CH1,(DspInputSrc_E)input,(DspVolume_E)(31 - SysInfo.config->dsp[DSP_OUTPUT_CH1].inputVol[input]));
            }
            break;

            default:
                break;

            }

            /* �ر����е�Ԫ */
            Conference_CloseAllMic(kType_Unit_Wired,aChairman);
            Conference_CloseAllMic(kType_Unit_Wired,aRepresentative);
            Conference_CloseAllMic(kType_Unit_Wifi,aChairman);
            Conference_CloseAllMic(kType_Unit_Wifi,aRepresentative);

            /* ���������� */
            Conference_ClearApplyMic();

            /* �������ݵ�FLASH */
            Database.saveSpecify(kType_Database_SysCfg,null);
            /* �ظ��յ� */
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,0xFF,null,null,null));

        }
        break;

        /* ��Ԫ������������ */
        case DSP_UNIT_OUT_CFG: {
            UnitInfo_S *unitInfo;
            uint8_t saveType;

            if(idUnitType == kType_Unit_Wired) {
                unitInfo = &UnitInfo.wired[id];
                saveType = kType_Database_WiredCfg;
            } else if(idUnitType == kType_Unit_Wifi) {
                unitInfo = &UnitInfo.wifi[id];
                saveType = kType_Database_WifiCfg;
            } else
                break;

            switch(pl) {
            /* ����ͨ������ */
            case 0x01: {
                memcpy(&unitInfo->config->chVol[0],&protPara[2],3);
                memcpy(&unitInfo->config->chVol[3],&notify->exDataHead,13);
                if(unitInfo->micSta == kStatus_UnitMic_Open) {
                    Conference_DspUnitCtrl(id,idUnitType,tOpenMic,unitInfo->channel);
                }
            }
            break;
            /* ��ԪEQ */
            case 0x02: {
                uint8_t i;

                exdata = &notify->exDataHead;

                memcpy(&unitInfo->config->eqFreq[0],&protPara[2],3);
                memcpy(&unitInfo->config->eqFreq[3],&exdata[0],2);
                memcpy(&unitInfo->config->eqVol[0],&exdata[2],5);

                for(i = 0; i < 5; i++) {
                    uint8_t para;

                    if(!(unitInfo->config->eqFreq[i] >= 0 && unitInfo->config->eqFreq[i] <= 3))
                        unitInfo->config->eqFreq[i] = 0;

                    if((!(unitInfo->config->eqVol[i] >= 0 && unitInfo->config->eqVol[i] <= 24)))
                        unitInfo->config->eqVol[i] = 24;

                    /* ����ԭȫ���ֻ���Э���·�����EQ */
                    para = ((unitInfo->config->eqFreq[i]) << 5) | (24 - unitInfo->config->eqVol[i]);


                    if(idUnitType == kType_Unit_Wired) {
                        WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,UNIT_CTRL,SET_UNTI_EQ,0x00,(i + 1) << 12 | para));
                    } else if(idUnitType == kType_Unit_Wifi) {
                        WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,SetUnitEQ_MtoU_G,i + 1,para));
                    }

                }
            }
            break;
            /* ������ */
            case 0x03: {
                if(!(protPara[2] >= 0 && protPara[2] <= 8))
                    break;

                unitInfo->config->sensitivity = protPara[2];

                if(idUnitType == kType_Unit_Wired) {
                    WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,UNIT_CTRL,0x43,0x00,protPara[2]));
                } else if(idUnitType == kType_Unit_Wifi) {
                    WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,SetUnitMICSensitivity_MtoU_D,protPara[2],0));
                }
            }
            break;
            }

            /* �ظ��յ� */
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,0xFF,null,null,null));
            /* �������ݵ�FLASH */
            Database.saveSpecify(saveType,id);
        }
        break;

        /* ��ͨ��������� */
        case DSP_NOR_OUT_CFG: {
            DspOutput_E dspOutput;

            if(!(pl >= 0x01 && pl <= 0x06 && id == WHOLE_BROADCAST_ID))
                break;

            /* ת��ΪDSP���ͨ��ö��  	     */
            dspOutput = (DspOutput_E)(pl + DSP_OUTPUT_CH16);

            switch(protPara[2]) {
            /* �������� */
            case 0x01: {
                /* �жϲ�������ͨ���ͨ������ֵ */
                SysInfo.config->dsp[dspOutput].vol = protPara[3] > 31 ? 31 : protPara[3];

                /* д��DSP */
                Dsp.norOutput(dspOutput,DSP_OUTPUT_VOLUME,(DspEqPart_E)null,(uint8_t)(31 - SysInfo.config->dsp[dspOutput].vol));
            }
            break;
            /* ����10��EQ���� */
            case 0x02: {
                exdata = &notify->exDataHead;
                memcpy(&SysInfo.config->dsp[dspOutput].eqVol[0],&protPara[3],2);
                memcpy(&SysInfo.config->dsp[dspOutput].eqVol[2],&exdata[0],8);
                for(i = 0; i < 10; i++) {
                    /* ���ÿһ��EQֵ�Ƿ�Ϸ� */
                    SysInfo.config->dsp[dspOutput].eqVol[i] = SysInfo.config->dsp[dspOutput].eqVol[i] > 20 ? 20 : SysInfo.config->dsp[dspOutput].eqVol[i];

                    /* д��DSP */
                    Dsp.norOutput(dspOutput,DSP_OUTPUT_EQ,(DspEqPart_E)i,(uint8_t)SysInfo.config->dsp[dspOutput].eqVol[i]);
                }
            }
            break;
            /* ���ö�Ӧ�������� */
            case 0x03: {
                exdata = &notify->exDataHead;
                memcpy(&SysInfo.config->dsp[dspOutput].inputVol[0],&protPara[3],2);
                memcpy(&SysInfo.config->dsp[dspOutput].inputVol[2],&exdata[0],5);

                for(i = 0; i < 7; i++) {
                    /* ���ÿһ�������Ƿ�Ϸ� */
                    SysInfo.config->dsp[dspOutput].inputVol[i] =  \
                            SysInfo.config->dsp[dspOutput].inputVol[i] > 31 ? 31 : SysInfo.config->dsp[dspOutput].inputVol[i];

                    /* д��DSP */
                    Dsp.inputSrc(dspOutput,(DspInputSrc_E)i,(DspVolume_E)(31 - SysInfo.config->dsp[dspOutput].inputVol[i]));
                }
            }
            break;
            /* �����´� */
            case 0x04: {
                /* Ŀǰֻ֧���������߻�Ͳ�´����� */
                if(dspOutput == DSP_OUTPUT_DOWN_WIFI) {
//                    SysInfo.config->dsp[DSP_OUTPUT_DOWN_WIFI].downTrans = protPara[3];
                    SysInfo.state.wifiAudDownward = protPara[3];
                    SlaveMcu.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,DSP_NOR_OUT_CFG,0x06,0x04,protPara[3] << 8));
                }
            }
            break;
            }

            /* �ظ��յ� */
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,0xFF,null,null,null));

            Database.saveSpecify(kType_Database_SysCfg,null);

        }
        break;

        /* ����ͨ����������� */
        case DSP_CHANNEL_OUT_CFG: {

            if(!(pl >= 0x00 && pl <= 0x0F && id == WHOLE_BROADCAST_ID))
                break;

            switch(protPara[2]) {
            case 0x01: {
                /* �жϲ�����������ͨ������ֵ */
                SysInfo.config->dsp[pl].vol = protPara[3] > 31 ? 31 : protPara[3];
                /* д��DSP */
                Dsp.chOutput((DspOutput_E)pl,DSP_OUTPUT_VOLUME,(DspEqPart_E)null,(uint8_t)(31 - SysInfo.config->dsp[pl].vol));
            }
            break;

            case 0x02: {
                exdata = &notify->exDataHead;
                memcpy(&SysInfo.config->dsp[pl].eqVol[0],&protPara[3],2);
                memcpy(&SysInfo.config->dsp[pl].eqVol[2],&exdata[0],8);

                for(i = 0; i < 10; i++) {
                    /* ���ÿһ��EQֵ�Ƿ�Ϸ� */
                    SysInfo.config->dsp[pl].eqVol[i] = SysInfo.config->dsp[pl].eqVol[i] > 20 ? 20 : SysInfo.config->dsp[pl].eqVol[i];

                    /* д��DSP */
                    Dsp.chOutput((DspOutput_E)pl,DSP_OUTPUT_EQ,(DspEqPart_E)i,(DspEqValue_E)SysInfo.config->dsp[pl].eqVol[i]);
                }
            }
            break;

            case 0x03: {
                exdata = &notify->exDataHead;
                memcpy(&SysInfo.config->dsp[pl].inputVol[0],&protPara[3],2);
                memcpy(&SysInfo.config->dsp[pl].inputVol[2],&exdata[0],5);

                for(i = 0; i < 7; i++) {
                    /* ���ÿһ�������Ƿ�Ϸ� */
                    SysInfo.config->dsp[pl].inputVol[i] = SysInfo.config->dsp[pl].inputVol[i] > 31 ? 31 : SysInfo.config->dsp[pl].inputVol[i];

                    /* д��DSP */
                    Dsp.chInputSrc((DspOutput_E)pl,(DspInputSrc_E)i,(DspVolume_E)(31 - SysInfo.config->dsp[pl].inputVol[i]));
                }
            }
            break;

            case 0x04: {
                SysInfo.config->dsp[pl].dly = protPara[3] > 100 ? 100 : protPara[3];

                /* д��DSP */
                Dsp.chOutput((DspOutput_E)pl,DSP_OUTPUT_DELAY,(DspEqPart_E)null,SysInfo.config->dsp[pl].dly);
            }
            break;
            }

            /* �ظ��յ� */
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,0xFF,null,null,null));

            Database.saveSpecify(kType_Database_SysCfg,null);
        }
        break;

        }
    }
    break;
    /* ��Ƶ�����ѯ 0xB1*/
    case AUDIO_MATRIX_INQUIRE: {
        uint8_t ph = protPara[0];

        switch(ph) {
        /* �ⲿ����������ѯDSPģʽ */
        case DSP_MODE: {
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,DSP_MODE,SysInfo.config->dspMode,null,null));
        }
        break;

        /* �ⲿ����������ѯ��Ԫ������������ */
        case DSP_UNIT_OUT_CFG: {
            UnitCfg_S *uCfg;

            if(idUnitType == kType_Unit_Wired) {
                uCfg = UnitInfo.wired[id].config;
            } else if(idUnitType == kType_Unit_Wifi) {
                uCfg = UnitInfo.wifi[id].config;
            } else
                break;

            /* �ظ���Ԫ����ͨ������ */
            ExternalCtrl.transWithExData(notify->nSrc,Protocol.conference(&confProt,id,AUDIO_MATRIX,DSP_UNIT_OUT_CFG,0x01,  \
                                         uCfg->chVol[0],uCfg->chVol[1] << 8 | uCfg->chVol[2]),13,&uCfg->chVol[3]);
            /* �ظ���ԪEQƵ�ʼ����� */
            ExternalCtrl.transWithExData(notify->nSrc,Protocol.conference(&confProt,id,AUDIO_MATRIX,DSP_UNIT_OUT_CFG,0x02,  \
                                         uCfg->eqFreq[0],uCfg->eqFreq[1] << 8 | uCfg->eqFreq[2]),7,&uCfg->eqFreq[3]);
            /* �ظ���Ԫ������ */
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,id,AUDIO_MATRIX,DSP_UNIT_OUT_CFG,0x03,uCfg->sensitivity,null));

        }
        break;

        /* �ⲿ����������ѯ��ͨ��������� */
        case DSP_NOR_OUT_CFG: {
            uint8_t i;

            if(id != WHOLE_BROADCAST_ID)
                break;

            switch(protPara[2]) {
            case 0x01: {
                for(i = 1; i <= 6; i++) {
                    /* �ظ�6�����ͨ������ */
                    ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,DSP_NOR_OUT_CFG,  \
                                          i, 0x01, SysInfo.config->dsp[i + DSP_OUTPUT_CH16].vol << 8 | 0x00));
                }
            }
            break;

            case 0x02: {
                uint8_t *eqvol;

                for(i = 1; i <= 2; i++) {
                    eqvol = SysInfo.config->dsp[i + DSP_OUTPUT_CH16].eqVol;
                    /* �ظ�LineOut1 2��EQƵ�ʼ�EQֵ */
                    ExternalCtrl.transWithExData(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,DSP_NOR_OUT_CFG,  \
                                                 i, 0x02, eqvol[0] << 8 | eqvol[1]),8,&eqvol[2]);
                }

            }
            break;

            case 0x03: {
                uint8_t *inputvol;

                for(i = 1; i <= 6; i++) {
                    inputvol = SysInfo.config->dsp[i + DSP_OUTPUT_CH16].inputVol;
                    /* �ظ�6�����ͨ����Ӧ����˵����� */
                    ExternalCtrl.transWithExData(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,DSP_NOR_OUT_CFG,  \
                                                 i, 0x03, inputvol[0] << 8 | inputvol[1]),5,&inputvol[2]);
                }

            }
            break;

            case 0x04: {
                /* �ظ��´�����״̬ */
                ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,DSP_NOR_OUT_CFG,  \
                                      DSP_OUTPUT_DOWN_WIFI - DSP_OUTPUT_CH16, 0x04, SysInfo.state.wifiAudDownward << 8 | 0x00));
            }
            break;
            }

        }
        break;

        /* �ⲿ����������ѯ����ͨ����������� */
        case DSP_CHANNEL_OUT_CFG: {
            uint8_t i;

            if(id != WHOLE_BROADCAST_ID)
                break;

            switch(protPara[2]) {
            case 0x01: {
                for(i = 0; i < 16; i++) {
                    /* �ظ�16������ͨ������ */
                    ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,id,AUDIO_MATRIX,DSP_CHANNEL_OUT_CFG,  \
                                          i, 0x01,SysInfo.config->dsp[i].vol << 8 | 0x00));
                }
            }
            break;

            case 0x02: {
                uint8_t *eqvol;

                for(i = 0; i < 16; i++) {
                    eqvol = SysInfo.config->dsp[i].eqVol;
                    /* �ظ�16������ͨ����EQƵ�ʼ�EQֵ */
                    ExternalCtrl.transWithExData(notify->nSrc,Protocol.conference(&confProt,id,AUDIO_MATRIX,DSP_CHANNEL_OUT_CFG,  \
                                                 i, 0x02, eqvol[0] << 8 | eqvol[1]),8,&eqvol[2]);
                }

            }
            break;

            case 0x03: {
                uint8_t *inputvol;

                for(i = 0; i < 16; i++) {
                    inputvol = SysInfo.config->dsp[i].inputVol;
                    /* �ظ�16������ͨ����Ӧ����˵����� */
                    ExternalCtrl.transWithExData(notify->nSrc,Protocol.conference(&confProt,id,AUDIO_MATRIX,DSP_CHANNEL_OUT_CFG,  \
                                                 i, 0x03, inputvol[0] << 8 | inputvol[1]),5,&inputvol[2]);
                }

            }
            break;

            case 0x04: {
                /* �ظ�16������ͨ������ʱ */
                for(i = 0; i < 16; i++) {
                    /* �ظ�16������ͨ����Ӧ����˵����� */
                    ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,id,AUDIO_MATRIX,DSP_CHANNEL_OUT_CFG,  \
                                          i, 0x04,SysInfo.config->dsp[i].dly << 8 | 0x00));
                }
            }
            break;
            }

        }
        break;

        }
    }
    break;

    default:
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
    Log.d("Wifi Unit Msg: id = %d, cmd = %d,ph = %d , pl = %d",id,cmd,ph,pl);
    if(notify->exLen) {
        uint8_t i;

        printf(" exData{ ");
        for(i=0; i<notify->exLen; i++)
            printf(" %X,",(&notify->exDataHead)[i]);
        printf(" } ");
    }
    printf("\r\n");
#endif

    switch(cmd) {

	/* ��Ԫ�ظ���ѯ */
    case RepPollUnitl_UtoM_D: {
        switch(SysInfo.state.sysMode) {
        case kMode_Sign: {
            if((ph & 0x01) && !UnitInfo.wifi[id].sign) {
                UnitInfo.wifi[id].sign = true;
                SysInfo.state.currentSignNum++;
                Conference_SignInstruction(id,kType_Unit_Wifi,WIFI_CMD(RepPollUnitl_UtoM_D,kMode_Sign,0),null,null);
                Conference_SignInstruction(null,(UnitType_EN)null,WIFI_CMD(EnterSignMode_MtoU_G,0,1),SysInfo.state.totalSignNum,SysInfo.state.currentSignNum);
            }
        }
        break;

        case kMode_Vote: {
            uint8_t unitVote = pl;

            /* ���ͶƱ����Ϊ��Ҫ��ǩ��������Ԫδ����ǩ�� */
//            if(IS_VOTE_NEED_SIGN(SysInfo.voteMode) && UnitInfo.wifi[id].sign != true)
//                break;
            if(IS_VOTE_FIRST_VALID(SysInfo.state.voteMode) && UnitInfo.wifi[id].vote != null)
                break;


            if(UnitInfo.wifi[id].vote != unitVote) {
                UnitInfo.wifi[id].vote = unitVote;
                ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),BASIC_MSG,VOTE_MODE,unitVote,null,null));
            }
        }
        break;

        default:
            break;
        }
    }
    break;

    /* ��Ԫ���� */
    case RepApplyEnterSystm_MtoU_D: {

        /* ��Ͳ�����Ѿ�����(�Ȱβ�) */
        if(UnitInfo.wifi[id].online) {
            /* �·�MIC״̬ */
            switch(UnitInfo.wifi[id].micSta) {
            case kStatus_UnitMic_Open: {
                WifiUnit.transWithExData(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,MasterOpenOrCloseMic_MtoU_D,0,null),1,&UnitInfo.wifi[id].channel);
            }
            break;
            case kStatus_UnitMic_Apply:
            case kStatus_UnitMic_Wait: {
                WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,RepApplyOpenMic_MtoU_D,3,null));
            }
            break;

            default:
                break;
            }

            Log.d("WifiUnit(%s) id = %d reonline\r\n",UnitInfo.wifi[id].attr == aChairman ? "CHM":"RPS",id);
        } else {
            if(UnitInfo.wifi[id].attr == aChairman)
                OnlineNum.wifiChm++;
            else if(UnitInfo.wifi[id].attr == aRepresentative)
                OnlineNum.wifiRps++;

            /* ������Ļ��ʾ���� */
            Conference_ScreenUpdataUnitNum();

            UnitInfo.wifi[id].online = true;

            Log.d("WifiUnit(%s) id = %d online ( Chm num = %d , Rps num = %d )\r\n",UnitInfo.wifi[id].attr == aChairman ? "CHM":"RPS",id,OnlineNum.wifiChm,OnlineNum.wifiRps);
        }

        /* ֪ͨ�ⲿ���ƻ�Ͳ���� */
        ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),PC_MSG,UNIT_LOGIN,0x01,null,null));

        NewUpline.wifiUnit = true;

        /* ͬ��EQ �����Ȳ��� */
        WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,RepReadUnitMICSensitivity_UtoM_D,null,null));
        WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,ReadUnitEQ_MtoU_D,null,null));

    }
    break;

    /* ��Ԫ���� */
    case MasterAskforReenterSysMtoU_D: {
        if(UnitInfo.wifi[id].attr == aChairman) {
            Conference_MicControl(id, kType_Unit_Wifi, WIFI_CMD(UnitCloseMic_UtoM_D,0,0));
            OnlineNum.wifiChm--;
        } else if(UnitInfo.wifi[id].attr == aRepresentative) {
            Conference_MicControl(id, kType_Unit_Wifi, WIFI_CMD(UnitCloseMic_UtoM_D,0,0));
            OnlineNum.wifiRps--;
        }

        /* ֪ͨ�ⲿ���ƻ�Ͳ���� */
        ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),PC_MSG,UNIT_LOGOUT,0x01,null,null));

        /* ������Ļ��ʾ���� */
        Conference_ScreenUpdataUnitNum();

        Log.d("WifiUnit(%s) id = %d offline ( Chm num = %d , Rps num = %d )\r\n",UnitInfo.wired[id].attr == aChairman ? "CHM":"RPS",id,OnlineNum.wiredChm,OnlineNum.wiredRps);
    }
    break;

    /*��Ԫ�����뿪��Ͳ*/
    case ApplyOpenMic_UtoM_D:
        Conference_MicControl(id, kType_Unit_Wifi, WIFI_CMD(cmd,0,0));
        break;

    /*��Ԫ���ؿ���Ͳ*/
    case UnitCloseMic_UtoM_D:
        Conference_MicControl(id, kType_Unit_Wifi, WIFI_CMD(cmd,0,0));
        break;

    /*��Ԫ���˳��ȴ�����*/
    case UnitGetoutWaitQuenen_UtoM_D:
        Conference_MicControl(id, kType_Unit_Wifi, WIFI_CMD(cmd,0,0));
        break;

    /*��ϯ��ִ������Ȩ*/
    case ChairExecutePriority_UtoM_D:
        Conference_ChairmanPriority(kType_Unit_Wifi,id);
        break;

    /*��ϯ��������ͬ����������Ͳ*/
    case ChairAgreeOpenOrNot_UtoM_D:
        Conference_MicControl(0, kType_Unit_Wifi, WIFI_CMD(ChairAgreeOpenOrNot_UtoM_D,ph,0));
        break;

    /*��Ԫ����������    */
    case UnitApplyWater_UtoM_D: {
        ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),BASIC_MSG,CONFERENCE_MODE,pl,null,null));
    }
    break;


    /*�ظ������ﵥԪ����ǩ��   */
    case RepSupplementSign_UtoM_D:
        Conference_SignInstruction(id,kType_Unit_Wifi,WIFI_CMD(RepSupplementSign_UtoM_D,0,0), null, null);
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

        Log.d("Wifi confirm ID = %d ip : %d.%d.%d.%d  mac : %X-%X-%X-%X-%X-%X \r\n",confirmID - 0x3000,ip[0],ip[1],ip[2],ip[3],mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
        if(confirmID == SysInfo.state.wifiCurEditID) {
            memcpy(&UnitInfo.wifi[id].ip,ip,NETWORK_IP_SIZE);
            memcpy(&UnitInfo.wifi[id].mac,mac,NETWORK_MAC_SIZE);
            /* �ظ���Ԫȷ�� */
            WifiUnit.transmit(kMode_Wifi_Unitcast,  \
                              Protocol.wifiUnit(&wifiProt,id,ConfirmUnitIDMtoU_D,(confirmID >> 8),(confirmID & 0xFF)));
            SysInfo.state.wifiCurEditID = (SysInfo.state.wifiCurEditID + 1) > WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM) ? WIFI_ID(1) : SysInfo.state.wifiCurEditID + 1;
            WifiUnit.transmit(kMode_Wifi_Multicast,  \
                              Protocol.wifiUnit(&wifiProt,0,EnterEditingIDMtoU_G,(SysInfo.state.wifiCurEditID >> 8),(SysInfo.state.wifiCurEditID & 0xFF)));
        }
    }
    break;

    /* ��Ԫ��֪ͨ�����������ź�ǿ�ȹ���\���� */
    case UnitCapacityStrChangeUtoM_D:
        break;

    /* �ظ�������ȡMIC������ */
    case RepReadUnitMICSensitivity_UtoM_D: {
        UnitInfo.wifi[id].config->sensitivity = ph;
    }
    break;

    /* �ظ�������ȡEQ */
    case RepReadUnitEQ_UtoM_D: {
        uint8_t eq,freq,vol;

        eq = ((&notify->exDataHead)[0] >> 4) - 1;
        freq = (&notify->exDataHead)[1] >> 5;
        vol = ((&notify->exDataHead)[1] & 0x1F);

        if(!(eq >= 0 && eq <= 3))
            break;

        freq = (freq >= 0 && freq <= 3) ? freq : 0;
        vol = (vol >= 0 && vol <= 24) ? vol : 24;

        UnitInfo.wifi[id].config->eqFreq[eq] = freq;
        UnitInfo.wifi[id].config->eqVol[eq] = 24 - vol;
        ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),BASIC_MSG,UNIT_CTRL,SET_UNTI_EQ,null,(&notify->exDataHead)[0] << 8 | (&notify->exDataHead)[1]));
    }
    break;

    case IDRepeatingMtoU_G: {
        SysInfo.state.dupId = WIFI_ID((ph << 8) | pl) ;

        Log.d("Wifi unit id repead !! ID = %X\r\n", SysInfo.state.dupId);

        /* �㲥֪ͨȫ���ֻ���ϵͳ */
        WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID, BASIC_MSG, CONFERENCE_MODE, ID_DUPICATE,null,SysInfo.state.dupId));
        /* �㲥֪ͨȫWIFI����ϵͳ */
        WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0, IDRepeatingMtoU_G, SysInfo.state.dupId >> 8, SysInfo.state.dupId));

        SysInfo.state.idDupicate = true;
//		SysInfo.state.idDupCnt = ID_DUPICATE_WAIT_TIME;
//        Screen.togglePage(SP_ID_PEPEAT);
    }
    break;

//			case RepReadUnitMICEQ_UtoM_D:
//			break;


    default:
        break;
    }


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
    ScreenProtType_EN type;
    uint8_t *para, exData[25];

    static UnitType_EN cfgWitchTypeUnit;

    ERR_CHECK(notify != null, return);

    type = notify->prot.screen.type;
    para = notify->prot.screen.para;

#if 1
    Log.d("Screen Msg: type = %X, para{%X %X %X %X %X}",type,para[0],para[1],para[2],para[3],para[4]);
    if(notify->exLen) {
        uint8_t i;

        printf(" exData{ ");
        for(i=0; i<notify->exLen; i++)
            printf(" %X,",(&notify->exDataHead)[i]);
        printf(" } ");
    }
    printf("\r\n");
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
                Screen.togglePage(SP_UNIT_TYPE);
            }
            break;

            /* ¼���ڽ��� */
            case 0x02: {
                if(!UsbAudio.isMcuConnectUsb && !UsbAudio.isWt2000ConnectUsb) {
                    Screen.togglePage(SP_USB_INIT);
                    break;
                }

                switch(UsbAudio.audSta) {
                /* �л���ͣҳ�� */
                case kStatus_Aud_Idle:
//					case kStatus_Aud_Pause:
                {
                    Screen.togglePage(SP_AUD_PLAY_OR_RECORD);
                }
                break;

                /* �л�����ҳ�� */
                case kStatus_Aud_Playing: {
                    char *name = UsbAudio.musicFile[UsbAudio.playIndex - 1]->fname;

                    Screen.setVariable(SVR_AUD_PLAY_NAME,strlen(name)+1,(uint8_t *)name);
                    Screen.togglePage(SP_AUD_PLAYER_PLAYING);
                }
                break;
                /* �л�¼��ҳ�� */
                case kStatus_Aud_Recording: {
                    Screen.setVariable(SVR_AUD_PLAY_NAME,strlen(UsbAudio.recFile)+1,(uint8_t *)UsbAudio.recFile);
                    Screen.togglePage(SP_AUD_RECORDING);
                }
                break;

                default:
                    break;
                }


            }
            break;

            /* ϵͳ״̬ */
            case 0x03: {


                /* �л�ҳ�� */
                Screen.togglePage(SP_SYS_STATE);

                /* ������Ļ��ʾ��Ԫ���� */
                Conference_ScreenUpdataUnitNum();


            }
            break;

            /* ��ID���� */
            case 0x04: {
                if(SysInfo.state.startID < 1 || SysInfo.state.startID > 4096)
                    SysInfo.state.startID = 1;
                /* �л�ҳ�� */
                Screen.togglePage(SP_SET_ID);
                sprintf((char *)&exData[0],"%04d  ",SysInfo.state.startID);
//                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_START_ID),6,exData);
                Screen.setVariable(SVR_START_ID,6,exData);
            }
            break;

            /* ϵͳ���� */
            case 0x05: {
//				uint16_t year,mon,day;

                /* �л�ҳ�� */
                Screen.togglePage(SP_SYS_SETTING);

//			    APP_GetBuildDate(&year, &mon, &day);

                /* ������ʾ��ǰϵͳ�汾�� */
//                sprintf((char *)&exData[0],"M_%s_%04d%02d%02d",APP_VERSION,year,mon,day);
//                Screen.setVariable(SVR_SOFTWARE_VERSION_M,16,exData);

                /* ������ʾ����������� */
//                sprintf((char *)&exData[0],"S_%s_%d%d%d",APP_VERSION,year,mon,day);
//                Screen.setVariable(SVR_SOFTWARE_VERSION_S,16,exData);
            }
            break;

            /*��Ա��*/
            case 0x06: {
//                /* �л�ҳ�� */
//                Screen.togglePage(StartTranSetID_Page);
            }
            break;
            /* ID ģʽ */
//            case 0x07: {
//                /* �л�ҳ�� */
//                Screen.togglePage(SP_SYS_SETTING);
//            }
//            break;
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
                    SysInfo.config->micMode = (MicMode_EN)mode;
            } else if(reg == 0x14) {
                /* �������ߵ�Ԫ */
                if(cfgWitchTypeUnit == kType_Unit_Wired && cmd >= 0 && cmd <= 3) {
                    static const uint8_t Num[4] = {1,2,4,8};

                    SysInfo.config->wiredAllowOpen = Num[cmd];
                    SysInfo.config->wiredAllowWait = Num[cmd];
                }

                /* ����WIFI��Ԫ */
                else if(cfgWitchTypeUnit == kType_Unit_Wifi && cmd >= 0 && cmd <= 3) {
                    static const uint8_t Num[4] = {1,2,4,6};

                    SysInfo.config->wifiAllowOpen = Num[cmd];
                    SysInfo.config->wifiAllowWait = Num[cmd];
                }
            }

            /* �ر����е�Ԫ */
            Conference_CloseAllMic(kType_Unit_Wired,aChairman);
            Conference_CloseAllMic(kType_Unit_Wired,aRepresentative);
            Conference_CloseAllMic(kType_Unit_Wifi,aChairman);
            Conference_CloseAllMic(kType_Unit_Wifi,aRepresentative);

            /* ���������� */
            Conference_ClearApplyMic();

            /* �㲥ģʽ������ */
            /* ���͵���Ԫ */
            WiredUnit.transmit(Protocol.conference(&confProt,MODE_BROADCAST_ID,STATE_MSG,SysInfo.config->micMode,SysInfo.config->wiredAllowOpen,null,null));
            WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,ChangeMicManage_MtoU_G,SysInfo.config->micMode,SysInfo.config->wifiAllowOpen));
            /* �ظ��ⲿ�����豸ȷ��״̬ */
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,MODE_BROADCAST_ID,STATE_MSG,SysInfo.config->micMode,SysInfo.config->wiredAllowOpen,null,null));
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_MODE_BROADCAST_ID,STATE_MSG,SysInfo.config->micMode,SysInfo.config->wifiAllowOpen,null,null));

            Database.saveSpecify(kType_Database_SysCfg,null);
        }
        break;
        case 0x2B:  //0 1 2 3���߻���ģʽ�У��л���Ͳģʽ
            break;
        /* ��ID��ʼID�� */
        case 0x22: {
            if(para[2] == 0x02) {
                char strId[5] = {0};

                strId[0] = para[3];
                strId[1] = para[4];
                strId[2] = (&notify->exDataHead)[0];
                strId[3] = (&notify->exDataHead)[1];

                sscanf(strId,"%d",&SysInfo.state.startID);
            }
        }
        break;
        case 0x24:  //��Ա��ģʽ����
            break;
        case 0x26:  //��Ա����ID
            break;
        /* ������߽�����ID */
        case 0x2A: {
            if(cmd == 0x02) {
                /* �л�ҳ�� */
                Screen.togglePage(SP_SET_ID_END);
                SysInfo.state.wiredCurEditID = SysInfo.state.startID;
                SysInfo.state.wifiCurEditID = WIFI_ID(SysInfo.state.startID > WIFI_UNIT_MAX_ONLINE_NUM ? WIFI_UNIT_MAX_ONLINE_NUM : SysInfo.state.startID);

                /* �л�����ģʽ */
                Conference_ChangeSysMode(kMode_EditID);
            } else if(cmd == 0x03) {
                /* �л�ҳ�� */
                Screen.togglePage(SP_MAIN_MENU);

                /* �����ߵ�Ԫ���ͽ�����ID */
                WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,EDIT_ID_MODE,END_EDIT_ID_MODE,null,null));

                /* �л�����ģʽ */
                Conference_ChangeSysMode(kMode_Conference);
//                WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,MasterStarUp_MtoU_G,0,0));
            }
        }
        break;

        case 0x16:  //����������
            break;
        case 0x18:  //����������
            break;
        case 0x20:  //������������������
            break;
        /* ϵͳ���� */
        case 0x3A: {
            switch(cmd) {
            /* ����ѡ�� */
            case 0x01: {
                /* �л�ҳ�� */
                Screen.togglePage(SP_SET_LANGUAGE);
//                exData[1] = SysInfo.config->language;
//                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0040),2,exData);
            }
            break;

            /* ������IP���ý��棬��ʱ��Ҫ����������ʾ */
            case 0x02: {
                uint8_t *ip,*mask,*gw;

                ip = SysInfo.config->ip;
                mask = SysInfo.config->mask;
                gw = SysInfo.config->gateWay;

                /* �л�ҳ�� */
                Screen.togglePage(SP_LOCAL_IP);

                sprintf((char *)&exData[0],"%d.%d.%d.%d",ip[0],ip[1],ip[2],ip[3]);
//                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_IP_ADDR),15,exData);
                Screen.setVariable(SVR_IP_ADDR,15,exData);

                sprintf((char *)&exData[0],"%d.%d.%d.%d",mask[0],mask[1],mask[2],mask[3]);
//                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_GATEWAY_ADDR),15,exData);
                Screen.setVariable(SVR_NETMASK_ADDR,15,exData);

                sprintf((char *)&exData[0],"%d.%d.%d.%d",gw[0],gw[1],gw[2],gw[3]);
//                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_NETMASK_ADDR),15,exData);
                Screen.setVariable(SVR_GATEWAY_ADDR,15,exData);

                sprintf((char *)&exData[0],"%d",SysInfo.config->port);
//                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_PORT),6,exData);
                Screen.setVariable(SVR_PORT,6,exData);

            }
            break;

            /* ��ʾ����-���ȵ��� */
            case 0x04: {
                /* �л�ҳ�� */
                Screen.togglePage(SP_SET_DISPLAY);

                exData[0] = 0;
                exData[1] = SysInfo.config->brightness;
//                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_BRIGHTNESS),2,exData);
                Screen.setVariable(SVR_BRIGHTNESS,2,exData);
            }
            break;

            /* �������� */
            case 0x06: {
                /* �л�ҳ�� */
                Screen.togglePage(SP_SET_VOLUME);

                exData[0] = 0;
                exData[1] = SysInfo.config->dsp[DSP_OUTPUT_LINE_OUT1].vol;
//                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_LINEOUT1_VOL),2,exData);
                Screen.setVariable(SVR_LINEOUT1_VOL,2,exData);

                exData[0] = 0;
                exData[1] = SysInfo.config->dsp[DSP_OUTPUT_LINE_OUT2].vol;
//                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_LINEOUT2_VOL),2,exData);
                Screen.setVariable(SVR_LINEOUT2_VOL,2,exData);
            }
            break;

            /* �´����� */
            case 0x07: {
                /* �л�ҳ�� */
                Screen.togglePage(SP_SET_DOWN_TRANS);

                exData[0] = 0;
                exData[1] = !SysInfo.state.wifiAudDownward;
//                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_DOWN_TRANSMIT),2,exData);
                Screen.setVariable(SVR_DOWN_TRANSMIT,2,exData);
            }
            break;

            default:
                break;
            }
        }
        break;

        /* ������¼�������� */
        case 0x3B: {
            switch(cmd) {
            case 0x00://��һ��
                Audio.previous();
                break;

            case 0x01://����
//            	if(UsbAudio.audSta == kStatus_Aud_Idle || UsbAudio.audSta == kStatus_Aud_Pause)
                Audio.playPause();
                break;

            case 0x02://��һ��
                Audio.next();
                break;

            /* ��ʼ¼�� */
            case 0x03:
                if(UsbAudio.audSta == kStatus_Aud_Idle) {
                    char *name;
                    uint8_t *slogan;

                    name = MALLOC(16);
                    slogan = MALLOC(2);

                    Screen.togglePage(SP_AUD_RECORD_WAIT);
                    slogan[1] = 0;
                    Screen.setVariable(0x0008,2,slogan);

                    sprintf(UsbAudio.recFile,"%s%03d","R",++UsbAudio.recFileMaxIndex);
                    Audio.record(UsbAudio.recFile);

                    strcpy(name,UsbAudio.recFile);
                    Screen.setVariable(SVR_AUD_PLAY_NAME,16,(uint8_t *)name);
                    FREE(name);
                    FREE(slogan);
                }
                break;

            /* ֹͣ¼�� */
            case 0x06: {
                if(UsbAudio.audSta == kStatus_Aud_Recording) {
                    uint8_t *slogan;

                    Audio.stopRec();
                    slogan = MALLOC(2);

                    Screen.togglePage(SP_AUD_RECORD_WAIT);
                    slogan[1] = 1;
                    Screen.setVariable(0x0008,2,slogan);
                    FREE(slogan);
                }
            }
            break;

            /* ������ͣ */
            case 0x05:
                Audio.playPause();
                break;



            /* ���� */
            case 0x0F: {
                if(UsbAudio.audSta == kStatus_Aud_Playing || UsbAudio.audSta == kStatus_Aud_Recording) {
                    Screen.togglePage(SP_MAIN_MENU);
                } else {
                    Screen.togglePage(SP_AUD_PLAY_OR_RECORD);
                }
            }
            break;

            }
        }
        break;

        /* �����ߵ�Ԫ & �ָ�Ĭ�� */
        case 0x3C:
            switch(cmd) {
            case 0x01: {
                Database.restoreDef(kType_Database_SysCfg);
                Database.saveSpecify(kType_Database_SysCfg,null);
                Screen.togglePage(SP_WELCOME);
            }
            break;

            case 0x02: {
                WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0, MasterStarUp_MtoU_G, 1, 0));
                Screen.togglePage(SP_WELCOME);
            }
            break;

            default:
                break;
            }
            break;

        case 0x40: {//������������ѡ��
            if(cmd >= Chinese && cmd <= French) {
                SysInfo.config->language = cmd;
                ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,UNIT_BROADCAST_ID,BASIC_MSG,UNIT_CTRL,SET_LANGUAGE(cmd),null,null));
            }
        }
        break;
        /* ���������´��������� */
        case 0x41: {
            SysInfo.state.wifiAudDownward = !cmd;
            SlaveMcu.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,DSP_NOR_OUT_CFG,0x06,0x04,(!cmd) << 8));
        }
        break;
        /* �����������������Ļ��ǰ����ֵ�� */
        case 0x44: {
            if(cmd >= 0 && cmd <= 64) {
                SysInfo.config->brightness = cmd;
                Screen.backlight(cmd);
                Database.saveSpecify(kType_Database_SysCfg,null);
            }
        }
        break;
        case 0x45://��������������µ���ֵ��
            break;
        case 0x50: //��������IDģʽ����
            break;

        /* ���±�ID */
        case 0x51: {
            /* �ر�ID�ظ� */
            SysInfo.state.idDupicate = false;

            /* �л�ҳ�� */
            Screen.unlock();
            Screen.togglePage(SP_RESET_ID);
            SysInfo.state.wiredCurEditID = 1;
            SysInfo.state.wifiCurEditID = WIFI_ID(1);
            Conference_ChangeSysMode(kMode_EditID);
        }
        break;
        /* �������±�ID */
        case 0x52: {
            /* �л�ҳ�� */
            Screen.togglePage(SP_MAIN_MENU);
            Conference_ChangeSysMode(kMode_Conference);
//            WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,MasterStarUp_MtoU_G,0,0));
        }
        break;
        /* ������¼/����ѡ�� */
        case 0x53: {
            if(!UsbAudio.isMcuConnectUsb && !UsbAudio.isWt2000ConnectUsb) {
                Screen.togglePage(SP_USB_INIT);
                break;
            }

            switch(cmd) {
            case 0x00:
                Screen.togglePage(SP_AUD_PLAY_STOPED);
                break;
            case 0x01:
                Screen.togglePage(SP_AUD_RECORD_STOPED);
                break;
            }
        }
        break;

        /* ������������������ѡ�񷵻� */
        case 0x54:
            switch(cmd) {
            /* ���� */
            case 0x00: {
                static const uint8_t Reg[7] = {null,0,1,null,2,null,3};

                cfgWitchTypeUnit = kType_Unit_Wifi;

                exData[0] = 0;
                exData[1] = 1;
                Screen.setVariable(SVR_CONF_MODE,2,exData);

                /* �л�ҳ�� */
                Screen.togglePage(SP_CONF_MODE);

                exData[1] = SysInfo.config->micMode - 1;
                Screen.setVariable(SVR_SET_MIC_MODE,2,exData);

                exData[1] = Reg[SysInfo.config->wifiAllowOpen];
                Screen.setVariable(SVR_SET_MIC_NUM,2,exData);
            }
            break;

            /* ���� */
            case 0x01: {
                static const uint8_t Reg[9] = {null,0,1,null,2,null,null,null,3};

                cfgWitchTypeUnit = kType_Unit_Wired;

                exData[0] = 0;
                exData[1] = 0;
                Screen.setVariable(SVR_CONF_MODE,2,exData);

                /* �л�ҳ�� */
                Screen.togglePage(SP_CONF_MODE);

                exData[1] = SysInfo.config->micMode - 1;
                Screen.setVariable(SVR_SET_MIC_MODE,2,exData);

                exData[1] = Reg[SysInfo.config->wiredAllowOpen];
                Screen.setVariable(SVR_SET_MIC_NUM,2,exData);
            }
            break;
            }
            break;

        /* ������������Ͳ���� */
        case 0x55:
            switch(cmd) {
            case 0x00://����
                break;
            case 0x01://�½�
                break;
            }
            break;
        case 0x56: { //������AFC����
            switch(cmd) {
            case 0x00://�ر�
                HAL_SetGpioLevel(AfcCtrl, 0);
                break;
            case 0x01://��
                HAL_SetGpioLevel(AfcCtrl, 1);
                break;
            }
        }
        break;
        /* ����LineOut���� */
        case 0x92:
        case 0x94: {
            DspOutput_E lineout = (reg == 0x92 ? DSP_OUTPUT_LINE_OUT1 : DSP_OUTPUT_LINE_OUT2);

            SysInfo.config->dsp[lineout].vol = cmd;
            Dsp.norOutput(lineout,DSP_OUTPUT_VOLUME,(DspEqPart_E)null,31 - cmd);
            Database.saveSpecify(kType_Database_SysCfg,null);
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



/*******************************************************************************
 * @Section:         Screen
 ******************************************************************************/

/**
* @Name  		Conference_ScreenUpdataUnitNum
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_ScreenUpdataUnitNum(void)
{
    uint16_t num;
    uint8_t *exData;

    exData = MALLOC(25);

    /* �������ߵ�Ԫ�������� */
    num = OnlineNum.wiredChm + OnlineNum.wiredRps;
    exData[0]= (uint8_t)(num >> 8);
    exData[1]= (uint8_t)(num & 0xFF);
    Screen.setVariable(SVR_WIRED_OL_TOTAL_NUM,2,exData);


    /* ����������ϯ���������� */
    exData[0]= (uint8_t)(OnlineNum.wiredChm >> 8);
    exData[1]= (uint8_t)(OnlineNum.wiredChm & 0xFF);
    Screen.setVariable(SVR_WIRED_OL_CHM_NUM,2,exData);

    /* �������ߴ������������  */
    exData[0]= (uint8_t)(OnlineNum.wiredRps >> 8);
    exData[1]= (uint8_t)(OnlineNum.wiredRps & 0xFF);
    Screen.setVariable(SVR_WIRED_OL_RPS_NUM,2,exData);

    /* �������ߵ�Ԫ�������� */
    num = OnlineNum.wifiChm + OnlineNum.wifiRps;
    exData[0]= (uint8_t)(num >> 8);
    exData[1]= (uint8_t)(num & 0xFF);
    Screen.setVariable(SVR_WIFI_OL_TOTAL_NUM,2,exData);

    /* ����������ϯ���������� */
    exData[0]= (uint8_t)(OnlineNum.wifiChm >> 8);
    exData[1]= (uint8_t)(OnlineNum.wifiChm & 0xFF);
    Screen.setVariable(SVR_WIFI_OL_CHM_NUM,2,exData);

    /* �������ߴ������������ */
    exData[0]= (uint8_t)(OnlineNum.wifiRps >> 8);
    exData[1]= (uint8_t)(OnlineNum.wifiRps & 0xFF);
    Screen.setVariable(SVR_WIFI_OL_RPS_NUM,2,exData);


    /* ������Ա���������� */
    exData[0]= (uint8_t)(OnlineNum.interpreter >> 8);
    exData[1]= (uint8_t)(OnlineNum.interpreter & 0xFF);
    Screen.setVariable(SVR_INTERP_OL_NUM,2,exData);

    FREE(exData);

}


/*******************************************************************************
 * @Section:         Sign
 ******************************************************************************/

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
    uint8_t *data;

    switch(cmd) {


    /* ��ʼǩ�� */
    /** @brief ���ߵ�ԪЭ���ǿ�ʼǩ����ǩ�������ֿ�����Э�飬wifi��ԪЭ����ͬһ��
               ������￪ʼǩ����ͬʱ�·�ǩ���������� */
    case WIRED_CMD(START_SIGN_MODE,0,0):
    case WIFI_CMD(EnterSignMode_MtoU_G,0,0): {
        WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,SIGN_MODE,START_SIGN_MODE,null,null));

    }

    /* �·�ǩ��������ǩ����ǰ���� */
    case WIRED_CMD(START_SIGN_MODE,0,1):
    case WIFI_CMD(EnterSignMode_MtoU_G,0,1): {
        data = MALLOC(4);

        data[0] = (uint8_t)(totalNum >> 8);
        data[1] = (uint8_t)(totalNum & 0xFF);
        data[2] = (uint8_t)(curNum >> 8);
        data[3] = (uint8_t)(curNum & 0xFF);

        WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,RESULT_MSG,SIGN_RESULT,data[0], data[1], curNum));
        WifiUnit.transWithExData(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,EnterSignMode_MtoU_G,0,null),4,data);

        FREE(data);
    }
    break;

    /* ��Ԫǩ�� */
    case WIRED_CMD(UNIT_SIGN_IN,0,0):
    case WIFI_CMD(RepPollUnitl_UtoM_D,kMode_Sign,0): {
        if(id != null) {
            id = type == kType_Unit_Wired ? id : WIFI_ID(id);
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id,BASIC_MSG,SIGN_MODE,UNIT_SIGN_IN,null,null));
        }
    }
    break;

    /* ���Ƶ�Ԫǩ�� */
    case WIRED_CMD(CONTROL_UNIT_SIGN,0,0):
    case WIFI_CMD(ControlSign_MtoU_D,0,0): {
        if(type == kType_Unit_Wired)
            WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,SIGN_MODE,CONTROL_UNIT_SIGN,null,null));
        else if(type == kType_Unit_Wifi)
            WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,ControlSign_MtoU_D,null,null));
    }
    break;

    /* ����ǩ�� */
    case WIRED_CMD(SUPPLEMENT_SIGN,0,0):
    case WIFI_CMD(SupplementSign_MtoU_D,0,0): {
        if(type == kType_Unit_Wired)
            WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,SIGN_MODE,SUPPLEMENT_SIGN,null,null));
        else if(type == kType_Unit_Wifi)
            WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,SupplementSign_MtoU_D,null,null));
    }
    break;

    /* �ظ�����ǩ�� */
    case WIRED_CMD(UNIT_SUPPLEMENT_SIGN,0,0):
    case WIFI_CMD(RepSupplementSign_UtoM_D,0,0): {
        if(id != null) {
            id = type == kType_Unit_Wired ? id : WIFI_ID(id);
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id,BASIC_MSG,SIGN_MODE,UNIT_SUPPLEMENT_SIGN,null,null));
        }
    }
    break;
    }

}


/*******************************************************************************
 * @Section:         Vote
 ******************************************************************************/
/**
* @Name 		Conference_VoteResult
* @Author		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_VoteResult(void)
{
    uint8_t *data,i;
    uint16_t *res,id,total;

    ConfProtocol_S confProt;
    WifiUnitProtocol_S wifiProt;

    data = MALLOC(10);

    memset(SysInfo.state.voteResult,0,sizeof(SysInfo.state.voteResult));
    res = SysInfo.state.voteResult;

    /* ͳ�������ߵ�ԪͶƱ��� */
    for(id = 1; id <= WIRED_UNIT_MAX_ONLINE_NUM; id++) {
        if(UnitInfo.wired[id].online && UnitInfo.wired[id].vote > 0 && UnitInfo.wired[id].vote <= 5)
            res[UnitInfo.wired[id].vote - 1] += 1;
    }

    /* ͳ����WIFI��ԪͶƱ��� */
    for(id = 1; id <= WIFI_UNIT_MAX_ONLINE_NUM; id++) {
        if(UnitInfo.wifi[id].online && UnitInfo.wifi[id].vote > 0 && UnitInfo.wifi[id].vote <= 5)
            res[UnitInfo.wifi[id].vote - 1] += 1;
    }

    total = UNIT_ONLINE_NUM;

    /* �·������� */
    for(i = 0; i < 10; i+=2) {
        data[i] = (uint8_t)(res[i/2] >> 8);
        data[i+1] = (uint8_t)(res[i/2] & 0xFF);
    }

    WiredUnit.transWithExData(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,RESULT_MSG,VOTE_RESULT,total >> 8,total & 0xFF,res[0]),8,&data[2]);
    WifiUnit.transWithExData(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,EnterVoteMode_MtoU_G,1,null),10,data);

    FREE(data);
}


/*******************************************************************************
 * @Section:         DSP
 ******************************************************************************/

/**
* @Name  		Conference_DspUnitCtrl
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_DspUnitCtrl(uint16_t id, UnitType_EN type, MicControlType_EN ctrlType,uint8_t channel)
{
    uint8_t dspOut[17],dspOutVol[17],i;
    DspUintSrc_E dspUnitSrc;
    UnitInfo_S *unitInfo;

    dspUnitSrc = (DspUintSrc_E)((type == kType_Unit_Wired) ? (channel - 0x81) : (channel - 0x81 + 8));
    unitInfo = (type == kType_Unit_Wired) ? UnitInfo.wired : UnitInfo.wifi;

    switch(ctrlType) {
    /* ����Ͳ */
    case tOpenMic: {
        switch(SysInfo.config->dspMode) {
        /* ͬ��ģʽ������ */
        case DSP_MODE_PARTITION: {
            dspOut[0] = dspOutVol[0] = 16;
            for(i = 0; i < 16; i++) {
                dspOut[i + 1] = i;
                dspOutVol[i + 1] = 31 - unitInfo[id].config->chVol[i];
            }
            Dsp.unitCtrl(dspUnitSrc,dspOut,dspOutVol);
        }
        break;
        /* ���߼�����ģʽ������ */
        case DSP_MODE_WIFI:
        case DSP_MODE_WIRE: {
            if(id >= 1 && id <= 16) {
                dspOut[0] = dspOutVol[0] = 1;
                dspOut[1] = id - 1;
                dspOutVol[1] = DSP_VOLUME_0_DB;
                Dsp.unitCtrl(dspUnitSrc,dspOut,dspOutVol);
            }
        }
        break;

        default:
            break;
        }
    }
    break;

    /* �ػ�Ͳ */
    case tCloseMic: {
        switch(SysInfo.config->dspMode) {
        /* ͬ��ģʽ������ */
        case DSP_MODE_PARTITION: {
            dspOut[0] = dspOutVol[0] = 16;
            for(i = 0; i < 16; i++) {
                dspOut[i + 1] = i;
                dspOutVol[i + 1] = DSP_VOLUME_N144_DB;
            }
            Dsp.unitCtrl(dspUnitSrc,dspOut,dspOutVol);
        }
        break;
        /* ���߼�����ģʽ������ */
        case DSP_MODE_WIFI:
        case DSP_MODE_WIRE: {
            if(id >= 1 && id <= 16) {
                dspOut[0] = dspOutVol[0] = 1;
                dspOut[1] = id - 1;
                dspOutVol[1] = DSP_VOLUME_N144_DB;
                Dsp.unitCtrl(dspUnitSrc,dspOut,dspOutVol);
            }
        }
        break;

        default:
            break;
        }
    }
    break;

    default:
        return;
    }

//
//		Log.d("Dsp unit ctrl data :  src = %d\r\n",dspUnitSrc);
//		for(i = 0;i <= 16;i++){
//			printf(" %X-%X \r\n",dspOut[i],dspOutVol[i]);
//		}
//		printf("\r\n");
}


/*******************************************************************************
 * @Section:         Unit
 ******************************************************************************/

/**
* @Name  		Conference_MicCtrlInstruction
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_MicCtrlInstruction(uint16_t id, UnitType_EN type,UnitAttr_EN attr, MicControlType_EN ctrlType,uint8_t ch)
{
    ConfProtocol_S confProt;
    WifiUnitProtocol_S wifiProt;
    uint8_t data[5] = {0},len =0,arg = 0;


    switch(ctrlType) {
    /* ����Ͳ */
    case tOpenMic: {
        arg = (attr == aChairman) ? CHM_OPEN_MIC : RPS_OPEN_MIC;
        if(type == kType_Unit_Wired) {
            Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,arg,ch,null);
            WiredUnit.transmit(&confProt);
            ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
            UnitInfo.wired[id].channel = ch;
            UnitInfo.wired[id].micSta = kStatus_UnitMic_Open;

            /* ����DSP */
            Conference_DspUnitCtrl(id,type,tOpenMic,ch);
            /* �����������Ԥ��λ */
            Camera.call(id,type);
        } else if(type == kType_Unit_Wifi) {
            data[0] = ch;
            len = 1;
            WifiUnit.transWithExData(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,MasterOpenOrCloseMic_MtoU_D,0,null),len,data);
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),BASIC_MSG,CONFERENCE_MODE,arg,ch,null));
            UnitInfo.wifi[id].channel = ch;
            UnitInfo.wifi[id].micSta = kStatus_UnitMic_Open;

            /* ����DSP */
            Conference_DspUnitCtrl(id,type,tOpenMic,ch);
            /* �����������Ԥ��λ */
            Camera.call(id,type);
        }
    }
    break;

    /* �ػ�Ͳ */
    case tCloseMic: {
        arg = (attr == aChairman) ? CHM_CLOSE_MIC : RPS_CLOSE_MIC;
        if(type == kType_Unit_Wired) {
            Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,arg,null,null);
            WiredUnit.transmit(&confProt);
            ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
            UnitInfo.wired[id].channel = null;
            UnitInfo.wired[id].micSta = kStatus_UnitMic_Close;

            /* ����DSP */
            Conference_DspUnitCtrl(id,type,tCloseMic,ch);
            /* �ͷ�Ԥ��λ */
            Camera.release(id,type);
        } else if(type == kType_Unit_Wifi) {
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),BASIC_MSG,CONFERENCE_MODE,arg,null,null));
            WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,MasterOpenOrCloseMic_MtoU_D,1,null));
            UnitInfo.wifi[id].channel = null;
            UnitInfo.wifi[id].micSta = kStatus_UnitMic_Close;

            /* ����DSP */
            Conference_DspUnitCtrl(id,type,tCloseMic,ch);
            /* �ͷ�Ԥ��λ */
            Camera.release(id,type);
        }

    }
    break;

    /* ��Ͳ�ȴ� */
    case tWaiting: {
        if(type == kType_Unit_Wired) {
            Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,MIC_WAIT,null,null);
            WiredUnit.transmit(&confProt);
            ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
            UnitInfo.wired[id].micSta = kStatus_UnitMic_Wait;
        } else if(type == kType_Unit_Wifi) {
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),BASIC_MSG,CONFERENCE_MODE,MIC_WAIT,null,null));
            WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,RepApplyOpenMic_MtoU_D,3,null));
            UnitInfo.wifi[id].micSta = kStatus_UnitMic_Wait;
        }
    }
    break;

    /* ��Ͳȡ���ȴ� */
    case tDisWait: {
        if(type == kType_Unit_Wired) {
            Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,MIC_DISWAIT,null,null);
            ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
            WiredUnit.transmit(&confProt);
            UnitInfo.wired[id].micSta = kStatus_UnitMic_Close;
        } else if(type == kType_Unit_Wifi) {
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),BASIC_MSG,CONFERENCE_MODE,MIC_DISWAIT,null,null));
            WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,MasterOpenOrCloseMic_MtoU_D,1,null));
            UnitInfo.wifi[id].micSta = kStatus_UnitMic_Close;
        }
    }
    break;

    /* ��Ͳ���� */
    case tMicFull: {
        if(type == kType_Unit_Wired) {
            Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,MIC_CHANNEL_FULL,null,null);
            ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
            WiredUnit.transmit(&confProt);
        } else if(type == kType_Unit_Wifi) {
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),BASIC_MSG,CONFERENCE_MODE,MIC_CHANNEL_FULL,null,null));
            WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,RepApplyOpenMic_MtoU_D,1,null));
        }

    }
    break;

    /* ��Ͳ���� */
    case tMicApply: {
        uint16_t applyId = (type == kType_Unit_Wifi) ? WIFI_ID(id) : id;
        UnitInfo_S *unitInfo = (type == kType_Unit_Wifi) ? UnitInfo.wifi : UnitInfo.wired;

        data[0] = (uint8_t)(applyId >> 8);
        data[1] = (uint8_t)(applyId& 0xFF);
        len = 2;
        Protocol.conference(&confProt,APPLY_OPEN_MIC_ID,APPLY_MSG,data[0],data[1],null,null);
        Protocol.wifiUnit(&wifiProt,0,RepresentApplyOpenMic_MtoU_G,0,null);

        ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
        WiredUnit.transmit(&confProt);
        WifiUnit.transWithExData(kMode_Wifi_Multicast,&wifiProt,len,data);

        unitInfo[id].micSta = kStatus_UnitMic_Apply;
    }
    break;

    /* �������� */
    case tRevokeApply: {
        /* �������벻��ҪID */
        data[0] = 0;
        data[1] = 0;
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
    uint8_t ch, chmMicNum, rpsMicNum, waitNum, index,allowOpen,allowWait;
    uint16_t applyId;
    DataQueueHandler_S chmMicQueue,rpsMicQueue,waitQueue;
    UnitInfo_S *unitInfo;


    ERR_CHECK(!(type == kType_Unit_Wired && (id < 0 || id > WIRED_UNIT_MAX_ONLINE_NUM)),return);
//    ERR_CHECK(!(type == kType_Unit_Wired && UnitInfo.wired[id].online != true),return);
    ERR_CHECK(!(type == kType_Unit_Wifi && (id < 0 || id > WIFI_UNIT_MAX_ONLINE_NUM)),return);
//    ERR_CHECK(!(type == kType_Unit_Wifi && UnitInfo.wifi[id].online != true),return);

    if(type == kType_Unit_Wired) {
        chmMicQueue = WiredChmMicQueue;
        rpsMicQueue = WiredRpsMicQueue;
        waitQueue = WiredWaitQueue;
        unitInfo = UnitInfo.wired;
        allowOpen = SysInfo.config->wiredAllowOpen;
        allowWait = SysInfo.config->wiredAllowWait;
    }

    else if(type == kType_Unit_Wifi) {
        chmMicQueue = WifiChmMicQueue;
        rpsMicQueue = WifiRpsMicQueue;
        waitQueue = WifiWaitQueue;
        unitInfo = UnitInfo.wifi;
        allowOpen = SysInfo.config->wifiAllowOpen;
        allowWait = SysInfo.config->wifiAllowWait;
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
            switch(SysInfo.config->micMode) {
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
                    ch = unitInfo[id].channel;
                    Conference_MicCtrlInstruction(id,type,aChairman,tOpenMic,ch);
                    break;
                }

                /* ��ϯ+������Ͳ����δ����������Ͳ�� */
                if(chmMicNum + rpsMicNum < allowOpen) {
                    DataQueue.enter(chmMicQueue,&id);
                    ch = Conference_GetAudioChannel(type);

                    Conference_MicCtrlInstruction(id,type,aChairman,tOpenMic,ch);
                }
                /* ��ϯ+������Ͳ��������������Ͳ�� */
                else {

                    if(rpsMicNum > 0) {
                        /* ����п��ŵĴ���Ͳ���ͼ�һ������ */
                        uint16_t closeId;

                        /* ����ȡ������Ԫ���ر� */
                        DataQueue.exit(rpsMicQueue,&closeId);
                        ch = unitInfo[closeId].channel;
                        Conference_MicCtrlInstruction(closeId,type,aRepresentative,tCloseMic,ch);

                        /* ����ϯ */
                        DataQueue.enter(chmMicQueue,&id);
                        Conference_MicCtrlInstruction(id,type,aChairman,tOpenMic,ch);
                    } else {
                        /* ��Ͳ���� */
                        Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tMicFull,null);
                    }
                }

                /* ���������ģʽ��,������(�ȴ�)���в�Ϊ�գ����������� */
                if(waitNum > 0) {
                    /* ���»�ȡ��ϯ���򿪶������� */
                    chmMicNum = DataQueue.getSize(chmMicQueue);

                    /* ������ڴ򿪻�Ͳ����ϯ+�������� + �ȴ����� > ����򿪵�����
                       ��ȡ���ȴ������еĻ�Ͳ��ȡ���ȴ� */
                    if(waitNum + chmMicNum + rpsMicNum > allowOpen) {
                        uint16_t closeId;

                        /* ����(�ȴ�)����ȡ����Ԫ��ȡ���ȴ� */
                        DataQueue.exit(waitQueue,&closeId);
                        Conference_MicCtrlInstruction(closeId,type,aRepresentative,tDisWait,null);

                        /* ����ģʽ�� ��Ҫ����������� */
                        if(SysInfo.config->micMode == kMode_Mic_Apply) {
                            closeId = (type == kType_Unit_Wired ? closeId : WIFI_ID(closeId));

                            /* �������ɾ����ͲID */
                            DataQueue.deleted(ApplyQueue,DataQueue.search(ApplyQueue,&closeId));

                            if(DataQueue.getSize(ApplyQueue) == 0) {
                                Conference_MicCtrlInstruction(null,(UnitType_EN)null, (UnitAttr_EN)null, tRevokeApply, null);
                            } else {
                                /* ����ͷ��ID���뿪��Ͳ */
                                DataQueue.front(ApplyQueue,&applyId);
                                if(applyId > WIFI_UNIT_START_ID && applyId <= (WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM)))
                                    Conference_MicCtrlInstruction(applyId - WIFI_UNIT_START_ID,kType_Unit_Wifi,(UnitAttr_EN)null,tMicApply,null);
                                else if(applyId >= 1 && applyId <= WIRED_UNIT_MAX_ONLINE_NUM)
                                    Conference_MicCtrlInstruction(applyId,kType_Unit_Wired,(UnitAttr_EN)null,tMicApply,null);
                            }
                        }
                    }
                }
            }
            break;

            default:
                break;
            }
        }

        /* �������뿪��Ͳ */
        else if(unitInfo[id].attr == aRepresentative) {
            switch(SysInfo.config->micMode) {
            /*** �Ƚ��ȳ�ģʽ ***/
            case kMode_Mic_Fifo: {
                /* ���һ�Ͳ�����Ƿ��Ѿ����ڶ�ӦID�Ļ�Ͳ */
                if(DataQueue.search(rpsMicQueue,&id)) {
                    ch = unitInfo[id].channel;
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tOpenMic,ch);
                    break;
                }

                /* ��ϯ+������Ͳ����δ����������Ͳ�� */
                if(chmMicNum + rpsMicNum < allowOpen) {
                    ch = Conference_GetAudioChannel(type);
                    DataQueue.enter(rpsMicQueue,&id);
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tOpenMic,ch);
                }
                /* ��ϯ+������Ͳ��������������Ͳ�� */
                else {
                    if(rpsMicNum > 0) {
                        uint16_t closeId;

                        /* ����ȡ������Ԫ���ر� */
                        DataQueue.exit(rpsMicQueue,&closeId);
                        ch = unitInfo[closeId].channel;
                        Conference_MicCtrlInstruction(closeId,type,aRepresentative,tCloseMic,ch);
                        /* �򿪴��� */
                        DataQueue.enter(rpsMicQueue,&id);
                        Conference_MicCtrlInstruction(id,type,aRepresentative,tOpenMic,ch);
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
                    ch = unitInfo[id].channel;
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tOpenMic,ch);
                    break;
                }

                /* ���ҵȴ������Ƿ���ڶ�ӦID��Ͳ */
                if(waitNum > 0) {
                    /* ȡ���û�Ͳ�ȴ� */
                    index = DataQueue.search(waitQueue,&id);
                    if(index != 0) {
                        DataQueue.deleted(waitQueue,index);
                        Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tDisWait,null);
                        break;
                    }
                }

                /* ��ϯ+������Ͳ����δ����������Ͳ�� */
                if(chmMicNum + rpsMicNum < allowOpen) {
                    ch = Conference_GetAudioChannel(type);
                    DataQueue.enter(rpsMicQueue,&id);
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tOpenMic,ch);
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
                /* ������в��������ߡ�WIFI��Ԫ���������ID��Ҫ������ʵIDת�� */
                applyId = (type == kType_Unit_Wired ? id : WIFI_ID(id));

                /* ���һ�Ͳ�����Ƿ��Ѿ����ڶ�ӦID�Ļ�Ͳ */
                if(DataQueue.search(rpsMicQueue,&id)) {
                    /* ���·��ʹ򿪻�Ͳָ�����Ҫ�������� */
                    ch = unitInfo[id].channel;
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tOpenMic,ch);
                    break;
                }

                /* ���ҵȴ������Ƿ���ڶ�ӦID��Ͳ */
                if(waitNum > 0 ) {
                    index = DataQueue.search(waitQueue,&id);
                    if(index != 0) {
                        Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tDisWait,null);
                        DataQueue.deleted(waitQueue,index);

                        /* ɾ��������ж�ӦID */
                        DataQueue.deleted(ApplyQueue,DataQueue.search(ApplyQueue,&applyId));
                        if(DataQueue.getSize(ApplyQueue) == 0) {
                            Conference_MicCtrlInstruction(null,(UnitType_EN)null, (UnitAttr_EN)null, tRevokeApply, null);
                        } else {
                            /* ����ͷ��ID���뿪��Ͳ */
                            DataQueue.front(ApplyQueue,&applyId);
                            if(applyId > WIFI_UNIT_START_ID && applyId <= (WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM)))
                                Conference_MicCtrlInstruction(applyId - WIFI_UNIT_START_ID,kType_Unit_Wifi,(UnitAttr_EN)null,tMicApply,null);
                            else if(applyId >= 1 && applyId <= WIRED_UNIT_MAX_ONLINE_NUM)
                                Conference_MicCtrlInstruction(applyId,kType_Unit_Wired,(UnitAttr_EN)null,tMicApply,null);
                        }
                        break;
                    }
                }

                /* ��ϯ+������Ͳ����δ����������Ͳ�� */
                if(chmMicNum + rpsMicNum < allowOpen && waitNum < allowWait) {
                    /* ����ȴ����� */
                    DataQueue.enter(waitQueue,&id);
                    Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tWaiting,null);

                    /* ����������� */
                    DataQueue.enter(ApplyQueue,&applyId);

                    /* ����ͷ��ID���뿪��Ͳ */
                    DataQueue.front(ApplyQueue,&applyId);
                    if(applyId > WIFI_UNIT_START_ID && applyId <= (WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM)))
                        Conference_MicCtrlInstruction(applyId - WIFI_UNIT_START_ID,kType_Unit_Wifi,(UnitAttr_EN)null,tMicApply,null);
                    else if(applyId >= 1 && applyId <= WIRED_UNIT_MAX_ONLINE_NUM)
                        Conference_MicCtrlInstruction(applyId,kType_Unit_Wired,(UnitAttr_EN)null,tMicApply,null);

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
        switch(SysInfo.config->micMode) {
        /*** �Ƚ��ȳ�ģʽ ***/
        case kMode_Mic_Fifo: {
            /* ��ϯ��Ԫ */
            if(unitInfo[id].attr == aChairman) {
                /* ���һ�Ͳ�����Ƿ��Ѿ����ڶ�ӦID�Ļ�Ͳ */
                index = DataQueue.search(chmMicQueue,&id);
                if(index) {
                    ch = unitInfo[id].channel;
                    Conference_GiveAudioChannel(type,ch);
                    DataQueue.deleted(chmMicQueue,index);
                    Conference_MicCtrlInstruction(id,type,aChairman,tCloseMic,ch);
                }
            }

            /* ����Ԫ */
            else if(unitInfo[id].attr == aRepresentative) {
                /* ���һ�Ͳ�����Ƿ��Ѿ����ڶ�ӦID�Ļ�Ͳ */
                index = DataQueue.search(rpsMicQueue,&id);
                if(index) {
                    ch = unitInfo[id].channel;
                    Conference_GiveAudioChannel(type,ch);
                    DataQueue.deleted(rpsMicQueue,index);
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tCloseMic,ch);
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
                    ch = unitInfo[id].channel;
                    Conference_GiveAudioChannel(type,ch);
                    DataQueue.deleted(chmMicQueue,index);
                    Conference_MicCtrlInstruction(id,type,aChairman,tCloseMic,ch);
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
                    ch = unitInfo[id].channel;
                    Conference_GiveAudioChannel(type,ch);
                    DataQueue.deleted(rpsMicQueue,index);
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tCloseMic,ch);
                }

                if(waitNum > 0) {
                    /* �ȴ�����ȡ����Ԫ���� */
                    uint16_t waitId;
                    DataQueue.exit(waitQueue,&waitId);
                    Conference_MicControl(waitId,type,WIRED_CMD(RPS_OPEN_MIC,0,0));
                }
            }
        }
        break;

        /*** ����ģʽ ***/
        case kMode_Mic_Apply: {
            /* ��ϯ��Ԫ */
            if(unitInfo[id].attr == aChairman) {
                /* ���һ�Ͳ�����Ƿ��Ѿ����ڶ�ӦID�Ļ�Ͳ */
                index = DataQueue.search(chmMicQueue,&id);
                if(index) {
                    ch = unitInfo[id].channel;
                    Conference_GiveAudioChannel(type,ch);
                    DataQueue.deleted(chmMicQueue,index);
                    Conference_MicCtrlInstruction(id,type,aChairman,tCloseMic,ch);
                }
            }

            /* ����Ԫ */
            else if(unitInfo[id].attr == aRepresentative) {
                /* ���һ�Ͳ�����Ƿ��Ѿ����ڶ�ӦID�Ļ�Ͳ */
                index = DataQueue.search(rpsMicQueue,&id);
                if(index) {
                    ch = unitInfo[id].channel;
                    Conference_GiveAudioChannel(type,ch);
                    DataQueue.deleted(rpsMicQueue,index);
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tCloseMic,ch);
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

        /* @brief ����ϯ��ͬ��/�ܾ�����Ͳ�����·�ͬ���ID��ֻ���·�һ��ͬ��/�ܾ�ָ�
                    ������Ҫ�ڶ���������ң����ն���˳��ͬ��/�ܾ���Ͳ�򿪣��ҵ���������
                    ��Ҫ�����ߺ�WIFI������ϵͳ�߼��ϲ����������������С���ApplyQueue��
                    ��������ϵͳID�ڶ����� */

        /* ���ⲿ����ͬ�⣬��ID */
        if(id != 0) {
            /* ������в��������ߡ�WIFI��Ԫ���������ID��Ҫ������ʵIDת�� */
            applyId = (type == kType_Unit_Wired ? id : WIFI_ID(id));

            /* �򿪻�Ͳ����ɾ���ȴ������ж�ӦID�Ļ�Ͳ */
            index = DataQueue.search(waitQueue,&id);
            if(index != 0) {
                DataQueue.deleted(waitQueue,index);

                /* ɾ��������ж�ӦID */
                DataQueue.deleted(ApplyQueue,DataQueue.search(waitQueue,&applyId));

                if(chmMicNum + rpsMicNum < allowOpen) {
                    DataQueue.enter(rpsMicQueue,&id);
                    ch = Conference_GetAudioChannel(type);
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tOpenMic,ch);
                } else {
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tDisWait,null);
                }
            }
        }

        else {

            if(DataQueue.getSize(ApplyQueue) > 0) {
                DataQueue.exit(ApplyQueue,&applyId);

                if(applyId > WIFI_UNIT_START_ID && applyId <= (WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM))) {
                    applyId -= WIFI_UNIT_START_ID;

                    DataQueue.deleted(WifiWaitQueue,DataQueue.search(WifiWaitQueue,&applyId));

                    if(DataQueue.getSize(WifiRpsMicQueue) + DataQueue.getSize(WifiChmMicQueue) < SysInfo.config->wifiAllowOpen) {
                        DataQueue.enter(WifiRpsMicQueue,&applyId);
                        ch = Conference_GetAudioChannel(kType_Unit_Wifi);
                        Conference_MicCtrlInstruction(applyId,kType_Unit_Wifi,aRepresentative,tOpenMic,ch);
                    } else {
                        Conference_MicCtrlInstruction(applyId,kType_Unit_Wifi,aRepresentative,tDisWait,null);
                    }
                } else if(applyId >= 1 && applyId <= WIRED_UNIT_MAX_ONLINE_NUM) {

                    DataQueue.deleted(WiredWaitQueue,DataQueue.search(WiredWaitQueue,&applyId));

                    if(DataQueue.getSize(WiredRpsMicQueue) + DataQueue.getSize(WiredChmMicQueue) < SysInfo.config->wiredAllowOpen) {
                        DataQueue.enter(WiredRpsMicQueue,&applyId);
                        ch = Conference_GetAudioChannel(kType_Unit_Wired);
                        Conference_MicCtrlInstruction(applyId,kType_Unit_Wired,aRepresentative,tOpenMic,ch);
                    } else {
                        Conference_MicCtrlInstruction(applyId,kType_Unit_Wired,aRepresentative,tDisWait,null);
                    }
                }

            }

#if 0
            if(DataQueue.getSize(WiredWaitQueue) > 0) {
                /* �Ȳ������ߵ�Ԫ�ȴ����� */
                if(DataQueue.exit(WiredWaitQueue,&waitId) && \
                   (DataQueue.getSize(WiredRpsMicQueue) + DataQueue.getSize(WiredChmMicQueue) < SysInfo.config->wiredAllowOpen)) {
                    DataQueue.enter(WiredRpsMicQueue,&waitId);
                    ch = Conference_GetAudioChannel(kType_Unit_Wired);
                    Conference_MicCtrlInstruction(waitId,kType_Unit_Wired,aRepresentative,tOpenMic,ch);
                } else {
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tDisWait,null);
                }


            }

            else if(DataQueue.getSize(WifiWaitQueue) > 0 ) {
                /* �ٲ���WIFI��Ԫ�ȴ����� */
                if(DataQueue.exit(WifiWaitQueue,&waitId) && \
                   (DataQueue.getSize(WifiRpsMicQueue) + DataQueue.getSize(WifiChmMicQueue) < SysInfo.config->wifiAllowOpen)) {
                    DataQueue.enter(WifiRpsMicQueue,&waitId);
                    ch = Conference_GetAudioChannel(kType_Unit_Wifi);
                    Conference_MicCtrlInstruction(waitId,kType_Unit_Wifi,aRepresentative,tOpenMic,ch);
                } else {
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tDisWait,null);
                }
            }
#endif
        }


        if(DataQueue.getSize(ApplyQueue) == 0) {
            Conference_MicCtrlInstruction(null, (UnitType_EN)null, (UnitAttr_EN)null, tRevokeApply, null);
        } else {
            /* ����ͷ��ID���뿪��Ͳ */
            DataQueue.front(ApplyQueue,&applyId);
            if(applyId > WIFI_UNIT_START_ID && applyId <= (WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM)))
                Conference_MicCtrlInstruction(applyId - WIFI_UNIT_START_ID,kType_Unit_Wifi,(UnitAttr_EN)null,tMicApply,null);
            else if(applyId >= 1 && applyId <= WIRED_UNIT_MAX_ONLINE_NUM)
                Conference_MicCtrlInstruction(applyId,kType_Unit_Wired,(UnitAttr_EN)null,tMicApply,null);
        }

//        if(DataQueue.getSize(WifiWaitQueue) == 0 && DataQueue.getSize(WiredWaitQueue) == 0) {
//            Conference_MicCtrlInstruction(null, null, (UnitAttr_EN)null, tRevokeApply, null);
//        }
    }
    break;

    /* ��ϯ��ͬ�⿪��Ͳ */
    case WIRED_CMD(DISAGREE_OPEN_MIC,0,0):
    case WIFI_CMD(ChairAgreeOpenOrNot_UtoM_D,1,0): {
        /* ���ⲿ���ƾܾ�����ID */
        if(id != 0) {
            /* ������в��������ߡ�WIFI��Ԫ���������ID��Ҫ������ʵIDת�� */
            applyId = (type == kType_Unit_Wired ? id : WIFI_ID(id));

            /* �򿪻�Ͳ����ɾ���ȴ������ж�ӦID�Ļ�Ͳ */
            index = DataQueue.search(waitQueue,&id);
            if(index != 0) {
                DataQueue.deleted(waitQueue,index);

                /* ɾ��������ж�ӦID */
                DataQueue.deleted(ApplyQueue,DataQueue.search(waitQueue,&applyId));

                Conference_MicCtrlInstruction(id,type,aRepresentative,tDisWait,null);
            }
        }

        else {

            if(DataQueue.getSize(ApplyQueue) > 0) {
                DataQueue.exit(ApplyQueue,&applyId);

                if(applyId > WIFI_UNIT_START_ID && applyId <= (WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM))) {
                    applyId -= WIFI_UNIT_START_ID;

                    DataQueue.deleted(WifiWaitQueue,DataQueue.search(WifiWaitQueue,&applyId));
                    Conference_MicCtrlInstruction(applyId,kType_Unit_Wifi,aRepresentative,tDisWait,null);
                } else if(applyId >= 1 && applyId <= WIRED_UNIT_MAX_ONLINE_NUM) {

                    DataQueue.deleted(WiredWaitQueue,DataQueue.search(WiredWaitQueue,&applyId));
                    Conference_MicCtrlInstruction(applyId,kType_Unit_Wired,aRepresentative,tDisWait,null);
                }

            }

        }


        if(DataQueue.getSize(ApplyQueue) == 0) {
            Conference_MicCtrlInstruction(null, (UnitType_EN)null, (UnitAttr_EN)null, tRevokeApply, null);
        } else {
            /* ����ͷ��ID���뿪��Ͳ */
            DataQueue.front(ApplyQueue,&applyId);
            if(applyId > WIFI_UNIT_START_ID && applyId <= (WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM)))
                Conference_MicCtrlInstruction(applyId - WIFI_UNIT_START_ID,kType_Unit_Wifi,(UnitAttr_EN)null,tMicApply,null);
            else if(applyId >= 1 && applyId <= WIRED_UNIT_MAX_ONLINE_NUM)
                Conference_MicCtrlInstruction(applyId,kType_Unit_Wired,(UnitAttr_EN)null,tMicApply,null);
        }
    }
    break;

    /* ��Ԫȡ���ȴ� */
    case WIRED_CMD(MIC_DISWAIT,0,0):
    case WIFI_CMD(UnitGetoutWaitQuenen_UtoM_D,0,0): {
        switch(SysInfo.config->micMode) {
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
            /* ������в��������ߡ�WIFI��Ԫ���������ID��Ҫ������ʵIDת�� */
            applyId = (type == kType_Unit_Wired ? id : WIFI_ID(id));

            if(waitNum > 0 && unitInfo[id].attr == aRepresentative) {
                /* ���ҵȴ������Ƿ��Ѿ����ڶ�ӦID�Ļ�Ͳ */
                index = DataQueue.search(waitQueue,&id);
                if(index) {
                    DataQueue.deleted(waitQueue,index);
                    DataQueue.deleted(ApplyQueue,DataQueue.search(ApplyQueue,&applyId));
                    Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tDisWait,null);
                }
            }

            if(DataQueue.getSize(ApplyQueue) == 0) {
                Conference_MicCtrlInstruction(null, (UnitType_EN)null, (UnitAttr_EN)null, tRevokeApply, null);
            } else {
                /* ����ͷ��ID���뿪��Ͳ */
                DataQueue.front(ApplyQueue,&applyId);
                if(applyId > WIFI_UNIT_START_ID && applyId <= (WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM)))
                    Conference_MicCtrlInstruction(applyId - WIFI_UNIT_START_ID,kType_Unit_Wifi,(UnitAttr_EN)null,tMicApply,null);
                else if(applyId >= 1 && applyId <= WIRED_UNIT_MAX_ONLINE_NUM)
                    Conference_MicCtrlInstruction(applyId,kType_Unit_Wired,(UnitAttr_EN)null,tMicApply,null);
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
    uint8_t ch;

    if(type == kType_Unit_Wired) {
        unitInfo = UnitInfo.wired;

        if(attr == aChairman)
            micQueue = WiredChmMicQueue;
        else if(attr == aRepresentative)
            micQueue = WiredRpsMicQueue;
        else
            return;
    } else if(type == kType_Unit_Wifi) {
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
            ch = unitInfo[idArr[i]].channel;
            if(attr == aChairman)
                Conference_MicCtrlInstruction(idArr[i], type, aChairman, tCloseMic, ch);
            else
                Conference_MicCtrlInstruction(idArr[i], type, aRepresentative, tCloseMic, ch);

            Conference_GiveAudioChannel(type,ch);
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
static void Conference_ClearApplyMic(void)
{
    uint8_t waitNum,i;
    uint16_t *idArr;

    /* �����ߵ�Ԫ */
    waitNum = DataQueue.getSize(WiredWaitQueue);
    if(waitNum > 0) {
        idArr = MALLOC(waitNum * sizeof(uint16_t));
        DataQueue.toArray(WiredWaitQueue,idArr);
        for(i = 0; i<waitNum; i++) {
            Conference_MicCtrlInstruction(idArr[i], kType_Unit_Wired, (UnitAttr_EN)null, tDisWait, null);
        }
        DataQueue.empty(WiredWaitQueue);
        FREE(idArr);
    }

    /* �����ߵ�Ԫ */
    waitNum = DataQueue.getSize(WifiWaitQueue);
    if(waitNum > 0) {
        idArr = MALLOC(waitNum * sizeof(uint16_t));
        DataQueue.toArray(WifiWaitQueue,idArr);
        for(i = 0; i<waitNum; i++) {
            Conference_MicCtrlInstruction(idArr[i], kType_Unit_Wifi, (UnitAttr_EN)null, tDisWait, null);
        }
        DataQueue.empty(WifiWaitQueue);
        FREE(idArr);
    }

    DataQueue.empty(ApplyQueue);
    if(SysInfo.config->micMode == kMode_Mic_Apply)
        Conference_MicCtrlInstruction(null, (UnitType_EN)null, (UnitAttr_EN)null, tRevokeApply, null);
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
	uint16_t id;

    Conference_CloseAllMic(kType_Unit_Wired,aChairman);
    Conference_CloseAllMic(kType_Unit_Wired,aRepresentative);
    Conference_CloseAllMic(kType_Unit_Wifi,aChairman);
    Conference_CloseAllMic(kType_Unit_Wifi,aRepresentative);
    Conference_ClearApplyMic();

    memset(&OnlineNum,0,sizeof(UnitOnlineNum));

	for(id = 1;id <= WIRED_UNIT_MAX_ONLINE_NUM;id++){
		UnitInfo.wired[id].online = false;
		UnitInfo.wired[id].sign = false;
		UnitInfo.wired[id].vote = null;
		UnitInfo.wired[id].channel = 0;
	}

	for(id = 1;id <= WIFI_UNIT_MAX_ONLINE_NUM;id++){
		UnitInfo.wifi[id].online = false;
		UnitInfo.wifi[id].sign = false;
		UnitInfo.wifi[id].vote = null;
		UnitInfo.wifi[id].channel = 0;
	}

    /* ������Ƶͨ������ */
    Conference_ResetAudioChannel(kType_Unit_Wired);
    Conference_ResetAudioChannel(kType_Unit_Wifi);

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
    ERR_CHECK(!(SysInfo.state.sysMode != kMode_Conference && mode != kMode_Conference), return);

    switch(mode) {
    case kMode_Conference: {

        /* �жϵ�ǰģʽ������ */
        switch(SysInfo.state.sysMode) {
        case kMode_Sign:
        case kMode_DevSign: {
            WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,SIGN_MODE,END_SIGN_MODE,null,null));
            WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,EnterSignMode_MtoU_G,1,null));
            SysInfo.state.initSignVoteId = 0;
        }
        break;

        case kMode_Vote:
        case kMode_DevVote: {
            WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,VOTE_MODE,FINISH_VOTE,null,null));
//                WifiUnit.transWithExData(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,EnterVoteMode_MtoU_G,1,null),10,data);
            SysInfo.state.initSignVoteId = 0;
        }
        break;

        case kMode_EditID: {
            /* �����ߵ�Ԫ���ͽ�����ID */
            WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,EDIT_ID_MODE,END_EDIT_ID_MODE,null,null));
        }
        break;
        default:
            break;
        }

        /* ��WIFIϵͳ��ȫ����ϵͳ�㲥�������ģʽ */
        Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,CONFERENCE_MODE,START_CONFERENCE_MODE,null,null);
        Protocol.wifiUnit(&wifiProt,0,EnterMeetingMode_MtoU_G,SysInfo.config->micMode,SysInfo.config->wifiAllowOpen);

        ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
        WiredUnit.transmit(&confProt);
        WifiUnit.transmit(kMode_Wifi_Multicast,&wifiProt);

        /* WIFIϵͳ�����⣬Ҫ���л������ģʽ�������һ��������� */
        if(SysInfo.state.sysMode == kMode_EditID) {
            WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,MasterStarUp_MtoU_G,0,0));
        }

    }
    break;

    case kMode_EditID: {
        /* �����������е�Ԫ */
        Conference_OfflineAllUnit();

        /* ��WIFIϵͳ��ȫ����ϵͳ�㲥�����IDģʽ */
        Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,EDIT_ID_MODE,START_EDIT_ID_MODE,null,null);
        Protocol.wifiUnit(&wifiProt,0,EnterEditingIDMtoU_G,(SysInfo.state.wifiCurEditID >> 8),(SysInfo.state.wifiCurEditID & 0xFF));

        ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
        WiredUnit.transmit(&confProt);
        WifiUnit.transmit(kMode_Wifi_Multicast,&wifiProt);


    }
    break;

    case kMode_DevSign:
    case kMode_Sign: {
        uint16_t id;

        /* �������ǩ����־ */
        for(id = 0; id < WIRED_UNIT_MAX_ONLINE_NUM; id++)
            UnitInfo.wired[id].sign = false;

        for(id = 0; id < WIFI_UNIT_MAX_ONLINE_NUM; id++)
            UnitInfo.wifi[id].sign = false;

        Conference_SignInstruction(null,(UnitType_EN)null,WIRED_CMD(START_SIGN_MODE,0,0),SysInfo.state.totalSignNum,SysInfo.state.currentSignNum);

    }
    break;

    case kMode_DevVote:
    case kMode_Vote: {
        uint16_t id;

        /* �������ͶƱ��־ */
        for(id = 0; id < WIRED_UNIT_MAX_ONLINE_NUM; id++)
            UnitInfo.wired[id].vote = null;

        for(id = 0; id < WIFI_UNIT_MAX_ONLINE_NUM; id++)
            UnitInfo.wifi[id].vote = null;

        memset(SysInfo.state.voteResult,0,sizeof(SysInfo.state.voteResult));

        Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,VOTE_MODE,VoteModeCode[SysInfo.state.voteMode],null,null);
        Protocol.wifiUnit(&wifiProt,0,EnterVoteMode_MtoU_G,0,SysInfo.state.voteMode);

        ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
        WiredUnit.transmit(&confProt);
        WifiUnit.transmit(kMode_Wifi_Multicast,&wifiProt);

    }
    break;

    default:
        break;
    }

    SysInfo.state.sysMode = mode;
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

    ERR_CHECK(!(type == kType_Unit_Wired && (id <= 0 || id > WIRED_UNIT_MAX_ONLINE_NUM || UnitInfo.wired[id].attr != aChairman)),return);
    ERR_CHECK(!(type == kType_Unit_Wifi && (id <= 0 || id > WIFI_UNIT_MAX_ONLINE_NUM || UnitInfo.wifi[id].attr != aChairman)),return);

    /* �ر����д���Ԫ */
    Conference_CloseAllMic(kType_Unit_Wired,aRepresentative);
    Conference_CloseAllMic(kType_Unit_Wifi,aRepresentative);

    /* ȡ�����л�Ͳ���� */
    Conference_ClearApplyMic();

    /* ����ϯ��Ԫ */
    Conference_MicControl(id,type,type == kType_Unit_Wired ? WIRED_CMD(CHM_OPEN_MIC,0,0) : WIFI_CMD(ApplyOpenMic_UtoM_D,0,0));
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

    if(type == kType_Unit_Wired) {
        DataQueue.empty(WiredChannelQueue);
        for(ch = 1; ch <= WIRED_UNIT_MAX_ALLWO_OPEN; ch++) {
            DataQueue.enter(WiredChannelQueue,&ch);
        }
    } else if(type == kType_Unit_Wifi) {
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

    if(type == kType_Unit_Wired)
        DataQueue.exit(WiredChannelQueue,&ch);
    else if(type == kType_Unit_Wifi)
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

    if(type == kType_Unit_Wired)
        DataQueue.enter(WiredChannelQueue,&ch);
    else if(type == kType_Unit_Wifi)
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
* @Name  		Conference_UpdataUnitTime
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_UpdataUnitTime(TimePara_S *time)
{
    ConfProtocol_S confProt;
    WifiUnitProtocol_S wifiProt;
    uint8_t *data;

    ERR_CHECK(time != null, return);

    data = (uint8_t *)time;
    WiredUnit.transWithExData(Protocol.conference(&confProt,NOW_DATE_TIME_ID,STATE_MSG,data[0],data[1],data[2],data[3] << 8 | data[4]),1,&data[5]);
    WifiUnit.transWithExData(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,RealTimeMtoU_G,null,null),6,data);
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
    return SysInfo.state.sysMode;
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
static ConfSysInfo_S *Conference_GetConfSysInfo(void)
{
    return &SysInfo;
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
    return SysInfo.state.wiredCurEditID;
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
    FILINFO *filinfo = null;
    uint8_t fileNum, i;
    char *suffix = null;

    UsbAudio.musicFileNum = 0;
    UsbAudio.recFileMaxIndex = 1;

    fileNum = UsbDisk.getFileList(path,&filinfo);

    if(filinfo == null) {
        Log.d("\"%s\" not exist\r\n",path);
        UsbDisk.mkdir(path,true);
    } else {
        Log.d("USB( %s ) music file : \r\n",path,fileNum);
        for(i = 0; i < fileNum; i++) {
            suffix = strrchr(filinfo[i].fname, '.');
            if(suffix != null) {
                if (strcmp(suffix, ".mp3") == 0 || strcmp(suffix, ".MP3") == 0) {
                    printf(" %s\r\n",filinfo[i].fname);
                    UsbAudio.musicFile[UsbAudio.musicFileNum++] = &filinfo[i];
                    /* �ж��ļ����Ƿ�Ϊ�ض��ı���¼���ļ�������¼�ļ�������
                       "Rxxx.mp3"*/
                    if(IS_SPECIFIC_AUD_REC_FILE(filinfo[i].fname)) {
                        uint8_t index;

                        index = GET_SPECIFIC_AUD_REC_INDEX(filinfo[i].fname);
                        UsbAudio.recFileMaxIndex = UsbAudio.recFileMaxIndex < index ? index : UsbAudio.recFileMaxIndex;
                    }
                } else if (strcmp(suffix, ".wav") == 0 || strcmp(suffix, ".MP3") == 0) {
                    printf(" %s\r\n",filinfo[i].fname);
                    UsbAudio.musicFile[UsbAudio.musicFileNum++] = &filinfo[i];
                }

                if(UsbAudio.musicFileNum >= MUSIC_FILE_LIST_MAX_LEN)
                    break;
            }
            suffix = null;
        }
//        Log.d("max index = %d\r\n",UsbAudio.recFileMaxIndex);
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
    char path[10];

    switch(sta) {
    case kStatus_DEV_Attached: {
        Log.d("Usb disk is attached!!\r\n");
    }
    break;
    case kStatus_DEV_Detached: {
        UsbAudio.isMcuConnectUsb = false;

        Log.d("Usb disk is detached!!\r\n");
    }
    break;
    case kStatus_FS_Mounted: {
        Log.d("Usb file system is mounted!!\r\n");
        Log.d("Usb disk free size : %dMB\r\n",(UsbDisk.freeSize()/1024));

        sprintf(path,"%s/%s",USBDISK_ROOT,AUDIO_DEFAULT_DIR);

        Conference_GetMusicFile(path);
        UsbAudio.isMcuConnectUsb = true;
    }
    break;
    case kStatus_DEV_Idle: {
        Log.d("Usb idle!!\r\n");
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
static void Conference_AudioStateListener(AudEvent_EN event,AudInfo_S *info)
{
    char *name,time[20];
    uint8_t currPage;

    const char nullLen = 32;
    const char nullChar[nullLen] = {0};


    UsbAudio.isWt2000ConnectUsb = info->usbConnect;
    UsbAudio.audSta = info->state;

    /* �жϵ�ǰ�Ƿ�Ϊ����ģʽ��������������ʾ�����ļ��� */
    if(UsbAudio.playIndex != info->playIndex) {
        UsbAudio.playIndex = info->playIndex;
        if(UsbAudio.audSta == kStatus_Aud_Playing) {
            name = UsbAudio.musicFile[UsbAudio.playIndex - 1]->fname;
            /* �������ʾ */
            Screen.setVariable(SVR_AUD_PLAY_NAME,nullLen,(uint8_t *)nullChar);
            Screen.setVariable(SVR_AUD_PLAY_NAME,strlen(name)+1,(uint8_t *)name);
            Log.d("Play audio file(index = %d) : %s\r\n",UsbAudio.playIndex,name);
        }
    }

    if(/*UsbAudio.audSta == kStatus_Aud_Playing || */UsbAudio.audSta == kStatus_Aud_Recording) {
        sprintf(time,"%02d:%02d:%02d",info->runTime / 3600,info->runTime / 60,info->runTime % 60);
//        Log.d("Audio index = %d, state = %d , time = %s\r\n",info->playIndex,UsbAudio.audSta,time);
        Screen.setVariable(SVR_AUD_REC_PLAY_TIME,strlen(time) + 1,(uint8_t *)time);
    }


    /* ��ȡ��ǰҳ�� */
    currPage = Screen.currentPage();

    switch(event) {
    case kEvent_Aud_None:
//        Log.d("Audio index = %d, state = %d , time = %s\r\n",info->playIndex,UsbAudio.audSta,time);
        break;
    case kEvent_Aud_UsbConnect:
        Log.d("Audio usb connect..\r\n");

        if(currPage == SP_USB_INIT) {
            Screen.togglePage(SP_AUD_PLAY_OR_RECORD);
        }
        break;
    case kEvent_Aud_UsbDisconnect:
        Log.d("Audio usb disconnect..\r\n");

        if(currPage == SP_AUD_PLAY_OR_RECORD || currPage == SP_AUD_PLAYER_PLAYING ||  currPage == SP_AUD_PLAY_STOPED || \
           currPage == SP_AUD_RECORD_WAIT || currPage == SP_AUD_RECORD_STOPED ||  currPage == SP_AUD_RECORDING ) {
            Screen.togglePage(SP_USB_INIT);
        }
        Screen.setVariable(SVR_AUD_PLAY_NAME,nullLen,(uint8_t *)nullChar);
        Screen.setVariable(SVR_AUD_REC_PLAY_TIME,nullLen,(uint8_t *)nullChar);
        break;
    case kEvent_Aud_RecordStar:
        Log.d("Audio usb Star Record..\r\n");
        if(currPage == SP_AUD_RECORD_STOPED || currPage == SP_AUD_RECORD_WAIT) {
            Screen.togglePage(SP_AUD_RECORDING);
            Screen.setVariable(SVR_AUD_PLAY_NAME,nullLen,(uint8_t *)nullChar);
            Screen.setVariable(SVR_AUD_PLAY_NAME,strlen(UsbAudio.recFile)+1,(uint8_t *)UsbAudio.recFile);
        }
        break;
    case kEvent_Aud_PlayStar:
        Log.d("Audio usb Star Play..\r\n");
        if(currPage == SP_AUD_PLAY_STOPED) {
            Screen.togglePage(SP_AUD_PLAYER_PLAYING);
            name = UsbAudio.musicFile[UsbAudio.playIndex - 1]->fname;
            /* �������ʾ */
            Screen.setVariable(SVR_AUD_PLAY_NAME,nullLen,(uint8_t *)nullChar);
            Screen.setVariable(SVR_AUD_PLAY_NAME,strlen(name)+1,(uint8_t *)name);
        }
        break;
    case kEvent_Aud_UsbFileUpdata: {
        char *path;
        TimePara_S *tPara;

        Log.d("Audio usb file updata..\r\n");

        if(UsbAudio.audSta == kStatus_Aud_Recording) {
            while(!UsbAudio.isMcuConnectUsb) {
                DELAY(50);
            }

            if(Time.getRst()) {
                path = MALLOC(30);
                tPara = MALLOC(sizeof(TimePara_S));

                Time.getNow(tPara);
                sprintf(path,"%s/%s/%s.MP3",USBDISK_ROOT,AUDIO_DEFAULT_DIR,UsbAudio.recFile);
                UsbDisk.setFileDateTime(path,tPara->year,tPara->month,tPara->day,tPara->hour,tPara->min);

                FREE(tPara);
                FREE(path);
            }
        }
    }
    break;

    case kEvent_Aud_RecordStop:
        Log.d("Audio usb Stop Record..\r\n");
        if(currPage == SP_AUD_RECORDING || currPage == SP_AUD_RECORD_WAIT) {
            Screen.togglePage(SP_AUD_RECORD_STOPED);
            Screen.setVariable(SVR_AUD_PLAY_NAME,nullLen,(uint8_t *)nullChar);
            Screen.setVariable(SVR_AUD_REC_PLAY_TIME,nullLen,(uint8_t *)nullChar);
        }
        break;
    case kEvent_Aud_PlayStop:
        Log.d("Audio usb Stop Play..\r\n");
        if(currPage == SP_AUD_PLAYER_PLAYING) {
            Screen.togglePage(SP_AUD_PLAY_STOPED);
//			Screen.setVariable(SVR_AUD_PLAY_NAME,16,(uint8_t *)nullChar);
//			Screen.setVariable(SVR_AUD_REC_PLAY_TIME,16,(uint8_t *)nullChar);
        }
        break;
    case kEvent_Aud_RecordErr:
        Log.d("Audio usb Record Err..\r\n");
        break;
    case kEvent_Aud_PlayErr:
        Log.d("Audio usb Play Err..\r\n");
        break;

    default:
        break;
    }

}

/**
* @Name  		Conference_FireAlarmSignalCallback
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Conference_FireAlarmSignalCallback(void *param)
{
    FireAlarm = !HAL_GetGpioLevel(FireAlarmSignal);
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

    Log.d("Open mic list : \r\n");
    printf("======================== Wired ===========================\r\n");
    printf("==      Attr      ==      ID      ==      Channel       ==\r\n");
    for(i=0; i<len; i++)
        printf("==    Chairman    ==      %02d      ==        %02X          ==\r\n",idArr[i],UnitInfo.wired[idArr[i]].channel);
    FREE(idArr);

    len = DataQueue.getSize(WiredRpsMicQueue);
    idArr = MALLOC(len * sizeof(uint16_t));
    DataQueue.toArray(WiredRpsMicQueue,idArr);

    for(i=0; i<len; i++)
        printf("== Representative ==      %02d      ==        %02X          ==\r\n",idArr[i],UnitInfo.wired[idArr[i]].channel);
    FREE(idArr);

    len = DataQueue.getSize(WifiChmMicQueue);
    idArr = MALLOC(len * sizeof(uint16_t));
    DataQueue.toArray(WifiChmMicQueue,idArr);

    printf("======================== Wifi ============================\r\n");
    for(i=0; i<len; i++)
        printf("==    Chairman    ==      %02d      ==        %02X          ==\r\n",idArr[i],UnitInfo.wifi[idArr[i]].channel);
    FREE(idArr);

    len = DataQueue.getSize(WifiRpsMicQueue);
    idArr = MALLOC(len * sizeof(uint16_t));
    DataQueue.toArray(WifiRpsMicQueue,idArr);

    for(i=0; i<len; i++)
        printf("== Representative ==      %02d      ==        %02X          ==\r\n",idArr[i],UnitInfo.wifi[idArr[i]].channel);
    printf("==========================================================\r\n");
    FREE(idArr);
}

