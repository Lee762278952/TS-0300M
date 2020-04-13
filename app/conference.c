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
/* 任务堆栈大小及优先级 */
#define CONFERENCE_TASK_STACK_SIZE						(2048)
#define CONFERENCE_TASK_PRIORITY						(configMAX_PRIORITIES - 1)

#define NOTICE_QUEUE_LENGTH								(128)
#define NOTICE_QUEUE_SIZE								(sizeof(void *))

/* WIFI单元ID转换   ，WIFI单元ID从0x3000开始 */
#define WIFI_ID(_id)									(_id + WIFI_UNIT_START_ID)

/* 单元指令(type 或 cmd)与参数(ph,pl)合并拓展到32位，并加上标志位，方便在同时处理两个类型的单元的指令时区分或避免数字重复
	第一个字节为设备类型表示 第二个字节为指令(type或cmd) 第三、四个字节是参数(ph,pl)*/
#define WIRED_CMD(_type,_ph,_pl)						( 0x80000000 | (_type << 16) | (_ph << 8) | _pl)
#define WIFI_CMD(_cmd,_ph,_pl)							( 0x40000000 | (_cmd << 16) | (_ph << 8) | _pl)

/* 单元在线数量 */
#define WIRED_ONLINE_NUM								(OnlineNum.wiredChm + OnlineNum.wiredRps)
#define WIFI_ONLINE_NUM									(OnlineNum.wifiChm + OnlineNum.wifiRps)
#define UNIT_ONLINE_NUM 								(WIRED_ONLINE_NUM + WIFI_ONLINE_NUM)


/* 判断投票模式是否需要签到 */
#define IS_VOTE_NEED_SIGN(_mode)						(_mode == Key3First_Sign_vote 		 || _mode == Key3Last_Sign_vote 		||   	\
														 _mode == Key5First_Sign_Select 	 || _mode == Key5Last_Sign_Select  		||  	\
														 _mode == Key5First_Sign_Rate		 || _mode == Key5Last_Sign_Rate			||  	\
														 _mode == Key2First_Sign_CustomTerm  || _mode == Key2Last_Sign_CustomTerm	||  	\
														 _mode == Key3First_Sign_CustomTerm  || _mode == Key3Last_Sign_CustomTerm	||  	\
														 _mode == Key4First_Sign_CustomTerm  || _mode == Key4Last_Sign_CustomTerm	||  	\
														 _mode == Key5First_Sign_CustomTerm  || _mode == Key5Last_Sign_CustomTerm	||  	\
														 _mode == Key3First_Sign_Satisfaction|| _mode == Key3Last_Sign_Satisfaction)

/* 判断投票模式是否第一次有效 */
#define IS_VOTE_FIRST_VALID(_mode)						(_mode == Key3First_Sign_vote 		 || _mode == Key3First_NoSign_vote		||		\
														 _mode == Key5First_Sign_Select 	 || _mode == Key5First_NoSign_Select   	||  	\
														 _mode == Key5First_Sign_Rate		 || _mode == Key5First_NoSign_Rate		||  	\
														 _mode == Key2First_Sign_CustomTerm  || _mode == Key2First_NoSign_CustomTerm||  	\
														 _mode == Key3First_Sign_CustomTerm  || _mode == Key3First_NoSign_CustomTerm||  	\
														 _mode == Key4First_Sign_CustomTerm  || _mode == Key4First_NoSign_CustomTerm||  	\
														 _mode == Key5First_Sign_CustomTerm  || _mode == Key5First_NoSign_CustomTerm||  	\
														 _mode == Key3First_Sign_Satisfaction|| _mode == Key3First_NoSign_Satisfaction)

/* 判断音频文件名是否为特定本地录音文件 */
#define IS_SPECIFIC_AUD_REC_FILE(_fname)				(strlen(_fname) == 8 && _fname[0] == 'R' && (_fname[1] >= '0' && _fname[1] <= '9') \
														 && (_fname[2] >= '0' && _fname[1] <= '9') && (_fname[3] >= '0' && _fname[1] <= '9'))

/* 从特定本地录音文件名中提取索引 */
#define GET_SPECIFIC_AUD_REC_INDEX(_fname)				((_fname[1] - '0') * 100 + (_fname[2] - '0') * 10 + (_fname[3] - '0'))


/* 音乐列表最大长度 */
#define MUSIC_FILE_LIST_MAX_LEN						(64)

/* 定时任务计时器运行间隔(ms) */
#define TIMING_TASK_INTERVAL						(1000)

/* 时间计数器重置间隔(S) */
#define TIME_COUNT_RESET_INTERVAL					(36000)
/* 定时下发Real Time Clock间隔(S) */
#define BROADCAST_RTC_INTERVAL						(900)

/* ID重复等待时间(S) */
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


/* 内部调用 */
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
/* 投票模式码（全数字协议） */
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


/* 会议通知队列 */
static QueueHandle_t noticeQueue;

/* 运行定时任务定时器 */
static TimerHandle_t TimingTask;

/* 单元信息保存堆栈
总大小为最大单元数+1 因为单元没有0号,方便索引 */
static struct {
    UnitInfo_S wired[WIRED_UNIT_MAX_ONLINE_NUM + 1];
    UnitInfo_S wifi[WIFI_UNIT_MAX_ONLINE_NUM + 1];
} UnitInfo __attribute__((section("OcramHeap")));

/* 在线话筒数 */
static UnitOnlineNum OnlineNum;

/* 主机系统信息 */
static ConfSysInfo_S SysInfo;

/* 全数字（有线）主席机队列 */
static DataQueueHandler_S WiredChmMicQueue;

/* 全数字（有线）代表机队列 */
static DataQueueHandler_S WiredRpsMicQueue;

/* 全数字（有线）等待(申请)队列 */
static DataQueueHandler_S WiredWaitQueue;

/* 全数字（有线）音频通道队列 */
static DataQueueHandler_S WiredChannelQueue;

/* WIFI 主席机队列 */
static DataQueueHandler_S WifiChmMicQueue;

/* WIFI 代表机队列 */
static DataQueueHandler_S WifiRpsMicQueue;

/* WIFI 等待(申请)队列 */
static DataQueueHandler_S WifiWaitQueue;

/* WIFI 音频通道队列 */
static DataQueueHandler_S WifiChannelQueue;

/* 有线及无线单元申请开话筒队列(只用于申请模式) */
static DataQueueHandler_S ApplyQueue;

/* AFC控制GPIO   (D14 GPIO2_29) */
static HAL_GpioHandler AfcCtrl;

/* LineOut声音输出控制GPIO (H13 GPIO1_24) */
static HAL_GpioHandler LineOutCtrl;

/* 分区通道 Out1 ~ Out16 声音输出控制GPIO (L10 GPIO1_15) */
static HAL_GpioHandler PartOutCtrl;

/* 设备运行信号 (L12 GPIO1_20) */
static HAL_GpioHandler RunSignal;

/* 火警信号输入 (H14 GPIO1_14) */
static HAL_GpioHandler FireAlarmSignal;

/* 火警信号 */
static bool FireAlarm = false;

/* 有新单元上线 (标志位用于记录是否有新
单元上线，然后再定时任务中下发广播类的信息)*/
static struct {
    bool wiredUnit;
    bool wifiUnit;
} NewUpline;

/* USB音频录放控制句柄 */
static struct {
    /* MCU是否连接上U盘 */
    bool isMcuConnectUsb;
    /* wt2000是否连接U盘 */
    bool isWt2000ConnectUsb;
    /* 音频录放状态 */
    AudStaType_EN audSta;
    /* 录音或放音时间 */
    uint32_t runtime;
    /* 录音文件 */
    char recFile[25];
    /* 当前播放音乐索引 */
    uint8_t playIndex;
    /* 音乐文件数量 */
    uint8_t musicFileNum;
    /* 本地录音文件最大索引 */
    uint16_t recFileMaxIndex;
    /* 音乐文件列表 */
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

    /* GPIO初始化 */
    AfcCtrl = HAL_GpioInit(GPIO2, 29, kGPIO_DigitalOutput, 0, (gpio_interrupt_mode_t)null);
    LineOutCtrl = HAL_GpioInit(GPIO1, 24, kGPIO_DigitalOutput, 0, (gpio_interrupt_mode_t)null);
    PartOutCtrl = HAL_GpioInit(GPIO1, 15, kGPIO_DigitalOutput, 1, (gpio_interrupt_mode_t)null);
    RunSignal = HAL_GpioInit(GPIO1, 20, kGPIO_DigitalOutput, 0, (gpio_interrupt_mode_t)null);
    FireAlarmSignal = HAL_GpioInit(GPIO1, 14, kGPIO_DigitalInput, null, kGPIO_IntRisingOrFallingEdge);


    /* 初始化话筒队列 */
    WiredChmMicQueue = DataQueue.creat(WIRED_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WiredRpsMicQueue = DataQueue.creat(WIRED_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WiredWaitQueue = DataQueue.creat(WIRED_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WiredChannelQueue = DataQueue.creat(WIRED_UNIT_MAX_ALLWO_OPEN,sizeof(uint8_t));

    WifiChmMicQueue = DataQueue.creat(WIFI_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WifiRpsMicQueue = DataQueue.creat(WIFI_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WifiWaitQueue = DataQueue.creat(WIFI_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WifiChannelQueue = DataQueue.creat(WIFI_UNIT_MAX_ALLWO_OPEN,sizeof(uint8_t));

    ApplyQueue = DataQueue.creat(WIFI_UNIT_MAX_ALLWO_OPEN + WIRED_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));

    /* 复位单元音频通道队列 */
    Conference_ResetAudioChannel(kType_Unit_Wired);
    Conference_ResetAudioChannel(kType_Unit_Wifi);

    /* 设置USB状态监听 */
    UsbDisk.setListener(Conference_UsbStateListener);

    /* 音频录放状态监听 */
    Audio.setListener(Conference_AudioStateListener);

    /* 从数据库获取并初始化会议参数 */
    SysInfo.config = (SysCfg_S *)Database.getInstance(kType_Database_SysCfg);

    /* 有线单元配置连接到数据库 */
    unitCfg = (UnitCfg_S *)Database.getInstance(kType_Database_WiredCfg);
    for(id = 1; id <= WIRED_UNIT_MAX_ONLINE_NUM; id++) {
        UnitInfo.wired[id].config = &unitCfg[id];
    }

    /* WIFI单元配置连接到数据库 */
    unitCfg = (UnitCfg_S *)Database.getInstance(kType_Database_WifiCfg);
    for(id = 1; id <= WIFI_UNIT_MAX_ONLINE_NUM; id++) {
        UnitInfo.wifi[id].config = &unitCfg[id];
    }

    /* 初始化定时任务计时器 */
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


    /* 检查RTC电池及切换主页面 */
    if(!Time.getRst())
        Screen.togglePage(SP_BATTERY_ERR);
    else
        Screen.togglePage(SP_WELCOME);

    xTimerStart(TimingTask, 0);

    while(1) {
        xQueueReceive(noticeQueue, &notify, MAX_NUM);

        /* 判断通知来源 */

        /* 通知来源为外部控制及有线单元通讯任务 */
        if(notify->nSrc & (EX_CTRL_DEST | kType_NotiSrc_WiredUnit)) {
            Conference_MessageProcess(notify);
        }

        /* 通知来源为WIFI单元通讯任务 */
        else if(notify->nSrc & kType_NotiSrc_WifiUnit) {
            Conference_WifiUnitMessageProcess(notify);
        }

        /* 通知来源为屏幕控制通讯任务 */
        else if(notify->nSrc & kType_NotiSrc_ScreenCtrl) {
            Conference_ScreenMessageProcess(notify);
        }

        /* 通知来源为从单片机通讯任务 */
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


    /*** @TimingTask0 ： 运行灯 ***/
    runSignal = !runSignal;
    HAL_SetGpioLevel(RunSignal, runSignal);


    /*** @TimingTask1 ： 主屏锁定,检测外部控制连接状态,并将屏幕切换到相应状态 ***/
    /* 火警信号锁屏 */
    if(FireAlarm) {
        Screen.lock(SP_FIRE_WARNING);
    }

    /* ID重复 */
    else if(SysInfo.state.idDupicate) {
        Screen.lock(SP_ID_PEPEAT);
    }

    /* PC连接锁屏 */
    else if(ExternalCtrl.connectSta(kType_NotiSrc_PC) || ExternalCtrl.connectSta(kType_NotiSrc_Web)) {
        Screen.lock(SP_EX_CTRL_CONNECT);
    }

    /* 投票、签到过程中锁屏 */
    else if(SysInfo.state.sysMode != kMode_Conference) {
        if(SysInfo.state.sysMode == kMode_DevSign)
            Screen.lock(SP_SIGNING_IN);

        else if(SysInfo.state.sysMode == kMode_DevVote)
            Screen.lock(SP_VOTING);
    }

    /* 正常非锁屏 */
    else {
        if(Screen.isLock())
            Screen.unlock();
    }


    /*** @TimingTask2 ： 检测火警状态,并广播状态 ***/
    /* 启动警报 */
    if(FireAlarm) {
        alarmTmp = true;

        Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,CONFERENCE_MODE,FIRE_ALARM_SIGNAL,null,null);
        WiredUnit.transmit(&confProt);
        ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);

        WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,AlarmOrNot_MtoU_G,0,0));
    }
    /* 解除警报 */
    else if(alarmTmp == true && FireAlarm == false) {
        alarmTmp = false;

        Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,CONFERENCE_MODE,FIRE_ALARM_SIGNAL_CANCEL,null,null);
        WiredUnit.transmit(&confProt);
        ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);

        WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,AlarmOrNot_MtoU_G,1,0));
    }


    /*** @TimingTask3 ： 定时下发真实时间到单元 ***/
    if(Time.getRst() && (timeCnt % BROADCAST_RTC_INTERVAL == 0)) {
        Time.getNow(&time);
        Conference_UpdataUnitTime(&time);
    }


    /*** @TimingTask4 ： 检测新单元上线标志，广播自定义表决项  ***/
    if(NewUpline.wiredUnit || NewUpline.wifiUnit) {


        /* 下发自定义表决项 */
        if(SysInfo.state.sysMode == kMode_Vote &&  \
           SysInfo.state.voteMode >= Key2First_Sign_CustomTerm && \
           SysInfo.state.voteMode <= Key5Last_NoSign_CustomTerm) {

            for(i = 0; i < 5; i++) {
                uint8_t *item = SysInfo.state.voteCustomItem[i];
                uint8_t len = strlen((char *)item);

                Log.d("len = %d content = %s\r\n",len,item);

                /* 有线单元 */
                if(NewUpline.wiredUnit) {
                    if(len <= 3) {
                        WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,CUSTOM_VOTE_ITEM,i+1, item[0],(item[1] << 8) | item[2]));
                    } else {
                        WiredUnit.transWithExData(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,CUSTOM_VOTE_ITEM,i+1,item[0],(item[1] << 8) | item[2]),len - 3,&item[3]);
                    }
                }

                /* WIFI单元 */
                if(NewUpline.wifiUnit)
                    WifiUnit.transWithExData(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,PCUpdateCustomTerm_MtoU_G,i+1,0), len,item);

            }

        }
    }

    /*** @TimingTask5 ： 检测ID重复标志 ***/
    if(SysInfo.state.idDupicate) {
#if 0
        if(SysInfo.state.idDupCnt-- > 0) {
            /* 广播通知全数字会议系统 */
            WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID, BASIC_MSG, CONFERENCE_MODE, ID_DUPICATE,null,SysInfo.state.dupId));
            /* 广播通知全WIFI会议系统 */
            WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0, IDRepeatingMtoU_G, SysInfo.state.dupId >> 8, SysInfo.state.dupId & 0xFF));
        }
        /* 经过ID_DUPICATE_WAIT_TIME秒后，没有重新收到ID重复，解除状态 */
        else {
            SysInfo.state.idDupicate = false;

            /* 广播通知全数字会议系统 */
            WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID, BASIC_MSG, CONFERENCE_MODE, ID_UNDUPICATE,null,null));
            /* 广播通知全WIFI会议系统 */

            /**@brief WIFI系统没有取消ID重复协议，后续加上 */
        }
#else
        /* 广播通知全数字会议系统 */
        WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID, BASIC_MSG, CONFERENCE_MODE, ID_DUPICATE,null,SysInfo.state.dupId));
        /* 广播通知全WIFI会议系统 */
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

    /* 复位新上线标志 */
    NewUpline.wiredUnit = false;
    NewUpline.wifiUnit = false;


    /* 计时器每36000秒（10小时）重置一次 */
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

    /* 通知源 */
    NotifySrcType_EN noticeSrc;
    /* ID对应的单元类型 */
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
    /* 基本消息 0x80 */
    case BASIC_MSG: {
        uint8_t mode = protPara[0], cmd = protPara[1];

        switch(mode) {
        /* 会议模式 0x00 */
        case CONFERENCE_MODE: {
            switch(cmd) {
            case START_CONFERENCE_MODE: {
                Conference_ChangeSysMode(kMode_Conference);
            }
            break;
            /*主席申请开话筒*/
            case CHM_OPEN_MIC:
            /*代表申请开话筒*/
            case RPS_OPEN_MIC:
            /* 话筒撤销等待 */
            case MIC_DISWAIT:
            /*主席申请关话筒*/
            case CHM_CLOSE_MIC:
            /*代表申请关话筒*/
            case RPS_CLOSE_MIC: {
                /* id范围是WIFI单元，且控制指令来源于外部控制 */
//                if(idUnitType == kType_Unit_Wifi && (notify->nSrc & EX_CTRL_DEST))
//                    Conference_MicControl(id - WIFI_UNIT_START_ID, kType_Unit_Wifi, WIRED_CMD(cmd,0,0));
//                else if(idUnitType == kType_Unit_Wired)
//                    Conference_MicControl(id, kType_Unit_Wired, WIRED_CMD(cmd,0,0));
                Conference_MicControl(id, idUnitType, WIRED_CMD(cmd,0,0));
            }
            break;

            /* 同意开话筒 */
            case AGREE_OPEN_MIC:
            /* 不同意开话筒 */
            case DISAGREE_OPEN_MIC: {
                /* PC下发同意或不同意 */
//                if(idUnitType == kType_Unit_Wifi && (notify->nSrc & EX_CTRL_DEST))
//                    Conference_MicControl(id - WIFI_UNIT_START_ID, kType_Unit_Wifi, WIRED_CMD(cmd,0,0));
//                else if(idUnitType == kType_Unit_Wired && (notify->nSrc & EX_CTRL_DEST))
//                    Conference_MicControl(id, kType_Unit_Wired, WIRED_CMD(cmd,0,0));
                if(notify->nSrc & EX_CTRL_DEST)
                    Conference_MicControl(id, idUnitType, WIRED_CMD(cmd,0,0));

                /* 有线主席单元下发同意或不同意 */
                else if(notify->nSrc == kType_NotiSrc_WiredUnit)
                    Conference_MicControl(0, (UnitType_EN)null, WIRED_CMD(cmd,0,0));
            }
            break;

            /* 主席优先权 */
            case CHM_PRIORITY: {
                if(idUnitType == kType_Unit_Wired)
                    Conference_ChairmanPriority(kType_Unit_Wired,id);
            }
            break;

            /* 话筒上线 */
            case UNIT_ACCESS_SYS: {
                /* 话筒本身已经在线 */
                if(UnitInfo.wired[id].online) {
                    /* 下发MIC状态 */
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

                    /* 更新屏幕显示数量 */
                    Conference_ScreenUpdataUnitNum();

                    UnitInfo.wired[id].online = true;

                    Log.d("WiredUnit(%s) id = %d online ( Chm num = %d , Rps num = %d )\r\n",UnitInfo.wired[id].attr == aChairman ? "CHM":"RPS",id,OnlineNum.wiredChm,OnlineNum.wiredRps);
                }

                /* 通知外部控制话筒上线 */
                ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id,PC_MSG,UNIT_LOGIN,0x01,null,null));

                NewUpline.wiredUnit = true;

                /* 同步EQ 灵敏度参数 */
            }
            break;

            /*话筒掉线*/
            case UNIT_OFFLINE: {
                /* 关闭话筒，取消等待 */
                if(UnitInfo.wired[id].attr == aChairman) {
                    Conference_MicControl(id, kType_Unit_Wired, WIRED_CMD(CHM_CLOSE_MIC,0,0));
                    OnlineNum.wiredChm--;
                } else if(UnitInfo.wired[id].attr == aRepresentative) {
                    Conference_MicControl(id, kType_Unit_Wired, WIRED_CMD(RPS_CLOSE_MIC,0,0));
                    /* 如果是代表机，清除等待 */
                    Conference_MicControl(id, kType_Unit_Wired, WIRED_CMD(MIC_DISWAIT,0,0));
                    OnlineNum.wiredRps--;
                }

                /* 通知外部控制话筒掉线 */
                ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id,PC_MSG,UNIT_LOGOUT,0x01,null,null));

                /* 更新屏幕显示数量 */
                Conference_ScreenUpdataUnitNum();

                /* 如果掉线话筒为发起签到表决的话筒，将系统模式恢复到会议模式 */
                if(SysInfo.state.initSignVoteId == id && (SysInfo.state.sysMode == kMode_DevSign || SysInfo.state.sysMode == kMode_DevVote)) {
                    Conference_ChangeSysMode(kMode_Conference);
                }

                Log.d("WiredUnit(%s) id = %d offline ( Chm num = %d , Rps num = %d )\r\n",UnitInfo.wired[id].attr == aChairman ? "CHM":"RPS",id,OnlineNum.wiredChm,OnlineNum.wiredRps);
            }
            break;

            /* ID重复 */
            case ID_DUPICATE: {
                SysInfo.state.dupId = protPara[3] << 8 | protPara[4] ;

                if(SysInfo.state.sysMode != kMode_EditID) {
                    Log.d("Wired unit id repead !! ID = %d\r\n",SysInfo.state.dupId);

                    /* 广播通知全数字会议系统 */
                    WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID, BASIC_MSG, CONFERENCE_MODE, ID_DUPICATE,null,SysInfo.state.dupId));
                    /* 广播通知全WIFI会议系统 */
                    WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0, IDRepeatingMtoU_G, SysInfo.state.dupId >> 8, SysInfo.state.dupId & 0xFF));

                    SysInfo.state.idDupicate = true;
//					SysInfo.state.idDupCnt = ID_DUPICATE_WAIT_TIME;

//                    Screen.togglePage(SP_ID_PEPEAT);
                }
            }
            break;
            /* 发言计时 */
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
            /* 限时发言 */
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

            /* 单元机呼叫服务 */
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

            /* 火警信号 */
            case FIRE_ALARM_SIGNAL:
            case FIRE_ALARM_SIGNAL_CANCEL: {
                Log.d("Fire signal = %X\r\n",cmd);
            }
            break;

            }
        }
        break;
        /* 签到模式 0x01 */
        case SIGN_MODE: {
            switch(cmd) {
            /* 进入签到模式(控制端发起) */
            case START_SIGN_MODE: {
                SysInfo.state.totalSignNum = (uint16_t)((protPara[3] << 8) | protPara[4]);
                SysInfo.state.currentSignNum = 0;

                Conference_ChangeSysMode(kMode_Sign);
            }
            break;
            /* 进入签到模式(主席单元发起) */
            case DEV_START_SIGN_MODE: {
                /* 系统处于会议模式才允许切换 */
                if(SysInfo.state.sysMode != kMode_Conference)
                    break;

                SysInfo.state.totalSignNum = (uint16_t)(UNIT_ONLINE_NUM);
                SysInfo.state.currentSignNum = 0;
                /* 保存发起ID */
                SysInfo.state.initSignVoteId = id;

                /* 回应主席机 */
                WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,SIGN_MODE,DEV_START_SIGN_MODE,null,null));

                /* 切模式 */
                Conference_ChangeSysMode(kMode_DevSign);
            }
            break;
            /* 结束签到(控制端发起) */
            case END_SIGN_MODE:
            /* 结束签到(主席单元发起) */
            case DEV_END_SIGN_MODE: {
                Conference_ChangeSysMode(kMode_Conference);
            }
            break;
            /* 单元签到 */
            case UNIT_SIGN_IN: {
                /* 单元签到 */
                if((notify->nSrc & kType_NotiSrc_WiredUnit) && !UnitInfo.wired[id].sign) {
                    UnitInfo.wired[id].sign = true;
                    SysInfo.state.currentSignNum++;
                    Conference_SignInstruction(id,kType_Unit_Wired,WIRED_CMD(UNIT_SIGN_IN,0,0),null,null);
                    /* 下发签到总人数及已签到人数结果 */
                    Conference_SignInstruction(null,(UnitType_EN)null,WIRED_CMD(START_SIGN_MODE,0,1),SysInfo.state.totalSignNum,SysInfo.state.currentSignNum);
                }
                /* 如果单元签到来自外部控制，则为控制签到 */
                else if(notify->nSrc & (kType_NotiSrc_PC | kType_NotiSrc_Web | kType_NotiSrc_UartCtrl)) {
                    /* 控制有线单元签到 */
                    if(idUnitType == kType_Unit_Wired && !UnitInfo.wired[id].sign) {
                        Conference_SignInstruction(id,kType_Unit_Wired,WIRED_CMD(CONTROL_UNIT_SIGN,0,0),null,null);
                    }
                    /* 控制WIFI单元签到 */
                    else if(idUnitType == kType_Unit_Wifi  && !UnitInfo.wifi[id].sign) {
                        Conference_SignInstruction(id,kType_Unit_Wifi,WIFI_CMD(ControlSign_MtoU_D,0,0),null,null);
                    }
                }
            }
            break;
            /* 允许/禁止签到 */
            case SIGN_ENABLE:
            case SIGN_DISABLE: {

                if(id == UNIT_BROADCAST_ID) {
                    SysInfo.config->signEn = (cmd == SIGN_ENABLE);
                    WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,EnableSignOrNot_MtoU_D,!SysInfo.config->signEn,null));
                    WiredUnit.transmit(Protocol.conference(&confProt,UNIT_BROADCAST_ID,BASIC_MSG,SIGN_MODE,cmd,null,null));
                } else {
                    /* 判断设备类型 */
                    if(idUnitType == kType_Unit_Wifi)
                        WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,EnableSignOrNot_MtoU_D,!(cmd == SIGN_ENABLE),null));
                    else if(idUnitType == kType_Unit_Wired)
                        WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,SIGN_MODE,cmd,null,null));
                }
            }
            break;
//            /* 禁止签到 */
//            case SIGN_DISABLE: {
//
//            }
//            break;
            /* 补充签到 */
            case SUPPLEMENT_SIGN: {
                /* 有线单元补充签到 */
                if(idUnitType == kType_Unit_Wired && !UnitInfo.wired[id].sign) {
                    Conference_SignInstruction(id,kType_Unit_Wired,WIRED_CMD(SUPPLEMENT_SIGN,0,0),null,null);
                }
                /* WIFI单元补充签到*/
                else if(idUnitType == kType_Unit_Wifi && !UnitInfo.wifi[id].sign) {
                    Conference_SignInstruction(id,kType_Unit_Wifi,WIFI_CMD(SupplementSign_MtoU_D,0,0),null,null);
                }
            }
            break;
            /* 单元应答补充签到 */
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
        /* 投票模式 0x02 */
        case VOTE_MODE: {
            uint8_t unitVote;

            switch(cmd) {
            /* PC发起的各类型投票、表决、评级、满意度 */
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

            /* 主席发起 */
            case DEV_LAUNCH_VOTE: {
                if(!(notify->nSrc & kType_NotiSrc_WiredUnit) || SysInfo.state.sysMode != kMode_Conference)
                    break;

                SysInfo.state.voteMode = Key3Last_NoSign_vote;
                /* 保存发起ID */
                SysInfo.state.initSignVoteId = id;

                /* 回应主席机 */
                WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,VOTE_MODE,DEV_LAUNCH_VOTE,null,null));

                /* 切模式 */
                Conference_ChangeSysMode(kMode_DevVote);

            }
            break;

            /* 暂停表决（下发结果） */
            case PAUSE_VOTE: {
                /* 有线单元广播暂停表决(WIFI单元与结果是同一条指令) */
                WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,VOTE_MODE,PAUSE_VOTE,null,null));
                /* 下发投票结果 */
                Conference_VoteResult();
                SysInfo.state.voteMode = VotePause;
            }
            break;

            /* 结束表决 */
            case FINISH_VOTE:
            case DEV_FINISH_VOTE: {
                if(SysInfo.state.sysMode == kMode_Vote || SysInfo.state.sysMode == kMode_DevVote) {
                    SysInfo.state.voteMode = VotePause;
                    Conference_ChangeSysMode(kMode_Conference);
                }
            }
            break;

            /* 允许/禁止表决 */
            case VOTE_ENABLE:
            case VOTE_DISABLE: {
                if(id == UNIT_BROADCAST_ID) {
                    SysInfo.config->voteEn = ENABLE;

                    WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,EnableVoteOrNot_MtoU_D,!SysInfo.config->voteEn,null));
                    WiredUnit.transmit(Protocol.conference(&confProt,UNIT_BROADCAST_ID,BASIC_MSG,VOTE_MODE,cmd,null,null));
                } else {
                    /* 判断设备类型 */
                    if(idUnitType == kType_Unit_Wifi)
                        WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,EnableVoteOrNot_MtoU_D,!(cmd == VOTE_ENABLE),null));
                    else if(idUnitType == kType_Unit_Wired)
                        WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,VOTE_MODE,cmd,null,null));

                }
            }
            break;


            /* 单元投票、表决、选举 */
            case 0x25:
                unitVote = 1;//赞成
                goto unitVoted;
            case 0x26:
                unitVote = 2;//弃权
                goto unitVoted;
            case 0x27:
                unitVote = 3;//反对
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
                unitVote = 1;//很不满意(五键)/不满意(三键)
                goto unitVoted;
            case 0x4B:
                unitVote = 2;//不满意(五键)/一般(三键)
                goto unitVoted;
            case 0x4C:
                unitVote = 3;//一般(五键)/满意(三键)
                goto unitVoted;
            case 0x4D:
                unitVote = 4;//满意(五键)
                goto unitVoted;
            case 0x4E:
                unitVote = 5;//很满意(五键)
                goto unitVoted;
            case 0x65:
                unitVote = 1;//自定义表决选项1
                goto unitVoted;
            case 0x66:
                unitVote = 2;//自定义表决选项2
                goto unitVoted;
            case 0x67:
                unitVote = 3;//自定义表决选项3
                goto unitVoted;
            case 0x68:
                unitVote = 4;//自定义表决选项4
                goto unitVoted;
            case 0x69:
                unitVote = 5;//自定义表决选项5
                {
unitVoted:
                    if(notify->nSrc & kType_NotiSrc_WiredUnit && (SysInfo.state.sysMode == kMode_Vote || SysInfo.state.sysMode == kMode_DevVote)   \
                       && SysInfo.state.voteMode != VotePause) {
//                        /* 如果投票类型为需要先签到，而单元未进行签到 */
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
        /* 编ID模式 0x03 */
        case EDIT_ID_MODE: {
            switch(cmd) {

            /* 单元确认当前ID为本机ID */
            case UNIT_CONFIRM_ID: {
                Network_Mac_S *devMac = (Network_Mac_S *)&notify->exDataHead;

                Log.d("Wired confirm id = %d , dev mac = %X:%X:%X:%X:%X:%X \r\n",id,devMac->mac0,devMac->mac1,devMac->mac2,devMac->mac3,devMac->mac4,devMac->mac5);
                if(id == SysInfo.state.wiredCurEditID) {
                    /* 回复单元确定 */
                    WiredUnit.transWithExData(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,POLLING_MSG,EDIT_ID_POLLING,CONFIRM_ID,null,id),NETWORK_MAC_SIZE,(uint8_t *)devMac);

                    SysInfo.state.wiredCurEditID = (SysInfo.state.wiredCurEditID + 1) > WIRED_UNIT_MAX_ONLINE_NUM ? 1 : SysInfo.state.wiredCurEditID + 1;

                    WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,POLLING_MSG,EDIT_ID_POLLING,CURRENT_ID,null,SysInfo.state.wiredCurEditID));
                }
            }
            break;
            }
        }
        break;

        /* 单元控制 0x04 */
        case UNIT_CTRL: {
            /** @brief: 这里的EQ和灵敏度区别于AUDIO_MATRIX、AUDIO_MATRIX_INQUIRE(0xB0,0xB1)
            			字段，这里使用的协议是兼容PC软件（2代会议系统）协议。这个协议里面涉
            			及位运算，WEB端不好处理，因此在音频矩阵功能种新增了关于EQ和灵敏度的
            			协议 */

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

                /* 来自有线单元（EQ 灵敏度保存在单元，主机或PC需要主动请求获取） */
                if(notify->nSrc & kType_NotiSrc_WiredUnit) {
                    UnitInfo.wired[id].config->eqFreq[eq] = freq;
                    UnitInfo.wired[id].config->eqVol[eq] = 24 - vol;
                    ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id,BASIC_MSG,UNIT_CTRL,SET_UNTI_EQ,null,protPara[3] << 8 | protPara[4]));
                }
                /* 来自外部控制 */
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

            /* 灵敏度 */
            case SET_UNIT_SENSITIVITY: {
                /* 来自有线单元（EQ 灵敏度保存在单元，主机或PC需要主动请求获取） */
                if(notify->nSrc & kType_NotiSrc_WiredUnit) {
                    UnitInfo.wired[id].config->sensitivity = protPara[4];
                    ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id, BASIC_MSG, UNIT_CTRL, SET_UNIT_SENSITIVITY,null,protPara[4]));
                }
                /* 来自外部控制 */
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

            /* 声控灵敏度&声控时间 */
            case SET_VOICE_CTRL_SENSITIVITY:
            case SET_VOICE_CTRL_CLOSE_TIME: {
                uint8_t wifiCmd = (cmd == SET_VOICE_CTRL_SENSITIVITY ? SetVoiceSensitivity_MtoU_D : SetVoiceCloseTime_MtoU_D);

                if(id == WHOLE_BROADCAST_ID) {
                    /* 广播通知全数字会议系统 */
                    WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID, BASIC_MSG, UNIT_CTRL, cmd,null,protPara[4]));
                    /* 广播通知全WIFI会议系统 */
                    WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0, wifiCmd, protPara[4], null));
                } else {
                    if(idUnitType == kType_Unit_Wifi)
                        WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id, wifiCmd, protPara[4], null));
                    else if(idUnitType == kType_Unit_Wired)
                        WiredUnit.transmit(Protocol.conference(&confProt,id, BASIC_MSG, UNIT_CTRL, cmd,null,protPara[4]));
                }
            }
            break;

            /* 语言设置 */
            case SET_LANGUAGE(Chinese):
            case SET_LANGUAGE(English):
            case SET_LANGUAGE(Russian):
            case SET_LANGUAGE(French): {
                Protocol.conference(&confProt,UNIT_BROADCAST_ID,BASIC_MSG,UNIT_CTRL,cmd,null,null);

                if(id == UNIT_BROADCAST_ID) {
                    SysInfo.config->language = (Language_EN)(cmd - 0xF0);
                    /* 回复控制主机确定 */
                    ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
                    /* 广播到单元 */
                    WiredUnit.transmit(&confProt);
                    WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,null, SetLanguage_MtoU_G, SysInfo.config->language, null));
                    /* 设置屏幕语言 */
                    Screen.setLanguage(SysInfo.config->language);
                }
                /* 允许单独设置单元语言 */
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
    /* 状态消息 0x82*/
    case STATE_MSG: {
        switch(id) {
        case MODE_BROADCAST_ID:
        case WIFI_MODE_BROADCAST_ID: {
            uint8_t mode = protPara[0], num = protPara[1];

            if(mode >= kMode_Mic_Fifo && mode <= kMode_Mic_Apply)
                SysInfo.config->micMode = (MicMode_EN)mode;

            /* 设置有线单元 */
            if(id == MODE_BROADCAST_ID && num >= 1 && num <= WIRED_UNIT_MAX_ALLWO_OPEN) {
                SysInfo.config->wiredAllowOpen = num;
                SysInfo.config->wiredAllowWait = num;
            }

            /* 设置WIFI单元 */
            else if(id == WIFI_MODE_BROADCAST_ID && num >= 1 && num <= WIFI_UNIT_MAX_ALLWO_OPEN) {
                SysInfo.config->wifiAllowOpen = num;
                SysInfo.config->wifiAllowWait = num;
            }

            Database.saveSpecify(kType_Database_SysCfg,null);


            /* 关闭所有单元 */
            Conference_CloseAllMic(kType_Unit_Wired,aChairman);
            Conference_CloseAllMic(kType_Unit_Wired,aRepresentative);
            Conference_CloseAllMic(kType_Unit_Wifi,aChairman);
            Conference_CloseAllMic(kType_Unit_Wifi,aRepresentative);

            /* 清除申请队列 */
            Conference_ClearApplyMic();

            /* 广播模式及数量 */
            /* 发送到单元 */
            WiredUnit.transmit(Protocol.conference(&confProt,MODE_BROADCAST_ID,STATE_MSG,SysInfo.config->micMode,SysInfo.config->wiredAllowOpen,null,null));
            WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,ChangeMicManage_MtoU_G,SysInfo.config->micMode,SysInfo.config->wifiAllowOpen));
            /* 回复外部控制设备确认状态 */
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
            /* 设置主机时间 */
            Time.setNow(time);

            /* 下发时间到单元 */
            Conference_UpdataUnitTime(time);

            FREE(time);
        }
        break;

        }
    }
    break;
    /* 通讯消息 0x86*/
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

            /* 回复当前会议模式及有线单元最大话筒数 */
            ExternalCtrl.transmit(notify->nSrc,  \
                                  Protocol.conference(&confProt,MODE_BROADCAST_ID,STATE_MSG,SysInfo.config->micMode,SysInfo.config->wiredAllowOpen,null,null));
            /* 回复当前会议模式及WIFI单元最大话筒数 */
            ExternalCtrl.transmit(notify->nSrc,  \
                                  Protocol.conference(&confProt,WIFI_MODE_BROADCAST_ID,STATE_MSG,SysInfo.config->micMode,SysInfo.config->wifiAllowOpen,null,null));

            /* 回复当前IP地址、掩码、网关 */
            memcpy(data,SysInfo.config->ip,4);
            ExternalCtrl.transWithExData(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,CFG_IP,0x04,data[0],data[1] << 8 | data[2]),1,&data[3]);
            memcpy(data,SysInfo.config->gateWay,4);
            ExternalCtrl.transWithExData(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,CFG_GW,0x04,data[0],data[1] << 8 | data[2]),1,&data[3]);
            memcpy(data,SysInfo.config->mask,4);
            ExternalCtrl.transWithExData(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,CFG_MASK,0x04,data[0],data[1] << 8 | data[2]),1,&data[3]);


            /* 回复当前语言 */
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,UNIT_BROADCAST_ID,BASIC_MSG,UNIT_CTRL,SET_LANGUAGE(SysInfo.config->language),null,null));
        }
        break;

        /* 摄像头预置位配置 */
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

        /* 摄像头控制 0x14 */
        case CAMERA_CONTROL: {
            uint8_t *data,len = protPara[1];

            data = MALLOC(len);
            memcpy(&data[0],&protPara[2],3);
            memcpy(&data[3],&notify->exDataHead,len - 3);

            Camera.cmdSend(data,len);

            FREE(data);
        }
        break;

        /* 下发自定义表决项 0x24 */
        case CUSTOM_VOTE_ITEM: {
            uint8_t item = protPara[1];

            if(item >= 1 && item <= 5) {

                memset(SysInfo.state.voteCustomItem[item - 1],0,VOTE_CUSTOM_ITEM_LEN);

                memcpy(&SysInfo.state.voteCustomItem[item - 1][0],&protPara[2],3);
                if(notify->exLen > 0)
                    memcpy(&SysInfo.state.voteCustomItem[item - 1][3],&notify->exDataHead,notify->exLen);

                /* WIFI单元有自定义表决 */
                WifiUnit.transWithExData(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,PCUpdateCustomTerm_MtoU_G,item,0), \
                                         3 + notify->exLen,SysInfo.state.voteCustomItem[item - 1]);

                /* 直接转发到有线单元 */
                WiredUnit.transWithExData(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,CUSTOM_VOTE_ITEM,item, \
                                          protPara[2],(protPara[3] << 8) | protPara[4]),notify->exLen,&notify->exDataHead);
            }
        }
        break;

        /* 发起编ID   	0x2F */
        case PC_EDIT_ID: {
            if(protPara[1] == PC_START_EDIT_ID) {
                /* 开始编ID */
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

        /* 扫描在线单元ID 0x30 */
        case SCAN_ONLINE_UNIT: {
            uint8_t cmd,ch;
            /* 上传在线有线单元 */
            for(id = 1; id <= WIRED_UNIT_MAX_ONLINE_NUM; id++) {
                if(UnitInfo.wired[id].online) {
                    ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,id,PC_MSG,ONLINE_UNIT,UnitInfo.wired[id].attr,null,null));
                }
            }
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,SCAN_ONLINE_UNIT,SCAN_UNIT_END,kType_Unit_Wired, \
                                  (OnlineNum.wiredChm + OnlineNum.wiredRps)));

            /* 上传在线WIFI单元 */
            for(id = 1; id <= WIFI_UNIT_MAX_ONLINE_NUM; id++) {
                if(UnitInfo.wifi[id].online) {
                    ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WIFI_ID(id),PC_MSG,ONLINE_UNIT,UnitInfo.wifi[id].attr,null,null));
                }
            }
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,SCAN_ONLINE_UNIT,SCAN_UNIT_END,kType_Unit_Wifi, \
                                  (OnlineNum.wifiChm + OnlineNum.wifiRps)));

            /* 上传在线有线单元话筒状态 */
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

            /* 上传在线WIFI单元话筒状态 */
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

        /* 外部控制查询当前状态 */
        case QUERY_PRIOR_SIGN:
        case QUERY_PRIOR_VOTE:
        case QUERY_PRIOR_SCAN: {
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,cmd,0x01,SysInfo.state.sysMode,null));
        }
        break;
        }
    }
    break;

    /* 电子桌牌 */
    case NAMEPLATE_CONTENT:
    case NAMEPLATE_COMP:
    case NAMEPLATE_POS:
    case NAMEPLATE_NAME:
    case NAMEPLATE_UPDATE:
    case CONSTANT_NOTIFY: {
        uint8_t *data,len,isSave = 1;

        if(protType == NAMEPLATE_CONTENT)
            /* 消息不保存 */
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

    /* 音频矩阵 0xB0*/
    case AUDIO_MATRIX: {
        uint8_t ph = protPara[0],pl = protPara[1], *exdata, i;

        switch(ph) {
        /* DSP模式 */
        case DSP_MODE: {
            uint8_t output,input;

            switch(pl) {
            /* 有线及无线模式 */
            case DSP_MODE_WIRE:
            case DSP_MODE_WIFI: {
                /* 设置DSP */
                SysInfo.config->dspMode = pl;
                Dsp.setMode((DspSysMode_E)SysInfo.config->dspMode);
            }
            break;

            /* 分区模式 */
            case DSP_MODE_PARTITION: {
                /* 设置DSP */
                SysInfo.config->dspMode = pl;
                Dsp.setMode((DspSysMode_E)SysInfo.config->dspMode);

                for(output = DSP_OUTPUT_CH1; output <= DSP_OUTPUT_CH16 ; output++) {
                    /* 恢复话筒混音音量为最大 */
                    SysInfo.config->dsp[output].inputVol[DSP_ALL_MIC_MIX] = 0x1F;
                    /* 配置对应输入通道音量 */
                    for(input = 0; input < 7; input++)
                        Dsp.chInputSrc((DspOutput_E)output,(DspInputSrc_E)input,(DspVolume_E)(31 - SysInfo.config->dsp[output].inputVol[input]));
                }

            }
            break;

            /* 同传模式  */
            case DSP_MODE_SI: {
                /* 设置DSP */
                SysInfo.config->dspMode = pl;
                Dsp.setMode((DspSysMode_E)SysInfo.config->dspMode);

                /* 配置Out1通道音量 */
                for(input = 0; input < 7; input++)
                    Dsp.chInputSrc((DspOutput_E)DSP_OUTPUT_CH1,(DspInputSrc_E)input,(DspVolume_E)(31 - SysInfo.config->dsp[DSP_OUTPUT_CH1].inputVol[input]));
            }
            break;

            default:
                break;

            }

            /* 关闭所有单元 */
            Conference_CloseAllMic(kType_Unit_Wired,aChairman);
            Conference_CloseAllMic(kType_Unit_Wired,aRepresentative);
            Conference_CloseAllMic(kType_Unit_Wifi,aChairman);
            Conference_CloseAllMic(kType_Unit_Wifi,aRepresentative);

            /* 清除申请队列 */
            Conference_ClearApplyMic();

            /* 保存数据到FLASH */
            Database.saveSpecify(kType_Database_SysCfg,null);
            /* 回复收到 */
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,0xFF,null,null,null));

        }
        break;

        /* 单元分区音量设置 */
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
            /* 分区通道音量 */
            case 0x01: {
                memcpy(&unitInfo->config->chVol[0],&protPara[2],3);
                memcpy(&unitInfo->config->chVol[3],&notify->exDataHead,13);
                if(unitInfo->micSta == kStatus_UnitMic_Open) {
                    Conference_DspUnitCtrl(id,idUnitType,tOpenMic,unitInfo->channel);
                }
            }
            break;
            /* 单元EQ */
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

                    /* 适配原全数字会议协议下发配置EQ */
                    para = ((unitInfo->config->eqFreq[i]) << 5) | (24 - unitInfo->config->eqVol[i]);


                    if(idUnitType == kType_Unit_Wired) {
                        WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,UNIT_CTRL,SET_UNTI_EQ,0x00,(i + 1) << 12 | para));
                    } else if(idUnitType == kType_Unit_Wifi) {
                        WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,SetUnitEQ_MtoU_G,i + 1,para));
                    }

                }
            }
            break;
            /* 灵敏度 */
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

            /* 回复收到 */
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,0xFF,null,null,null));
            /* 保存数据到FLASH */
            Database.saveSpecify(saveType,id);
        }
        break;

        /* 普通输出口配置 */
        case DSP_NOR_OUT_CFG: {
            DspOutput_E dspOutput;

            if(!(pl >= 0x01 && pl <= 0x06 && id == WHOLE_BROADCAST_ID))
                break;

            /* 转换为DSP输出通道枚举  	     */
            dspOutput = (DspOutput_E)(pl + DSP_OUTPUT_CH16);

            switch(protPara[2]) {
            /* 配置音量 */
            case 0x01: {
                /* 判断并保存普通输出通道音量值 */
                SysInfo.config->dsp[dspOutput].vol = protPara[3] > 31 ? 31 : protPara[3];

                /* 写入DSP */
                Dsp.norOutput(dspOutput,DSP_OUTPUT_VOLUME,(DspEqPart_E)null,(uint8_t)(31 - SysInfo.config->dsp[dspOutput].vol));
            }
            break;
            /* 配置10段EQ音量 */
            case 0x02: {
                exdata = &notify->exDataHead;
                memcpy(&SysInfo.config->dsp[dspOutput].eqVol[0],&protPara[3],2);
                memcpy(&SysInfo.config->dsp[dspOutput].eqVol[2],&exdata[0],8);
                for(i = 0; i < 10; i++) {
                    /* 检查每一个EQ值是否合法 */
                    SysInfo.config->dsp[dspOutput].eqVol[i] = SysInfo.config->dsp[dspOutput].eqVol[i] > 20 ? 20 : SysInfo.config->dsp[dspOutput].eqVol[i];

                    /* 写入DSP */
                    Dsp.norOutput(dspOutput,DSP_OUTPUT_EQ,(DspEqPart_E)i,(uint8_t)SysInfo.config->dsp[dspOutput].eqVol[i]);
                }
            }
            break;
            /* 配置对应输入音量 */
            case 0x03: {
                exdata = &notify->exDataHead;
                memcpy(&SysInfo.config->dsp[dspOutput].inputVol[0],&protPara[3],2);
                memcpy(&SysInfo.config->dsp[dspOutput].inputVol[2],&exdata[0],5);

                for(i = 0; i < 7; i++) {
                    /* 检查每一个音量是否合法 */
                    SysInfo.config->dsp[dspOutput].inputVol[i] =  \
                            SysInfo.config->dsp[dspOutput].inputVol[i] > 31 ? 31 : SysInfo.config->dsp[dspOutput].inputVol[i];

                    /* 写入DSP */
                    Dsp.inputSrc(dspOutput,(DspInputSrc_E)i,(DspVolume_E)(31 - SysInfo.config->dsp[dspOutput].inputVol[i]));
                }
            }
            break;
            /* 配置下传 */
            case 0x04: {
                /* 目前只支持配置无线话筒下传开关 */
                if(dspOutput == DSP_OUTPUT_DOWN_WIFI) {
//                    SysInfo.config->dsp[DSP_OUTPUT_DOWN_WIFI].downTrans = protPara[3];
                    SysInfo.state.wifiAudDownward = protPara[3];
                    SlaveMcu.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,DSP_NOR_OUT_CFG,0x06,0x04,protPara[3] << 8));
                }
            }
            break;
            }

            /* 回复收到 */
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,0xFF,null,null,null));

            Database.saveSpecify(kType_Database_SysCfg,null);

        }
        break;

        /* 分区通道输出口配置 */
        case DSP_CHANNEL_OUT_CFG: {

            if(!(pl >= 0x00 && pl <= 0x0F && id == WHOLE_BROADCAST_ID))
                break;

            switch(protPara[2]) {
            case 0x01: {
                /* 判断并保存分区输出通道音量值 */
                SysInfo.config->dsp[pl].vol = protPara[3] > 31 ? 31 : protPara[3];
                /* 写入DSP */
                Dsp.chOutput((DspOutput_E)pl,DSP_OUTPUT_VOLUME,(DspEqPart_E)null,(uint8_t)(31 - SysInfo.config->dsp[pl].vol));
            }
            break;

            case 0x02: {
                exdata = &notify->exDataHead;
                memcpy(&SysInfo.config->dsp[pl].eqVol[0],&protPara[3],2);
                memcpy(&SysInfo.config->dsp[pl].eqVol[2],&exdata[0],8);

                for(i = 0; i < 10; i++) {
                    /* 检查每一个EQ值是否合法 */
                    SysInfo.config->dsp[pl].eqVol[i] = SysInfo.config->dsp[pl].eqVol[i] > 20 ? 20 : SysInfo.config->dsp[pl].eqVol[i];

                    /* 写入DSP */
                    Dsp.chOutput((DspOutput_E)pl,DSP_OUTPUT_EQ,(DspEqPart_E)i,(DspEqValue_E)SysInfo.config->dsp[pl].eqVol[i]);
                }
            }
            break;

            case 0x03: {
                exdata = &notify->exDataHead;
                memcpy(&SysInfo.config->dsp[pl].inputVol[0],&protPara[3],2);
                memcpy(&SysInfo.config->dsp[pl].inputVol[2],&exdata[0],5);

                for(i = 0; i < 7; i++) {
                    /* 检查每一个音量是否合法 */
                    SysInfo.config->dsp[pl].inputVol[i] = SysInfo.config->dsp[pl].inputVol[i] > 31 ? 31 : SysInfo.config->dsp[pl].inputVol[i];

                    /* 写入DSP */
                    Dsp.chInputSrc((DspOutput_E)pl,(DspInputSrc_E)i,(DspVolume_E)(31 - SysInfo.config->dsp[pl].inputVol[i]));
                }
            }
            break;

            case 0x04: {
                SysInfo.config->dsp[pl].dly = protPara[3] > 100 ? 100 : protPara[3];

                /* 写入DSP */
                Dsp.chOutput((DspOutput_E)pl,DSP_OUTPUT_DELAY,(DspEqPart_E)null,SysInfo.config->dsp[pl].dly);
            }
            break;
            }

            /* 回复收到 */
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,0xFF,null,null,null));

            Database.saveSpecify(kType_Database_SysCfg,null);
        }
        break;

        }
    }
    break;
    /* 音频矩阵查询 0xB1*/
    case AUDIO_MATRIX_INQUIRE: {
        uint8_t ph = protPara[0];

        switch(ph) {
        /* 外部控制主机查询DSP模式 */
        case DSP_MODE: {
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,DSP_MODE,SysInfo.config->dspMode,null,null));
        }
        break;

        /* 外部控制主机查询单元分区音量设置 */
        case DSP_UNIT_OUT_CFG: {
            UnitCfg_S *uCfg;

            if(idUnitType == kType_Unit_Wired) {
                uCfg = UnitInfo.wired[id].config;
            } else if(idUnitType == kType_Unit_Wifi) {
                uCfg = UnitInfo.wifi[id].config;
            } else
                break;

            /* 回复单元分区通道音量 */
            ExternalCtrl.transWithExData(notify->nSrc,Protocol.conference(&confProt,id,AUDIO_MATRIX,DSP_UNIT_OUT_CFG,0x01,  \
                                         uCfg->chVol[0],uCfg->chVol[1] << 8 | uCfg->chVol[2]),13,&uCfg->chVol[3]);
            /* 回复单元EQ频率及音量 */
            ExternalCtrl.transWithExData(notify->nSrc,Protocol.conference(&confProt,id,AUDIO_MATRIX,DSP_UNIT_OUT_CFG,0x02,  \
                                         uCfg->eqFreq[0],uCfg->eqFreq[1] << 8 | uCfg->eqFreq[2]),7,&uCfg->eqFreq[3]);
            /* 回复单元灵敏度 */
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,id,AUDIO_MATRIX,DSP_UNIT_OUT_CFG,0x03,uCfg->sensitivity,null));

        }
        break;

        /* 外部控制主机查询普通输出口配置 */
        case DSP_NOR_OUT_CFG: {
            uint8_t i;

            if(id != WHOLE_BROADCAST_ID)
                break;

            switch(protPara[2]) {
            case 0x01: {
                for(i = 1; i <= 6; i++) {
                    /* 回复6个输出通道音量 */
                    ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,DSP_NOR_OUT_CFG,  \
                                          i, 0x01, SysInfo.config->dsp[i + DSP_OUTPUT_CH16].vol << 8 | 0x00));
                }
            }
            break;

            case 0x02: {
                uint8_t *eqvol;

                for(i = 1; i <= 2; i++) {
                    eqvol = SysInfo.config->dsp[i + DSP_OUTPUT_CH16].eqVol;
                    /* 回复LineOut1 2的EQ频率及EQ值 */
                    ExternalCtrl.transWithExData(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,DSP_NOR_OUT_CFG,  \
                                                 i, 0x02, eqvol[0] << 8 | eqvol[1]),8,&eqvol[2]);
                }

            }
            break;

            case 0x03: {
                uint8_t *inputvol;

                for(i = 1; i <= 6; i++) {
                    inputvol = SysInfo.config->dsp[i + DSP_OUTPUT_CH16].inputVol;
                    /* 回复6个输出通道对应输入端的音量 */
                    ExternalCtrl.transWithExData(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,DSP_NOR_OUT_CFG,  \
                                                 i, 0x03, inputvol[0] << 8 | inputvol[1]),5,&inputvol[2]);
                }

            }
            break;

            case 0x04: {
                /* 回复下传开关状态 */
                ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,DSP_NOR_OUT_CFG,  \
                                      DSP_OUTPUT_DOWN_WIFI - DSP_OUTPUT_CH16, 0x04, SysInfo.state.wifiAudDownward << 8 | 0x00));
            }
            break;
            }

        }
        break;

        /* 外部控制主机查询分区通道输出口配置 */
        case DSP_CHANNEL_OUT_CFG: {
            uint8_t i;

            if(id != WHOLE_BROADCAST_ID)
                break;

            switch(protPara[2]) {
            case 0x01: {
                for(i = 0; i < 16; i++) {
                    /* 回复16个分区通道音量 */
                    ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,id,AUDIO_MATRIX,DSP_CHANNEL_OUT_CFG,  \
                                          i, 0x01,SysInfo.config->dsp[i].vol << 8 | 0x00));
                }
            }
            break;

            case 0x02: {
                uint8_t *eqvol;

                for(i = 0; i < 16; i++) {
                    eqvol = SysInfo.config->dsp[i].eqVol;
                    /* 回复16个分区通道的EQ频率及EQ值 */
                    ExternalCtrl.transWithExData(notify->nSrc,Protocol.conference(&confProt,id,AUDIO_MATRIX,DSP_CHANNEL_OUT_CFG,  \
                                                 i, 0x02, eqvol[0] << 8 | eqvol[1]),8,&eqvol[2]);
                }

            }
            break;

            case 0x03: {
                uint8_t *inputvol;

                for(i = 0; i < 16; i++) {
                    inputvol = SysInfo.config->dsp[i].inputVol;
                    /* 回复16个分区通道对应输入端的音量 */
                    ExternalCtrl.transWithExData(notify->nSrc,Protocol.conference(&confProt,id,AUDIO_MATRIX,DSP_CHANNEL_OUT_CFG,  \
                                                 i, 0x03, inputvol[0] << 8 | inputvol[1]),5,&inputvol[2]);
                }

            }
            break;

            case 0x04: {
                /* 回复16个分区通道的延时 */
                for(i = 0; i < 16; i++) {
                    /* 回复16个分区通道对应输入端的音量 */
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

	/* 单元回复轮询 */
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

            /* 如果投票类型为需要先签到，而单元未进行签到 */
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

    /* 单元上线 */
    case RepApplyEnterSystm_MtoU_D: {

        /* 话筒本身已经在线(热拔插) */
        if(UnitInfo.wifi[id].online) {
            /* 下发MIC状态 */
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

            /* 更新屏幕显示数量 */
            Conference_ScreenUpdataUnitNum();

            UnitInfo.wifi[id].online = true;

            Log.d("WifiUnit(%s) id = %d online ( Chm num = %d , Rps num = %d )\r\n",UnitInfo.wifi[id].attr == aChairman ? "CHM":"RPS",id,OnlineNum.wifiChm,OnlineNum.wifiRps);
        }

        /* 通知外部控制话筒上线 */
        ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),PC_MSG,UNIT_LOGIN,0x01,null,null));

        NewUpline.wifiUnit = true;

        /* 同步EQ 灵敏度参数 */
        WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,RepReadUnitMICSensitivity_UtoM_D,null,null));
        WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,ReadUnitEQ_MtoU_D,null,null));

    }
    break;

    /* 单元掉线 */
    case MasterAskforReenterSysMtoU_D: {
        if(UnitInfo.wifi[id].attr == aChairman) {
            Conference_MicControl(id, kType_Unit_Wifi, WIFI_CMD(UnitCloseMic_UtoM_D,0,0));
            OnlineNum.wifiChm--;
        } else if(UnitInfo.wifi[id].attr == aRepresentative) {
            Conference_MicControl(id, kType_Unit_Wifi, WIFI_CMD(UnitCloseMic_UtoM_D,0,0));
            OnlineNum.wifiRps--;
        }

        /* 通知外部控制话筒掉线 */
        ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),PC_MSG,UNIT_LOGOUT,0x01,null,null));

        /* 更新屏幕显示数量 */
        Conference_ScreenUpdataUnitNum();

        Log.d("WifiUnit(%s) id = %d offline ( Chm num = %d , Rps num = %d )\r\n",UnitInfo.wired[id].attr == aChairman ? "CHM":"RPS",id,OnlineNum.wiredChm,OnlineNum.wiredRps);
    }
    break;

    /*单元机申请开话筒*/
    case ApplyOpenMic_UtoM_D:
        Conference_MicControl(id, kType_Unit_Wifi, WIFI_CMD(cmd,0,0));
        break;

    /*单元机关开话筒*/
    case UnitCloseMic_UtoM_D:
        Conference_MicControl(id, kType_Unit_Wifi, WIFI_CMD(cmd,0,0));
        break;

    /*单元机退出等待队列*/
    case UnitGetoutWaitQuenen_UtoM_D:
        Conference_MicControl(id, kType_Unit_Wifi, WIFI_CMD(cmd,0,0));
        break;

    /*主席机执行优先权*/
    case ChairExecutePriority_UtoM_D:
        Conference_ChairmanPriority(kType_Unit_Wifi,id);
        break;

    /*主席机（不）同意代表机开话筒*/
    case ChairAgreeOpenOrNot_UtoM_D:
        Conference_MicControl(0, kType_Unit_Wifi, WIFI_CMD(ChairAgreeOpenOrNot_UtoM_D,ph,0));
        break;

    /*单元机服务申请    */
    case UnitApplyWater_UtoM_D: {
        ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),BASIC_MSG,CONFERENCE_MODE,pl,null,null));
    }
    break;


    /*回复主机帮单元机补签到   */
    case RepSupplementSign_UtoM_D:
        Conference_SignInstruction(id,kType_Unit_Wifi,WIFI_CMD(RepSupplementSign_UtoM_D,0,0), null, null);
        break;

    /*回复主席机发起签到   */
    case RepChairStarupOrEndSign_MtoU_D:
        break;

    /*回复主席机发起表决   */
    case RepChairStarupOrEndVote_MtoU_D:

        break;

    /*单元机以当前广播的ID作为本机ID   */
    case UnitGraspIDUtoM_D: {
        uint8_t *ip = &(&notify->exDataHead)[6],*mac = &(&notify->exDataHead)[0];
        uint16_t confirmID = (uint16_t)(ph << 8) | pl;

        Log.d("Wifi confirm ID = %d ip : %d.%d.%d.%d  mac : %X-%X-%X-%X-%X-%X \r\n",confirmID - 0x3000,ip[0],ip[1],ip[2],ip[3],mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
        if(confirmID == SysInfo.state.wifiCurEditID) {
            memcpy(&UnitInfo.wifi[id].ip,ip,NETWORK_IP_SIZE);
            memcpy(&UnitInfo.wifi[id].mac,mac,NETWORK_MAC_SIZE);
            /* 回复单元确定 */
            WifiUnit.transmit(kMode_Wifi_Unitcast,  \
                              Protocol.wifiUnit(&wifiProt,id,ConfirmUnitIDMtoU_D,(confirmID >> 8),(confirmID & 0xFF)));
            SysInfo.state.wifiCurEditID = (SysInfo.state.wifiCurEditID + 1) > WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM) ? WIFI_ID(1) : SysInfo.state.wifiCurEditID + 1;
            WifiUnit.transmit(kMode_Wifi_Multicast,  \
                              Protocol.wifiUnit(&wifiProt,0,EnterEditingIDMtoU_G,(SysInfo.state.wifiCurEditID >> 8),(SysInfo.state.wifiCurEditID & 0xFF)));
        }
    }
    break;

    /* 单元机通知主机电量、信号强度过低\正常 */
    case UnitCapacityStrChangeUtoM_D:
        break;

    /* 回复主机读取MIC灵敏度 */
    case RepReadUnitMICSensitivity_UtoM_D: {
        UnitInfo.wifi[id].config->sensitivity = ph;
    }
    break;

    /* 回复主机读取EQ */
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

        /* 广播通知全数字会议系统 */
        WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID, BASIC_MSG, CONFERENCE_MODE, ID_DUPICATE,null,SysInfo.state.dupId));
        /* 广播通知全WIFI会议系统 */
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
        case 0://出错返回功能
            if(para[0]==0x01) {
                switch(cmd) {
                case 0x01://IP
                    break;
                case 0x02://网关
                    break;
                case 0x03://子网掩码
                    break;
                case 0x04://端口配置
                    break;
                case 0x05:
                    break;
                default:
                    break;
                }
            }
            break;
        case 0x10://主界面中，屏向主机请求指令
            switch(cmd) {
            /* 会议模式 */
            case 0x01: {
                Screen.togglePage(SP_UNIT_TYPE);
            }
            break;

            /* 录音节界面 */
            case 0x02: {
                if(!UsbAudio.isMcuConnectUsb && !UsbAudio.isWt2000ConnectUsb) {
                    Screen.togglePage(SP_USB_INIT);
                    break;
                }

                switch(UsbAudio.audSta) {
                /* 切换暂停页面 */
                case kStatus_Aud_Idle:
//					case kStatus_Aud_Pause:
                {
                    Screen.togglePage(SP_AUD_PLAY_OR_RECORD);
                }
                break;

                /* 切换播放页面 */
                case kStatus_Aud_Playing: {
                    char *name = UsbAudio.musicFile[UsbAudio.playIndex - 1]->fname;

                    Screen.setVariable(SVR_AUD_PLAY_NAME,strlen(name)+1,(uint8_t *)name);
                    Screen.togglePage(SP_AUD_PLAYER_PLAYING);
                }
                break;
                /* 切换录音页面 */
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

            /* 系统状态 */
            case 0x03: {


                /* 切换页面 */
                Screen.togglePage(SP_SYS_STATE);

                /* 更新屏幕显示单元数量 */
                Conference_ScreenUpdataUnitNum();


            }
            break;

            /* 编ID界面 */
            case 0x04: {
                if(SysInfo.state.startID < 1 || SysInfo.state.startID > 4096)
                    SysInfo.state.startID = 1;
                /* 切换页面 */
                Screen.togglePage(SP_SET_ID);
                sprintf((char *)&exData[0],"%04d  ",SysInfo.state.startID);
//                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_START_ID),6,exData);
                Screen.setVariable(SVR_START_ID,6,exData);
            }
            break;

            /* 系统设置 */
            case 0x05: {
//				uint16_t year,mon,day;

                /* 切换页面 */
                Screen.togglePage(SP_SYS_SETTING);

//			    APP_GetBuildDate(&year, &mon, &day);

                /* 更新显示当前系统版本号 */
//                sprintf((char *)&exData[0],"M_%s_%04d%02d%02d",APP_VERSION,year,mon,day);
//                Screen.setVariable(SVR_SOFTWARE_VERSION_M,16,exData);

                /* 更新显示代码编译日期 */
//                sprintf((char *)&exData[0],"S_%s_%d%d%d",APP_VERSION,year,mon,day);
//                Screen.setVariable(SVR_SOFTWARE_VERSION_S,16,exData);
            }
            break;

            /*译员机*/
            case 0x06: {
//                /* 切换页面 */
//                Screen.togglePage(StartTranSetID_Page);
            }
            break;
            /* ID 模式 */
//            case 0x07: {
//                /* 切换页面 */
//                Screen.togglePage(SP_SYS_SETTING);
//            }
//            break;
            default:
                break;
            }
            break;
        /* 切换话筒模式(0x12)及数量(0x14) */
        case 0x12:
        case 0x14: {
            uint8_t mode;

            if(reg == 0x12) {
                mode = cmd + 1;
                if(mode >= kMode_Mic_Fifo && mode <= kMode_Mic_Apply)
                    SysInfo.config->micMode = (MicMode_EN)mode;
            } else if(reg == 0x14) {
                /* 设置有线单元 */
                if(cfgWitchTypeUnit == kType_Unit_Wired && cmd >= 0 && cmd <= 3) {
                    static const uint8_t Num[4] = {1,2,4,8};

                    SysInfo.config->wiredAllowOpen = Num[cmd];
                    SysInfo.config->wiredAllowWait = Num[cmd];
                }

                /* 设置WIFI单元 */
                else if(cfgWitchTypeUnit == kType_Unit_Wifi && cmd >= 0 && cmd <= 3) {
                    static const uint8_t Num[4] = {1,2,4,6};

                    SysInfo.config->wifiAllowOpen = Num[cmd];
                    SysInfo.config->wifiAllowWait = Num[cmd];
                }
            }

            /* 关闭所有单元 */
            Conference_CloseAllMic(kType_Unit_Wired,aChairman);
            Conference_CloseAllMic(kType_Unit_Wired,aRepresentative);
            Conference_CloseAllMic(kType_Unit_Wifi,aChairman);
            Conference_CloseAllMic(kType_Unit_Wifi,aRepresentative);

            /* 清除申请队列 */
            Conference_ClearApplyMic();

            /* 广播模式及数量 */
            /* 发送到单元 */
            WiredUnit.transmit(Protocol.conference(&confProt,MODE_BROADCAST_ID,STATE_MSG,SysInfo.config->micMode,SysInfo.config->wiredAllowOpen,null,null));
            WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,ChangeMicManage_MtoU_G,SysInfo.config->micMode,SysInfo.config->wifiAllowOpen));
            /* 回复外部控制设备确认状态 */
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,MODE_BROADCAST_ID,STATE_MSG,SysInfo.config->micMode,SysInfo.config->wiredAllowOpen,null,null));
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_MODE_BROADCAST_ID,STATE_MSG,SysInfo.config->micMode,SysInfo.config->wifiAllowOpen,null,null));

            Database.saveSpecify(kType_Database_SysCfg,null);
        }
        break;
        case 0x2B:  //0 1 2 3无线会议模式中，切换话筒模式
            break;
        /* 编ID起始ID号 */
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
        case 0x24:  //译员机模式配置
            break;
        case 0x26:  //译员机编ID
            break;
        /* 发起或者结束编ID */
        case 0x2A: {
            if(cmd == 0x02) {
                /* 切换页面 */
                Screen.togglePage(SP_SET_ID_END);
                SysInfo.state.wiredCurEditID = SysInfo.state.startID;
                SysInfo.state.wifiCurEditID = WIFI_ID(SysInfo.state.startID > WIFI_UNIT_MAX_ONLINE_NUM ? WIFI_UNIT_MAX_ONLINE_NUM : SysInfo.state.startID);

                /* 切换会议模式 */
                Conference_ChangeSysMode(kMode_EditID);
            } else if(cmd == 0x03) {
                /* 切换页面 */
                Screen.togglePage(SP_MAIN_MENU);

                /* 向有线单元发送结束编ID */
                WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,EDIT_ID_MODE,END_EDIT_ID_MODE,null,null));

                /* 切换会议模式 */
                Conference_ChangeSysMode(kMode_Conference);
//                WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,MasterStarUp_MtoU_G,0,0));
            }
        }
        break;

        case 0x16:  //屏发来低音
            break;
        case 0x18:  //屏发来高音
            break;
        case 0x20:  //屏发来背景音乐音量
            break;
        /* 系统设置 */
        case 0x3A: {
            switch(cmd) {
            /* 语言选择 */
            case 0x01: {
                /* 切换页面 */
                Screen.togglePage(SP_SET_LANGUAGE);
//                exData[1] = SysInfo.config->language;
//                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,0x0040),2,exData);
            }
            break;

            /* 屏进入IP配置界面，此时需要主机更新显示 */
            case 0x02: {
                uint8_t *ip,*mask,*gw;

                ip = SysInfo.config->ip;
                mask = SysInfo.config->mask;
                gw = SysInfo.config->gateWay;

                /* 切换页面 */
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

            /* 显示设置-亮度调节 */
            case 0x04: {
                /* 切换页面 */
                Screen.togglePage(SP_SET_DISPLAY);

                exData[0] = 0;
                exData[1] = SysInfo.config->brightness;
//                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_BRIGHTNESS),2,exData);
                Screen.setVariable(SVR_BRIGHTNESS,2,exData);
            }
            break;

            /* 音量调节 */
            case 0x06: {
                /* 切换页面 */
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

            /* 下传功能 */
            case 0x07: {
                /* 切换页面 */
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

        /* 屏发来录音处理按键 */
        case 0x3B: {
            switch(cmd) {
            case 0x00://上一曲
                Audio.previous();
                break;

            case 0x01://播放
//            	if(UsbAudio.audSta == kStatus_Aud_Idle || UsbAudio.audSta == kStatus_Aud_Pause)
                Audio.playPause();
                break;

            case 0x02://下一曲
                Audio.next();
                break;

            /* 开始录音 */
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

            /* 停止录音 */
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

            /* 播放暂停 */
            case 0x05:
                Audio.playPause();
                break;



            /* 返回 */
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

        /* 关无线单元 & 恢复默认 */
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

        case 0x40: {//屏发过来语言选择
            if(cmd >= Chinese && cmd <= French) {
                SysInfo.config->language = cmd;
                ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,UNIT_BROADCAST_ID,BASIC_MSG,UNIT_CTRL,SET_LANGUAGE(cmd),null,null));
            }
        }
        break;
        /* 屏发来的下传功能设置 */
        case 0x41: {
            SysInfo.state.wifiAudDownward = !cmd;
            SlaveMcu.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,DSP_NOR_OUT_CFG,0x06,0x04,(!cmd) << 8));
        }
        break;
        /* 屏发过来请求更新屏幕当前亮度值的 */
        case 0x44: {
            if(cmd >= 0 && cmd <= 64) {
                SysInfo.config->brightness = cmd;
                Screen.backlight(cmd);
                Database.saveSpecify(kType_Database_SysCfg,null);
            }
        }
        break;
        case 0x45://屏发过来请求更新低音值的
            break;
        case 0x50: //屏发来的ID模式设置
            break;

        /* 重新编ID */
        case 0x51: {
            /* 关闭ID重复 */
            SysInfo.state.idDupicate = false;

            /* 切换页面 */
            Screen.unlock();
            Screen.togglePage(SP_RESET_ID);
            SysInfo.state.wiredCurEditID = 1;
            SysInfo.state.wifiCurEditID = WIFI_ID(1);
            Conference_ChangeSysMode(kMode_EditID);
        }
        break;
        /* 结束重新编ID */
        case 0x52: {
            /* 切换页面 */
            Screen.togglePage(SP_MAIN_MENU);
            Conference_ChangeSysMode(kMode_Conference);
//            WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,MasterStarUp_MtoU_G,0,0));
        }
        break;
        /* 屏发来录/放音选择 */
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

        /* 屏发来会议主机类型选择返回 */
        case 0x54:
            switch(cmd) {
            /* 无线 */
            case 0x00: {
                static const uint8_t Reg[7] = {null,0,1,null,2,null,3};

                cfgWitchTypeUnit = kType_Unit_Wifi;

                exData[0] = 0;
                exData[1] = 1;
                Screen.setVariable(SVR_CONF_MODE,2,exData);

                /* 切换页面 */
                Screen.togglePage(SP_CONF_MODE);

                exData[1] = SysInfo.config->micMode - 1;
                Screen.setVariable(SVR_SET_MIC_MODE,2,exData);

                exData[1] = Reg[SysInfo.config->wifiAllowOpen];
                Screen.setVariable(SVR_SET_MIC_NUM,2,exData);
            }
            break;

            /* 有线 */
            case 0x01: {
                static const uint8_t Reg[9] = {null,0,1,null,2,null,null,null,3};

                cfgWitchTypeUnit = kType_Unit_Wired;

                exData[0] = 0;
                exData[1] = 0;
                Screen.setVariable(SVR_CONF_MODE,2,exData);

                /* 切换页面 */
                Screen.togglePage(SP_CONF_MODE);

                exData[1] = SysInfo.config->micMode - 1;
                Screen.setVariable(SVR_SET_MIC_MODE,2,exData);

                exData[1] = Reg[SysInfo.config->wiredAllowOpen];
                Screen.setVariable(SVR_SET_MIC_NUM,2,exData);
            }
            break;
            }
            break;

        /* 屏发来升降话筒控制 */
        case 0x55:
            switch(cmd) {
            case 0x00://上升
                break;
            case 0x01://下降
                break;
            }
            break;
        case 0x56: { //屏发来AFC控制
            switch(cmd) {
            case 0x00://关闭
                HAL_SetGpioLevel(AfcCtrl, 0);
                break;
            case 0x01://打开
                HAL_SetGpioLevel(AfcCtrl, 1);
                break;
            }
        }
        break;
        /* 设置LineOut音量 */
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

    /* 更新有线单元在线总数 */
    num = OnlineNum.wiredChm + OnlineNum.wiredRps;
    exData[0]= (uint8_t)(num >> 8);
    exData[1]= (uint8_t)(num & 0xFF);
    Screen.setVariable(SVR_WIRED_OL_TOTAL_NUM,2,exData);


    /* 更新有线主席机在线数量 */
    exData[0]= (uint8_t)(OnlineNum.wiredChm >> 8);
    exData[1]= (uint8_t)(OnlineNum.wiredChm & 0xFF);
    Screen.setVariable(SVR_WIRED_OL_CHM_NUM,2,exData);

    /* 更新有线代表机在线数量  */
    exData[0]= (uint8_t)(OnlineNum.wiredRps >> 8);
    exData[1]= (uint8_t)(OnlineNum.wiredRps & 0xFF);
    Screen.setVariable(SVR_WIRED_OL_RPS_NUM,2,exData);

    /* 更新无线单元在线总数 */
    num = OnlineNum.wifiChm + OnlineNum.wifiRps;
    exData[0]= (uint8_t)(num >> 8);
    exData[1]= (uint8_t)(num & 0xFF);
    Screen.setVariable(SVR_WIFI_OL_TOTAL_NUM,2,exData);

    /* 更新无线主席机在线数量 */
    exData[0]= (uint8_t)(OnlineNum.wifiChm >> 8);
    exData[1]= (uint8_t)(OnlineNum.wifiChm & 0xFF);
    Screen.setVariable(SVR_WIFI_OL_CHM_NUM,2,exData);

    /* 更新无线代表机在线数量 */
    exData[0]= (uint8_t)(OnlineNum.wifiRps >> 8);
    exData[1]= (uint8_t)(OnlineNum.wifiRps & 0xFF);
    Screen.setVariable(SVR_WIFI_OL_RPS_NUM,2,exData);


    /* 更新译员机在线数量 */
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


    /* 开始签到 */
    /** @brief 有线单元协议是开始签到和签到数量分开两条协议，wifi单元协议是同一条
               因此这里开始签到会同时下发签到数量连着 */
    case WIRED_CMD(START_SIGN_MODE,0,0):
    case WIFI_CMD(EnterSignMode_MtoU_G,0,0): {
        WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,SIGN_MODE,START_SIGN_MODE,null,null));

    }

    /* 下发签到总数和签到当前数量 */
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

    /* 单元签到 */
    case WIRED_CMD(UNIT_SIGN_IN,0,0):
    case WIFI_CMD(RepPollUnitl_UtoM_D,kMode_Sign,0): {
        if(id != null) {
            id = type == kType_Unit_Wired ? id : WIFI_ID(id);
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id,BASIC_MSG,SIGN_MODE,UNIT_SIGN_IN,null,null));
        }
    }
    break;

    /* 控制单元签到 */
    case WIRED_CMD(CONTROL_UNIT_SIGN,0,0):
    case WIFI_CMD(ControlSign_MtoU_D,0,0): {
        if(type == kType_Unit_Wired)
            WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,SIGN_MODE,CONTROL_UNIT_SIGN,null,null));
        else if(type == kType_Unit_Wifi)
            WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,ControlSign_MtoU_D,null,null));
    }
    break;

    /* 补充签到 */
    case WIRED_CMD(SUPPLEMENT_SIGN,0,0):
    case WIFI_CMD(SupplementSign_MtoU_D,0,0): {
        if(type == kType_Unit_Wired)
            WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,SIGN_MODE,SUPPLEMENT_SIGN,null,null));
        else if(type == kType_Unit_Wifi)
            WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,SupplementSign_MtoU_D,null,null));
    }
    break;

    /* 回复补充签到 */
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

    /* 统计线有线单元投票结果 */
    for(id = 1; id <= WIRED_UNIT_MAX_ONLINE_NUM; id++) {
        if(UnitInfo.wired[id].online && UnitInfo.wired[id].vote > 0 && UnitInfo.wired[id].vote <= 5)
            res[UnitInfo.wired[id].vote - 1] += 1;
    }

    /* 统计线WIFI单元投票结果 */
    for(id = 1; id <= WIFI_UNIT_MAX_ONLINE_NUM; id++) {
        if(UnitInfo.wifi[id].online && UnitInfo.wifi[id].vote > 0 && UnitInfo.wifi[id].vote <= 5)
            res[UnitInfo.wifi[id].vote - 1] += 1;
    }

    total = UNIT_ONLINE_NUM;

    /* 下发表决结果 */
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
    /* 开话筒 */
    case tOpenMic: {
        switch(SysInfo.config->dspMode) {
        /* 同传模式下配置 */
        case DSP_MODE_PARTITION: {
            dspOut[0] = dspOutVol[0] = 16;
            for(i = 0; i < 16; i++) {
                dspOut[i + 1] = i;
                dspOutVol[i + 1] = 31 - unitInfo[id].config->chVol[i];
            }
            Dsp.unitCtrl(dspUnitSrc,dspOut,dspOutVol);
        }
        break;
        /* 有线及无线模式下配置 */
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

    /* 关话筒 */
    case tCloseMic: {
        switch(SysInfo.config->dspMode) {
        /* 同传模式下配置 */
        case DSP_MODE_PARTITION: {
            dspOut[0] = dspOutVol[0] = 16;
            for(i = 0; i < 16; i++) {
                dspOut[i + 1] = i;
                dspOutVol[i + 1] = DSP_VOLUME_N144_DB;
            }
            Dsp.unitCtrl(dspUnitSrc,dspOut,dspOutVol);
        }
        break;
        /* 有线及无线模式下配置 */
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
    /* 开话筒 */
    case tOpenMic: {
        arg = (attr == aChairman) ? CHM_OPEN_MIC : RPS_OPEN_MIC;
        if(type == kType_Unit_Wired) {
            Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,arg,ch,null);
            WiredUnit.transmit(&confProt);
            ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
            UnitInfo.wired[id].channel = ch;
            UnitInfo.wired[id].micSta = kStatus_UnitMic_Open;

            /* 配置DSP */
            Conference_DspUnitCtrl(id,type,tOpenMic,ch);
            /* 配置摄像跟踪预制位 */
            Camera.call(id,type);
        } else if(type == kType_Unit_Wifi) {
            data[0] = ch;
            len = 1;
            WifiUnit.transWithExData(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,MasterOpenOrCloseMic_MtoU_D,0,null),len,data);
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),BASIC_MSG,CONFERENCE_MODE,arg,ch,null));
            UnitInfo.wifi[id].channel = ch;
            UnitInfo.wifi[id].micSta = kStatus_UnitMic_Open;

            /* 配置DSP */
            Conference_DspUnitCtrl(id,type,tOpenMic,ch);
            /* 配置摄像跟踪预制位 */
            Camera.call(id,type);
        }
    }
    break;

    /* 关话筒 */
    case tCloseMic: {
        arg = (attr == aChairman) ? CHM_CLOSE_MIC : RPS_CLOSE_MIC;
        if(type == kType_Unit_Wired) {
            Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,arg,null,null);
            WiredUnit.transmit(&confProt);
            ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
            UnitInfo.wired[id].channel = null;
            UnitInfo.wired[id].micSta = kStatus_UnitMic_Close;

            /* 配置DSP */
            Conference_DspUnitCtrl(id,type,tCloseMic,ch);
            /* 释放预置位 */
            Camera.release(id,type);
        } else if(type == kType_Unit_Wifi) {
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),BASIC_MSG,CONFERENCE_MODE,arg,null,null));
            WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,MasterOpenOrCloseMic_MtoU_D,1,null));
            UnitInfo.wifi[id].channel = null;
            UnitInfo.wifi[id].micSta = kStatus_UnitMic_Close;

            /* 配置DSP */
            Conference_DspUnitCtrl(id,type,tCloseMic,ch);
            /* 释放预置位 */
            Camera.release(id,type);
        }

    }
    break;

    /* 话筒等待 */
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

    /* 话筒取消等待 */
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

    /* 话筒已满 */
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

    /* 话筒申请 */
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

    /* 撤销申请 */
    case tRevokeApply: {
        /* 撤销申请不需要ID */
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

    /* 获取队列中当前打开话筒数量 */
    chmMicNum = DataQueue.getSize(chmMicQueue);
    rpsMicNum = DataQueue.getSize(rpsMicQueue);
    waitNum = DataQueue.getSize(waitQueue);

    switch(cmd) {

    /* 开话筒 */
    case WIRED_CMD(CHM_OPEN_MIC,0,0):
    case WIRED_CMD(RPS_OPEN_MIC,0,0):
    case WIFI_CMD(ApplyOpenMic_UtoM_D,0,0): {

        /* 主席申请开话筒 */
        if(unitInfo[id].attr == aChairman) {
            switch(SysInfo.config->micMode) {
            /*** 先进先出模式 ***/
            case kMode_Mic_Fifo:
            /*** 正常模式 ***/
            case kMode_Mic_Normal:
            /*** 声控模式 ***/
            case kMode_Mic_VoiceCtrl:
            /*** 申请模式 ***/
            case kMode_Mic_Apply: {
                /* 查找话筒队列是否已经存在对应ID的话筒 */
                if(DataQueue.search(chmMicQueue,&id)) {
                    ch = unitInfo[id].channel;
                    Conference_MicCtrlInstruction(id,type,aChairman,tOpenMic,ch);
                    break;
                }

                /* 主席+代表开话筒数量未超过允许开话筒数 */
                if(chmMicNum + rpsMicNum < allowOpen) {
                    DataQueue.enter(chmMicQueue,&id);
                    ch = Conference_GetAudioChannel(type);

                    Conference_MicCtrlInstruction(id,type,aChairman,tOpenMic,ch);
                }
                /* 主席+代表开话筒数量等于允许开话筒数 */
                else {

                    if(rpsMicNum > 0) {
                        /* 如果有开着的代表话筒，就挤一个出来 */
                        uint16_t closeId;

                        /* 队列取出代表单元并关闭 */
                        DataQueue.exit(rpsMicQueue,&closeId);
                        ch = unitInfo[closeId].channel;
                        Conference_MicCtrlInstruction(closeId,type,aRepresentative,tCloseMic,ch);

                        /* 打开主席 */
                        DataQueue.enter(chmMicQueue,&id);
                        Conference_MicCtrlInstruction(id,type,aChairman,tOpenMic,ch);
                    } else {
                        /* 话筒已满 */
                        Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tMicFull,null);
                    }
                }

                /* 如果在申请模式下,且申请(等待)队列不为空，检查申请队列 */
                if(waitNum > 0) {
                    /* 重新获取主席机打开队列数量 */
                    chmMicNum = DataQueue.getSize(chmMicQueue);

                    /* 如果正在打开话筒（主席+代表）数量 + 等待数量 > 允许打开的数量
                       则取出等待队列中的话筒并取消等待 */
                    if(waitNum + chmMicNum + rpsMicNum > allowOpen) {
                        uint16_t closeId;

                        /* 申请(等待)队列取出单元并取消等待 */
                        DataQueue.exit(waitQueue,&closeId);
                        Conference_MicCtrlInstruction(closeId,type,aRepresentative,tDisWait,null);

                        /* 申请模式下 需要处理申请队列 */
                        if(SysInfo.config->micMode == kMode_Mic_Apply) {
                            closeId = (type == kType_Unit_Wired ? closeId : WIFI_ID(closeId));

                            /* 申请队列删除话筒ID */
                            DataQueue.deleted(ApplyQueue,DataQueue.search(ApplyQueue,&closeId));

                            if(DataQueue.getSize(ApplyQueue) == 0) {
                                Conference_MicCtrlInstruction(null,(UnitType_EN)null, (UnitAttr_EN)null, tRevokeApply, null);
                            } else {
                                /* 队列头的ID申请开话筒 */
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

        /* 代表申请开话筒 */
        else if(unitInfo[id].attr == aRepresentative) {
            switch(SysInfo.config->micMode) {
            /*** 先进先出模式 ***/
            case kMode_Mic_Fifo: {
                /* 查找话筒队列是否已经存在对应ID的话筒 */
                if(DataQueue.search(rpsMicQueue,&id)) {
                    ch = unitInfo[id].channel;
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tOpenMic,ch);
                    break;
                }

                /* 主席+代表开话筒数量未超过允许开话筒数 */
                if(chmMicNum + rpsMicNum < allowOpen) {
                    ch = Conference_GetAudioChannel(type);
                    DataQueue.enter(rpsMicQueue,&id);
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tOpenMic,ch);
                }
                /* 主席+代表开话筒数量等于允许开话筒数 */
                else {
                    if(rpsMicNum > 0) {
                        uint16_t closeId;

                        /* 队列取出代表单元并关闭 */
                        DataQueue.exit(rpsMicQueue,&closeId);
                        ch = unitInfo[closeId].channel;
                        Conference_MicCtrlInstruction(closeId,type,aRepresentative,tCloseMic,ch);
                        /* 打开代表 */
                        DataQueue.enter(rpsMicQueue,&id);
                        Conference_MicCtrlInstruction(id,type,aRepresentative,tOpenMic,ch);
                    } else {
                        /* 话筒已满 */
                        Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tMicFull,null);
                    }
                }
            }
            break;
            /*** 正常模式 ***/
            case kMode_Mic_Normal:
            /*** 声控模式 ***/
            case kMode_Mic_VoiceCtrl: {
                /* 查找话筒队列是否已经存在对应ID的话筒 */
                if(DataQueue.search(rpsMicQueue,&id)) {
                    /* 重新发送打开话筒指令，不需要操作队列 */
                    ch = unitInfo[id].channel;
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tOpenMic,ch);
                    break;
                }

                /* 查找等待队列是否存在对应ID话筒 */
                if(waitNum > 0) {
                    /* 取消该话筒等待 */
                    index = DataQueue.search(waitQueue,&id);
                    if(index != 0) {
                        DataQueue.deleted(waitQueue,index);
                        Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tDisWait,null);
                        break;
                    }
                }

                /* 主席+代表开话筒数量未超过允许开话筒数 */
                if(chmMicNum + rpsMicNum < allowOpen) {
                    ch = Conference_GetAudioChannel(type);
                    DataQueue.enter(rpsMicQueue,&id);
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tOpenMic,ch);
                }
                /* 主席+代表开话筒数量等于允许开话筒数 */
                else {
                    /* 等待数量未满 */
                    if(waitNum < allowWait) {
                        /* 进入等待队列 */
                        DataQueue.enter(waitQueue,&id);
                        Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tWaiting,null);
                    } else {
                        /* 话筒已满 */
//                        EXE_MIC_FULL(id,kType_NotiSrc_PC | kType_NotiSrc_WiredUnit);
                        Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tMicFull,null);
                    }
                }
            }
            break;
            /*** 申请模式 ***/
            case kMode_Mic_Apply: {
                /* 申请队列不区分有线、WIFI单元，因此入列ID需要进行真实ID转换 */
                applyId = (type == kType_Unit_Wired ? id : WIFI_ID(id));

                /* 查找话筒队列是否已经存在对应ID的话筒 */
                if(DataQueue.search(rpsMicQueue,&id)) {
                    /* 重新发送打开话筒指令，不需要操作队列 */
                    ch = unitInfo[id].channel;
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tOpenMic,ch);
                    break;
                }

                /* 查找等待队列是否存在对应ID话筒 */
                if(waitNum > 0 ) {
                    index = DataQueue.search(waitQueue,&id);
                    if(index != 0) {
                        Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tDisWait,null);
                        DataQueue.deleted(waitQueue,index);

                        /* 删除申请队列对应ID */
                        DataQueue.deleted(ApplyQueue,DataQueue.search(ApplyQueue,&applyId));
                        if(DataQueue.getSize(ApplyQueue) == 0) {
                            Conference_MicCtrlInstruction(null,(UnitType_EN)null, (UnitAttr_EN)null, tRevokeApply, null);
                        } else {
                            /* 队列头的ID申请开话筒 */
                            DataQueue.front(ApplyQueue,&applyId);
                            if(applyId > WIFI_UNIT_START_ID && applyId <= (WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM)))
                                Conference_MicCtrlInstruction(applyId - WIFI_UNIT_START_ID,kType_Unit_Wifi,(UnitAttr_EN)null,tMicApply,null);
                            else if(applyId >= 1 && applyId <= WIRED_UNIT_MAX_ONLINE_NUM)
                                Conference_MicCtrlInstruction(applyId,kType_Unit_Wired,(UnitAttr_EN)null,tMicApply,null);
                        }
                        break;
                    }
                }

                /* 主席+代表开话筒数量未超过允许开话筒数 */
                if(chmMicNum + rpsMicNum < allowOpen && waitNum < allowWait) {
                    /* 进入等待队列 */
                    DataQueue.enter(waitQueue,&id);
                    Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tWaiting,null);

                    /* 进入申请队列 */
                    DataQueue.enter(ApplyQueue,&applyId);

                    /* 队列头的ID申请开话筒 */
                    DataQueue.front(ApplyQueue,&applyId);
                    if(applyId > WIFI_UNIT_START_ID && applyId <= (WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM)))
                        Conference_MicCtrlInstruction(applyId - WIFI_UNIT_START_ID,kType_Unit_Wifi,(UnitAttr_EN)null,tMicApply,null);
                    else if(applyId >= 1 && applyId <= WIRED_UNIT_MAX_ONLINE_NUM)
                        Conference_MicCtrlInstruction(applyId,kType_Unit_Wired,(UnitAttr_EN)null,tMicApply,null);

                }
                /* 主席+代表开话筒数量等于允许开话筒数 */
                else {
                    /* 话筒已满 */
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

    /* 关话筒 */
    case WIRED_CMD(CHM_CLOSE_MIC,0,0):
    case WIRED_CMD(RPS_CLOSE_MIC,0,0):
    case WIFI_CMD(UnitCloseMic_UtoM_D,0,0): {
        switch(SysInfo.config->micMode) {
        /*** 先进先出模式 ***/
        case kMode_Mic_Fifo: {
            /* 主席单元 */
            if(unitInfo[id].attr == aChairman) {
                /* 查找话筒队列是否已经存在对应ID的话筒 */
                index = DataQueue.search(chmMicQueue,&id);
                if(index) {
                    ch = unitInfo[id].channel;
                    Conference_GiveAudioChannel(type,ch);
                    DataQueue.deleted(chmMicQueue,index);
                    Conference_MicCtrlInstruction(id,type,aChairman,tCloseMic,ch);
                }
            }

            /* 代表单元 */
            else if(unitInfo[id].attr == aRepresentative) {
                /* 查找话筒队列是否已经存在对应ID的话筒 */
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

        /*** 正常模式 ***/
        case kMode_Mic_Normal:
        /*** 声控模式 ***/
        case kMode_Mic_VoiceCtrl: {
            /* 主席单元 */
            if(unitInfo[id].attr == aChairman) {
                /* 查找话筒队列是否已经存在对应ID的话筒 */
                index = DataQueue.search(chmMicQueue,&id);
                if(index) {
                    ch = unitInfo[id].channel;
                    Conference_GiveAudioChannel(type,ch);
                    DataQueue.deleted(chmMicQueue,index);
                    Conference_MicCtrlInstruction(id,type,aChairman,tCloseMic,ch);
                } else
                    break;

                if(waitNum > 0) {
                    /* 等待队列取出单元并打开 */
                    uint16_t waitId;
                    DataQueue.exit(waitQueue,&waitId);
                    Conference_MicControl(waitId,type,WIRED_CMD(RPS_OPEN_MIC,0,0));
                }
            }

            /* 代表单元 */
            else if(unitInfo[id].attr == aRepresentative) {
                /* 查找话筒队列是否已经存在对应ID的话筒 */
                index = DataQueue.search(rpsMicQueue,&id);
                if(index) {
                    ch = unitInfo[id].channel;
                    Conference_GiveAudioChannel(type,ch);
                    DataQueue.deleted(rpsMicQueue,index);
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tCloseMic,ch);
                }

                if(waitNum > 0) {
                    /* 等待队列取出单元并打开 */
                    uint16_t waitId;
                    DataQueue.exit(waitQueue,&waitId);
                    Conference_MicControl(waitId,type,WIRED_CMD(RPS_OPEN_MIC,0,0));
                }
            }
        }
        break;

        /*** 申请模式 ***/
        case kMode_Mic_Apply: {
            /* 主席单元 */
            if(unitInfo[id].attr == aChairman) {
                /* 查找话筒队列是否已经存在对应ID的话筒 */
                index = DataQueue.search(chmMicQueue,&id);
                if(index) {
                    ch = unitInfo[id].channel;
                    Conference_GiveAudioChannel(type,ch);
                    DataQueue.deleted(chmMicQueue,index);
                    Conference_MicCtrlInstruction(id,type,aChairman,tCloseMic,ch);
                }
            }

            /* 代表单元 */
            else if(unitInfo[id].attr == aRepresentative) {
                /* 查找话筒队列是否已经存在对应ID的话筒 */
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

    /* 主席同意开话筒 */
    case WIRED_CMD(AGREE_OPEN_MIC,0,0):
    case WIFI_CMD(ChairAgreeOpenOrNot_UtoM_D,0,0): {

        /* @brief ：主席机同意/拒绝开话筒不会下发同意的ID，只会下发一个同意/拒绝指令，
                    主机需要在队列里面查找，按照队列顺序同意/拒绝话筒打开，且第三代主机
                    需要把有线和WIFI两部分系统逻辑合并，因此做了申请队列——ApplyQueue。
                    整合两个系统ID在队列中 */

        /* 由外部控制同意，有ID */
        if(id != 0) {
            /* 申请队列不区分有线、WIFI单元，因此入列ID需要进行真实ID转换 */
            applyId = (type == kType_Unit_Wired ? id : WIFI_ID(id));

            /* 打开话筒，并删除等待队列中对应ID的话筒 */
            index = DataQueue.search(waitQueue,&id);
            if(index != 0) {
                DataQueue.deleted(waitQueue,index);

                /* 删除申请队列对应ID */
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
                /* 先查找有线单元等待队列 */
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
                /* 再查找WIFI单元等待队列 */
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
            /* 队列头的ID申请开话筒 */
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

    /* 主席不同意开话筒 */
    case WIRED_CMD(DISAGREE_OPEN_MIC,0,0):
    case WIFI_CMD(ChairAgreeOpenOrNot_UtoM_D,1,0): {
        /* 由外部控制拒绝，有ID */
        if(id != 0) {
            /* 申请队列不区分有线、WIFI单元，因此入列ID需要进行真实ID转换 */
            applyId = (type == kType_Unit_Wired ? id : WIFI_ID(id));

            /* 打开话筒，并删除等待队列中对应ID的话筒 */
            index = DataQueue.search(waitQueue,&id);
            if(index != 0) {
                DataQueue.deleted(waitQueue,index);

                /* 删除申请队列对应ID */
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
            /* 队列头的ID申请开话筒 */
            DataQueue.front(ApplyQueue,&applyId);
            if(applyId > WIFI_UNIT_START_ID && applyId <= (WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM)))
                Conference_MicCtrlInstruction(applyId - WIFI_UNIT_START_ID,kType_Unit_Wifi,(UnitAttr_EN)null,tMicApply,null);
            else if(applyId >= 1 && applyId <= WIRED_UNIT_MAX_ONLINE_NUM)
                Conference_MicCtrlInstruction(applyId,kType_Unit_Wired,(UnitAttr_EN)null,tMicApply,null);
        }
    }
    break;

    /* 单元取消等待 */
    case WIRED_CMD(MIC_DISWAIT,0,0):
    case WIFI_CMD(UnitGetoutWaitQuenen_UtoM_D,0,0): {
        switch(SysInfo.config->micMode) {
        /*** 正常模式 ***/
        case kMode_Mic_Normal:
        /*** 声控模式 ***/
        case kMode_Mic_VoiceCtrl: {
            if(waitNum > 0 && unitInfo[id].attr == aRepresentative) {
                /* 查找等待队列是否已经存在对应ID的话筒 */
                index = DataQueue.search(waitQueue,&id);
                if(index) {
                    DataQueue.deleted(waitQueue,index);
                    Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tDisWait,null);
                }
            }
        }
        break;
        /*** 申请模式 ***/
        case kMode_Mic_Apply: {
            /* 申请队列不区分有线、WIFI单元，因此入列ID需要进行真实ID转换 */
            applyId = (type == kType_Unit_Wired ? id : WIFI_ID(id));

            if(waitNum > 0 && unitInfo[id].attr == aRepresentative) {
                /* 查找等待队列是否已经存在对应ID的话筒 */
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
                /* 队列头的ID申请开话筒 */
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

    /* 清有线单元 */
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

    /* 清无线单元 */
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

    /* 重置音频通道队列 */
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

    /* 检查当前模式和目标模式，如果当前是“会议模式”下，目标模式可随便切换，
    	如果当前是“非会议模式”下，则除了会议模式别的模式都不能切换
    (换句话说在非会议模式[EditID,Sign,Vote]下，必须先结束该模式，切换到
    会议模式[conference]，才可以再切换成别的非会议模式) */
    ERR_CHECK(!(SysInfo.state.sysMode != kMode_Conference && mode != kMode_Conference), return);

    switch(mode) {
    case kMode_Conference: {

        /* 判断当前模式并结束 */
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
            /* 向有线单元发送结束编ID */
            WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,EDIT_ID_MODE,END_EDIT_ID_MODE,null,null));
        }
        break;
        default:
            break;
        }

        /* 向WIFI系统及全数字系统广播进入会议模式 */
        Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,CONFERENCE_MODE,START_CONFERENCE_MODE,null,null);
        Protocol.wifiUnit(&wifiProt,0,EnterMeetingMode_MtoU_G,SysInfo.config->micMode,SysInfo.config->wifiAllowOpen);

        ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
        WiredUnit.transmit(&confProt);
        WifiUnit.transmit(kMode_Wifi_Multicast,&wifiProt);

        /* WIFI系统很奇葩，要在切换完会议模式后给它发一个开机完成 */
        if(SysInfo.state.sysMode == kMode_EditID) {
            WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,MasterStarUp_MtoU_G,0,0));
        }

    }
    break;

    case kMode_EditID: {
        /* 主动离线所有单元 */
        Conference_OfflineAllUnit();

        /* 向WIFI系统及全数字系统广播进入编ID模式 */
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

        /* 清除所有签到标志 */
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

        /* 清除所有投票标志 */
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

    /* 关闭所有代表单元 */
    Conference_CloseAllMic(kType_Unit_Wired,aRepresentative);
    Conference_CloseAllMic(kType_Unit_Wifi,aRepresentative);

    /* 取消所有话筒申请 */
    Conference_ClearApplyMic();

    /* 打开主席单元 */
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
* @Description	对单元处理任务下发单元信息结构指针
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
* @Description	对单元处理任务下发单元信息结构指针
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
                    /* 判断文件名是否为特定的本地录音文件，并记录文件名索引
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

    /* 判断当前是否为播放模式，并根据索引显示播放文件名 */
    if(UsbAudio.playIndex != info->playIndex) {
        UsbAudio.playIndex = info->playIndex;
        if(UsbAudio.audSta == kStatus_Aud_Playing) {
            name = UsbAudio.musicFile[UsbAudio.playIndex - 1]->fname;
            /* 先清空显示 */
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


    /* 获取当前页面 */
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
            /* 先清空显示 */
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

