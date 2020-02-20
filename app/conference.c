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
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* 任务堆栈大小及优先级 */
#define CONFERENCE_TASK_STACK_SIZE						(2048)
#define CONFERENCE_TASK_PRIORITY						(configMAX_PRIORITIES - 1)

#define NOTICE_QUEUE_LENGTH								(128)
#define NOTICE_QUEUE_SIZE								(sizeof(void *))

/* WIFI单元ID转换   ，WIFI单元ID从0x3000开始 */
#define WIFI_ID(_id)									(_id + 0x3000)

/* 单元指令(type 或 cmd)与参数(ph,pl)合并拓展到32位，并加上标志位，方便在同时处理两个类型的单元的指令时区分或避免数字重复
	第一个字节为设备类型表示 第二个字节为指令(type或cmd) 第三、四个字节是参数(ph,pl)*/
#define WIRED_CMD(_type,_ph,_pl)						( 0x80000000 | (_type << 16) | (_ph << 8) | _pl)
#define WIFI_CMD(_cmd,_ph,_pl)							( 0x40000000 | (_cmd << 16) | (_ph << 8) | _pl)

/* 单元在线数量 */
#define WIRED_ONLINE_NUM								(OnlineNum.wiredChm + OnlineNum.wiredRps)
#define WIFI_ONLINE_NUM									(OnlineNum.wifiChm + OnlineNum.wifiRps)
#define UNIT_ONLINE_NUM 								(WIRED_ONLINE_NUM + WIFI_ONLINE_NUM)
								

#if 0
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

#endif


/* 音乐列表最大长度 */
#define MUSIC_FILE_LIST_MAX_LEN						(64)

/* 定时任务计时器运行间隔(ms) */
#define TIMING_TASK_INTERVAL						(1000)



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
static ConfSysInfo_S *Conference_GetConfSysInfo(void);


/* 内部调用 */
static void Conference_NoticeProcessTask(void *pvParameters);
static void Conference_TimingTask(TimerHandle_t xTimer);
static void Conference_MessageProcess(Notify_S *notify);
static void Conference_WifiUnitMessageProcess(Notify_S *notify);

static void Conference_ScreenPageSwitch(uint8_t page);
static void Conference_ScreenUpdataUnitNum(void);
static void Conference_ScreenMessageProcess(Notify_S *notify);

static void Conference_ResetAudioChannel(UnitType_EN type);
static uint8_t Conference_GetAudioChannel(UnitType_EN type);
static void Conference_GiveAudioChannel(UnitType_EN type,uint8_t ch);

static void Conference_SignInstruction(uint16_t id,UnitType_EN type,uint32_t cmd,uint16_t totalNum,uint16_t curNum);
static void Conference_MicCtrlInstruction(uint16_t id, UnitType_EN type,UnitAttr_EN attr, MicControlType_EN ctrlType,uint8_t channel);
static void Conference_MicControl(uint16_t id, UnitType_EN type, uint32_t cmd);
static void Conference_DspUnitCtrl(uint16_t id, UnitType_EN type, MicControlType_EN ctrlType,uint8_t channel);

static void Conference_CloseAllMic(UnitType_EN type,UnitAttr_EN attr);
static void Conference_OfflineAllUnit(void);
static void Conference_ClearApplyMic(UnitType_EN type);
static void Conference_ChangeSysMode(SysMode_EN mode);
static void Conference_ChairmanPriority(UnitType_EN type,uint16_t id);

static void Conference_UsbStateListener(status_t sta);
static void Conference_AudioStateListener(AudState_S *sta);


static void Conference_OpenMicListDebug(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* 会议通知队列 */
static QueueHandle_t noticeQueue;

///* 会议系统通讯协议 */
//static ConfProtocol_S confProt;
//
///* WIFI单元通讯协议 */
//static WifiUnitProtocol_S wifiProt;

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

/* AFC控制GPIO   (D14 GPIO2_29) */
static HAL_GpioIndex AfcCtrl;

/* LineOut声音输出控制GPIO (H13 GPIO1_24) */
static HAL_GpioIndex LineOutCtrl;

/* 分区通道 Out1 ~ Out16 声音输出控制GPIO (L10 GPIO1_15) */
static HAL_GpioIndex PartOutCtrl;

/* 设备运行信号 (L12 GPIO1_20) */
static HAL_GpioIndex RunSignal;

/* 运行定时任务定时器 */
static TimerHandle_t TimingTask;


/* 投票模式码（全数字协议） */
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

/* 记录当前显示页面 */
static uint8_t ScreenCurrentPage = SP_WELCOME;


/* USB音频录放控制句柄 */
static struct {
    /* MCU连接上U盘 */
    bool mcuConnectUsb;
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
    /* 音乐文件列表 */
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
    uint16_t id;
    UnitCfg_S *unitCfg;

    noticeQueue = xQueueCreate(NOTICE_QUEUE_LENGTH,NOTICE_QUEUE_SIZE);

    memset(&UnitInfo,0,sizeof(UnitInfo));
    memset(&UsbAudio,0,sizeof(UsbAudio));

    /* GPIO初始化 */
    AfcCtrl = HAL_GpioInit(GPIO2, 29, kGPIO_DigitalOutput, null, (gpio_interrupt_mode_t)null);
    LineOutCtrl = HAL_GpioInit(GPIO1, 24, kGPIO_DigitalOutput, null, (gpio_interrupt_mode_t)null);
    PartOutCtrl = HAL_GpioInit(GPIO1, 15, kGPIO_DigitalOutput, null, (gpio_interrupt_mode_t)null);
    RunSignal = HAL_GpioInit(GPIO1, 20, kGPIO_DigitalOutput, null, (gpio_interrupt_mode_t)null);


    /* 初始化话筒队列 */
    WiredChmMicQueue = DataQueue.creat(WIRED_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WiredRpsMicQueue = DataQueue.creat(WIRED_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WiredWaitQueue = DataQueue.creat(WIRED_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WiredChannelQueue = DataQueue.creat(WIRED_UNIT_MAX_ALLWO_OPEN,sizeof(uint8_t));

    WifiChmMicQueue = DataQueue.creat(WIFI_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WifiRpsMicQueue = DataQueue.creat(WIFI_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WifiWaitQueue = DataQueue.creat(WIFI_UNIT_MAX_ALLWO_OPEN,sizeof(uint16_t));
    WifiChannelQueue = DataQueue.creat(WIFI_UNIT_MAX_ALLWO_OPEN,sizeof(uint8_t));

    /* 复位单元音频通道队列 */
    Conference_ResetAudioChannel(tWired);
    Conference_ResetAudioChannel(tWifi);

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

	/* 检查RTC电池 */
	if(!Time.getRst())
		Conference_ScreenPageSwitch(SP_BATTERY_ERR);
	else
		Conference_ScreenPageSwitch(SP_WELCOME);
	

    /* 初始化定时任务计时器 */
    TimingTask = xTimerCreate("TimingTask",TIMING_TASK_INTERVAL,pdTRUE,null,Conference_TimingTask);


	/* 启动会议主机主处理任务 */
    if (xTaskCreate(Conference_NoticeProcessTask, "NoticeProcessTask", CONFERENCE_TASK_STACK_SIZE, null, CONFERENCE_TASK_PRIORITY, null) != pdPASS) {
        Log.e("create host task error\r\n");
    }

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

    Log.d("Conference notice process task launch!\r\n");

    HAL_SetGpioLevel(PartOutCtrl, 0);
    HAL_SetGpioLevel(LineOutCtrl, 1);
    HAL_SetGpioLevel(AfcCtrl, 0);


    xTimerStart(TimingTask, 0);

    while(1) {
        xQueueReceive(noticeQueue, &notify, MAX_NUM);

        /* 判断通知来源 */
        if(notify->nSrc & (EX_CTRL_DEST | kType_NotiSrc_WiredUnit)) {
            /* 通知来源为外部控制及有线单元 */
            Conference_MessageProcess(notify);
        }

        else if(notify->nSrc & kType_NotiSrc_WifiUnit) {
            /* 通知来源为WIFI单元 */
            Conference_WifiUnitMessageProcess(notify);
        }

        else if(notify->nSrc & kType_NotiSrc_ScreenCtrl) {
            /* 通知来源为屏幕控制 */
            Conference_ScreenMessageProcess(notify);
        }
		else if(notify->nSrc & kType_NotiSrc_SlaveMcu){
			/* 通知来源为从单片机 */
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

    /* 运行灯 */
    runSignal = !runSignal;
    HAL_SetGpioLevel(RunSignal, runSignal);

    /* 检测外部控制连接状态,并将屏幕切换到相应状态 */
    if(ExternalCtrl.connectSta(kType_NotiSrc_PC) || ExternalCtrl.connectSta(kType_NotiSrc_Web)) {
        if(ScreenCurrentPage != SP_EX_CTRL_CONNECT)
            Conference_ScreenPageSwitch(SP_EX_CTRL_CONNECT);
    } else {
        if(ScreenCurrentPage == SP_EX_CTRL_CONNECT)
            Conference_ScreenPageSwitch(SP_MAIN_MENU);
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


}



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
        sprintf(srcStr,"%s",notify->nSrc == kType_NotiSrc_WiredUnit ? "Wired Unit" :  \
                notify->nSrc == kType_NotiSrc_PC ? "PC" : \
                notify->nSrc == kType_NotiSrc_Web ? "WEB" : \
                notify->nSrc == kType_NotiSrc_UartCtrl ? "UART" : "unknow");
        Log.d("%s Msg: id = 0x%X, type = 0x%X,para{ %X , %X , %X , %X , %X }",srcStr,id,type,para[0],para[1],para[2],para[3],para[4]);
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

    switch(type) {
    /* 基本消息 0x80 */
    case BASIC_MSG: {
        uint8_t mode = para[0], cmd = para[1];

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
                if((id > WIFI_UNIT_START_ID && id <= (WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM))) && (notify->nSrc & EX_CTRL_DEST))
                    Conference_MicControl(id - WIFI_UNIT_START_ID, tWifi, WIRED_CMD(cmd,0,0));
                else if(id >= 1 && id <= WIRED_UNIT_MAX_ONLINE_NUM)
                    Conference_MicControl(id, tWired, WIRED_CMD(cmd,0,0));
            }
            break;

            /* 同意开话筒 */
            case AGREE_OPEN_MIC:
            /* 不同意开话筒 */
            case DISAGREE_OPEN_MIC: {
                if(notify->nSrc & EX_CTRL_DEST && id > 0x3000 )
                    Conference_MicControl(id - WIFI_UNIT_START_ID, tWifi, WIRED_CMD(cmd,0,0));
                else if(notify->nSrc & EX_CTRL_DEST && id > 0 && id < 4096 )
                    Conference_MicControl(id, tWired, WIRED_CMD(cmd,0,0));
                else if(notify->nSrc == kType_NotiSrc_WifiUnit || notify->nSrc == kType_NotiSrc_WiredUnit)
                    Conference_MicControl(null, (UnitType_EN)null, WIRED_CMD(cmd,0,0));
            }
            break;

            /* 主席优先权 */
            case CHM_PRIORITY: {
                Conference_ChairmanPriority(tWired,id);
            }
            break;

            /* 话筒上线 */
            case UNIT_ACCESS_SYS: {
                if(UnitInfo.wired[id].attr == aChairman)
                    OnlineNum.wiredChm++;
                else if(UnitInfo.wired[id].attr == aRepresentative)
                    OnlineNum.wiredRps++;
#ifdef	CONFIRM_UNIT_NUM
                /* 打开定时器，3S后数一下话筒列表数量对不对，防止话筒重复上线 */
                xTimerStart(ConfirmUnitNumTimer,0);
#endif
                /* 通知外部控制话筒上线 */
                ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id,PC_MSG,UNIT_LOGIN,0x01,null,null));

                /* 更新屏幕显示数量 */
                Conference_ScreenUpdataUnitNum();

                Log.d("WiredUnit(%s) id = %d online ( Chm num = %d , Rps num = %d )\r\n",UnitInfo.wired[id].attr == aChairman ? "CHM":"RPS",id,OnlineNum.wiredChm,OnlineNum.wiredRps);
            }
            break;

            /*话筒掉线*/
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
                /* 通知外部控制话筒掉线 */
                ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id,PC_MSG,UNIT_LOGOUT,0x01,null,null));

                /* 更新屏幕显示数量 */
                Conference_ScreenUpdataUnitNum();

                Log.d("WiredUnit(%s) id = %d offline ( Chm num = %d , Rps num = %d )\r\n",UnitInfo.wired[id].attr == aChairman ? "CHM":"RPS",id,OnlineNum.wiredChm,OnlineNum.wiredRps);
            }
            break;

			/* ID重复 */
            case ID_DUPICATE: {
                uint16_t repeatId = para[2];

                if(SysInfo.state.sysMode != kMode_EditID) {
                    Log.d("Wired unit id repead !! ID = %d\r\n",repeatId);

                    /* 广播通知全数字会议系统 */
                    WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID, BASIC_MSG, CONFERENCE_MODE, ID_DUPICATE,null,repeatId));
                    /* 广播通知全WIFI会议系统 */
                    WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0, IDRepeatingMtoU_G, repeatId >> 8, repeatId & 0xFF));

                    Conference_ScreenPageSwitch(SP_ID_PEPEAT);
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
			case SERVICE_RESERVED_2:{
				ExternalCtrl.transmit(id,Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,cmd,null,null));
			}break;
            }
        }
        break;
        /* 签到模式 0x01 */
        case SIGN_MODE: {
            switch(cmd) {
            /* 进入签到模式(控制端发起) */
            case START_SIGN_MODE: {
                SysInfo.state.totalSignNum = (uint16_t)((para[3] << 8) | para[4]);
                SysInfo.state.currentSignNum = 0;

                Conference_ChangeSysMode(kMode_Sign);
            }
            break;
			/* 进入签到模式(主席单元发起) */
			case DEV_START_SIGN_MODE:{
				SysInfo.state.totalSignNum = (uint16_t)(UNIT_ONLINE_NUM);
                SysInfo.state.currentSignNum = 0;

				Conference_ChangeSysMode(kMode_Sign);
			}
			break;
			/* 结束签到(控制端发起) */
            case END_SIGN_MODE: 
			/* 结束签到(主席单元发起) */
			case DEV_END_SIGN_MODE:
			{
                WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,SIGN_MODE,END_SIGN_MODE,null,null));
                WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,EnterSignMode_MtoU_G,1,null));
                Conference_ChangeSysMode(kMode_Conference);
            }
            break;
            /* 单元签到 */
            case UNIT_SIGN_IN: {
                /* 单元签到 */
                if((notify->nSrc & kType_NotiSrc_WiredUnit) && !UnitInfo.wired[id].sign) {
                    UnitInfo.wired[id].sign = true;
                    SysInfo.state.currentSignNum++;
                    Conference_SignInstruction(id,tWired,WIRED_CMD(UNIT_SIGN_IN,0,0),null,null);
                    Conference_SignInstruction(null,(UnitType_EN)null,WIRED_CMD(START_SIGN_MODE,0,0),SysInfo.state.totalSignNum,SysInfo.state.currentSignNum);
                }
                /* 如果单元签到来自外部控制，则为控制签到 */
                else if(notify->nSrc & (kType_NotiSrc_PC | kType_NotiSrc_Web | kType_NotiSrc_UartCtrl)) {
                    /* 控制有线单元签到 */
                    if(id >= 1 && id <= WIRED_UNIT_MAX_ONLINE_NUM && !UnitInfo.wired[id].sign) {
                        Conference_SignInstruction(id,tWired,WIRED_CMD(CONTROL_UNIT_SIGN,0,0),null,null);
                    }
                    /* 控制WIFI单元签到 */
                    else if(id > WIFI_UNIT_START_ID && id <= WIFI_UNIT_START_ID + WIFI_UNIT_MAX_ONLINE_NUM  \
                            && !UnitInfo.wifi[id - WIFI_UNIT_START_ID].sign) {
                        Conference_SignInstruction(id - WIFI_UNIT_START_ID,tWifi,WIFI_CMD(ControlSign_MtoU_D,0,0),null,null);
                    }
                }
            }
            break;
			/* 允许签到 */
			case SIGN_ENABLE:{
				if(id == 0xFFF1){
					SysInfo.config->signEn = ENABLE;
				}
				else{
					/* id范围判断设备类型 */
	                if((id > WIFI_UNIT_START_ID && id <= (WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM))))
						WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id - WIFI_UNIT_START_ID,EnableSignOrNot_MtoU_D,0,null));
	                else if(id >= 1 && id <= WIRED_UNIT_MAX_ONLINE_NUM)
						WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,SIGN_MODE,SIGN_ENABLE,null,null));
	                    
				}
			}
			break;
			/* 禁止签到 */
			case SIGN_DISABLE:{
				if(id == 0xFFF1){
					SysInfo.config->signEn = DISABLE;
				}else{
					/* id范围判断设备类型 */
	                if((id > WIFI_UNIT_START_ID && id <= (WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM))))
						WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id - WIFI_UNIT_START_ID,EnableSignOrNot_MtoU_D,1,null));
	                else if(id >= 1 && id <= WIRED_UNIT_MAX_ONLINE_NUM)
						WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,SIGN_MODE,SIGN_DISABLE,null,null));
				}
			}
			break;
            /* 补充签到 */
            case SUPPLEMENT_SIGN: {
                /* 有线单元补充签到 */
                if(id >= 1 && id <= WIRED_UNIT_MAX_ONLINE_NUM && !UnitInfo.wired[id].sign) {
                    Conference_SignInstruction(id,tWired,WIRED_CMD(SUPPLEMENT_SIGN,0,0),null,null);
                }
                /* WIFI单元补充签到*/
                else if(id > WIFI_UNIT_START_ID && id <= WIFI_UNIT_START_ID + WIFI_UNIT_MAX_ONLINE_NUM  \
                        && !UnitInfo.wifi[id - WIFI_UNIT_START_ID].sign) {
                    Conference_SignInstruction(id - WIFI_UNIT_START_ID,tWifi,WIFI_CMD(SupplementSign_MtoU_D,0,0),null,null);
                }
            }
            break;
            /* 单元应答补充签到 */
            case UNIT_SUPPLEMENT_SIGN: {
                if((notify->nSrc & kType_NotiSrc_WiredUnit) && !UnitInfo.wired[id].sign) {
                    UnitInfo.wired[id].sign = true;
                    SysInfo.state.currentSignNum++;
                    Conference_SignInstruction(id,tWired,WIRED_CMD(UNIT_SUPPLEMENT_SIGN,0,0),null,null);
                }
            }
            break;
            }
        }
        break;
        /* 投票模式 0x02 */
        case VOTE_MODE: {
            uint8_t voteResult;

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
            case VOTE_LAUNCH_BY_CHAIRMAN: {
                if(notify->nSrc & kType_NotiSrc_WiredUnit || notify->nSrc & kType_NotiSrc_WifiUnit) {

                }
            }
            break;

            /* 结束表决 */
            case FINISH_VOTE: {
                if(SysInfo.state.sysMode == kMode_Vote) {
                    Conference_ChangeSysMode(kMode_Conference);
                }
            }
            break;

            /* 单元投票、表决、选举结果 */
            case 0x25:
                voteResult = 1;//赞成
                goto unitVoted;
            case 0x26:
                voteResult = 2;//弃权
                goto unitVoted;
            case 0x27:
                voteResult = 3;//反对
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
                voteResult = 1;//很不满意(五键)/不满意(三键)
                goto unitVoted;
            case 0x4B:
                voteResult = 2;//不满意(五键)/一般(三键)
                goto unitVoted;
            case 0x4C:
                voteResult = 3;//一般(五键)/满意(三键)
                goto unitVoted;
            case 0x4D:
                voteResult = 4;//满意(五键)
                goto unitVoted;
            case 0x4E:
                voteResult = 5;//很满意(五键)
                goto unitVoted;
            case 0x65:
                voteResult = 1;//自定义表决选项1
                goto unitVoted;
            case 0x66:
                voteResult = 2;//自定义表决选项2
                goto unitVoted;
            case 0x67:
                voteResult = 3;//自定义表决选项3
                goto unitVoted;
            case 0x68:
                voteResult = 4;//自定义表决选项4
                goto unitVoted;
            case 0x69:
                voteResult = 5;//自定义表决选项5
                {
unitVoted:
                    if(notify->nSrc & kType_NotiSrc_WiredUnit && SysInfo.state.sysMode == kMode_Vote) {
                        /* 如果投票类型为需要先签到，而单元未进行签到 */
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
            switch(cmd) {
            /* 声控灵敏度&声控时间 */
            case 0x53:
            case 0x54: {
                uint8_t wifiCmd = (cmd == 53 ? SetVoiceSensitivity_MtoU_D : SetVoiceCloseTime_MtoU_D);

                if(id == 0xFFF0) {
                    /* 广播通知全数字会议系统 */
                    WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID, BASIC_MSG, UNIT_CTRL, cmd,null,para[4]));
                    /* 广播通知全WIFI会议系统 */
                    WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0, wifiCmd, para[4], null));
                } else {
                    if(id > WIFI_UNIT_START_ID && id <= (WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM)))
                        WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id - WIFI_UNIT_START_ID, wifiCmd, para[4], null));
                    else if(id >= 1 && id <= WIRED_UNIT_MAX_ONLINE_NUM)
                        WiredUnit.transmit(Protocol.conference(&confProt,id, BASIC_MSG, UNIT_CTRL, cmd,null,para[4]));
                }
            }
            /* 灵敏度 */
            case 0x43: {
                if(id > WIFI_UNIT_START_ID && id <= (WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM))) {
                    UnitInfo.wifi[id - WIFI_UNIT_START_ID].config->sensitivity = para[4];
                    WifiUnit.transmit(kMode_Wifi_Unitcast,  \
                                      Protocol.wifiUnit(&wifiProt,id - WIFI_UNIT_START_ID, SetUnitMICSensitivity_MtoU_D, para[4], null));
                    Database.saveSpecify(kType_Database_WifiCfg,null);
                } else if(id >= 1 && id <= WIRED_UNIT_MAX_ONLINE_NUM) {
                    UnitInfo.wired[id].config->sensitivity = para[4];
                    WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID, BASIC_MSG, UNIT_CTRL, cmd,null,para[4]));
                    Database.saveSpecify(kType_Database_WiredCfg,id);
                }
            }
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
            uint8_t mode = para[0], num = para[1];

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
            Conference_CloseAllMic(tWired,aChairman);
            Conference_CloseAllMic(tWired,aRepresentative);
            Conference_CloseAllMic(tWifi,aChairman);
            Conference_CloseAllMic(tWifi,aRepresentative);

            /* 清除申请队列 */
            Conference_ClearApplyMic(tWired);
            Conference_ClearApplyMic(tWifi);


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

            time->year = para[0];
            time->month = para[1];
            time->day = para[2];
            time->hour = para[3];
            time->min = para[4];
            time->sec = (&notify->exDataHead)[0];

            Time.setNow(time);

            FREE(time);
        }
        break;

        }
    }
    break;
    /* 通讯消息 0x86*/
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
        case CFG_IP: {
            SysInfo.config->ip[0] = para[2];
            SysInfo.config->ip[1] = para[3];
            SysInfo.config->ip[2] = para[4];
            SysInfo.config->ip[3] = (&notify->exDataHead)[0];

            Database.saveSpecify(kType_Database_SysCfg,null);
        }
        break;
        /* 0x08 */
        case CFG_MASK: {
            SysInfo.config->mask[0] = para[2];
            SysInfo.config->mask[1] = para[3];
            SysInfo.config->mask[2] = para[4];
            SysInfo.config->mask[3] = (&notify->exDataHead)[0];

            Database.saveSpecify(kType_Database_SysCfg,null);
        }
        break;
        /* 0x09 */
        case CFG_GW: {
            SysInfo.config->gateWay[0] = para[2];
            SysInfo.config->gateWay[1] = para[3];
            SysInfo.config->gateWay[2] = para[4];
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

        }
        break;

        /* 摄像头预置位配置 */
        case CAMERA_POSITION: {
            if(id == 0) {
                SysInfo.config->panorPos = para[2];
                SysInfo.config->panorCh = para[3];
                Database.saveSpecify(kType_Database_SysCfg,null);
            } else if(id >= 1 && id <= WIRED_UNIT_MAX_ONLINE_NUM) {
                UnitInfo.wired[id].config->camPos = para[2];
                UnitInfo.wired[id].config->camCh = para[3];
                Database.saveSpecify(kType_Database_WiredCfg,id);
            } else if(id > WIFI_UNIT_START_ID && id <= (WIFI_UNIT_START_ID + WIFI_UNIT_MAX_ONLINE_NUM)) {
                uint16_t wifiId = id - WIFI_UNIT_START_ID;

                UnitInfo.wifi[wifiId].config->camPos = para[2];
                UnitInfo.wifi[wifiId].config->camCh = para[3];
                Database.saveSpecify(kType_Database_WifiCfg,null);
            }


        }
        break;

        /* 摄像头控制 0x14 */
        case CAMERA_CONTROL: {
            uint8_t *data,len = para[1];

            data = MALLOC(len);
            memcpy(&data[0],&para[2],3);
            memcpy(&data[3],&notify->exDataHead,len - 3);

            Camera.cmdSend(data,len);

            FREE(data);
        }
        break;

        /* 下发自定义表决项 0x24 */
        case CUSTOM_VOTE_ITEM: {
            uint8_t item = para[1];

            if(item >= 1 && item <= 5) {
                uint8_t *customItem;

                customItem = MALLOC(3 + notify->exLen);
                memcpy(&customItem[0],&para[2],3);
                if(notify->exLen > 0)
                    memcpy(&customItem[3],&notify->exDataHead,notify->exLen);

                /* 目前只有WIFI单元有自定义表决 */
                WifiUnit.transWithExData(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,PCUpdateCustomTerm_MtoU_G,item,0),3 + notify->exLen,customItem);
                FREE(customItem);
            }
        }
        break;

        /* 发起编ID   	0x2F*/
        case PC_EDIT_ID: {
            if(para[1] == PC_START_EDIT_ID) {
                /* 开始编ID */
                uint8_t startID = (notify->prot.conference.sec == 0 ? 1 : notify->prot.conference.sec);

                SysInfo.state.wiredCurEditID = startID;
                SysInfo.state.wifiCurEditID = WIFI_ID(startID);
                Conference_ChangeSysMode(kMode_EditID);
            }

            else if(para[1] == PC_STOP_EDIT_ID) {
                Conference_ChangeSysMode(kMode_Conference);
                WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,MasterStarUp_MtoU_G,0,0));
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
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,SCAN_ONLINE_UNIT,SCAN_UNIT_END,tWired, \
                                  (OnlineNum.wiredChm + OnlineNum.wiredRps)));

            /* 上传在线WIFI单元 */
            for(id = 1; id <= WIFI_UNIT_MAX_ONLINE_NUM; id++) {
                if(UnitInfo.wifi[id].online) {
                    ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WIFI_ID(id),PC_MSG,ONLINE_UNIT,UnitInfo.wifi[id].attr,null,null));
                }
            }
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,SCAN_ONLINE_UNIT,SCAN_UNIT_END,tWifi, \
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

        case QUERY_PRIOR_SIGN:
        case QUERY_PRIOR_VOTE:
        case QUERY_PRIOR_SCAN: {
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,PC_MSG,cmd,0x01,SysInfo.state.sysMode,null));
        }
        break;
        }
    }
    break;
    /* 音频矩阵 0xB0*/
    case AUDIO_MATRIX: {
        uint8_t ph = para[0],pl = para[1], *exdata, i;

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
            Conference_CloseAllMic(tWired,aChairman);
            Conference_CloseAllMic(tWired,aRepresentative);
            Conference_CloseAllMic(tWifi,aChairman);
            Conference_CloseAllMic(tWifi,aRepresentative);

            /* 清除申请队列 */
            Conference_ClearApplyMic(tWired);
            Conference_ClearApplyMic(tWifi);

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
            UnitType_EN unitType;

            if(id >= 1 && id <= WIRED_UNIT_MAX_ONLINE_NUM) {
                unitInfo = &UnitInfo.wired[id];
                saveType = kType_Database_WiredCfg;
                unitType = tWired;
            } else if(id > WIFI_UNIT_START_ID && id <= WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM)) {
                id -= WIFI_UNIT_START_ID;
                unitInfo = &UnitInfo.wifi[id];
                saveType = kType_Database_WifiCfg;
                unitType = tWifi;
            } else
                break;

            switch(pl) {
            /* 分区通道音量 */
            case 0x01: {
                memcpy(&unitInfo->config->chVol[0],&para[2],3);
                memcpy(&unitInfo->config->chVol[3],&notify->exDataHead,13);
                if(unitInfo->micSta == kStatus_UnitMic_Open) {
                    Conference_DspUnitCtrl(id,unitType,tOpenMic,unitInfo->channel);
                }
            }
            break;
            /* 单元EQ */
            case 0x02: {
                uint8_t i;

                exdata = &notify->exDataHead;

                memcpy(&unitInfo->config->eqFreq[0],&para[2],3);
                memcpy(&unitInfo->config->eqFreq[3],&exdata[0],2);
                memcpy(&unitInfo->config->eqVol[0],&exdata[2],5);

                for(i = 0; i < 5; i++) {
                    uint8_t para;

                    if(!(unitInfo->config->eqFreq[i] >= 0 && unitInfo->config->eqFreq[i] <= 3))
                        unitInfo->config->eqFreq[i] = 0;
                    if((!(unitInfo->config->eqVol[i] >= 0 && unitInfo->config->eqVol[i] <= 24)))
                        unitInfo->config->eqVol[i] = 24;

                    /* 适配原全数字会议协议下发配置EQ */
                    para = ((3 - unitInfo->config->eqFreq[i]) << 5) | (24 - unitInfo->config->eqVol[i]);

                    WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,UNIT_CTRL,0x40,0x00,i << 12 | para));
                }
            }
            break;

            case 0x03: {
                if(!(para[2] >= 0 && para[2] <= 8))
                    break;

                unitInfo->config->sensitivity = para[2];

                if(unitType == tWired) {
                    WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,UNIT_CTRL,0x43,0x00,para[2]));
                } else if(unitType == tWifi) {
                    WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,SetUnitMICSensitivity_MtoU_D,para[2],0));
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

            if(!(pl >= 0x01 && pl <= 0x06 && id == 0xFFF0))
                break;

            /* 转换为DSP输出通道枚举  	     */
            dspOutput = (DspOutput_E)(pl + DSP_OUTPUT_CH16);

            switch(para[2]) {
            /* 配置音量 */
            case 0x01: {
                /* 判断并保存普通输出通道音量值 */
                SysInfo.config->dsp[dspOutput].vol = para[3] > 31 ? 31 : para[3];

                /* 写入DSP */
                Dsp.norOutput(dspOutput,DSP_OUTPUT_VOLUME,(DspEqPart_E)null,(uint8_t)(31 - SysInfo.config->dsp[dspOutput].vol));
            }
            break;
            /* 配置10段EQ音量 */
            case 0x02: {
                exdata = &notify->exDataHead;
                memcpy(&SysInfo.config->dsp[dspOutput].eqVol[0],&para[3],2);
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
                memcpy(&SysInfo.config->dsp[dspOutput].inputVol[0],&para[3],2);
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
				if(dspOutput == DSP_OUTPUT_DOWN_WIFI){
                	SysInfo.config->dsp[DSP_OUTPUT_DOWN_WIFI].downTrans = para[3];
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

            if(!(pl >= 0x00 && pl <= 0x0F && id == 0xFFF0))
                break;

            switch(para[2]) {
            case 0x01: {
                /* 判断并保存分区输出通道音量值 */
                SysInfo.config->dsp[pl].vol = para[3] > 31 ? 31 : para[3];
                /* 写入DSP */
                Dsp.chOutput((DspOutput_E)pl,DSP_OUTPUT_VOLUME,(DspEqPart_E)null,(uint8_t)(31 - SysInfo.config->dsp[pl].vol));
            }
            break;

            case 0x02: {
                exdata = &notify->exDataHead;
                memcpy(&SysInfo.config->dsp[pl].eqVol[0],&para[3],2);
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
                memcpy(&SysInfo.config->dsp[pl].inputVol[0],&para[3],2);
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
                SysInfo.config->dsp[pl].dly = para[3] > 100 ? 100 : para[3];

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
        uint8_t ph = para[0];

        switch(ph) {
        /* 外部控制主机查询DSP模式 */
        case DSP_MODE: {
            ExternalCtrl.transmit(notify->nSrc,Protocol.conference(&confProt,WHOLE_BROADCAST_ID,AUDIO_MATRIX,DSP_MODE,SysInfo.config->dspMode,null,null));
        }
        break;

        /* 外部控制主机查询单元分区音量设置 */
        case DSP_UNIT_OUT_CFG: {
            UnitCfg_S *uCfg;

            if(id >= 1 && id <= WIRED_UNIT_MAX_ONLINE_NUM) {
                uCfg = UnitInfo.wired[id].config;
            } else if(id > WIFI_UNIT_START_ID && id <= WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM)) {
                uCfg = UnitInfo.wifi[id - WIFI_UNIT_START_ID].config;
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

            if(id != 0xFFF0)
                break;

            switch(para[2]) {
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
                                      DSP_OUTPUT_DOWN_WIFI - DSP_OUTPUT_CH16, 0x04, SysInfo.config->dsp[DSP_OUTPUT_DOWN_WIFI].downTrans << 8 | 0x00));
            }
            break;
            }

        }
        break;

        /* 外部控制主机查询分区通道输出口配置 */
        case DSP_CHANNEL_OUT_CFG: {
            uint8_t i;

            if(id != 0xFFF0)
                break;

            switch(para[2]) {
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

    case RepPollUnitl_UtoM_D: {
        switch(SysInfo.state.sysMode) {
        case kMode_Sign: {
            if((ph & 0x01) && !UnitInfo.wifi[id].sign) {
                UnitInfo.wifi[id].sign = true;
                SysInfo.state.currentSignNum++;
                Conference_SignInstruction(id,tWifi,WIFI_CMD(RepPollUnitl_UtoM_D,kMode_Sign,0),null,null);
                Conference_SignInstruction(null,(UnitType_EN)null,WIFI_CMD(EnterSignMode_MtoU_G,0,0),SysInfo.state.totalSignNum,SysInfo.state.currentSignNum);
            }
        }
        break;

        case kMode_Vote: {
            uint8_t voteResult = pl;

            /* 如果投票类型为需要先签到，而单元未进行签到 */
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

    /* 单元上线 */
    case RepApplyEnterSystm_MtoU_D: {
        if(UnitInfo.wifi[id].attr == aChairman)
            OnlineNum.wifiChm++;
        else if(UnitInfo.wifi[id].attr == aRepresentative)
            OnlineNum.wifiRps++;
#ifdef	CONFIRM_UNIT_NUM
        /* 打开定时器，3S后数一下话筒列表数量对不对，防止话筒重复上线 */
        xTimerStart(ConfirmUnitNumTimer,0);
#endif
        /* 通知外部控制话筒上线 */
        ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id,PC_MSG,UNIT_LOGIN,0x01,null,null));

        /* 更新屏幕显示数量 */
        Conference_ScreenUpdataUnitNum();

        Log.d("WifiUnit(%s) id = %d online ( Chm num = %d , Rps num = %d )\r\n",UnitInfo.wired[id].attr == aChairman ? "CHM":"RPS",id,OnlineNum.wiredChm,OnlineNum.wiredRps);
    }
    break;

    /* 单元掉线 */
    case MasterAskforReenterSysMtoU_D: {
        if(UnitInfo.wifi[id].attr == aChairman) {
            Conference_MicControl(id, tWifi, WIFI_CMD(UnitCloseMic_UtoM_D,0,0));
            OnlineNum.wifiChm--;
        } else if(UnitInfo.wifi[id].attr == aRepresentative) {
            Conference_MicControl(id, tWifi, WIFI_CMD(UnitCloseMic_UtoM_D,0,0));
            OnlineNum.wifiRps--;
        }
#ifdef	CONFIRM_UNIT_NUM
        /* 打开定时器，3S后数一下话筒列表数量对不对，防止话筒重复上线 */
        xTimerStart(ConfirmUnitNumTimer,0);
#endif
        /* 通知外部控制话筒掉线 */
        ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id,PC_MSG,UNIT_LOGOUT,0x01,null,null));

        /* 更新屏幕显示数量 */
        Conference_ScreenUpdataUnitNum();

        Log.d("WifiUnit(%s) id = %d offline ( Chm num = %d , Rps num = %d )\r\n",UnitInfo.wired[id].attr == aChairman ? "CHM":"RPS",id,OnlineNum.wiredChm,OnlineNum.wiredRps);
    }
    break;

    /*单元机申请开话筒*/
    case ApplyOpenMic_UtoM_D:
        Conference_MicControl(id, tWifi, WIFI_CMD(cmd,0,0));
        break;

    /*单元机关开话筒*/
    case UnitCloseMic_UtoM_D:
        Conference_MicControl(id, tWifi, WIFI_CMD(cmd,0,0));
        break;

    /*单元机退出等待队列*/
    case UnitGetoutWaitQuenen_UtoM_D:
        Conference_MicControl(id, tWifi, WIFI_CMD(cmd,0,0));
        break;

    /*主席机执行优先权*/
    case ChairExecutePriority_UtoM_D:
        Conference_ChairmanPriority(tWifi,id);
        break;

    /*主席机（不）同意代表机开话筒*/
    case ChairAgreeOpenOrNot_UtoM_D:
        Conference_MicControl(0, tWifi, WIFI_CMD(ChairAgreeOpenOrNot_UtoM_D,ph,0));
        break;

    /*单元机茶水申请    */
    case UnitApplyWater_UtoM_D:
        break;


    /*回复主机帮单元机补签到   */
    case RepSupplementSign_UtoM_D:
        Conference_SignInstruction(id,tWifi,WIFI_CMD(RepSupplementSign_UtoM_D,0,0), null, null);
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

    case UnitCapacityStrChangeUtoM_D:
        break;

    case RepReadUnitMICSensitivity_UtoM_D:
        break;

    case IDRepeatingMtoU_G: {
        uint16_t repeatId = (ph << 8) | pl ;

        Log.d("Wifi unit id repead !! ID = %d\r\n",repeatId);

        /* 广播通知全数字会议系统 */
        WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID, BASIC_MSG, CONFERENCE_MODE, ID_DUPICATE,null,null));
        /* 广播通知全WIFI会议系统 */
        WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0, IDRepeatingMtoU_G, ph, pl));

        Conference_ScreenPageSwitch(SP_ID_PEPEAT);
    }
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
static void Conference_ScreenPageSwitch(uint8_t page)
{
    ScreenProtocol_S screenProt;
    ScreenCurrentPage = page;
    Screen.transmit(Protocol.screen(&screenProt,tType_Screen_Page,ScreenCurrentPage));
}


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
    ScreenProtocol_S screenProt;
    uint16_t num;
    uint8_t *exData;

    exData = MALLOC(25);

    /* 更新有线单元在线总数 */
    num = OnlineNum.wiredChm + OnlineNum.wiredRps;
    exData[0]= (uint8_t)(num >> 8);
    exData[1]= (uint8_t)(num & 0xFF);
    Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_WIRED_OL_TOTAL_NUM),2,exData);

    /* 更新有线主席机在线数量 */
    exData[0]= (uint8_t)(OnlineNum.wiredChm >> 8);
    exData[1]= (uint8_t)(OnlineNum.wiredChm & 0xFF);
    Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_WIRED_OL_CHM_NUM),2,exData);

    /* 更新有线代表机在线数量  */
    exData[0]= (uint8_t)(OnlineNum.wiredRps >> 8);
    exData[1]= (uint8_t)(OnlineNum.wiredRps & 0xFF);
    Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_WIRED_OL_RPS_NUM),2,exData);


    /* 更新无线单元在线总数 */
    num = OnlineNum.wifiChm + OnlineNum.wifiRps;
    exData[0]= (uint8_t)(num >> 8);
    exData[1]= (uint8_t)(num & 0xFF);
    Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_WIFI_OL_TOTAL_NUM),2,exData);

    /* 更新无线主席机在线数量 */
    exData[0]= (uint8_t)(OnlineNum.wifiChm >> 8);
    exData[1]= (uint8_t)(OnlineNum.wifiChm & 0xFF);
    Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_WIFI_OL_CHM_NUM),2,exData);

    /* 更新无线代表机在线数量 */
    exData[0]= (uint8_t)(OnlineNum.wifiRps >> 8);
    exData[1]= (uint8_t)(OnlineNum.wifiRps & 0xFF);
    Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_WIFI_OL_RPS_NUM),2,exData);


    /* 更新译员机在线数量 */
    exData[0]= (uint8_t)(OnlineNum.interpreter >> 8);
    exData[1]= (uint8_t)(OnlineNum.interpreter & 0xFF);
    Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_INTERP_OL_NUM),2,exData);

    FREE(exData);

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
                Conference_ScreenPageSwitch(SP_UNIT_TYPE);
            }
            break;

            /* 录音节界面 */
            case 0x02: {
                if(UsbAudio.mcuConnectUsb || UsbAudio.audSta == kStatus_Aud_Playing ||   \
                   UsbAudio.audSta == kStatus_Aud_Pause || UsbAudio.audSta == kStatus_Aud_Recording) {
                    switch(UsbAudio.audSta) {
                    /* 切换暂停页面 */
                    case kStatus_Aud_Idle:
                    case kStatus_Aud_Pause: {
                        Conference_ScreenPageSwitch(SP_AUD_PLAYER_PAUSE);
                    }
                    break;
                    /* 切换播放页面 */
                    case kStatus_Aud_Playing: {
                        char *name = UsbAudio.musicFile[UsbAudio.playIndex - 1]->fname;
                        Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_AUD_PLAY_NAME),strlen(name)+1,(uint8_t *)name);
                        Conference_ScreenPageSwitch(SP_AUD_PLAYER_PLAY);
                    }
                    break;
                    /* 切换录音页面 */
                    case kStatus_Aud_Recording: {
                        Conference_ScreenPageSwitch(SP_AUD_RECORD);
                    }
                    break;

                    default:
                        break;
                    }
                } else {
                    Conference_ScreenPageSwitch(SP_USB_INIT);
                }

            }
            break;

            /* 系统状态 */
            case 0x03: {
                uint16_t year,mon,day;

                /* 切换页面 */
                Conference_ScreenPageSwitch(SP_SYS_STATE);

                /* 更新屏幕显示单元数量 */
                Conference_ScreenUpdataUnitNum();

                APP_GetBuildDate(&year, &mon, &day);

                /* 更新显示当前系统版本号 */
                sprintf((char *)&exData[0],"M_%s_%d%d%d",APP_VERSION,year,mon,day);
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_SOFTWARE_VERSION_M),16,exData);

                /* 更新显示代码编译日期 */
                sprintf((char *)&exData[0],"S_%s_%d%d%d",APP_VERSION,year,mon,day);
				Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_SOFTWARE_VERSION_S),16,exData);
            }
            break;

            /* 编ID界面 */
            case 0x04: {
				if(SysInfo.state.startID < 1 || SysInfo.state.startID > 4096)
					SysInfo.state.startID = 1;
                /* 切换页面 */
                Conference_ScreenPageSwitch(SP_SET_ID);
                sprintf((char *)&exData[0],"%04d  ",SysInfo.state.startID);
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_START_ID),6,exData);
            }
            break;

            /* 中繁英版本 */
            case 0x05: {
                /* 切换页面 */
                Conference_ScreenPageSwitch(SP_SYS_SETTING);
            }
            break;

            /*译员机*/
            case 0x06: {
//                /* 切换页面 */
//                Conference_ScreenPageSwitch(StartTranSetID_Page);
            }
            break;
            /* ID 模式 */
            case 0x07: {
                /* 切换页面 */
                Conference_ScreenPageSwitch(SP_SYS_SETTING);
            }
            break;
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
                if(cfgWitchTypeUnit == tWired && cmd >= 0 && cmd <= 3) {
                    static const uint8_t Num[4] = {1,2,4,8};

                    SysInfo.config->wiredAllowOpen = Num[cmd];
                    SysInfo.config->wiredAllowWait = Num[cmd];
                }

                /* 设置WIFI单元 */
                else if(cfgWitchTypeUnit == tWifi && cmd >= 0 && cmd <= 3) {
                    static const uint8_t Num[4] = {1,2,4,6};

                    SysInfo.config->wifiAllowOpen = Num[cmd];
                    SysInfo.config->wifiAllowWait = Num[cmd];
                }
            }

            /* 关闭所有单元 */
            Conference_CloseAllMic(tWired,aChairman);
            Conference_CloseAllMic(tWired,aRepresentative);
            Conference_CloseAllMic(tWifi,aChairman);
            Conference_CloseAllMic(tWifi,aRepresentative);

            /* 清除申请队列 */
            Conference_ClearApplyMic(tWired);
            Conference_ClearApplyMic(tWifi);


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
        case 0x22:{
			if(para[2] == 0x02){
				char strId[5] = {0};
				
				strId[0] = para[3];
				strId[1] = para[4];
				strId[2] = (&notify->exDataHead)[0];
				strId[3] = (&notify->exDataHead)[1];
				
				sscanf(strId,"%d",&SysInfo.state.startID);
			}
		}break;
        case 0x24:  //译员机模式配置
            break;
        case 0x26:  //译员机编ID
            break;
        /* 发起或者结束编ID */
        case 0x2A: {
            if(cmd == 0x02) {
                /* 切换页面 */
                Conference_ScreenPageSwitch(SP_SET_ID_END);
                SysInfo.state.wiredCurEditID = SysInfo.state.startID;
                SysInfo.state.wifiCurEditID = WIFI_ID(SysInfo.state.startID > WIFI_UNIT_MAX_ONLINE_NUM ? WIFI_UNIT_MAX_ONLINE_NUM : SysInfo.state.startID);

                /* 切换会议模式 */
                Conference_ChangeSysMode(kMode_EditID);
            } else if(cmd == 0x03) {
                /* 切换页面 */
                Conference_ScreenPageSwitch(SP_MAIN_MENU);

                /* 向有线单元发送结束编ID */
                WiredUnit.transmit(Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,EDIT_ID_MODE,END_EDIT_ID_MODE,null,null));

                /* 切换会议模式 */
                Conference_ChangeSysMode(kMode_Conference);
                WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,MasterStarUp_MtoU_G,0,0));
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
                Conference_ScreenPageSwitch(SP_SET_LANGUAGE);
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
                Conference_ScreenPageSwitch(SP_LOCAL_IP);

                sprintf((char *)&exData[0],"%d.%d.%d.%d",ip[0],ip[1],ip[2],ip[3]);
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_IP_ADDR),15,exData);

                sprintf((char *)&exData[0],"%d.%d.%d.%d",mask[0],mask[1],mask[2],mask[3]);
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_GATEWAY_ADDR),15,exData);

                sprintf((char *)&exData[0],"%d.%d.%d.%d",gw[0],gw[1],gw[2],gw[3]);
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_NETMASK_ADDR),15,exData);

                sprintf((char *)&exData[0],"%d",SysInfo.config->port);
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_PORT),6,exData);

            }
            break;

            /* 显示设置-亮度调节 */
            case 0x04: {
                /* 切换页面 */
                Conference_ScreenPageSwitch(SP_SET_DISPLAY);

				exData[0] = 0;
                exData[1] = SysInfo.config->brightness;
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_BRIGHTNESS),2,exData);
            }
            break;

            /* 音量调节 */
            case 0x06: {
                /* 切换页面 */
                Conference_ScreenPageSwitch(SP_SET_VOLUME);

				exData[0] = 0;
                exData[1] = SysInfo.config->dsp[DSP_OUTPUT_LINE_OUT1].vol;
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_LINEOUT1_VOL),2,exData);

				exData[0] = 0;
                exData[1] = SysInfo.config->dsp[DSP_OUTPUT_LINE_OUT2].vol;
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_LINEOUT2_VOL),2,exData);
            }
            break;

            /* 下传功能 */
            case 0x07: {
                /* 切换页面 */
                Conference_ScreenPageSwitch(SP_SET_DOWN_TRANS);
            }
            break;

            default:
                break;
            }
        }
        break;

        /* 屏发来录音处理按键 */
        case 0x3B:{
            switch(cmd) {
            case 0x00://上一曲
                Audio.previous();
                break;

            case 0x01://播放
                Audio.playPause();
                break;

            case 0x02://下一曲
                Audio.next();
                break;

            case 0x03://开始/停止录音
                if(UsbAudio.audSta == kStatus_Aud_Recording) {
                    Audio.stopRec();
                    /* 切换到音频录播主页 */
//					Conference_ScreenPageSwitch(SP_AUD_PLAYER_PAUSE);
                    Conference_ScreenPageSwitch(SP_AUD_RECORD_WAIT);
                } else if(UsbAudio.audSta == kStatus_Aud_Idle || UsbAudio.audSta == kStatus_Aud_Pause) {
                    TimePara_S *time;
                    char *name;

                    time = MALLOC(sizeof(TimePara_S));
                    name = MALLOC(72);

                    Time.getNow(time);

                    Conference_ScreenPageSwitch(SP_AUD_RECORD_WAIT);
                    sprintf(UsbAudio.recFile,"%s%02d%02d","RE",time->hour,time->min);
                    Audio.record(UsbAudio.recFile);

                    strcpy(name,UsbAudio.recFile);
                    Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_AUD_PLAY_NAME),72,(uint8_t *)name);
                    FREE(name);
                    FREE(time);
                }
                break;

            case 0x04://返回
                /* 切换页面 */
                Conference_ScreenPageSwitch(SP_MAIN_MENU);
                break;

            case 0x05://暂停
                Audio.playPause();
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
                Conference_ScreenPageSwitch(SP_WELCOME);
            }
            break;

            case 0x02: {
                WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0, MasterStarUp_MtoU_G, 1, 0));
                Conference_ScreenPageSwitch(SP_WELCOME);
            }
            break;

            default:
                break;
            }
            break;

        case 0x40: //屏发过来语言选择
            break;
		/* 屏发来的下传功能设置 */
        case 0x41:{
			
        }break;
		/* 屏发过来请求更新屏幕当前亮度值的 */
        case 0x44:{
			if(cmd >= 0 && cmd <= 64){
				SysInfo.config->brightness = cmd;
        		Screen.backlight(cmd);
				Database.saveSpecify(kType_Database_SysCfg,null);
			}
        }break;
        case 0x45://屏发过来请求更新低音值的
            break;
        case 0x50: //屏发来的ID模式设置
            break;

        /* 重新编ID */
        case 0x51: {
            /* 切换页面 */
            Conference_ScreenPageSwitch(SP_RESET_ID);
            SysInfo.state.wiredCurEditID = 1;
            SysInfo.state.wifiCurEditID = WIFI_ID(1);
            Conference_ChangeSysMode(kMode_EditID);
        }
        break;
        /* 结束重新编ID */
        case 0x52: {
            /* 切换页面 */
            Conference_ScreenPageSwitch(SP_MAIN_MENU);
            Conference_ChangeSysMode(kMode_Conference);
            WifiUnit.transmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&wifiProt,0,MasterStarUp_MtoU_G,0,0));
        }
        break;
		/* 屏发来会议主机类型选择返回 */
        case 0x54:
            switch(cmd) {
            /* 无线 */
            case 0x00: {
                static const uint8_t Reg[7] = {null,0,1,null,2,null,3};

                cfgWitchTypeUnit = tWifi;

                exData[0] = 0;
                exData[1] = 1;
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_CONF_MODE),2,exData);

                /* 切换页面 */
                Conference_ScreenPageSwitch(SP_CONF_MODE);

                exData[1] = SysInfo.config->micMode - 1;
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_SET_MIC_MODE),2,exData);

                exData[1] = Reg[SysInfo.config->wifiAllowOpen];
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_SET_MIC_NUM),2,exData);
            }
            break;

            /* 有线 */
            case 0x01: {
                static const uint8_t Reg[9] = {null,0,1,null,2,null,null,null,3};

                cfgWitchTypeUnit = tWired;

                exData[0] = 0;
                exData[1] = 0;
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_CONF_MODE),2,exData);

                /* 切换页面 */
                Conference_ScreenPageSwitch(SP_CONF_MODE);

                exData[1] = SysInfo.config->micMode - 1;
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_SET_MIC_MODE),2,exData);

                exData[1] = Reg[SysInfo.config->wiredAllowOpen];
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_SET_MIC_NUM),2,exData);
            }
            break;
            }
            break;
        case 0x55://屏发来升降话筒控制 20190819
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
		case 0x94:{
			DspOutput_E lineout = (reg == 0x92 ? DSP_OUTPUT_LINE_OUT1 : DSP_OUTPUT_LINE_OUT2);
		
			SysInfo.config->dsp[lineout].vol = cmd;
			Dsp.norOutput(lineout,DSP_OUTPUT_VOLUME,(DspEqPart_E)null,cmd);
			Database.saveSpecify(kType_Database_SysCfg,null);
		}break;
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
    /* 开始签到以及下发签到总数和签到当前数量 */
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

    /* 单元签到 */
    case WIRED_CMD(UNIT_SIGN_IN,0,0):
    case WIFI_CMD(RepPollUnitl_UtoM_D,kMode_Sign,0): {
        if(id != null) {
            id = type == tWired ? id : WIFI_ID(id);
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,id,BASIC_MSG,SIGN_MODE,UNIT_SIGN_IN,null,null));
        }
    }
    break;

    /* 控制单元签到 */
    case WIRED_CMD(CONTROL_UNIT_SIGN,0,0):
    case WIFI_CMD(ControlSign_MtoU_D,0,0): {
        if(type == tWired)
            WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,SIGN_MODE,CONTROL_UNIT_SIGN,null,null));
        else if(type == tWifi)
            WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,ControlSign_MtoU_D,null,null));
    }
    break;

    /* 补充签到 */
    case WIRED_CMD(SUPPLEMENT_SIGN,0,0):
    case WIFI_CMD(SupplementSign_MtoU_D,0,0): {
        if(type == tWired)
            WiredUnit.transmit(Protocol.conference(&confProt,id,BASIC_MSG,SIGN_MODE,SUPPLEMENT_SIGN,null,null));
        else if(type == tWifi)
            WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,SupplementSign_MtoU_D,null,null));
    }
    break;

    /* 回复补充签到 */
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

    dspUnitSrc = (DspUintSrc_E)((type == tWired) ? (channel - 0x81) : (channel - 0x81 + 8));
    unitInfo = (type == tWired) ? UnitInfo.wired : UnitInfo.wifi;
//	id = (type == tWired) ? id : id - WIFI_UNIT_START_ID;

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
        if(type == tWired) {
            Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,arg,ch,null);
            WiredUnit.transmit(&confProt);
            ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
            UnitInfo.wired[id].channel = ch;
            UnitInfo.wired[id].micSta = kStatus_UnitMic_Open;

            /* 配置DSP */
            Conference_DspUnitCtrl(id,type,tOpenMic,ch);
            /* 配置摄像跟踪预制位 */
            Camera.call(id,type);
        } else if(type == tWifi) {
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
        if(type == tWired) {
            Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,arg,null,null);
            WiredUnit.transmit(&confProt);
            ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
            UnitInfo.wired[id].channel = null;
            UnitInfo.wired[id].micSta = kStatus_UnitMic_Close;

            /* 配置DSP */
            Conference_DspUnitCtrl(id,type,tCloseMic,ch);
            /* 释放预置位 */
            Camera.release(id,type);
        } else if(type == tWifi) {
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
        if(type == tWired) {
            Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,MIC_WAIT,null,null);
            WiredUnit.transmit(&confProt);
            ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
            UnitInfo.wired[id].micSta = kStatus_UnitMic_Wait;
        } else if(type == tWifi) {
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),BASIC_MSG,CONFERENCE_MODE,MIC_WAIT,null,null));
            WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,RepApplyOpenMic_MtoU_D,3,null));
            UnitInfo.wifi[id].micSta = kStatus_UnitMic_Wait;
        }
    }
    break;

    /* 话筒取消等待 */
    case tDisWait: {
        if(type == tWired) {
            Protocol.conference(&confProt,id,BASIC_MSG,CONFERENCE_MODE,MIC_DISWAIT,null,null);
            ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
            WiredUnit.transmit(&confProt);
            UnitInfo.wired[id].micSta = kStatus_UnitMic_Close;
        } else if(type == tWifi) {
            ExternalCtrl.transmit(EX_CTRL_DEST,Protocol.conference(&confProt,WIFI_ID(id),BASIC_MSG,CONFERENCE_MODE,MIC_DISWAIT,null,null));
            WifiUnit.transmit(kMode_Wifi_Unitcast,Protocol.wifiUnit(&wifiProt,id,MasterOpenOrCloseMic_MtoU_D,1,null));
            UnitInfo.wifi[id].micSta = kStatus_UnitMic_Close;
        }
    }
    break;

    /* 话筒已满 */
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

    /* 话筒申请 */
    case tMicApply: {
        uint16_t applyId = (type == tWifi) ? WIFI_ID(id) : id;
        UnitInfo_S *unitInfo = (type == tWifi) ? UnitInfo.wifi : UnitInfo.wired;

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
        uint16_t applyId = (type == tWifi) ? WIFI_ID(id) : id;
        UnitInfo_S *unitInfo = (type == tWifi) ? UnitInfo.wifi : UnitInfo.wired;

        data[0] = (uint8_t)(applyId >> 8);
        data[1] = (uint8_t)(applyId& 0xFF);
        len = 2;
        Protocol.conference(&confProt,REVOKE_APPLY_ID,APPLY_MSG,data[0],data[1],null,null);
        Protocol.wifiUnit(&wifiProt,0,RepresentApplyOpenMic_MtoU_G,1,null);

        ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
        WiredUnit.transmit(&confProt);
        WifiUnit.transWithExData(kMode_Wifi_Multicast,&wifiProt,len,data);

        unitInfo[id].micSta = kStatus_UnitMic_Close;
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
        allowOpen = SysInfo.config->wiredAllowOpen;
        allowWait = SysInfo.config->wiredAllowWait;
    }

    else if(type == tWifi) {
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
                    }
                }
            }
            break;

            default:
                break;
            }
        }

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
                if(waitNum > 0 ) {
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
//                    unitInfo[id].channel = ch;
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
                /* 查找话筒队列是否已经存在对应ID的话筒 */
                if(DataQueue.search(rpsMicQueue,&id)) {
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tMicApply,ch);
                    break;
                }

                /* 查找等待队列是否存在对应ID话筒 */
                if(waitNum > 0 ) {
                    index = DataQueue.search(waitQueue,&id);
                    if(index != 0) {
                        Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tDisWait,null);
                        Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tRevokeApply,null);
                        DataQueue.deleted(waitQueue,index);
                        break;
                    }
                }

                /* 主席+代表开话筒数量未超过允许开话筒数 */
                if(chmMicNum + rpsMicNum < allowOpen && waitNum < allowWait) {
                    /* 进入等待队列 */
                    DataQueue.enter(waitQueue,&id);
                    Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tWaiting,null);
                    Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tMicApply,null);
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
        uint8_t waitId;


        /* @brief ：主席机同意/拒绝开话筒不会下发同意的ID，只会下发一个同意/拒绝指令，
                    主机需要在队列里面查找，按照队列顺序同意/拒绝话筒打开，且第三代主机
                    需要把有线和WIFI两部分系统逻辑合并，因此在同意/拒绝开话筒这里需要
                    查找两方面的队列   */

        /* 由外部控制同意，有ID */
        if(id != 0) {
            /* 打开话筒，并删除等待队列中对应ID的话筒 */
            index = DataQueue.search(waitQueue,&id);
            if(index != 0) {
                DataQueue.deleted(waitQueue,index);

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

            if(DataQueue.getSize(WiredWaitQueue) > 0) {
                /* 先查找有线单元等待队列 */
                if(DataQueue.exit(WiredWaitQueue,&waitId) && \
                   (DataQueue.getSize(WiredRpsMicQueue) + DataQueue.getSize(WiredChmMicQueue) < SysInfo.config->wiredAllowOpen)) {
                    DataQueue.enter(WiredRpsMicQueue,&waitId);
                    ch = Conference_GetAudioChannel(tWired);
                    Conference_MicCtrlInstruction(waitId,tWired,aRepresentative,tOpenMic,ch);
                } else {
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tDisWait,null);
                }


            }

            else if(DataQueue.getSize(WifiWaitQueue) > 0 ) {
                /* 再查找WIFI单元等待队列 */
                if(DataQueue.exit(WifiWaitQueue,&waitId) && \
                   (DataQueue.getSize(WifiRpsMicQueue) + DataQueue.getSize(WifiChmMicQueue) < SysInfo.config->wifiAllowOpen)) {
                    DataQueue.enter(WifiRpsMicQueue,&waitId);
                    ch = Conference_GetAudioChannel(tWifi);
                    Conference_MicCtrlInstruction(waitId,tWifi,aRepresentative,tOpenMic,ch);
                } else {
                    Conference_MicCtrlInstruction(id,type,aRepresentative,tDisWait,null);
                }
            }
        }



        if(DataQueue.getSize(WifiWaitQueue) == 0 && DataQueue.getSize(WiredWaitQueue) == 0) {
            Conference_MicCtrlInstruction(null,(UnitType_EN)null,(UnitAttr_EN)null,tRevokeApply,null);
        }
    }
    break;

    /* 主席不同意开话筒 */
    case WIRED_CMD(DISAGREE_OPEN_MIC,0,0):
    case WIFI_CMD(ChairAgreeOpenOrNot_UtoM_D,1,0): {
        /* 由外部控制拒绝，有ID */
        if(id != 0) {
            /* 打开话筒，并删除等待队列中对应ID的话筒 */
            index = DataQueue.search(waitQueue,&id);
            if(index != 0) {
                DataQueue.deleted(waitQueue,index);
                Conference_MicCtrlInstruction(id,type,(UnitAttr_EN)null,tDisWait,null);
            }
        }

        /* 主席机拒绝 */
        else {
            uint8_t waitId;

            if(DataQueue.getSize(WiredWaitQueue) > 0) {
                /* 先查找有线单元等待队列 */
                if(DataQueue.exit(WiredWaitQueue,&waitId)) {
                    Conference_MicCtrlInstruction(waitId,tWired,(UnitAttr_EN)null,tDisWait,null);
                }

            }

            else if(DataQueue.getSize(WifiWaitQueue) > 0 ) {
                /* 再查找WIFI单元等待队列 */
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
            if(waitNum > 0 && unitInfo[id].attr == aRepresentative) {
                /* 查找等待队列是否已经存在对应ID的话筒 */
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
    uint8_t ch;

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
static void Conference_ClearApplyMic(UnitType_EN type)
{
    uint8_t waitNum,i;
    uint16_t *idArr;
    DataQueueHandler_S *micQueue;

    if(type == tWired) {
        micQueue = WiredWaitQueue;
    } else if(type == tWifi) {
        micQueue = WifiWaitQueue;
    } else
        return;

    waitNum = DataQueue.getSize(micQueue);

    if(waitNum > 0) {
        idArr = MALLOC(waitNum * sizeof(uint16_t));
        DataQueue.toArray(micQueue,idArr);
        for(i = 0; i<waitNum; i++) {
            Conference_MicCtrlInstruction(idArr[i], type, (UnitAttr_EN)null, tDisWait, null);
            if(SysInfo.config->micMode == kMode_Mic_Apply)
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

    /* 重置音频通道队列 */
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

    /* 检查当前模式和目标模式，如果当前是“会议模式”下，目标模式可随便切换，
    	如果当前是“非会议模式”下，则除了会议模式别的模式都不能切换
    (换句话说在非会议模式[EditID,Sign,Vote]下，必须先结束该模式，切换到
    会议模式[conference]，才可以再切换成别的非会议模式) */
    ERR_CHECK(!(SysInfo.state.sysMode != kMode_Conference && mode != kMode_Conference), return);

    switch(mode) {
    case kMode_Conference: {
        Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,CONFERENCE_MODE,START_CONFERENCE_MODE,null,null);
        Protocol.wifiUnit(&wifiProt,0,EnterMeetingMode_MtoU_G,SysInfo.state.sysMode,SysInfo.config->wifiAllowOpen);

        ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
        WiredUnit.transmit(&confProt);
        WifiUnit.transmit(kMode_Wifi_Multicast,&wifiProt);

        SysInfo.state.sysMode = kMode_Conference;
    }
    break;

    case kMode_EditID: {
        Conference_OfflineAllUnit();

        Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,EDIT_ID_MODE,START_EDIT_ID_MODE,null,null);
        Protocol.wifiUnit(&wifiProt,0,EnterEditingIDMtoU_G,(SysInfo.state.wifiCurEditID >> 8),(SysInfo.state.wifiCurEditID & 0xFF));

        ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
        WiredUnit.transmit(&confProt);
        WifiUnit.transmit(kMode_Wifi_Multicast,&wifiProt);

        SysInfo.state.sysMode = kMode_EditID;

    }
    break;

    case kMode_Sign: {
        uint16_t id;

        for(id = 0; id < WIRED_UNIT_MAX_ONLINE_NUM; id++)
            UnitInfo.wired[id].sign = false;

        for(id = 0; id < WIFI_UNIT_MAX_ONLINE_NUM; id++)
            UnitInfo.wifi[id].sign = false;


        Conference_SignInstruction(null,(UnitType_EN)null,WIRED_CMD(START_SIGN_MODE,0,0),SysInfo.state.totalSignNum,SysInfo.state.currentSignNum);

        SysInfo.state.sysMode = kMode_Sign;
    }
    break;

    case kMode_Vote: {
        uint16_t id;

        for(id = 0; id < WIRED_UNIT_MAX_ONLINE_NUM; id++)
            UnitInfo.wired[id].vote = null;

        for(id = 0; id < WIFI_UNIT_MAX_ONLINE_NUM; id++)
            UnitInfo.wifi[id].vote = null;

        Protocol.conference(&confProt,WHOLE_BROADCAST_ID,BASIC_MSG,VOTE_MODE,VoteModeCode[SysInfo.state.voteMode],null,null);
        Protocol.wifiUnit(&wifiProt,0,EnterVoteMode_MtoU_G,0,SysInfo.state.voteMode);

        ExternalCtrl.transmit(EX_CTRL_DEST,&confProt);
        WiredUnit.transmit(&confProt);
        WifiUnit.transmit(kMode_Wifi_Multicast,&wifiProt);

        SysInfo.state.sysMode = kMode_Vote;
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

    /* 关闭所有代表单元 */
    Conference_CloseAllMic(tWired,aRepresentative);
    Conference_CloseAllMic(tWifi,aRepresentative);

    /* 取消所有话筒申请 */
    Conference_ClearApplyMic(tWired);
    Conference_ClearApplyMic(tWifi);

    /* 打开主席单元 */
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
    FILINFO *filinfo;
    uint8_t fileNum, i;
    char *suffix = null;

    UsbAudio.musicFileNum = 0;

    fileNum = UsbDisk.getFileList(path,&filinfo);


    Log.d("USB music file : \r\n");
    for(i = 0; i < fileNum; i++) {
        suffix = strrchr(filinfo[i].fname, '.');
        if(suffix != null) {
            if (strcmp(suffix, ".mp3") == 0) {
                printf(" %s\r\n",filinfo[i].fname);
                UsbAudio.musicFile[UsbAudio.musicFileNum++] = &filinfo[i];
            } else if (strcmp(suffix, ".wav") == 0) {
                printf(" %s\r\n",filinfo[i].fname);
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
        Log.i("Usb disk is attached!!\r\n");
    }
    break;
    case kStatus_DEV_Detached: {
        UsbAudio.mcuConnectUsb = false;

//		/* 当MCU的USB断开时，判断当前音频录放功能的状态，如果是空闲，而且页面停留在录放功能里面，就切换页面 */
//		if(UsbAudio.audSta == kStatus_Aud_Idle && (ScreenCurrentPage == SP_AUD_PLAYER_PAUSE || ScreenCurrentPage == SP_AUD_PLAYER_PLAY ||   \
//			ScreenCurrentPage == SP_AUD_RECORD || 	ScreenCurrentPage ==SP_AUD_RECORD_WAIT)){
//			Conference_ScreenPageSwitch(SP_USB_INIT);
//		}
        Log.i("Usb disk is detached!!\r\n");
    }
    break;
    case kStatus_FS_Mounted: {
//			uint8_t i;
        UsbAudio.mcuConnectUsb = true;

        Log.i("Usb file system is mounted!!\r\n");
        Log.i("Usb disk free size : %dMB\r\n",(UsbDisk.freeSize()/1024));

        Conference_GetMusicFile(USBDISK_ROOT);

//			Log.d("Music file num = %d \r\n",UsbAudHandler.num);
//			for(i = 0;i < UsbAudHandler.num;i++){
//				printf("%s \r\n",UsbAudHandler.fMusic[i]->fname);
//			}

    }
    break;
    case kStatus_DEV_Idle: {
        Log.i("Usb idle!!\r\n");
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
            Conference_ScreenPageSwitch(SP_AUD_PLAYER_PLAY);
            Log.d("Music playing .. \r\n");
        }

        if(UsbAudio.playIndex != sta->playIndex) {
            UsbAudio.playIndex = sta->playIndex;
            if(UsbAudio.playIndex > 0 && UsbAudio.playIndex <= UsbAudio.musicFileNum) {
                name = MALLOC(72);
                strcpy(name,UsbAudio.musicFile[UsbAudio.playIndex - 1]->fname);
                Log.d("Music name : \'%s\'  ... \r\n",name);
                Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_AUD_PLAY_NAME),72,(uint8_t *)name);
                FREE(name);
            }
        }
    }
    break;
    case kStatus_Aud_Pause: {
        if(UsbAudio.audSta != sta->state) {
            UsbAudio.audSta = sta->state;
            Conference_ScreenPageSwitch(SP_AUD_PLAYER_PAUSE);
            Log.d("Music pause .. \r\n");
        }

    }
    break;
    case kStatus_Aud_Recording: {
        char time[10] = {0};

        if(UsbAudio.audSta != sta->state) {
            UsbAudio.audSta = sta->state;
            Conference_ScreenPageSwitch(SP_AUD_RECORD);
            /* 显示停止录音的按钮 */
            exdata[1] = 0x01;
            Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_AUD_REC_ICO),2,exdata);
        }
//		Log.d("Audio recording sec = %d \r\n",sta->runTime);
        sprintf(time,"%02d:%02d:%02d",sta->runTime / 3600,sta->runTime / 60,sta->runTime % 60);
        Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_AUD_REC_TIME),9,(uint8_t *)time);
    }
    break;

    case kStatus_Aud_RecStoped: {
        /* 隐藏录音的按钮 */
        exdata[1] = 0x00;
        Screen.transWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,SVR_AUD_REC_ICO),2,exdata);
        UsbAudio.audSta = kStatus_Aud_Idle;
        Conference_ScreenPageSwitch(SP_AUD_PLAYER_PAUSE);
    }
    break;
    case kStatus_Aud_UsbBroken:
    case kStatus_Aud_StarPlayErr:
    case kStatus_Aud_StarRecErr:
    case kStatus_Aud_UsbTimeout: {
        Conference_ScreenPageSwitch(SP_USB_INIT);
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

