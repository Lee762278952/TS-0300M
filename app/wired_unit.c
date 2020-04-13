/**
 *
 *	@name				wired_unit.c
 *	@author 			KT
 *	@date 				2019/07/23
 *	@brief
 *  @include			wired_unit.h
 *
 *  @API
 *
 *  @description
 *
 **/
/*******************************************************************************
 * includes
 ******************************************************************************/
/* OS */
#include "FreeRTOS.h"
#include "timers.h"

/* API */
#include "network.h"
#include "ram.h"

/* APP */
#include "wired_unit.h"
#include "conference.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/**** 网络相关参数定义 ****/
#define HOST_LOCAL_IP										{192,168,0,117}
#define HOST_GATEWAY										{192,168,0,1}
#define HOST_NETMASK										{255,255,255,0}
#define HOST_PORT											(1026)

/* 单元指令数据监听多播地址及端口 */
#define UNIT_DATA_MULTICAST_IP								{224,0,2,9}
#define UNIT_DATA_MULTICAST_PORT							(1028)

/* 单元指令数据目的多播地址及端口 */
#define UNIT_DEST_IP										{224,0,2,7}
#define UNIT_DEST_PORT										(1026)

/* 网络接口 */
#define NETWORK_ENET_TYPE									eth1

/* 网络类型 */
#define NETWORK_TYPE										NETWORK_TYPE_ETHERNET

/* 任务堆栈大小及优先级 */
#define WIRED_UNIT_TASK_STACK_SIZE							(1024)
#define WIRED_UNIT_TASK_PRIORITY							(15)

/* 接收单元数据BUF大小 */
#define UNIT_DATA_RECEIVE_BUF_SIZE							(64)

/* 轮询时间 */
#define POLLING_TIME_MS										(1000)

/* 单元发送间隔 */
#define TRANSMIT_TIME_INTERVAL_MS							(1)

/* 轮询无应答判断为离线次数 */
#if 0   
/* 测试用 */
#define POLLING_NO_REPLY_OFFLINE_COUNT						(1200)
#else
#define POLLING_NO_REPLY_OFFLINE_COUNT						(6)
#endif

#define IS_APPLY_ACCESS_SYS(_type,_ph,_pl)					((_pl == RPS_APPLY_ACCESS_SYS || _pl == CHM_APPLY_ACCESS_SYS) && _type == BASIC_MSG && _ph == CONFERENCE_MODE)
#define IS_CONFIRM_ID(_type,_ph,_pl)						(_pl == UNIT_CONFIRM_ID && _type == BASIC_MSG && _ph == EDIT_ID_MODE)
#define IS_UNIT_REPLY_ONLINE(_type,_ph,_pl)					((_pl == RPS_REPLY_ONLINE || _pl == CHM_REPLY_ONLINE) && _type == BASIC_MSG && _ph == CONFERENCE_MODE)
#define IS_UNIT_REPLY_SIGN(_type,_ph,_pl)					(_pl == UNIT_SIGN_IN && _type == BASIC_MSG && _ph == SIGN_MODE)
//#define IS_UNIT_REPLY_VOTE(_type,_ph,_pl)					(_pl == UNIT_SIGN_IN && _type == BASIC_MSG && _ph == VOTE_MODE)


/* 单元主机通讯协议格式 */
typedef struct {
    char header[4];
    ConfProtocol_S prot;
    uint16_t exLen;
    uint8_t exDataHead;
} NetData_S;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* LAUNCHER */
static void WiredUnit_ConfigTask(void *pvParameters);
static void WiredUnit_NetDataProcessTask(void *pvParameters);
static void WiredUnit_NetDataTransmitTask(void *pvParameters);

/* API调用 */
static void WiredUnit_NetDataTransmitWithExData(ConfProtocol_S *prot, uint16_t exLen, uint8_t *exData);
static void WiredUnit_NetDataTransmit(ConfProtocol_S *prot);

/* 内部调用 */
static void WiredUnit_EthStaListener(bool sta);
static void WiredUnit_PollingTimer(TimerHandle_t xTimer);
static void WiredUnit_AccessSystem(uint16_t id,Network_Mac_S *unitSrcMac,UnitAttr_EN attr);
static void WiredUnit_NotifyConference(ConfProtocol_S *prot);
static void WiredUnit_NotifyConferenceWithExData(ConfProtocol_S *prot, uint16_t exLen, uint8_t *exData);
/*******************************************************************************
 * Variables
 ******************************************************************************/
static const NETWORK_IP hostIp = HOST_LOCAL_IP;
static const NETWORK_GW hostGw = HOST_GATEWAY;
static const NETWORK_MASK hostMask = HOST_NETMASK;
static const NETWORK_IP multicastIp = UNIT_DATA_MULTICAST_IP;
static const NETWORK_IP unitIp = UNIT_DEST_IP;
static const NETWORK_PORT unitPort = UNIT_DEST_PORT;
static const NETWORK_MAC nullMac = {0x00,0x00,0x00,0x00,0x00,0x00};
static NETWORK_MAC unitMac;


/* 网络控制句柄 */
static Network_TaskHandler_S *netTaskHandler;

/* 网路连接标志 */
static bool isEthConnected;

/* 发送队列 */
static QueueHandle_t SendQueue;

/* 单元信息 */
static UnitInfo_S *unitInfo;

/* 轮询码 */
static const uint8_t VotePollingCode[] = 
{ 	POLLING_VOTE_FSIGN, POLLING_VOTE_F,POLLING_VOTE_SIGN,  POLLING_VOTE,
	POLLING_ELECT_FSIGN, POLLING_ELECT_F,POLLING_ELECT_SIGN,POLLING_ELECT,   
  	POLLING_RATE_FSIGN, POLLING_RATE_F, POLLING_RATE_SIGN, POLLING_RATE, 
	POLLING_VOTECUSTOM_F_2_S, POLLING_VOTECUSTOM_F_2, POLLING_VOTECUSTOM_L_2_S, POLLING_VOTECUSTOM_L_2, 
	POLLING_VOTECUSTOM_F_3_S, POLLING_VOTECUSTOM_F_3, POLLING_VOTECUSTOM_L_3_S, POLLING_VOTECUSTOM_L_3, 
	POLLING_VOTECUSTOM_F_4_S, POLLING_VOTECUSTOM_F_4, POLLING_VOTECUSTOM_L_4_S, POLLING_VOTECUSTOM_L_4, 
	POLLING_VOTECUSTOM_F_5_S, POLLING_VOTECUSTOM_F_5, POLLING_VOTECUSTOM_L_5_S, POLLING_VOTECUSTOM_L_5, 
	POLLING_SATISFACTION_FSIGN,POLLING_SATISFACTION_F, POLLING_SATISFACTION_SIGN, POLLING_SATISFACTION, 
 };

/* 轮询定时器 */
static TimerHandle_t pollingTimer;


/*******************************************************************************
 * Task & API
 ******************************************************************************/
static AppTask_S ConfigTask = {
	.task = WiredUnit_ConfigTask,
	.name = "WiredUnit.Config",	
	.stack = WIRED_UNIT_TASK_STACK_SIZE,
	.para = null,
	.prio = WIRED_UNIT_TASK_PRIORITY,
	.handle = null
};

static AppTask_S NetDataProcess = {
	.task = WiredUnit_NetDataProcessTask,
	.name = "WiredUnit.NetDataProcess",	
	.stack = WIRED_UNIT_TASK_STACK_SIZE,
	.para = null,
	.prio = WIRED_UNIT_TASK_PRIORITY,
	.handle = null
};

static AppTask_S NetDataTransmit = {
	.task = WiredUnit_NetDataTransmitTask,
	.name = "WiredUnit.NetDataTransmit",	
	.stack = WIRED_UNIT_TASK_STACK_SIZE,
	.para = null,
	.prio = WIRED_UNIT_TASK_PRIORITY,
	.handle = null
};

static AppTask_S *FuncTask[] = {&NetDataProcess,&NetDataTransmit};

static AppLauncher_S Launcher = {
	.init = null,
	.configTask = &ConfigTask,
	.funcNum  = 2,
	.funcTask = FuncTask,
};


WiredUnit_S WiredUnit = {
	.launcher = &Launcher,

    .transmit = WiredUnit_NetDataTransmit,
    .transWithExData = WiredUnit_NetDataTransmitWithExData,
};
/*******************************************************************************
 * Code
 ******************************************************************************/

/**
* @Name  		WiredUnit_ConfigTask
* @Author  		KT
* @Description	启动有线单元功能逻辑任务
* @para			
*
*
* @return
*/
static void WiredUnit_ConfigTask(void *pvParameters) {
    Network_EthPara_S *ethPara;
    Network_EthernetTaskPara_S *taskPara;

    isEthConnected = false;

    ethPara = MALLOC(sizeof(Network_EthPara_S));
    taskPara = MALLOC(sizeof(Network_EthernetTaskPara_S));

    /* 网口初始化 */
    ethPara->index = NETWORK_ENET_TYPE;
    ethPara->type = NETWORK_TYPE;
    NETWORK_SET_ADDR(ethPara->ip,hostIp.addr0,hostIp.addr1,hostIp.addr2,hostIp.addr3);
    NETWORK_SET_ADDR(ethPara->gateway,hostGw.addr0,hostGw.addr1,hostGw.addr2,hostGw.addr3);
    NETWORK_SET_ADDR(ethPara->netmask,hostMask.addr0,hostMask.addr1,hostMask.addr2,hostMask.addr3);
    ethPara->ethStaListener = WiredUnit_EthStaListener;

	/* 配置网卡 */
    Network.ethConfig(ethPara);

    /* 等待网络连接 */
    while(!isEthConnected){
		DELAY(100);
	};

    /* 网络功能初始化 */
    taskPara->port = HOST_PORT;

    /* 多播地址 */
    taskPara->multicastNum = 1;
    taskPara->multiPort = MALLOC(sizeof(NETWORK_PORT) * taskPara->multicastNum);
    taskPara->multiIp = MALLOC(sizeof(NETWORK_IP) * taskPara->multicastNum);
    NETWORK_SET_ADDR(taskPara->multiIp[0],multicastIp.addr0,multicastIp.addr1,multicastIp.addr2,multicastIp.addr3);
    taskPara->multiPort[0] = UNIT_DATA_MULTICAST_PORT;
    NETWORK_SET_MAC((&unitMac),0x01,0x00,0x5E,(unitIp.addr1 & 0x7F),unitIp.addr2,unitIp.addr3);

	/* 初始化网络功能 */
    netTaskHandler = Network.creatTask(NETWORK_ENET_TYPE,tEthernet,taskPara);

	/* 获取单元数据指针 */
	unitInfo = Conference.wiredUnitInfo();

	/* 初始化发送队列 */
	SendQueue = xQueueCreate(100, sizeof(void *));


    /* 初始化轮询定时器 */
    pollingTimer = xTimerCreate("PollingTimer",POLLING_TIME_MS,pdTRUE,null,WiredUnit_PollingTimer);

	Log.i("Wired unit configuration finish ... \r\n");

    
    vTaskDelete(null);
}

/**
* @Name  		WiredUnit_NetDataTransmitTask
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void WiredUnit_NetDataTransmitTask(void *pvParameters)
{
    Network_DataBuf_S *netBuf;

	Log.i("Wired unit data transmit task start!!\r\n");

    while(1) {
        xQueueReceive(SendQueue, &netBuf, MAX_NUM);
        ERR_CHECK_DBG(netBuf != null,"netBuf == null!!\r\n",continue);

		Network.transmit(netTaskHandler,netBuf);
    	FREE(netBuf->data);
    	FREE(netBuf);

        /* 发送延时 */
        DELAY(TRANSMIT_TIME_INTERVAL_MS);
    }
}



/**
* @Name  		WiredUnit_PollingTimer
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void WiredUnit_PollingTimer(TimerHandle_t xTimer) {
    uint16_t id;
    ConfProtocol_S prot;
	ConfSysInfo_S *info;

    memset(&prot,0,sizeof(ConfProtocol_S));

	info = Conference.getConfSysInfo();

	/* 编ID模式下轮询 */
	if(info->state.sysMode == kMode_EditID){
		WiredUnit_NetDataTransmit(Protocol.conference(&prot, WHOLE_BROADCAST_ID, POLLING_MSG, EDIT_ID_POLLING, CURRENT_ID,0,Conference.getCurrentEditId()));
	}

	/* 非编ID模式下 */
	else{
		/* 轮询从1号ID开始 */
		for(id = 1; id<WIRED_UNIT_MAX_ONLINE_NUM; id++) {
			
			/* 如果当前不为编ID模式且单元在线，发轮询指令 */
			if(unitInfo[id].online) {
				/* 根据会议模式轮询 */
				switch(info->state.sysMode){
					/* 会议模式 */
					case kMode_Conference:{
						Protocol.conference(&prot,id,BASIC_MSG,CONFERENCE_MODE,QUERY_UNIT,0,0);
					}break;
					/* 签到模式 */
					case kMode_DevSign:
					case kMode_Sign:{
						if(unitInfo[id].sign){
							Protocol.conference(&prot,id,BASIC_MSG,CONFERENCE_MODE,QUERY_UNIT,0,0);
						}else{
							Protocol.conference(&prot,id,POLLING_MSG,SIGN_POLLING,HOST_SIGN_POLLIGN,0,0);
						}
					}break;
					/* 投票模式 */
					case kMode_DevVote:
					case kMode_Vote:{
						if(info->state.voteMode == VotePause){
							Protocol.conference(&prot,id,BASIC_MSG,CONFERENCE_MODE,QUERY_UNIT,0,0);
						}else{
							Protocol.conference(&prot,id,POLLING_MSG,VOTE_POLLING,VotePollingCode[info->state.voteMode],0,0);
						}
					}break;
					
					default:break;
				}

				if(unitInfo[id].pollCount++ < POLLING_NO_REPLY_OFFLINE_COUNT){
					WiredUnit_NetDataTransmit(&prot);
				}
				/* 超过轮询次数，单元下线 */
				else{
					unitInfo[id].online = false;

					Protocol.conference(&prot,id,BASIC_MSG,CONFERENCE_MODE,UNIT_OFFLINE,0,0);
					WiredUnit_NetDataTransmit(&prot);
					
					/* 通知会议任务 */
					WiredUnit_NotifyConference(&prot);
				}
			}
		}
	}
}

/**
* @Name
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void WiredUnit_EthStaListener(bool sta) {
    isEthConnected = sta;
}

/**
* @Name  		WiredUnit_NetDataProcessTask
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void WiredUnit_NetDataProcessTask(void *pvParameters) {
    Network_DataBuf_S *taskBuf;
    Network_Mac_S *unitSrcMac;
    NetData_S *netData;
	ConfSysInfo_S *info;

    ConfProtocol_S prot;

    taskBuf	= MALLOC(sizeof(Network_DataBuf_S));
    taskBuf->data = MALLOC(UNIT_DATA_RECEIVE_BUF_SIZE);
    taskBuf->maxLen = UNIT_DATA_RECEIVE_BUF_SIZE;
	
    Log.i("Wired unit data process task start!!\r\n");

	
	xTimerStart(pollingTimer,0);
	Log.i("Wired unit polling timer start!!\r\n");

	/* 主机开机 */
	WiredUnit_NetDataTransmit(Protocol.conference(&prot,WHOLE_BROADCAST_ID,BASIC_MSG,CONFERENCE_MODE,HOST_LAUNCH,null,null));

	/* 广播模式及数量 */
	info = Conference.getConfSysInfo();
	WiredUnit_NetDataTransmit(Protocol.conference(&prot,MODE_BROADCAST_ID,STATE_MSG,info->config->micMode,info->config->wiredAllowOpen,null,null));
    while(1) {

        Network.receive(netTaskHandler,taskBuf,MAX_NUM);

        if(taskBuf->len < 14) {
            continue;
        }
        netData = (NetData_S *)taskBuf->data;

        /* 复制参数 */
        memcpy(&prot,&netData->prot,sizeof(ConfProtocol_S));
        prot.id = lwip_htons(prot.id);
        unitSrcMac = &taskBuf->srcMac;

        if(prot.id >= 1 && prot.id <= WIRED_UNIT_MAX_ONLINE_NUM) {

            /* 检查单元在线标志位 */
            if(unitInfo[prot.id].online) {
                /* 单元状态为在线 */
				
                unitInfo[prot.id].pollCount = 0;

                /* 单元申请进入系统 */
				if(IS_APPLY_ACCESS_SYS(prot.type,prot.ph,prot.pl))	goto access_sys;

                /* 单元回复在线 */
				else if(IS_UNIT_REPLY_ONLINE(prot.type,prot.ph,prot.pl))	goto clear_buf;


                /* 其他会议消息发送给会议任务处理 */
                WiredUnit_NotifyConference(&prot);
                goto clear_buf;
            } else {
                /* 单元状态为离线 */
				
				/* 单元申请进入系统 */
                if(IS_APPLY_ACCESS_SYS(prot.type,prot.ph,prot.pl)) {
                    goto access_sys;
                }
				
				/* 单元确认当前ID(编ID),需要向会议任务报告申请ID的MAC */
				else if(IS_CONFIRM_ID(prot.type,prot.ph,prot.pl)){
					WiredUnit_NotifyConferenceWithExData(&prot,6,(uint8_t *)unitSrcMac);
					goto clear_buf;
				}
                /* 如果为非在线状态而且不是申请进入系统及编ID，不处理当前数据，直接返回给单元离线 */
                else {
                    Protocol.conference(&prot,prot.id,BASIC_MSG,CONFERENCE_MODE, UNIT_OFFLINE,null,null);
                    WiredUnit_NetDataTransmit(&prot);
                    goto clear_buf;
                }
            }
            
access_sys:
			/* 单元机进系统 */
            if(prot.pl == RPS_APPLY_ACCESS_SYS)
                WiredUnit_AccessSystem(prot.id,unitSrcMac,aRepresentative);
            else if(prot.pl == CHM_APPLY_ACCESS_SYS)
                WiredUnit_AccessSystem(prot.id,unitSrcMac,aChairman);

        }

        /* 清空缓存 */
clear_buf:
        taskBuf->len = 0;
        memset(taskBuf->data,0,UNIT_DATA_RECEIVE_BUF_SIZE);
    }
}

/**
* @Name  		WiredUnit_AccessSystem
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void WiredUnit_AccessSystem(uint16_t id,Network_Mac_S *unitSrcMac,UnitAttr_EN attr) {
    ConfProtocol_S prot;
    UnitInfo_S *devInfo;
	ConfSysInfo_S *info;
	uint8_t content[20],contLen = 0;

    if(id <= 0 || id > WIRED_UNIT_MAX_ONLINE_NUM || unitSrcMac == null || NETWORK_COMPARISON_MAC(unitSrcMac,(&nullMac)))
        return;

	info = Conference.getConfSysInfo();
    devInfo = &unitInfo[id];

	/* 获取系统状态并下发 */
        content[6] = info->state.sysMode;
        switch(info->state.sysMode) {
        case kMode_Conference:
        default:
            content[7] = info->config->micMode;
            content[8] = info->config->wiredAllowOpen;
            contLen = 9;
            break;
        case kMode_Sign:
            content[7] = (uint8_t)info->state.totalSignNum >> 8;
            content[8] = (uint8_t)info->state.totalSignNum;
            content[9] = (uint8_t)info->state.currentSignNum >> 8;
            content[10] = (uint8_t)info->state.currentSignNum;
            content[11] = devInfo->sign;
            content[12] = info->config->micMode;
            content[13] = info->config->wiredAllowOpen;
            contLen = 14;
            break;
        case kMode_Vote:
            content[7] = info->state.voteMode;
            content[8] = info->config->micMode;
            content[9] = info->config->wiredAllowOpen;
            content[10] = devInfo->vote;
            content[11] = devInfo->sign;
            contLen = 12;
            break;
        case kMode_EditID:
            contLen = 7;
            break;
		case kMode_DevSign:{
			content[7] = (uint8_t)info->state.totalSignNum >> 8;
            content[8] = (uint8_t)info->state.totalSignNum;
            content[9] = (uint8_t)info->state.currentSignNum >> 8;
            content[10] = (uint8_t)info->state.currentSignNum;
            content[11] = devInfo->sign;
            content[12] = info->config->micMode;
            content[13] = info->config->wiredAllowOpen;
			content[14] = (uint8_t)info->state.initSignVoteId >> 8;
			content[15] = (uint8_t)info->state.initSignVoteId;
            contLen = 16;
		}break;
		case kMode_DevVote:{
			content[7] = info->state.voteMode;
            content[8] = info->config->micMode;
            content[9] = info->config->wiredAllowOpen;
            content[10] = devInfo->vote;
            content[11] = devInfo->sign;
			content[12] = (uint8_t)info->state.initSignVoteId >> 8;
			content[13] = (uint8_t)info->state.initSignVoteId;
            contLen = 14;
		}break;
        }

	memcpy(&content[0],unitSrcMac,NETWORK_MAC_SIZE);

    /* 设备ID已在线，且MAC与申请进系统MAC不同 */
    if(devInfo->online && !NETWORK_COMPARISON_MAC((&devInfo->mac),unitSrcMac)) {

		/* 先告诉单元允许进入系统(由于单元移植二代逻辑，因此这里增加先允许单元进入系统) */
		Protocol.conference(&prot,id, BASIC_MSG, CONFERENCE_MODE, UNIT_ACCESS_SYS,info->config->micMode,info->config->wiredAllowOpen << 8);
		WiredUnit_NetDataTransmitWithExData(&prot,contLen,content);
	
		/* 然后通知会议主线程ID重复 */
        WiredUnit_NotifyConference(Protocol.conference(&prot,WHOLE_BROADCAST_ID, BASIC_MSG, CONFERENCE_MODE, ID_DUPICATE,null,id));
    }
    /* 设备ID不在线,或已在线但MAC相同 */
    else {
		devInfo->attr = attr;
        devInfo->pollCount = 0;
        NETWORK_SET_MAC((&devInfo->mac),unitSrcMac->mac0,unitSrcMac->mac1,unitSrcMac->mac2,unitSrcMac->mac3,unitSrcMac->mac4,unitSrcMac->mac5);

		/* 同意进入系统并通知话筒模式 */
		Protocol.conference(&prot,id, BASIC_MSG, CONFERENCE_MODE, UNIT_ACCESS_SYS,info->config->micMode,info->config->wiredAllowOpen << 8);
		WiredUnit_NetDataTransmitWithExData(&prot,contLen,content);

		WiredUnit_NotifyConference(&prot);

    }
}

/**
* @Name  		WiredUnit_NotifyConferenceWithExData
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void WiredUnit_NotifyConferenceWithExData(ConfProtocol_S *prot, uint16_t exLen, uint8_t *exData) {

    Notify_S *notify;

    if(prot == null || (exLen != 0 && exData == null))
        return;

    notify = MALLOC(sizeof(Notify_S) + exLen);

    notify->nSrc = kType_NotiSrc_WiredUnit;

    memcpy(&notify->prot.conference,prot,sizeof(ConfProtocol_S));
    notify->exLen = exLen;
    if(exLen != 0) {
        memcpy(&notify->exDataHead,exData,exLen);
    }

    Conference.notify(notify);
}

/**
* @Name  		WiredUnit_NotifyConference
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void WiredUnit_NotifyConference(ConfProtocol_S *prot) {
    WiredUnit_NotifyConferenceWithExData(prot, null, null);
}


/**
* @Name  		WiredUnit_NetDataTransmitWithExData
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void WiredUnit_NetDataTransmitWithExData(ConfProtocol_S *prot, uint16_t exLen, uint8_t *exData) {
    Network_DataBuf_S *netBuf;
    NetData_S *netData;
    uint32_t dataLen;

    if(prot == null || (exLen != 0 && exData == null))
        return;

    dataLen = exLen + sizeof(NetData_S);
    netData = MALLOC(dataLen);
    netBuf = MALLOC(sizeof(Network_DataBuf_S));
    memset(netData,0,dataLen);

    netData->header[0] = 'h';
    netData->header[1] = 'y';
    netData->header[2] = 'x';
    netData->header[3] = 't';
    memcpy(&netData->prot,prot,sizeof(ConfProtocol_S));
    netData->prot.id = lwip_htons(netData->prot.id);
	netData->prot.sec = lwip_htons(netData->prot.sec);
    netData->exLen = lwip_htons(exLen);
    if(exLen != 0) {
        memcpy(&netData->exDataHead,exData,exLen);
    }

    NETWORK_SET_MAC((&(netBuf->destMac)),unitMac.mac0,unitMac.mac1,unitMac.mac2,unitMac.mac3,unitMac.mac4,unitMac.mac5);
    NETWORK_SET_ADDR(netBuf->destIp,unitIp.addr0,unitIp.addr1,unitIp.addr2,unitIp.addr3);
    netBuf->destPort = unitPort;
    netBuf->len = dataLen;
    netBuf->data = (uint8_t *)netData;
	
#if 0
	/* 直接发送 */
    Network.transmit(netTaskHandler,netBuf);
    FREE(netData);
    FREE(netBuf);
#endif

	/* 添加到发送队列 */
	xQueueSendToBack(SendQueue, &netBuf, MAX_NUM);

}


/**
* @Name  		WiredUnit_NetDataTransmit
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void WiredUnit_NetDataTransmit(ConfProtocol_S *prot) {
    WiredUnit_NetDataTransmitWithExData(prot,null,null);
}



