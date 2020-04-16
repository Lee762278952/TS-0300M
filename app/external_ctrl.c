/**
 *
 *	@name				external_ctrl.c
 *	@author 			KT
 *	@date 				2019/07/31
 *	@brief
 *  @include			external_ctrl.h
 *
 *  @API
 *
 *  @description
 *
 **/
/*******************************************************************************
 * includes
 ******************************************************************************/
/* CLIB*/
#include "stdio.h"
#include "ctype.h"

/* HAL */
#include "hal_uart.h"
#include "hal_gpio.h"

/* OS */
#include "FreeRTOS.h"
#include "timers.h"

/* LWIP */
#include "lwip\sys.h"
#include "mdns.h"

/* GLOBAL */
#include "log.h"

/* APP */
#include "global_config.h"
#include "external_ctrl.h"
#include "conference.h"
#include "network.h"
#include "ram.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/**** ������ز������� ****/
//#define HOST_LOCAL_IP								EX_CTRL_LOCAL_IP_DEF
//#define HOST_GATEWAY								EX_CTRL_GATEWAY_DEF
//#define HOST_NETMASK								EX_CTRL_NETMASK_DEF
//#define HOST_PORT									EX_CTRL_PORT_DEF

/* ����ӿ� */
#define NETWORK_ENET_TYPE							eth0

/* �������� */
#define NETWORK_TYPE								NETWORK_TYPE_TCPIP

/* �ⲿ����/������ٴ��ڼ������� */
#define EXCTRL_UART									(LPUART3)
#define EXCTRL_UART_BAUDRATE						(9600U)


/* �����ջ��С�����ȼ� */
#define EXTERNAL_CTRL_TASK_STACK_SIZE				(1024)
#define EXTERNAL_CTRL_TASK_PRIORITY					(12)

/* ����������м��(ms) */
#define HEARTBEAT_MONITOR_INTERVAL					(1000)

/* Э��ָ�����ݻ����С */
#define EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE			(260)

/* ��Э�����ݰ���С���� */
#define CTRL_CMD_MIN_LEN							(CONF_PROT_MIN_LEN)

/* ���ְ��� */
#define DATA_PACK_MAX_NUM							(EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE / CTRL_CMD_MIN_LEN)



/* �ⲿ���ơ�������Э��ṹ */
//#pragma pack(1)	/* ���ֽڶ��� */
typedef struct {
    uint8_t header[2];
    uint8_t len;
    ConfProtocol_S prot;
    uint8_t exDataHead;
} ExCtrlData_S;
//#pragma pack()

/* Э�����ݼ�� */
typedef struct {
    uint8_t packNum;
    ExCtrlData_S *ctrlData[DATA_PACK_MAX_NUM];
} DataPack_S;

//static PcCtrl_S pcCtrl;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* LAUNCHER���� */
static void ExternalCtrl_ConfigTask(void *pvParameters);
static void PcCtrl_ProcessTask(void *pvParameters);
static void WebCtrl_ProcessTask(void *pvParameters);
static void UartCtrl_ProcessTask(void *pvParameters);

/* API���� */
//static void ExternalCtrl_Launch(void);
static void ExternalCtrl_Transmit(EXE_DEST dest, ConfProtocol_S *prot);
static void ExternalCtrl_TransmitWithExData(EXE_DEST dest, ConfProtocol_S *prot, uint16_t exLen, uint8_t *exData);
static void ExternalCtrl_TransmitByByte(EXE_DEST dest, uint8_t *data, uint16_t len);
static bool ExternalCtrl_ConnectState(EXE_DEST dest);

/* �ڲ����� */
static void ExternalCtrl_HeartbeatMonitor(TimerHandle_t xTimer);
static void ExternalCtrl_EthStaListener(bool sta);
static DataPack_S *ExternalCtrl_FetchDataFromNetBuf(Network_DataBuf_S *taskBuf);
static void ExternalCtrl_NotifyConference(NotifySrcType_EN nSrc, ConfProtocol_S *prot);
static void ExternalCtrl_NotifyConferenceWithExData(NotifySrcType_EN nSrc, ConfProtocol_S *prot, uint16_t exLen, uint8_t *exData);
static void ExternalCtrl_ReplyHeartbeat(EXE_DEST dest);
static void ExternalCtrl_ReplyQuery(EXE_DEST dest, uint8_t para);
static void PcCtrl_ProcessTaskReset(void);

static void UartCtrl_UartCallback(uint8_t count,void *para);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* ����������� */
static NETWORK_IP HostIp;
static NETWORK_GW HostGw;
static NETWORK_MASK HostMask;
static NETWORK_PORT	HostPort;

/* �������Ӽ���־ */
static bool IsEthConnected;

/* �������Ӽ���־ */
static bool IsPcCtrlEnable;
static bool IsWebCtrlEnable;
//static bool isUartCtrlEnable;

/* �������� */
static uint8_t PcHeartbeatCnt;
static uint8_t WebHeartbeatCnt;

/* ���������� */
static TaskHandle_t PcTaskHandler;

/* ����������ƾ�� */
static Network_TaskHandler_S *PcNetTaskHandler;
static Network_TaskHandler_S *WebNetTaskHandler;

/* ���ڿ��ƾ�� */
static HAL_UartHandler_S UartCtrltHandler;

/* ������ⶨʱ�� */
static TimerHandle_t HeartbeatMonitor;

/* ����ָ�������м�ȡָ�����ݰ� */
static DataPack_S FetchDataPack;
/* ����ָ����ź��� */
static SemaphoreHandle_t CtrlProcessSemaphore;

/* ���ڽ����ź��� */
static SemaphoreHandle_t UartRecvSemaphore;

/* ��ҳ�ļ� */
extern const HTTPSRV_FS_DIR_ENTRY httpsrv_fs_data[];

/* ���վ�̬���漰���� */
static uint8_t UartCount = 0;
//static uint8_t UartBuf[EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE];
static uint8_t *UartBuf;

/* 485��232��Դ����GPIO��� */
static HAL_GpioHandler SerialPower;


/*******************************************************************************
 * Task & API
 ******************************************************************************/

static AppTask_S ConfigTask = {
	.task = ExternalCtrl_ConfigTask,
	.name = "ExternalCtrl.Config",	
	.stack = EXTERNAL_CTRL_TASK_STACK_SIZE,
	.para = null,
	.prio = EXTERNAL_CTRL_TASK_PRIORITY,
	.handle = null
};


static AppTask_S PcCtrlProcess = {
	.task = PcCtrl_ProcessTask,
	.name = "App.ExternalCtrl.PcCtrlProcess",	
	.stack = EXTERNAL_CTRL_TASK_STACK_SIZE,
	.para = null,
	.prio = EXTERNAL_CTRL_TASK_PRIORITY,
	.handle = &PcTaskHandler
};

static AppTask_S WebCtrlProcess = {
	.task = WebCtrl_ProcessTask,
	.name = "App.ExternalCtrl.WebCtrlProcess",	
	.stack = EXTERNAL_CTRL_TASK_STACK_SIZE,
	.para = null,
	.prio = EXTERNAL_CTRL_TASK_PRIORITY,
	.handle = null
};

static AppTask_S UartCtrlProcess = {
	.task = UartCtrl_ProcessTask,
	.name = "App.ExternalCtrl.UartCtrlProcess",	
	.stack = EXTERNAL_CTRL_TASK_STACK_SIZE,
	.para = null,
	.prio = EXTERNAL_CTRL_TASK_PRIORITY,
	.handle = null
};


static AppTask_S *FuncTask[] = {&PcCtrlProcess, &WebCtrlProcess, &UartCtrlProcess};


static AppLauncher_S Launcher = {
	.init = null,
	.configTask = &ConfigTask,
	.funcNum  = 3,
	.funcTask = FuncTask,
};


ExternalCtrl_S ExternalCtrl = {
	.launcher = &Launcher,

    .transmit = ExternalCtrl_Transmit,
    .transWithExData = ExternalCtrl_TransmitWithExData,
    .transByByte = ExternalCtrl_TransmitByByte,
    .connectSta = ExternalCtrl_ConnectState,

};



/*******************************************************************************
 * Code
 ******************************************************************************/

/**
* @Name  		ExternalCtrl_ConfigTask
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void ExternalCtrl_ConfigTask(void *pvParameters)
{
    Network_EthPara_S *ethPara;
    SysCfg_S *sysCfg;

    IsEthConnected = false;

    ethPara = MALLOC(sizeof(Network_EthPara_S));
    ethPara->mac = MALLOC(sizeof(Network_Mac_S));

    /* �����ݿ��ȡ������� */
    sysCfg = (SysCfg_S *)Database.getInstance(kType_Database_SysCfg);

    memcpy(&HostIp,sysCfg->ip,NETWORK_IP_SIZE);
    memcpy(&HostGw,sysCfg->gateWay,NETWORK_IP_SIZE);
    memcpy(&HostMask,sysCfg->mask,NETWORK_IP_SIZE);

    /* ���ڳ�ʼ�� */
    ethPara->index = NETWORK_ENET_TYPE;
    ethPara->type = NETWORK_TYPE;
    NETWORK_SET_ADDR(ethPara->ip,HostIp.addr0,HostIp.addr1,HostIp.addr2,HostIp.addr3);
    NETWORK_SET_ADDR(ethPara->gateway,HostGw.addr0,HostGw.addr1,HostGw.addr2,HostGw.addr3);
    NETWORK_SET_ADDR(ethPara->netmask,HostMask.addr0,HostMask.addr1,HostMask.addr2,HostMask.addr3);
    HostPort = EX_CTRL_PORT_DEF;

    /* MAC��ַ����IP�仯������˫��״̬MAC��ͻ�� */
    NETWORK_SET_MAC(ethPara->mac,0x02,0x12,HostIp.addr0,HostIp.addr1,HostIp.addr2,HostIp.addr3);

    ethPara->ethStaListener = ExternalCtrl_EthStaListener;

	/* �������� */
    Network.ethConfig(ethPara);

	/* ��ʼ����ʱ�� */
	HeartbeatMonitor = xTimerCreate("Heartbeat",HEARTBEAT_MONITOR_INTERVAL,pdTRUE,null,ExternalCtrl_HeartbeatMonitor);

    /* ��ʼ�������ź��� ���� �������� */
    CtrlProcessSemaphore = xSemaphoreCreateMutex();
    /* ��ʼ����ֵ�ź��� ���� ���ڽ��� */
    UartRecvSemaphore = xSemaphoreCreateBinary();


	IsPcCtrlEnable = IsWebCtrlEnable = false;


	Log.i("External control configuration finish ... \r\n");
    

    vTaskDelete(null);
}


/**
* @Name  		ExternalCtrl_HeartbeatMonitor
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void ExternalCtrl_HeartbeatMonitor(TimerHandle_t xTimer){

	if(IsPcCtrlEnable){
		if(PcHeartbeatCnt++ > 5){
			Log.d("Pc connect time out \r\n");
			PcCtrl_ProcessTaskReset();
		}
	}
}


/**
* @Name  		ExternalCtrl_EthStaListener
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void ExternalCtrl_EthStaListener(bool sta)
{
    IsEthConnected = sta;
	if(!sta){
		IsPcCtrlEnable = IsWebCtrlEnable = false;
	}
    Log.i("External control (net port) %s \r\n",sta ? "connected" : "disconnected");
}


/**
* @Name  		ExternalCtrl_FetchDataFromBuf
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static DataPack_S *ExternalCtrl_FetchDataFromBuf(uint8_t *data,uint32_t len)
{
    DataPack_S *dataPack = &FetchDataPack;
    uint16_t i;
//	uint8_t *data;

    dataPack->packNum = 0;
    memset(dataPack->ctrlData,0,DATA_PACK_MAX_NUM);

    ERR_CHECK(data != null,return dataPack);
    ERR_CHECK(len >= CTRL_CMD_MIN_LEN,return dataPack);

//	data = buf->data;

    /* ������ͷ�����ݰ�������֤��β*/
    /*	(taskBuf-> - CTRL_CMD_MIN_LEN)�����������Ϊһ�������Ȳ�Ӧ
    	��С��CTRL_CMD_MIN_LEN����˼�����ͷ�����ݰ������ֽھͲ���Ҫ��*/
    for(i = 0; i <= len - CTRL_CMD_MIN_LEN; i++) {
        if(data[i] == 0xAA && data[i+1] == 0xEE && \
           data[i + 3 + data[i+2]] == 0xEE && \
           data[i + 4 + data[i+2]] == 0xFC) {

            /* ��������ָ�뵽���ݰ�����������һ */
            dataPack->ctrlData[dataPack->packNum] = (ExCtrlData_S *)(&data[i]);
            dataPack->packNum++;
            /* �������������Ѿ������������ݶ� */
            i = i + 4 + data[i+2];
        }
    }

    return dataPack;
}



/**
* @Name  		ExternalCtrl_FetchDataFromNetBuf
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static DataPack_S *ExternalCtrl_FetchDataFromNetBuf(Network_DataBuf_S *buf)
{

    return ExternalCtrl_FetchDataFromBuf(buf->data,buf->len);
}




/**
* @Name  		ExternalCtrl_DataProcess
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void ExternalCtrl_DataProcess(NotifySrcType_EN nSrc, ExCtrlData_S *ctrlData)
{
    ConfProtocol_S prot;
    uint8_t exLen = 0, *exData = null;

    ERR_CHECK(ctrlData != null,return);

    memcpy(&prot,&ctrlData->prot,sizeof(ConfProtocol_S));
    prot.id = lwip_htons(prot.id);

	if(nSrc & kType_NotiSrc_PC){
		PcHeartbeatCnt = 0;
	}

    /* ����ҪƵ����������ٻظ������Ĳ������ڽ����߳�ֱ�Ӵ��������ķ��͸����������̴߳��� */
    /* �ظ����� */
    if(prot.ph == HEARTBEAT && prot.pl == HEARTBEAT && prot.type == PC_MSG ) {
        ExternalCtrl_ReplyHeartbeat(nSrc);
        return;
    }
    /* �ظ�״̬��ѯ */
    else if(prot.ph >= QUERY_PRIOR_SIGN && prot.ph <= QUERY_PRIOR_SCAN && prot.type == PC_MSG ) {
        ExternalCtrl_ReplyQuery(nSrc,prot.ph);
        return;
    }

    /* ֪ͨ�������� */
    if(ctrlData->len > sizeof(ConfProtocol_S)) {
        exLen = ctrlData->len - sizeof(ConfProtocol_S);
        exData = MALLOC(exLen);
        memcpy(exData,&ctrlData->exDataHead,exLen);
        ExternalCtrl_NotifyConferenceWithExData(nSrc,&prot,exLen,exData);
    } else
        ExternalCtrl_NotifyConference(nSrc,&prot);
}

/**
* @Name  		ExternalCtrl_NotifyConferenceWithExData
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void ExternalCtrl_NotifyConferenceWithExData(NotifySrcType_EN nSrc, ConfProtocol_S *prot, uint16_t exLen, uint8_t *exData)
{
    Notify_S *notify;

    ERR_CHECK(prot != null,return);
    ERR_CHECK(!(exLen != 0 && exData == null),return);

    notify = MALLOC(sizeof(Notify_S) + exLen);

    notify->nSrc = nSrc;
//	notify->kWord = kWord;
    memcpy(&notify->prot.conference,prot,sizeof(ConfProtocol_S));
    notify->exLen = exLen;

    if(exLen != 0) {
        memcpy(&notify->exDataHead,exData,exLen);
    }

    Conference.notify(notify);
}

/**
* @Name  		ExternalCtrl_NotifyConference
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void ExternalCtrl_NotifyConference(NotifySrcType_EN nSrc, ConfProtocol_S *prot)
{
    ExternalCtrl_NotifyConferenceWithExData(nSrc, prot, null, null);
}


/**
* @Name  		ExternalCtrl_TransmitByByte
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void ExternalCtrl_TransmitByByte(EXE_DEST dest, uint8_t *data, uint16_t len)
{
    Network_DataBuf_S *taskBuf;

    taskBuf = MALLOC(sizeof(Network_DataBuf_S));

    taskBuf->len = len;
    taskBuf->data = data;

    if((dest & kType_NotiSrc_PC) && IsPcCtrlEnable)
        Network.transmit(PcNetTaskHandler,taskBuf);

    if((dest & kType_NotiSrc_Web) && IsWebCtrlEnable) {
        taskBuf->webType = tWebsocket;
        taskBuf->wsType = WS_DATA_BINARY;
        Network.transmit(WebNetTaskHandler,taskBuf);
    }

    if(dest & kType_NotiSrc_UartCtrl) {
        HAL_UartSend(UartCtrltHandler, data, len);
    }

    FREE(taskBuf);
}



/**
* @Name  		ExternalCtrl_TransmitWithExData
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void ExternalCtrl_TransmitWithExData(EXE_DEST dest, ConfProtocol_S *prot, uint16_t exLen, uint8_t *exData)
{
    ExCtrlData_S *ctrlData;
    uint8_t dataLen;

    ERR_CHECK(prot != null,return);
    ERR_CHECK(!(exLen != 0 && exData == null),return);
    ERR_CHECK(((dest & kType_NotiSrc_PC) && IsPcCtrlEnable) ||   \
              ((dest & kType_NotiSrc_Web) && IsWebCtrlEnable) ||  \
              (dest & kType_NotiSrc_UartCtrl),return);

    dataLen = exLen + CTRL_CMD_MIN_LEN;
    ctrlData = MALLOC(dataLen);
    memset(ctrlData,0,dataLen);

    ctrlData->header[0] = 0xAA;
    ctrlData->header[1] = 0xEE;
    ctrlData->len = dataLen - 5;
    memcpy(&ctrlData->prot,prot,sizeof(ConfProtocol_S));
    ctrlData->prot.id = lwip_htons(ctrlData->prot.id);
    ctrlData->prot.sec = lwip_htons(ctrlData->prot.sec);
    if(exLen != 0) {
        memcpy(&ctrlData->exDataHead,exData,exLen);
    }
    ((uint8_t *)ctrlData)[dataLen - 2] = 0xEE;
    ((uint8_t *)ctrlData)[dataLen - 1] = 0xFC;

    ExternalCtrl_TransmitByByte(dest,(uint8_t *)ctrlData,dataLen);

    FREE(ctrlData);
}

/**
* @Name  		ExternalCtrl_TransmitWithExData
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void ExternalCtrl_Transmit(EXE_DEST dest, ConfProtocol_S *prot)
{
    ExternalCtrl_TransmitWithExData(dest, prot, null, null);
}


/**
* @Name  		ExternalCtrl_ReplyHeartbeat
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void ExternalCtrl_ReplyHeartbeat(EXE_DEST dest)
{
    UnitOnlineNum onlineNum;
    ConfProtocol_S prot;

    Conference.getOnlineNum(&onlineNum);

    Protocol.conference(&prot,WHOLE_BROADCAST_ID,WIRED_UNIT_NUM,0,onlineNum.interpreter,onlineNum.wiredChm,onlineNum.wiredRps);
    ExternalCtrl_Transmit(dest,&prot);
    Protocol.conference(&prot,WHOLE_BROADCAST_ID,WIFI_UNIT_NUM,0,0,onlineNum.wifiChm,onlineNum.wifiRps);
    ExternalCtrl_Transmit(dest,&prot);
}

/**
* @Name  		ExternalCtrl_ReplyQuery
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void ExternalCtrl_ReplyQuery(EXE_DEST dest, uint8_t para)
{
    ConfProtocol_S prot;
    uint8_t sta;

    sta = (uint8_t)Conference.getSysMode();
    Protocol.conference(&prot, WHOLE_BROADCAST_ID, PC_MSG, para, 0x01, sta,null);
    ExternalCtrl_Transmit(dest,&prot);
}


/**
* @Name  		ExternalCtrl_ConnectState
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static bool ExternalCtrl_ConnectState(EXE_DEST dest)
{
    bool state = false;

	if(dest & kType_NotiSrc_PC)
		state = IsPcCtrlEnable;
	else if(dest & kType_NotiSrc_Web)
		state = IsWebCtrlEnable;

	return state;
}

/*****************************************  PC  ****************************************************************/
/**
* @Name  		PcCtrl_TcpStaListener
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void PcCtrl_TcpStaListener(bool sta)
{
    IsPcCtrlEnable = sta;
    Log.i("PC control is %s \r\n",sta ? "online" : "offline");
}

/**
* @Name  		PcCtrl_ProcessTask
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void PcCtrl_ProcessTask(void *pvParameters)
{
    Network_TcpTaskPara_S *taskPara;
    Network_DataBuf_S *taskBuf;
    DataPack_S *dataPack;

    uint32_t index;

	Log.i("PC control process task start!!\r\n");

	 /* �ȴ��������� */
    while(!IsEthConnected) {
        DELAY(500);
    }

    taskBuf	= MALLOC(sizeof(Network_DataBuf_S));
    taskBuf->data = MALLOC(EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE);
    taskBuf->maxLen = EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE;
    taskBuf->len = 0;

    /* ��ʼ���������� */
    taskPara = MALLOC(sizeof(Network_TcpTaskPara_S));
    taskPara->port = HostPort;
    taskPara->type = tServer;
    taskPara->tcpListener = PcCtrl_TcpStaListener;

    PcNetTaskHandler = Network.creatTask(NETWORK_ENET_TYPE,tTcp,taskPara);

	/* ������ʱ�� */
	xTimerStart(HeartbeatMonitor, 0);

    while(1) {
        Network.receive(PcNetTaskHandler,taskBuf,MAX_NUM);

        xSemaphoreTake(CtrlProcessSemaphore, MAX_NUM);
		/* ��� */
        dataPack = ExternalCtrl_FetchDataFromNetBuf(taskBuf);
        if(dataPack->packNum > 0) {
            for(index = 0; index < dataPack->packNum; index++) {
                ExternalCtrl_DataProcess(kType_NotiSrc_PC,dataPack->ctrlData[index]);
            }
        }
        xSemaphoreGive(CtrlProcessSemaphore);

        taskBuf->len = 0;
        memset(taskBuf->data,0,EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE);

    }
}

/**
* @Name  		PcCtrl_ProcessTaskReset
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void PcCtrl_ProcessTaskReset(void){
	/* �ر��������� */
	Network.destoryTask(PcNetTaskHandler);

	/* �ر����ݴ������� */
	vTaskDelete(PcTaskHandler);
		
	PcHeartbeatCnt = 0;
	IsPcCtrlEnable = false;
			
	if (xTaskCreate(PcCtrlProcess.task, PcCtrlProcess.name, PcCtrlProcess.stack, PcCtrlProcess.para, PcCtrlProcess.prio, PcCtrlProcess.handle) != pdPASS)
	{
		Log.e("Function task \"%s\" creat error\r\n", PcCtrlProcess.name);
	}
}



/*****************************************  WEB  ****************************************************************/
/**
* @Name  		WebCtrl_TcpStaListener
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void WebCtrl_WsStaListener(bool sta)
{
    IsWebCtrlEnable = sta;
    Log.i("Websocket is %s \r\n",sta ? "online" : "offline");
}

/**
* @Name  		WebCtrl_ProcessTask
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void WebCtrl_ProcessTask(void *pvParameters)
{
    Network_HttpTaskPara_S *taskPara;
    Network_DataBuf_S *taskBuf;
    DataPack_S *dataPack;
    uint32_t index;

	Log.i("WEB control process task start!!\r\n");

	 /* �ȴ��������� */
    while(!IsEthConnected) {
        DELAY(500);
    }

    taskBuf	= MALLOC(sizeof(Network_DataBuf_S));
    taskBuf->data = MALLOC(EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE);
    taskBuf->maxLen = EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE;
    taskBuf->len = 0;

    /* ����HTTP�������� */
    taskPara = MALLOC(sizeof(Network_HttpTaskPara_S));
    taskPara->httpFsDir = httpsrv_fs_data;
    taskPara->rootDir = "";
    taskPara->indexPage = "/index.html";

    /* �˿�����Ϊnull ��ʾʹ��Ĭ�϶˿� */
    taskPara->port = null;

    /* ʹ��CGI���ܲ����� */
#if 0
    taskPara->cgi.enable = true;
    taskPara->cgi.postName = "post_input";
#endif

    /* ʹ��Websocket���ܲ����� */
    taskPara->websocket.enable = true;
    taskPara->websocket.wsListener = WebCtrl_WsStaListener;

    /* ��ʼ���������� */
    WebNetTaskHandler = Network.creatTask(NETWORK_ENET_TYPE,tHttp,taskPara);

    while(1) {
        Network.receive(WebNetTaskHandler,taskBuf,MAX_NUM);

        if(taskBuf->webType == tWebsocket && taskBuf->wsType == WS_DATA_BINARY) {
            xSemaphoreTake(CtrlProcessSemaphore, MAX_NUM);
            dataPack = ExternalCtrl_FetchDataFromNetBuf(taskBuf);

            if(dataPack->packNum > 0) {
                for(index = 0; index < dataPack->packNum; index++) {
                    ExternalCtrl_DataProcess(kType_NotiSrc_Web,dataPack->ctrlData[index]);
                }
            }
            xSemaphoreGive(CtrlProcessSemaphore);
        }

        taskBuf->len = 0;
        memset(taskBuf->data,0,EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE);
    }
}
/*****************************************  Uart  ****************************************************************/
/**
* @Name  		UartCtrl_ProcessTask
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void UartCtrl_ProcessTask(void *pvParameters)
{
    HAL_UartConfig_S *config;
    DataPack_S *dataPack;
    uint32_t index;


	Log.i("Uart control process task start!!\r\n");

    /* ����IO�򿪵�Դ */
    SerialPower = HAL_GpioInit(GPIO3, 3, kGPIO_DigitalOutput, null, (gpio_interrupt_mode_t)null);
    HAL_SetGpioLevel(SerialPower, 0);

    UartBuf = MALLOC(EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE);

    /* ��ʼ������ */
    config = MALLOC(sizeof(HAL_UartConfig_S));
    UartCtrltHandler = MALLOC(HAL_UART_HANDLER_SIZE);

    config->base = EXCTRL_UART;
    config->baudRate = EXCTRL_UART_BAUDRATE;
    config->enableRx = true;
    config->enableTx = true;
    config->rxIrq = true;
    config->txIrq = false;

    HAL_UartInit(UartCtrltHandler, config);
    HAL_UartSetCallback(UartCtrltHandler, UartBuf, EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE, UartCtrl_UartCallback, null);

   
    while(1) {
//		DELAY(100);
//		if(UartCount < 13)
//			continue;
        /* ��ȡ���ź�������ʱ��Ȼ���ٻ�ȡһ���ź��������ȴ�����
        ��������жϽ��ն��ֽ�ʱ���ظ��ź��� */
        xSemaphoreTake(UartRecvSemaphore, MAX_NUM);
        DELAY(100);
        xSemaphoreTake(UartRecvSemaphore, 0);

        xSemaphoreTake(CtrlProcessSemaphore, MAX_NUM);
        dataPack = ExternalCtrl_FetchDataFromBuf(UartBuf,UartCount);
        if(dataPack->packNum > 0) {
            for(index = 0; index < dataPack->packNum; index++) {
                ExternalCtrl_DataProcess(kType_NotiSrc_UartCtrl,dataPack->ctrlData[index]);
            }
        }
        xSemaphoreGive(CtrlProcessSemaphore);

        HAL_UartClearCount(UartCtrltHandler);
        UartCount = 0;
    }
}

/**
* @Name  		UartCtrl_UartCallback
* @Author  		KT
* @Description
* @para
*
*
* @return
*/

static void UartCtrl_UartCallback(uint8_t count,void *para)
{
    portBASE_TYPE taskToWake = pdFALSE;

    UartCount = count;
    if(count >= CTRL_CMD_MIN_LEN)
        xSemaphoreGiveFromISR(UartRecvSemaphore, &taskToWake);
}



