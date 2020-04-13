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

/* OS */
#include "FreeRTOS.h"
#include "timers.h"

/* LWIP */
#include "lwip\sys.h"
#include "mdns.h"

/* HAL */
#include "hal_uart.h"
#include "hal_gpio.h"

/* API */
#include "network.h"
#include "ram.h"

/* APP */
#include "global_config.h"
#include "external_ctrl.h"
#include "conference.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/**** 网络相关参数定义 ****/
//#define HOST_LOCAL_IP								EX_CTRL_LOCAL_IP_DEF
//#define HOST_GATEWAY								EX_CTRL_GATEWAY_DEF
//#define HOST_NETMASK								EX_CTRL_NETMASK_DEF
//#define HOST_PORT									EX_CTRL_PORT_DEF

/* 网络接口 */
#define NETWORK_ENET_TYPE							eth0

/* 网络类型 */
#define NETWORK_TYPE								NETWORK_TYPE_TCPIP

/* 外部控制/摄像跟踪串口及波特率 */
#define EXCTRL_UART									(LPUART3)
#define EXCTRL_UART_BAUDRATE						(9600U)


/* 任务堆栈大小及优先级 */
#define EXTERNAL_CTRL_TASK_STACK_SIZE				(1024)
#define EXTERNAL_CTRL_TASK_PRIORITY					(12)

/* 协议指令数据缓存大小 */
#define EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE			(260)

/* 议协议数据包最小长度 */
#define CTRL_CMD_MIN_LEN							(CONF_PROT_MIN_LEN)

/* 最大分包数 */
#define DATA_PACK_MAX_NUM							(EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE / CTRL_CMD_MIN_LEN)



/* 外部控制——主机协议结构 */
//#pragma pack(1)	/* 单字节对齐 */
typedef struct {
    uint8_t header[2];
    uint8_t len;
    ConfProtocol_S prot;
    uint8_t exDataHead;
} ExCtrlData_S;
//#pragma pack()

/* 协议数据捡包 */
typedef struct {
    uint8_t packNum;
    ExCtrlData_S *ctrlData[DATA_PACK_MAX_NUM];
} DataPack_S;

//static PcCtrl_S pcCtrl;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* LAUNCHER调用 */
static void ExternalCtrl_ConfigTask(void *pvParameters);
static void PcCtrl_ProcessTask(void *pvParameters);
static void WebCtrl_ProcessTask(void *pvParameters);
static void UartCtrl_ProcessTask(void *pvParameters);

/* API调用 */
//static void ExternalCtrl_Launch(void);
static void ExternalCtrl_Transmit(EXE_DEST dest, ConfProtocol_S *prot);
static void ExternalCtrl_TransmitWithExData(EXE_DEST dest, ConfProtocol_S *prot, uint16_t exLen, uint8_t *exData);
static void ExternalCtrl_TransmitByByte(EXE_DEST dest, uint8_t *data, uint16_t len);
static bool ExternalCtrl_ConnectState(EXE_DEST dest);

/* 内部调用 */

static void ExternalCtrl_EthStaListener(bool sta);
static DataPack_S *ExternalCtrl_FetchDataFromNetBuf(Network_DataBuf_S *taskBuf);
static void ExternalCtrl_NotifyConference(NotifySrcType_EN nSrc, ConfProtocol_S *prot);
static void ExternalCtrl_NotifyConferenceWithExData(NotifySrcType_EN nSrc, ConfProtocol_S *prot, uint16_t exLen, uint8_t *exData);
static void ExternalCtrl_ReplyHeartbeat(EXE_DEST dest);
static void ExternalCtrl_ReplyQuery(EXE_DEST dest, uint8_t para);
static void UartCtrl_UartCallback(uint8_t count,void *para);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* 网络参数变量 */
static NETWORK_IP hostIp;
static NETWORK_GW hostGw;
static NETWORK_MASK hostMask;
static NETWORK_PORT	hostPort;

/* 网线连接检测标志 */
static bool isEthConnected;

/* 网线连接检测标志 */
static bool isPcCtrlEnable;
static bool isWebCtrlEnable;
//static bool isUartCtrlEnable;

/* 网络任务控制句柄 */
static Network_TaskHandler_S *pcNetTaskHandler;
static Network_TaskHandler_S *webNetTaskHandler;

/* 串口控制句柄 */
static HAL_UartHandler_S UartCtrltHandler;


/* 控制指令数据中捡取指令数据包 */
static DataPack_S fetchDataPack;
/* 控制指令处理信号量 */
static SemaphoreHandle_t CtrlProcessSemaphore;

/* 串口接收信号量 */
static SemaphoreHandle_t UartRecvSemaphore;


/* 网页文件 */
extern const HTTPSRV_FS_DIR_ENTRY httpsrv_fs_data[];

/* 接收静态缓存及计数 */
static uint8_t UartCount = 0;
//static uint8_t UartBuf[EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE];
static uint8_t *UartBuf;

/* 485、232电源控制GPIO句柄 */
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
	.name = "ExternalCtrl.PcCtrlProcess",	
	.stack = EXTERNAL_CTRL_TASK_STACK_SIZE,
	.para = null,
	.prio = EXTERNAL_CTRL_TASK_PRIORITY,
	.handle = null
};

static AppTask_S WebCtrlProcess = {
	.task = WebCtrl_ProcessTask,
	.name = "ExternalCtrl.WebCtrlProcess",	
	.stack = EXTERNAL_CTRL_TASK_STACK_SIZE,
	.para = null,
	.prio = EXTERNAL_CTRL_TASK_PRIORITY,
	.handle = null
};

static AppTask_S UartCtrlProcess = {
	.task = UartCtrl_ProcessTask,
	.name = "ExternalCtrl.UartCtrlProcess",	
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
//    .launch = ExternalCtrl_Launch,
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
* @Name  		ExternalCtrl_Launch
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
//static void ExternalCtrl_Launch(void)
//{
//    if (xTaskCreate(ExternalCtrl_ConfigTask, "ExternalCtrlLaunchTask", EXTERNAL_CTRL_TASK_STACK_SIZE, null, EXTERNAL_CTRL_TASK_PRIORITY, null) != pdPASS) {
//        Log.e("create launch task error\r\n");
//    }
//}


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

    isEthConnected = false;

    ethPara = MALLOC(sizeof(Network_EthPara_S));
    ethPara->mac = MALLOC(sizeof(Network_Mac_S));

    /* 从数据库读取网络参数 */
    sysCfg = (SysCfg_S *)Database.getInstance(kType_Database_SysCfg);

    memcpy(&hostIp,sysCfg->ip,NETWORK_IP_SIZE);
    memcpy(&hostGw,sysCfg->gateWay,NETWORK_IP_SIZE);
    memcpy(&hostMask,sysCfg->mask,NETWORK_IP_SIZE);

    /* 网口初始化 */
    ethPara->index = NETWORK_ENET_TYPE;
    ethPara->type = NETWORK_TYPE;
    NETWORK_SET_ADDR(ethPara->ip,hostIp.addr0,hostIp.addr1,hostIp.addr2,hostIp.addr3);
    NETWORK_SET_ADDR(ethPara->gateway,hostGw.addr0,hostGw.addr1,hostGw.addr2,hostGw.addr3);
    NETWORK_SET_ADDR(ethPara->netmask,hostMask.addr0,hostMask.addr1,hostMask.addr2,hostMask.addr3);
    hostPort = EX_CTRL_PORT_DEF;

    /* MAC地址根据IP变化（避免双机状态MAC冲突） */
    NETWORK_SET_MAC(ethPara->mac,0x02,0x12,hostIp.addr0,hostIp.addr1,hostIp.addr2,hostIp.addr3);

    ethPara->ethStaListener = ExternalCtrl_EthStaListener;

    Network.ethConfig(ethPara);

    /* 初始化互斥信号量 —— 多任务互锁 */
    CtrlProcessSemaphore = xSemaphoreCreateMutex();
    /* 初始化二值信号量 —— 串口接收 */
    UartRecvSemaphore = xSemaphoreCreateBinary();


	isPcCtrlEnable = isWebCtrlEnable = false;


	Log.i("External control configuration finish ... \r\n");
    

    vTaskDelete(null);
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
    isEthConnected = sta;
	if(!sta){
		isPcCtrlEnable = isWebCtrlEnable = false;
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
    DataPack_S *dataPack = &fetchDataPack;
    uint16_t i;
//	uint8_t *data;

    dataPack->packNum = 0;
    memset(dataPack->ctrlData,0,DATA_PACK_MAX_NUM);

    ERR_CHECK(data != null,return dataPack);
    ERR_CHECK(len >= CTRL_CMD_MIN_LEN,return dataPack);

//	data = buf->data;

    /* 检索包头并根据包长度验证包尾*/
    /*	(taskBuf-> - CTRL_CMD_MIN_LEN)这个操作是因为一个包长度不应
    	该小于CTRL_CMD_MIN_LEN，因此检索包头到数据包最后的字节就不需要了*/
    for(i = 0; i <= len - CTRL_CMD_MIN_LEN; i++) {
        if(data[i] == 0xAA && data[i+1] == 0xEE && \
           data[i + 3 + data[i+2]] == 0xEE && \
           data[i + 4 + data[i+2]] == 0xFC) {

            /* 保存索引指针到数据包，包数量加一 */
            dataPack->ctrlData[dataPack->packNum] = (ExCtrlData_S *)(&data[i]);
            dataPack->packNum++;
            /* 数据索引跳过已经检索到的数据段 */
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

    /* 将需要频繁操作或快速回复参数的操作放在接收线程直接处理，其他的发送给会议任务线程处理 */
    /* 回复心跳 */
    if(prot.ph == HEARTBEAT && prot.pl == HEARTBEAT && prot.type == PC_MSG ) {
        ExternalCtrl_ReplyHeartbeat(nSrc);
        return;
    }
    /* 回复状态查询 */
    else if(prot.ph >= QUERY_PRIOR_SIGN && prot.ph <= QUERY_PRIOR_SCAN && prot.type == PC_MSG ) {
        ExternalCtrl_ReplyQuery(nSrc,prot.ph);
        return;
    }

    /* 通知会议任务 */
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

    if((dest & kType_NotiSrc_PC) && isPcCtrlEnable)
        Network.transmit(pcNetTaskHandler,taskBuf);

    if((dest & kType_NotiSrc_Web) && isWebCtrlEnable) {
        taskBuf->webType = tWebsocket;
        taskBuf->wsType = WS_DATA_BINARY;
        Network.transmit(webNetTaskHandler,taskBuf);
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
    ERR_CHECK(((dest & kType_NotiSrc_PC) && isPcCtrlEnable) ||   \
              ((dest & kType_NotiSrc_Web) && isWebCtrlEnable) ||  \
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
		state = isPcCtrlEnable;
	else if(dest & kType_NotiSrc_Web)
		state = isWebCtrlEnable;

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
    isPcCtrlEnable = sta;
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

	 /* 等待网络连接 */
    while(!isEthConnected) {
        DELAY(500);
    }

    taskBuf	= MALLOC(sizeof(Network_DataBuf_S));
    taskBuf->data = MALLOC(EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE);
    taskBuf->maxLen = EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE;
    taskBuf->len = 0;

    /* 初始化网络任务 */
    taskPara = MALLOC(sizeof(Network_TcpTaskPara_S));
    taskPara->port = hostPort;
    taskPara->type = tServer;
    taskPara->tcpListener = PcCtrl_TcpStaListener;
    pcNetTaskHandler = Network.creatTask(NETWORK_ENET_TYPE,tTcp,taskPara);

    while(1) {
        Network.receive(pcNetTaskHandler,taskBuf,MAX_NUM);

        xSemaphoreTake(CtrlProcessSemaphore, MAX_NUM);
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
    isWebCtrlEnable = sta;
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

	 /* 等待网络连接 */
    while(!isEthConnected) {
        DELAY(500);
    }

    taskBuf	= MALLOC(sizeof(Network_DataBuf_S));
    taskBuf->data = MALLOC(EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE);
    taskBuf->maxLen = EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE;
    taskBuf->len = 0;

    /* 配置HTTP网络任务 */
    taskPara = MALLOC(sizeof(Network_HttpTaskPara_S));
    taskPara->httpFsDir = httpsrv_fs_data;
    taskPara->rootDir = "";
    taskPara->indexPage = "/index.html";

    /* 端口配置为null 表示使用默认端口 */
    taskPara->port = null;

    /* 使能CGI功能并配置 */
#if 0
    taskPara->cgi.enable = true;
    taskPara->cgi.postName = "post_input";
#endif

    /* 使能Websocket功能并配置 */
    taskPara->websocket.enable = true;
    taskPara->websocket.wsListener = WebCtrl_WsStaListener;

    /* 初始化网络任务 */
    webNetTaskHandler = Network.creatTask(NETWORK_ENET_TYPE,tHttp,taskPara);

    while(1) {
        Network.receive(webNetTaskHandler,taskBuf,MAX_NUM);

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

    /* 控制IO打开电源 */
    SerialPower = HAL_GpioInit(GPIO3, 3, kGPIO_DigitalOutput, null, (gpio_interrupt_mode_t)null);
    HAL_SetGpioLevel(SerialPower, 0);

    UartBuf = MALLOC(EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE);

    /* 初始化串口 */
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
        /* 获取到信号量后延时，然后再获取一次信号量（不等待），
        清除串口中断接收多字节时的重复信号量 */
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



