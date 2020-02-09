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

/* Э��ָ�����ݻ����С */
#define EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE			(260)

/* ȫ���ֻ���Э�����ݰ���С���� */
#define CTRL_CMD_MIN_LEN							(13)

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
/* API���� */
static void ExternalCtrl_Launch(void);
static void ExternalCtrl_Transmit(EXE_DEST dest, ConfProtocol_S *prot);
static void ExternalCtrl_TransmitWithExData(EXE_DEST dest, ConfProtocol_S *prot, uint16_t exLen, uint8_t *exData);
static void ExternalCtrl_TransmitByByte(EXE_DEST dest, uint8_t *data, uint16_t len);
static bool ExternalCtrl_ConnectState(EXE_DEST dest);

/* �ڲ����� */
static void ExternalCtrl_LaunchTask(void *pvParameters);
static void ExternalCtrl_EthStaListener(bool sta);
static DataPack_S *ExternalCtrl_FetchDataFromNetBuf(Network_DataBuf_S *taskBuf);
static void ExternalCtrl_NotifyConference(NotifySrcType_EN nSrc, ConfProtocol_S *prot);
static void ExternalCtrl_NotifyConferenceWithExData(NotifySrcType_EN nSrc, ConfProtocol_S *prot, uint16_t exLen, uint8_t *exData);
static void ExternalCtrl_ReplyHeartbeat(EXE_DEST dest);
static void ExternalCtrl_ReplyQuery(EXE_DEST dest, uint8_t para);

static void PcCtrl_ProcessTask(void *pvParameters);
static void WebCtrl_ProcessTask(void *pvParameters);
static void UartCtrl_ProcessTask(void *pvParameters);

static void UartCtrl_UartCallback(uint8_t count,void *para);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* ����������� */
static NETWORK_IP hostIp;
static NETWORK_GW hostGw;
static NETWORK_MASK hostMask;
static NETWORK_PORT	hostPort;

/* �������Ӽ���־ */
static bool isEthConnected;

/* �������Ӽ���־ */
static bool isPcCtrlEnable;
static bool isWebCtrlEnable;
//static bool isUartCtrlEnable;

/* ����������ƾ�� */
static Network_TaskHandler_S *pcNetTaskHandler;
static Network_TaskHandler_S *webNetTaskHandler;

/* ���ڿ��ƾ�� */
static HAL_UartHandler_S UartCtrltHandler;


/* ����ָ�������м�ȡָ�����ݰ� */
static DataPack_S fetchDataPack;
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
static HAL_GpioIndex SerialPower;



ExternalCtrl_S ExternalCtrl = {
    .launch = ExternalCtrl_Launch,
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
static void ExternalCtrl_Launch(void)
{
    if (xTaskCreate(ExternalCtrl_LaunchTask, "ExternalCtrlLaunchTask", EXTERNAL_CTRL_TASK_STACK_SIZE, null, EXTERNAL_CTRL_TASK_PRIORITY, null) != pdPASS) {
        Log.e("create launch task error\r\n");
    }
}


/**
* @Name  		ExternalCtrl_LaunchTask
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void ExternalCtrl_LaunchTask(void *pvParameters)
{
    Network_EthPara_S *ethPara;
    SysCfg_S *sysCfg;

    isEthConnected = false;

    ethPara = MALLOC(sizeof(Network_EthPara_S));
    ethPara->mac = MALLOC(sizeof(Network_Mac_S));

    /* �����ݿ��ȡ������� */
    sysCfg = (SysCfg_S *)Database.getInstance(kType_Database_SysCfg);

    memcpy(&hostIp,sysCfg->ip,NETWORK_IP_SIZE);
    memcpy(&hostGw,sysCfg->gateWay,NETWORK_IP_SIZE);
    memcpy(&hostMask,sysCfg->mask,NETWORK_IP_SIZE);

    /* ���ڳ�ʼ�� */
    ethPara->index = NETWORK_ENET_TYPE;
    ethPara->type = NETWORK_TYPE;
    NETWORK_SET_ADDR(ethPara->ip,hostIp.addr0,hostIp.addr1,hostIp.addr2,hostIp.addr3);
    NETWORK_SET_ADDR(ethPara->gateway,hostGw.addr0,hostGw.addr1,hostGw.addr2,hostGw.addr3);
    NETWORK_SET_ADDR(ethPara->netmask,hostMask.addr0,hostMask.addr1,hostMask.addr2,hostMask.addr3);
    hostPort = EX_CTRL_PORT_DEF;

    /* MAC��ַ����IP�仯������˫��״̬MAC��ͻ�� */
    NETWORK_SET_MAC(ethPara->mac,0x02,0x12,hostIp.addr0,hostIp.addr1,hostIp.addr2,hostIp.addr3);

    ethPara->ethStaListener = ExternalCtrl_EthStaListener;

    Network.ethConfig(ethPara);

    /* ��ʼ�������ź��� ���� �������� */
    CtrlProcessSemaphore = xSemaphoreCreateMutex();
    /* ��ʼ����ֵ�ź��� ���� ���ڽ��� */
    UartRecvSemaphore = xSemaphoreCreateBinary();


    /* ����PC�����߳� */
    if (xTaskCreate(PcCtrl_ProcessTask, "PCProcessTask", EXTERNAL_CTRL_TASK_STACK_SIZE, null, EXTERNAL_CTRL_TASK_PRIORITY, null) != pdPASS) {
        Log.e("create PC data receive task error\r\n");
    }

    /* ����WEB�����߳� */
    if (xTaskCreate(WebCtrl_ProcessTask, "WEBProcessTask", EXTERNAL_CTRL_TASK_STACK_SIZE, null, EXTERNAL_CTRL_TASK_PRIORITY, null) != pdPASS) {
        Log.e("create WEB data receive task error\r\n");
    }

    /* ����UART�����߳� */
    if (xTaskCreate(UartCtrl_ProcessTask, "UartProcessTask", EXTERNAL_CTRL_TASK_STACK_SIZE, null, EXTERNAL_CTRL_TASK_PRIORITY, null) != pdPASS) {
        Log.e("create UART data receive task error\r\n");
    }

    isPcCtrlEnable = isWebCtrlEnable = false;

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

	 /* �ȴ��������� */
    while(!isEthConnected) {
        DELAY(200);
    }

    taskBuf	= MALLOC(sizeof(Network_DataBuf_S));
    taskBuf->data = MALLOC(EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE);
    taskBuf->maxLen = EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE;
    taskBuf->len = 0;

    /* ��ʼ���������� */
    taskPara = MALLOC(sizeof(Network_TcpTaskPara_S));
    taskPara->port = hostPort;
    taskPara->type = tServer;
    taskPara->tcpListener = PcCtrl_TcpStaListener;
    pcNetTaskHandler = Network.creatTask(NETWORK_ENET_TYPE,tTcp,taskPara);

    Log.d("PC control process task start!!\r\n");

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

	 /* �ȴ��������� */
    while(!isEthConnected) {
        DELAY(200);
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
    webNetTaskHandler = Network.creatTask(NETWORK_ENET_TYPE,tHttp,taskPara);

    Log.d("WEB control process task start!!\r\n");

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

    Log.d("Uart control process task start!!\r\n");

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



