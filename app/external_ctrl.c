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

/* API */
#include "app.h"
#include "network.h"
#include "ram.h"

#include "external_ctrl.h"
#include "conference.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/**** ������ز������� ****/
#define HOST_LOCAL_IP								{172,16,14,117}
#define HOST_GATEWAY								{172,16,14,254}
#define HOST_NETMASK								{255,255,255,0}
#define HOST_PORT									(50000)

/* ����ӿ� */
#define NETWORK_ENET_TYPE							eth0

/* �������� */
#define NETWORK_TYPE								NETWORK_TYPE_TCPIP
 
/* �����ջ��С�����ȼ� */
#define EXTERNAL_CTRL_TASK_STACK_SIZE				(1024)
#define EXTERNAL_CTRL_TASK_PRIORITY					(12)

/* ������������BUF��С */
#define EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE			(260)

/* ȫ���ֻ���Э�����ݰ���С���� */
#define CTRL_CMD_MIN_LEN							(13)

/* ���ְ��� */
#define DATA_PACK_MAX_NUM							(EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE / CTRL_CMD_MIN_LEN)

/* �ⲿ���ơ�������Э��ṹ */
//#pragma pack(1)	/* ���ֽڶ��� */
typedef struct{
	uint8_t header[2];
	uint8_t len;
	ConfProtocol_S prot;
	uint8_t exDataHead;
}ExCtrlData_S;
//#pragma pack()

/* Э�����ݼ�� */
typedef struct {
	uint8_t packNum;
	ExCtrlData_S *ctrlData[DATA_PACK_MAX_NUM];
}DataPack_S;

//static PcCtrl_S pcCtrl;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
 /* API���� */
static void ExternalCtrl_Launch(void);
static void ExternalCtrl_ctrlDataTransmit(EXE_DEST dest, ConfProtocol_S *prot);
static void ExternalCtrl_ctrlDataTransmitWithExData(EXE_DEST dest, ConfProtocol_S *prot, uint16_t exLen, uint8_t *exData);


/* �ڲ����� */
static void ExternalCtrl_LaunchTask(void *pvParameters);
static void ExternalCtrl_EthStaListener(bool sta);
static DataPack_S *ExternalCtrl_FetchDataFromBuf(Network_DataBuf_S *taskBuf);
static void ExternalCtrl_NotifyConference(NotifySrcType_EN nSrc, ConfProtocol_S *prot);
static void ExternalCtrl_NotifyConferenceWithExData(NotifySrcType_EN nSrc, ConfProtocol_S *prot, uint16_t exLen, uint8_t *exData);
static void ExternalCtrl_ReplyHeartbeat(EXE_DEST dest);
static void ExternalCtrl_ReplyQuery(EXE_DEST dest, uint8_t para);

static void PcCtrl_ProcessTask(void *pvParameters);
static void WebCtrl_ProcessTask(void *pvParameters);
/*******************************************************************************
 * Variables
 ******************************************************************************/

 
/* ����������� */
static NETWORK_IP hostIp = HOST_LOCAL_IP;
static NETWORK_GW hostGw = HOST_GATEWAY;
static NETWORK_MASK hostMask = HOST_NETMASK;
static NETWORK_PORT	hostPort = HOST_PORT;

/* �������Ӽ���־ */
static bool isEthConnected; 

/* �������Ӽ���־ */
static bool isPcCtrlEnable;
static bool isWebCtrlEnable;
static bool isUartCtrlEnable;

/* ����������ƾ�� */
static Network_TaskHandler_S *pcNetTaskHandler;
static Network_TaskHandler_S *webNetTaskHandler;

/* ����ָ�������м�ȡָ�����ݰ� */
static DataPack_S fetchDataPack;
/* ����ָ����ź��� */
static SemaphoreHandle_t ctrlProcessSemaphore;

/* ��ҳ�ļ� */
extern const HTTPSRV_FS_DIR_ENTRY httpsrv_fs_data[];

ExternalCtrl_S ExternalCtrl = {
	.launch = ExternalCtrl_Launch,
	.transmit = ExternalCtrl_ctrlDataTransmit,
	.transWithExData = ExternalCtrl_ctrlDataTransmitWithExData,
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
static void ExternalCtrl_Launch(void){
	if (xTaskCreate(ExternalCtrl_LaunchTask, "ExternalCtrlLaunchTask", EXTERNAL_CTRL_TASK_STACK_SIZE, null, EXTERNAL_CTRL_TASK_PRIORITY, null) != pdPASS)
    {
        debug("create launch task error\r\n");
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
static void ExternalCtrl_LaunchTask(void *pvParameters){
	Network_EthPara_S *ethPara;

	isEthConnected = false;

	ethPara = MALLOC(sizeof(Network_EthPara_S));

	/* ���ڳ�ʼ�� */
	ethPara->index = NETWORK_ENET_TYPE;
	ethPara->type = NETWORK_TYPE;
	NETWORK_SET_ADDR(ethPara->ip,hostIp.addr0,hostIp.addr1,hostIp.addr2,hostIp.addr3);
	NETWORK_SET_ADDR(ethPara->gateway,hostGw.addr0,hostGw.addr1,hostGw.addr2,hostGw.addr3);
	NETWORK_SET_ADDR(ethPara->netmask,hostMask.addr0,hostMask.addr1,hostMask.addr2,hostMask.addr3);
	ethPara->ethStaListener = ExternalCtrl_EthStaListener;
	
	Network.ethConfig(ethPara);
	
	/* ��ʼ���ź��� */
	ctrlProcessSemaphore = xSemaphoreCreateMutex();
	
	/* �ȴ��������� */
	while(!isEthConnected){
		DELAY(200);
	}
	
	/* ����PC�����߳� */
	if (xTaskCreate(PcCtrl_ProcessTask, "PCProcessTask", EXTERNAL_CTRL_TASK_STACK_SIZE, null, EXTERNAL_CTRL_TASK_PRIORITY, null) != pdPASS)
    {
        debug("create PC data receive task error\r\n");
    }
	
	/* ����WEB�����߳� */
	if (xTaskCreate(WebCtrl_ProcessTask, "WEBProcessTask", EXTERNAL_CTRL_TASK_STACK_SIZE, null, EXTERNAL_CTRL_TASK_PRIORITY, null) != pdPASS)
    {
        debug("create WEB data receive task error\r\n");
    }
	
	isPcCtrlEnable = isWebCtrlEnable = isUartCtrlEnable = false;
	
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
static void ExternalCtrl_EthStaListener(bool sta){
	isEthConnected = sta;
	debug("External control (net port) %s \r\n",sta ? "connected" : "disconnected");
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
static DataPack_S *ExternalCtrl_FetchDataFromBuf(Network_DataBuf_S *buf){
	DataPack_S *dataPack = &fetchDataPack;
	uint16_t i;
	uint8_t *data;

	dataPack->packNum = 0;
	memset(dataPack->ctrlData,0,DATA_PACK_MAX_NUM);
	
	ERR_CHECK(buf != null,return dataPack);
	ERR_CHECK(buf->data != null,return dataPack);
	ERR_CHECK(buf->len >= CTRL_CMD_MIN_LEN,return dataPack);
		
	data = buf->data;
	
	/* ������ͷ�����ݰ�������֤��β*/
	/*	(taskBuf-> - CTRL_CMD_MIN_LEN)�����������Ϊһ�������Ȳ�Ӧ
		��С��CTRL_CMD_MIN_LEN����˼�����ͷ�����ݰ������ֽھͲ���Ҫ��*/
	for(i = 0;i <= buf->len - CTRL_CMD_MIN_LEN;i++){
		if(data[i] == 0xAA && data[i+1] == 0xEE && \
		   data[i + 3 + data[i+2]] == 0xEE && \
		   data[i + 4 + data[i+2]] == 0xFC){

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
* @Name  		ExternalCtrl_DataProcess
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static void ExternalCtrl_DataProcess(NotifySrcType_EN nSrc, ExCtrlData_S *ctrlData){
	ConfProtocol_S prot;
	uint8_t exLen = 0, *exData = null;
	
	ERR_CHECK(ctrlData != null,return);	

	memcpy(&prot,&ctrlData->prot,sizeof(ConfProtocol_S));
	prot.id = lwip_htons(prot.id);

	/* ����ҪƵ����������ٻظ������Ĳ������ڽ����߳�ֱ�Ӵ��������ķ��͸����������̴߳��� */
	/* �ظ����� */
	if(prot.ph == HEARTBEAT && prot.pl == HEARTBEAT && prot.type == PC_MSG ){
		ExternalCtrl_ReplyHeartbeat(nSrc);
		return;
	}
	/* �ظ�״̬��ѯ */
	else if(prot.ph >= QUERY_PRIOR_SIGN && prot.ph <= QUERY_PRIOR_SCAN && prot.type == PC_MSG ){
		ExternalCtrl_ReplyQuery(nSrc,prot.ph);
		return;	
	}

	/* ֪ͨ�������� */
	if(ctrlData->len > sizeof(ConfProtocol_S)){
		exLen = ctrlData->len - sizeof(ConfProtocol_S);
		exData = MALLOC(exLen);
		memcpy(exData,&ctrlData->exDataHead,exLen);
		ExternalCtrl_NotifyConferenceWithExData(nSrc,&prot,exLen,exData);
	}
	else
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
static void ExternalCtrl_NotifyConferenceWithExData(NotifySrcType_EN nSrc, ConfProtocol_S *prot, uint16_t exLen, uint8_t *exData){
	Notify_S *notify;
	
	ERR_CHECK(prot != null,return);
	ERR_CHECK(!(exLen != 0 && exData == null),return);
		
	notify = MALLOC(sizeof(Notify_S) + exLen);
	
	notify->nSrc = nSrc;
//	notify->kWord = kWord;
	memcpy(&notify->prot.conference,prot,sizeof(ConfProtocol_S));
	notify->exLen = exLen;
	
	if(exLen != 0){
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
static void ExternalCtrl_NotifyConference(NotifySrcType_EN nSrc, ConfProtocol_S *prot){
	ExternalCtrl_NotifyConferenceWithExData(nSrc, prot, null, null);
}



/**
* @Name  		ExternalCtrl_ctrlDataTransmitWithExData
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static void ExternalCtrl_ctrlDataTransmitWithExData(EXE_DEST dest, ConfProtocol_S *prot, uint16_t exLen, uint8_t *exData){
	Network_DataBuf_S *taskBuf;
	ExCtrlData_S *ctrlData;
	uint8_t dataLen;
	
	ERR_CHECK(prot != null,return);
	ERR_CHECK(!(exLen != 0 && exData == null),return);
	ERR_CHECK(isEthConnected && (isPcCtrlEnable || isWebCtrlEnable),return);
//	ERR_CHECK((isPcCtrlEnable || isWebCtrlEnable || isUartCtrlEnable),return);
		
	dataLen = exLen + CTRL_CMD_MIN_LEN;
	ctrlData = MALLOC(dataLen);
	taskBuf = MALLOC(sizeof(Network_DataBuf_S));
	memset(ctrlData,0,dataLen);
	
	ctrlData->header[0] = 0xAA;
	ctrlData->header[1] = 0xEE;
	ctrlData->len = dataLen - 5;
	memcpy(&ctrlData->prot,prot,sizeof(ConfProtocol_S));
	ctrlData->prot.id = lwip_htons(ctrlData->prot.id);
	ctrlData->prot.sec = lwip_htons(ctrlData->prot.sec);
	if(exLen != 0){
		memcpy(&ctrlData->exDataHead,exData,exLen);
	}
	((uint8_t *)ctrlData)[dataLen - 2] = 0xEE; 
	((uint8_t *)ctrlData)[dataLen - 1] = 0xFC; 
	
	taskBuf->len = dataLen;
	taskBuf->data = (uint8_t *)ctrlData;
	
	if((dest & kType_NotiSrc_PC) && isPcCtrlEnable)
		Network.transmit(pcNetTaskHandler,taskBuf);
	
	if((dest & kType_NotiSrc_Web) && isWebCtrlEnable){
		taskBuf->webType = tWebsocket;
		taskBuf->wsType = WS_DATA_BINARY;
		Network.transmit(webNetTaskHandler,taskBuf);
	}
	
	FREE(ctrlData);
	FREE(taskBuf);
}

/**
* @Name  		ExternalCtrl_ctrlDataTransmitWithExData
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static void ExternalCtrl_ctrlDataTransmit(EXE_DEST dest, ConfProtocol_S *prot){
	ExternalCtrl_ctrlDataTransmitWithExData(dest, prot, null, null);
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
static void ExternalCtrl_ReplyHeartbeat(EXE_DEST dest){
	UnitOnlineNum onlineNum;
	ConfProtocol_S prot;
	
	Conference.getOnlineNum(&onlineNum);
	
	Protocol.conference(&prot,WHOLE_BROADCAST_ID,WIRED_UNIT_NUM,0,onlineNum.interpreter,onlineNum.wiredChm,onlineNum.wiredRps);
	ExternalCtrl_ctrlDataTransmit(dest,&prot);
	Protocol.conference(&prot,WHOLE_BROADCAST_ID,WIFI_UNIT_NUM,0,0,onlineNum.wifiChm,onlineNum.wifiRps);
	ExternalCtrl_ctrlDataTransmit(dest,&prot);
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
static void ExternalCtrl_ReplyQuery(EXE_DEST dest, uint8_t para){
	ConfProtocol_S prot;
	uint8_t sta;

	sta = (uint8_t)Conference.getSysMode();
	Protocol.conference(&prot, WHOLE_BROADCAST_ID, PC_MSG, para, 0x01, sta,null);
	ExternalCtrl_ctrlDataTransmit(dest,&prot);
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
static void PcCtrl_TcpStaListener(bool sta){
	isPcCtrlEnable = sta;
	debug("PC control is %s \r\n",sta ? "online" : "offline");
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
static void PcCtrl_ProcessTask(void *pvParameters){
	Network_TcpTaskPara_S *taskPara;
	Network_DataBuf_S *taskBuf;
	DataPack_S *dataPack;
	
	uint32_t index;
	
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
	
	debug("PC control process task start!!\r\n");
	
	while(1){
		Network.receive(pcNetTaskHandler,taskBuf,MAX_NUM);
		
		xSemaphoreTake(ctrlProcessSemaphore, MAX_NUM);
		dataPack = ExternalCtrl_FetchDataFromBuf(taskBuf);
		if(dataPack->packNum > 0){
			for(index = 0;index < dataPack->packNum;index++){
				ExternalCtrl_DataProcess(kType_NotiSrc_PC,dataPack->ctrlData[index]);
			}
		}
		xSemaphoreGive(ctrlProcessSemaphore);
		
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
static void WebCtrl_WsStaListener(bool sta){
	isWebCtrlEnable = sta;
	debug("Websocket is %s \r\n",sta ? "online" : "offline");
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
static void WebCtrl_ProcessTask(void *pvParameters){
	Network_HttpTaskPara_S *taskPara;
	Network_DataBuf_S *taskBuf;
	DataPack_S *dataPack;
	uint32_t index;
	
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
	
	debug("WEB control process task start!!\r\n");
	
	while(1){
		Network.receive(webNetTaskHandler,taskBuf,MAX_NUM);
		
		if(taskBuf->webType == tWebsocket && taskBuf->wsType == WS_DATA_BINARY){
			xSemaphoreTake(ctrlProcessSemaphore, MAX_NUM);
			dataPack = ExternalCtrl_FetchDataFromBuf(taskBuf);
			
			if(dataPack->packNum > 0){
				for(index = 0;index < dataPack->packNum;index++){
					ExternalCtrl_DataProcess(kType_NotiSrc_Web,dataPack->ctrlData[index]);
				}
			}
			xSemaphoreGive(ctrlProcessSemaphore);
		}
		
		taskBuf->len = 0;
		memset(taskBuf->data,0,EXTERNAL_CTRL_DATA_RECEIVE_BUF_SIZE);
	}
}
/*****************************************  Uart  ****************************************************************/
