/**
 *
 *	@name				network.c
 *	@author 			KT
 *	@date 				2019/06/22
 *	@brief				
 *  @include			network.h
 *
 *  @API				
 *
 *  @description   		
 *
 **/
/*******************************************************************************
 * includes
 ******************************************************************************/
/* clib */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>

/* OS */
#include "FreeRTOS.h"
#include "timers.h"

/* lwip */
#include "lwip/opt.h"
#include "lwip/netif.h"
#include "lwip/sys.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/tcpip.h"
#include "lwip/ip.h"
#include "lwip/netifapi.h"
#include "lwip/sockets.h"
#include "netif/etharp.h"
#include "ethernetif.h"
#include "board.h"
#include "httpsrv.h"
#include "mdns.h"

#include "ram.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "network.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*网卡配置复位相关配置,在线程中配置，使用系统延时*/
#define ETH_PHY_RESET_OS_DELAY 				1

#ifdef ETH_PHY_RESET_OS_DELAY
#define ETH_PHY_RESET_DELAY()				DELAY(10);
#else
#define ETH_PHY_RESET_DELAY()				Network_ResetDelay()
#endif

/*网卡复位*/
#define ETH_RESET(index)					{do{GPIO_WritePinOutput(GPIO1, EthResetIO[index], 0);ETH_PHY_RESET_DELAY();GPIO_WritePinOutput(GPIO1, EthResetIO[index], 1);}while(0);}

/* MAC地址 */
#define ETH0_MAC_DEF   						{0x02, 0x12, 0x13, 0x10, 0x15, 0x11}
#define ETH1_MAC_DEF   						{0x02, 0x12, 0x13, 0x10, 0x15, 0x12}

/* 网卡物理接口 */
#define ETH0_PHY_ADDRESS 					BOARD_ENET0_PHY_ADDRESS
#define ETH1_PHY_ADDRESS 					BOARD_ENET1_PHY_ADDRESS

/* 网络初始化函数 */
#define ETH0_INIT_FUNC						ethernetif0_init
#define ETH1_INIT_FUNC						ethernetif1_init

/* 网络接收函数 */
#define ETH0_INPUT_DEF_FUNC					tcpip_input
#define ETH1_INPUT_DEF_FUNC					tcpip_input

/* System clock name. */
#define SYS_CLOCK_NAME 						kCLOCK_CoreSysClk

/* 网卡配置线程堆栈大小、优先级 */
#define ETH_CONFIG_TASK_STACK				1024
#define ETH_CONFIG_TASK_PRIORITY			10

/* 网络任务线程堆栈大小、优先级 */
#define NETWORK_TASK_STACK					1024
#define NETWORK_TASK_PRIORITY				14

/* HTTP任务优先级（初始化中会规定HTTP优先级
不能大于TCP任务优先级，因此这里直接赋值等于TCP优先级） */
#define HTTP_TASK_PRIORITY					TCPIP_THREAD_PRIO

/* 网络任务收发通讯队列长度 */
#define NETWORK_TASK_QUEUE_LEN				30
#define NETWORK_TASK_QUEUE_SIZE				(sizeof(void*))

#define NETWORK_MAC_SIZE					6
#define NETWORK_IP_SIZE						4


#define IS_ETH_CONFIGURED(cfg)				(bool)(	cfg->netif != null 	&& 	\
											cfg->config != null && 	\
											cfg->ip != null 	&& 	\
											cfg->netmask != null &&	\
											cfg->gateway != null	)
											
#define GET_ETH_CONNECT_TIME_MS				(1000)
#define GET_AUTO_NEGOTIATION_TIME_MS		(100)

/* 网口配置函数 */
typedef struct {
	err_t (*funcInit)(struct netif *netif);
	err_t (*funcInput)(struct pbuf *p, struct netif *inp);
} EthConfigFunc_S;

/* 重定义网络接口（network interfaces）结构 */
typedef struct netif Netif_S;

/* 网络任务控制句柄结构 */
typedef struct {
	
	Network_EthIndex_EN index;

	Network_TaskType_EN type;

	Network_TaskPara *para;
	
	/* 消息队列 */
	QueueHandle_t recBufQueue;
	QueueHandle_t sendBufQueue;
	
	/* 任务线程控制句柄 */
	TaskHandle_t task;
	
	/* 网络连接句柄 */
	struct netconn *srcConn,*destConn;
	
	/* Http服务句柄 */
	uint32_t httpServHandler;
	/* Websocket连接句柄 */
	uint32_t wsHandler;
	
} TaskHandler_S;

/* 网口配置控制句柄 */
typedef struct {

	/* 网口是否正在被配置 */
	bool isConfiguring;
	
	/* 网口连接状态 */
	bool connectSta;
	
	/* 网络类型 */
	uint8_t type;
	
	/* 网口索引 */
	Network_EthIndex_EN index;
	
	/* LWIP网络接口的通用数据结构指针 */
	Netif_S *netif;
	
	/* 网口配置数据结构指针 */
	ethernetif_config_t *config;
	
	/* IP地址、掩码、网关 */
	ip4_addr_t *ip, *netmask, *gateway;
	
	/* 配置函数指针 */
	EthConfigFunc_S *ethFunc;
	
	/* 网口状态监听定时器 */
	TimerHandle_t ethStaListenTimer;
	
	/* 网口连接状态监听函数 */
	Network_EthStaListener ethStaListener;
	
	/* 网口初始化参数 */
	Network_EthPara_S *para;
	
	/* 网络任务控制句柄指针数组 */
	TaskHandler_S *taskHandler[NETWORK_TASK_MAX_NUM];
} EthHandler_S;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
#ifndef ETH_PHY_RESET_OS_DELAY
static void Network_ResetDelay(void);
#endif

/* API */
static void Network_Init(void);
static void Network_EthConfig(Network_EthPara_S *para);
static Network_TaskHandler_S *Network_CreatNetworkTask(Network_EthIndex_EN index,Network_TaskType_EN type,Network_TaskPara para);
static void Network_TaskReceive(Network_TaskHandler_S *handler,Network_DataBuf_S *buf,uint32_t waitTime);
static void Network_TaskTransmit(Network_TaskHandler_S *handler,Network_DataBuf_S *buf);

/* 内部调用 */
static void Network_EthConfigTask(void *para);
static bool Network_GetEthConnectSta(EthHandler_S *ethHandler);
static void Network_EthStaListenTimerCallback(TimerHandle_t xTimer);
static void Network_WaitEthConnect(EthHandler_S *ethHandler);
static bool Network_CheckAutoNegotiation(EthHandler_S *ethHandler);
static err_t Network_EthernetInput(struct pbuf *p, struct netif *netif);
static void Network_EthernetOutput(struct netif *netif,struct pbuf *p);
static void Network_TaskEthernet(void *pvParameters);
static void Network_TaskTcp(void *pvParameters);

static uint32_t Network_CgiGetContent(char * src, const char * var_name, char **dst, uint32_t length);
static int Network_WebCgiProcess(HTTPSRV_CGI_REQ_STRUCT *param);

static uint32_t Network_WsConnected(void *param, WS_USER_CONTEXT_STRUCT context);
static uint32_t Network_WsDisconnect(void *param, WS_USER_CONTEXT_STRUCT context);
static uint32_t Network_WsError(void *param, WS_USER_CONTEXT_STRUCT context);
static uint32_t Network_WsReceive(void *param, WS_USER_CONTEXT_STRUCT context);
static void Network_WsSend(Network_DataBuf_S * dataBuf);

/*******************************************************************************
 * Variables
 ******************************************************************************/
Network_S Network = {
	.init = Network_Init,
	.ethConfig = Network_EthConfig,
	.creatTask = Network_CreatNetworkTask,
	.receive = Network_TaskReceive,
	.transmit = Network_TaskTransmit,
};

const static uint8_t EthResetIO[NETWORK_ETH_NUM] = {2,3};

static ethernetif_config_t ethernetif_config[NETWORK_ETH_NUM] = {
	/* eth0 netif config */
	{ ETH0_PHY_ADDRESS,SYS_CLOCK_NAME, ETH0_MAC_DEF, 0},
	/* eth1 netif config */
	{ ETH1_PHY_ADDRESS,SYS_CLOCK_NAME, ETH1_MAC_DEF, 0}
};

static EthHandler_S EthHandler[NETWORK_ETH_NUM] = {0};

static EthConfigFunc_S EthConfigFunc[NETWORK_ETH_NUM] = {
	/*eth0 init func,input func*///noArp_input
	{ETH0_INIT_FUNC,ETH0_INPUT_DEF_FUNC},
	/*eth1 init func,input func*/
	{ETH1_INIT_FUNC,ETH1_INPUT_DEF_FUNC}
};

/* Http web 功能相关参数  */

/* 本地全局变量用于记录web功能网络任务句柄
(因为web的API调用Callback处理函数不能将该参数传入) */
static TaskHandler_S *webTaskHandler = null;

static const HTTPSRV_CGI_LINK_STRUCT cgiTable[] = {
    {"get", Network_WebCgiProcess},
    {"post", Network_WebCgiProcess},
    {0, 0} // DO NOT REMOVE - last item - end of table
};


/* websocket相关参数 */
#if HTTPSRV_CFG_WEBSOCKET_ENABLED
static const WS_PLUGIN_STRUCT wsTable[] = {{"/echo", Network_WsConnected, Network_WsReceive, Network_WsError, Network_WsDisconnect, NULL},
                             		{0, 0, 0, 0, 0, 0}}; //不能删
#endif /* HTTPSRV_CFG_WEBSOCKET_ENABLED */


/*******************************************************************************
 * Code
 ******************************************************************************/
/**
* @Name  		
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static void Network_Init(void)
{
	gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

	IOMUXC_EnableMode(IOMUXC_GPR, kIOMUXC_GPR_ENET1TxClkOutputDir, true);
	IOMUXC_EnableMode(IOMUXC_GPR, kIOMUXC_GPR_ENET2TxClkOutputDir, true);
	
	//GPIO1_IO02 == M11,GPIO1_IO03 == G11
	GPIO_PinInit(GPIO1, 2, &gpio_config);
	GPIO_PinInit(GPIO1, 3, &gpio_config);	
    GPIO_PinInit(GPIO1, 10, &gpio_config);
    /* pull up the ENET_INT before RESET. */
    GPIO_WritePinOutput(GPIO1, 10, 1);
	
	tcpip_init(NULL, NULL);
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
static void Network_EthConfig(Network_EthPara_S *para){
	if(para == null)
		return;
		
	if (xTaskCreate(Network_EthConfigTask, "Network_EthConfigTask", ETH_CONFIG_TASK_STACK, para, ETH_CONFIG_TASK_PRIORITY, NULL) != pdPASS)
    {
        debug("create Network_EthConfigTask error\r\n");
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
static void Network_EthConfigTask(void *pvParameters){

//	uint32_t temp;
//	err_t err;
	Network_EthPara_S *ethPara = (Network_EthPara_S *)pvParameters;
	EthHandler_S *ethHandler = &EthHandler[ethPara->index];
	
	if(ethPara == null || ethPara->index < eth0 || ethPara->index > eth1 || ethHandler->isConfiguring)
		goto eth_config_end;
		
	taskENTER_CRITICAL();	
	
	debug("Start to configure Eth%d ...\r\n",ethPara->index);
	ethHandler->isConfiguring = true;
	ethHandler->index = ethPara->index;
	ethHandler->connectSta = false;
	
	ETH_RESET(ethHandler->index);	
	ethHandler->config = &ethernetif_config[ethHandler->index];
	ethHandler->ethFunc = &EthConfigFunc[ethHandler->index];
	
	ethHandler->netif = MALLOC(sizeof(Netif_S));
	ethHandler->ip = MALLOC(sizeof(ip4_addr_t));
	ethHandler->gateway = MALLOC(sizeof(ip4_addr_t));
	ethHandler->netmask = MALLOC(sizeof(ip4_addr_t));

	if(ethPara->mac != null){
		ethHandler->config->macAddress[0] = ethPara->mac->mac0;
		ethHandler->config->macAddress[1] = ethPara->mac->mac1;
		ethHandler->config->macAddress[2] = ethPara->mac->mac2;
		ethHandler->config->macAddress[3] = ethPara->mac->mac3;
		ethHandler->config->macAddress[4] = ethPara->mac->mac4;
		ethHandler->config->macAddress[5] = ethPara->mac->mac5;
	}
	else{
		ethPara->mac = MALLOC(NETWORK_MAC_SIZE);
		ethPara->mac->mac0 = ethHandler->config->macAddress[0];
		ethPara->mac->mac1 = ethHandler->config->macAddress[1];
		ethPara->mac->mac2 = ethHandler->config->macAddress[2];
		ethPara->mac->mac3 = ethHandler->config->macAddress[3];
		ethPara->mac->mac4 = ethHandler->config->macAddress[4];
		ethPara->mac->mac5 = ethHandler->config->macAddress[5];
	}
	
	if(ethPara->type == NETWORK_TYPE_ETHERNET){
		ethHandler->config->flag = NETIF_FLAG_ETHERNET;
		ethHandler->type = NETWORK_TYPE_ETHERNET;
		ethHandler->ethFunc->funcInput = Network_EthernetInput;
	}
	else{
		ethHandler->config->flag = NETIF_FLAG_ETHARP;
		ethHandler->type = NETWORK_TYPE_TCPIP;
	}

	IP4_ADDR(ethHandler->ip, ethPara->ip.addr0, ethPara->ip.addr1, ethPara->ip.addr2, ethPara->ip.addr3);
    IP4_ADDR(ethHandler->netmask, ethPara->netmask.addr0, ethPara->netmask.addr1, ethPara->netmask.addr2, ethPara->netmask.addr3);
    IP4_ADDR(ethHandler->gateway, ethPara->gateway.addr0, ethPara->gateway.addr1, ethPara->gateway.addr2, ethPara->gateway.addr3);

	enet_phy_init(ethHandler->config);
	do{
		/* 检查及等待网口连接(退出临界区等待) */
		debug("Eth%d waiting physical interface connected\r\n",ethHandler->index);
		taskEXIT_CRITICAL();
		Network_WaitEthConnect(ethHandler);
		taskENTER_CRITICAL();
		debug("Eth%d physical interface connected\r\n",ethHandler->index);
		
		if(netifapi_netif_add(ethHandler->netif,ethHandler->ip,ethHandler->netmask,ethHandler->gateway, 
							  ethHandler->config,ethHandler->ethFunc->funcInit,ethHandler->ethFunc->funcInput))
		{
			debug("Eth%d netif init fail\r\n",ethHandler->index);
			DELAY(3000);
			continue;
		}
		
		/* 检查网口自适应(退出临界区等待) */
		taskEXIT_CRITICAL();
		Network_CheckAutoNegotiation(ethHandler);
		taskENTER_CRITICAL();
//		debug("Eth%d check auto negotiation success!!\r\n ",ethHandler->index);
		
		break;
	}while(1);
							 
	ethHandler->connectSta = true;
							 
    netifapi_netif_set_default(ethHandler->netif);
    netifapi_netif_set_up(ethHandler->netif);
	
	debug("\r\n****** Configure Eth%d Finish !! ************\r\n",ethHandler->index);
	debug("****        IP  : %u.%u.%u.%u		****\r\n",ethPara->ip.addr0, ethPara->ip.addr1, ethPara->ip.addr2, ethPara->ip.addr3);
	debug("****   Netmask  : %u.%u.%u.%u		****\r\n",ethPara->netmask.addr0, ethPara->netmask.addr1, ethPara->netmask.addr2, ethPara->netmask.addr3);
	debug("****   Gateway  : %u.%u.%u.%u		****\r\n",ethPara->gateway.addr0, ethPara->gateway.addr1, ethPara->gateway.addr2, ethPara->gateway.addr3);
	debug("****       Mac  : %x.%x.%x.%x.%x.%x	****\r\n",ethHandler->config->macAddress[0],ethHandler->config->macAddress[1],ethHandler->config->macAddress[2], \
														  ethHandler->config->macAddress[3],ethHandler->config->macAddress[4],ethHandler->config->macAddress[5]);
	debug("********************************************\r\n");
	
	if(ethPara->ethStaListener != null){
		ethHandler->ethStaListener = ethPara->ethStaListener;
		ethHandler->ethStaListenTimer = xTimerCreate("ethStaListenTimer",GET_ETH_CONNECT_TIME_MS,pdTRUE,ethHandler,Network_EthStaListenTimerCallback);
		xTimerStart(ethHandler->ethStaListenTimer,0);
		ethPara->ethStaListener(true);
	}
	
	ethHandler->para = ethPara;
	ethHandler->isConfiguring = false;
	
	taskEXIT_CRITICAL();
	
eth_config_end:
	vTaskDelete(null);
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
static void Network_EthStaListenTimerCallback(TimerHandle_t xTimer) {
	EthHandler_S *ethHandler;
	bool sta;
	
	ethHandler = pvTimerGetTimerID(xTimer);
	sta = Network_GetEthConnectSta(ethHandler);
	if(sta != ethHandler->connectSta){
		if(sta == true)
			Network_CheckAutoNegotiation(ethHandler);
		ethHandler->connectSta = sta;
		ethHandler->ethStaListener(sta);
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
static bool Network_GetEthConnectSta(EthHandler_S *ethHandler){
	return enet_link(ethHandler->config->phyAddress);
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
static void Network_WaitEthConnect(EthHandler_S *ethHandler){
	while(!Network_GetEthConnectSta(ethHandler)){
		DELAY(GET_ETH_CONNECT_TIME_MS);
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
static bool Network_CheckAutoNegotiation(EthHandler_S *ethHandler){
	while(!enet_phy_check_auto_negotiation(ethHandler->config->phyAddress)){
		DELAY(GET_AUTO_NEGOTIATION_TIME_MS);
	}
	return true;
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
static Network_TaskHandler_S *Network_CreatNetworkTask(Network_EthIndex_EN index,Network_TaskType_EN type,Network_TaskPara para){
	EthHandler_S *ethHandler;
	TaskHandler_S *taskHandler;

	if((index != eth0 && index != eth1) || type < tEthernet || type > tHttp || para == null){
		goto creat_net_task_err;
	}
	
	ethHandler = &EthHandler[index];
	
	taskHandler = MALLOC(sizeof(TaskHandler_S));
	taskHandler->index = index;
	taskHandler->type = type;
	taskHandler->para = para;
	
	switch(type){
		/* Ethernet任务 */
		case tEthernet:{
			Network_EthernetTaskPara_S *ethernetTaskPara = para;
			if(ethHandler->type != NETWORK_TYPE_ETHERNET || ethHandler->taskHandler[type] != null){
				goto creat_net_task_err;
			}
			
			taskHandler->recBufQueue = xQueueCreate(NETWORK_TASK_QUEUE_LEN,NETWORK_TASK_QUEUE_SIZE);
			taskHandler->sendBufQueue = xQueueCreate(NETWORK_TASK_QUEUE_LEN,NETWORK_TASK_QUEUE_SIZE);
			
			if (xTaskCreate(Network_TaskEthernet, "Network_TaskEthernet", NETWORK_TASK_STACK, taskHandler, NETWORK_TASK_PRIORITY, taskHandler->task) != pdPASS)
			{
				debug("create Network_TaskEthernet error\r\n");
				goto creat_net_task_err;
			}
			
			ethHandler->taskHandler[type] = taskHandler;
			return (Network_TaskHandler_S *)taskHandler;
		}
		
		/* Udp任务 */
		case tUdp:{}break;
		
		/* Tpc任务 */
		case tTcp:{
			Network_TcpTaskPara_S *tcpTaskPara = para;
			if(ethHandler->type != NETWORK_TYPE_TCPIP || ethHandler->taskHandler[type] != null){
				goto creat_net_task_err;
			}
			
			taskHandler->recBufQueue = xQueueCreate(NETWORK_TASK_QUEUE_LEN,NETWORK_TASK_QUEUE_SIZE);
//			taskHandler->sendBufQueue = xQueueCreate(NETWORK_TASK_QUEUE_LEN,NETWORK_TASK_QUEUE_SIZE);
			
			if (xTaskCreate(Network_TaskTcp, "Network_TaskTcp", NETWORK_TASK_STACK, taskHandler, NETWORK_TASK_PRIORITY, taskHandler->task) != pdPASS)
			{
				debug("create Network_TaskTcp error\r\n");
				goto creat_net_task_err;
			}
			
			ethHandler->taskHandler[type] = taskHandler;
			return (Network_TaskHandler_S *)taskHandler;
		}
		
		/* Http任务 */
		case tHttp:{
			Network_HttpTaskPara_S *httpTaskPara = para;
			HTTPSRV_PARAM_STRUCT params;

			ERR_CHECK((httpTaskPara->cgi.enable || httpTaskPara->websocket.enable), break);
		
			/* Init Fs*/
			HTTPSRV_FS_init(httpTaskPara->httpFsDir);

			/* Init HTTPSRV parameters.*/
		    memset(&params, 0, sizeof(HTTPSRV_PARAM_STRUCT));
		    params.root_dir = httpTaskPara->rootDir;
		    params.index_page = httpTaskPara->indexPage;
			if(httpTaskPara->cgi.enable)		
			    params.cgi_lnk_tbl = cgiTable;
			
#if HTTPSRV_CFG_WEBSOCKET_ENABLED
			if(httpTaskPara->websocket.enable)
			    params.ws_tbl = wsTable;
#endif /* HTTPSRV_CFG_WEBSOCKET_ENABLED */

			/* 使用网口IP */
			((struct sockaddr_in *)(&params.address))->sin_addr = *(struct in_addr *)ethHandler->ip;
			/* 配置端口号 */
			((struct sockaddr_in *)(&params.address))->sin_port = lwip_htonl(httpTaskPara->port == null ? 80 : httpTaskPara->port);
			
			/* http任务优先级*/
			params.task_prio = HTTP_TASK_PRIORITY;

			/* 初始化数据队列 */
			taskHandler->recBufQueue = xQueueCreate(NETWORK_TASK_QUEUE_LEN,NETWORK_TASK_QUEUE_SIZE);
			taskHandler->sendBufQueue = xQueueCreate(NETWORK_TASK_QUEUE_LEN,NETWORK_TASK_QUEUE_SIZE);

			/* Init HTTP Server.*/
			webTaskHandler = taskHandler;
			taskHandler->httpServHandler = HTTPSRV_init(&params);
			
			return (Network_TaskHandler_S *)taskHandler;
		}
	}
	
	
creat_net_task_err:
	debug("creat network task fail !!\r\n");
	if(para != null) FREE(para);
	if(taskHandler != null)	FREE(taskHandler);
	return null;
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
static void Network_TaskReceive(Network_TaskHandler_S *handler,Network_DataBuf_S *buf,uint32_t waitTime){
	TaskHandler_S *taskHandler = (TaskHandler_S *)handler;
	
	if(handler == null || buf == null || buf->data == null || taskHandler->recBufQueue == null)
		return;
	
	switch(taskHandler->type){
		case tEthernet:
		{
			Network_EthernetFrame_S *frame;
			struct pbuf *p;
			
			xQueueReceive(taskHandler->recBufQueue, &p, waitTime);
			
			taskENTER_CRITICAL();
			frame = p->payload;
			memcpy(&buf->destMac,&frame->destMac,NETWORK_MAC_SIZE);
			memcpy(&buf->srcMac,&frame->srcMac,NETWORK_MAC_SIZE);
			memcpy(&buf->destIp,&frame->destIp,NETWORK_IP_SIZE);
			memcpy(&buf->srcIp,&frame->srcIp,NETWORK_IP_SIZE);
			buf->destPort = lwip_htons(frame->destPort);
			buf->srcPort = lwip_htons(frame->srcPort);
			
			/* 计算包长度，如果接收包长度大于接收内存分配的最大长度，按最大长度复制
			   (这里基本用不到，自组的包不会超过指定长度，但以防万一，内存溢出就GG) */
			buf->len = (lwip_htons(frame->length) - 8);
			buf->len = buf->len > buf->maxLen ? buf->maxLen : buf->len;
			memcpy(buf->data,&frame->dataHead,buf->len);
			taskEXIT_CRITICAL();
			
			pbuf_free(p);
		}
		break;
		
		case tUdp:
		break;
		
		case tTcp:
		case tHttp:{
			Network_DataBuf_S *dataBuf;
		
			xQueueReceive(taskHandler->recBufQueue, &dataBuf, waitTime);
			
			taskENTER_CRITICAL();
			buf->len = dataBuf->len > buf->maxLen ? buf->maxLen : dataBuf->len;
			memcpy(buf->data,dataBuf->data,buf->len);

			if(taskHandler->type == tHttp){
				buf->webType = dataBuf->webType;
				buf->wsType = dataBuf->wsType;
			}
			taskEXIT_CRITICAL();
			
			FREE(dataBuf->data);
			FREE(dataBuf);
		}
		break;
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
static void Network_TaskTransmit(Network_TaskHandler_S *handler,Network_DataBuf_S *buf){
	TaskHandler_S *taskHandler = (TaskHandler_S *)handler;
	if(taskHandler == null || buf == null)
		return;

	switch(taskHandler->type){
		case tEthernet:{
			Network_EthernetFrame_S *frame;
			static uint16_t frameId;
			struct pbuf *p;
			EthHandler_S *ethHandler = &EthHandler[taskHandler->index];
			Network_EthernetTaskPara_S *taskPara;
			
			p = pbuf_alloc(PBUF_RAW, NETWORK_ETH_FRAME_SIZE(buf->len), PBUF_RAM);
			
			if(p == null)
				break;
				
			taskPara = (Network_EthernetTaskPara_S *)taskHandler->para;
			
			taskENTER_CRITICAL();
			frame = p->payload;
			
			memset(frame,0,NETWORK_ETH_FRAME_SIZE(buf->len));
			memcpy(&frame->destMac,&buf->destMac,NETWORK_MAC_SIZE);
			memcpy(&frame->destIp,&buf->destIp,NETWORK_IP_SIZE);
			frame->destPort = lwip_htons(buf->destPort);
			
			memcpy(&frame->srcMac,ethHandler->para->mac,NETWORK_MAC_SIZE);
			memcpy(&frame->srcIp,&ethHandler->para->ip,NETWORK_IP_SIZE);
			frame->srcPort = lwip_htons(taskPara->port);
			
			/* 封装网络包 */
			frame->type = lwip_htons(0x0800);
			frame->ver_len = 0x45;
			frame->totalLen = lwip_htons(buf->len+8+20);
			frame->id = lwip_htons(frameId++);
			frame->ttl = 128;
			frame->prot = 17;
			frame->length = lwip_htons(buf->len+8);
			memcpy(&(frame->dataHead),buf->data,buf->len);
			taskEXIT_CRITICAL();
			
			xQueueSend(taskHandler->sendBufQueue, &p, 0);
		}
		break;
		
		case tHttp:
			ERR_CHECK(webTaskHandler != null,break);
			
			switch(buf->webType){
				case tCgi:{
					Network_DataBuf_S *dataBuf;

					dataBuf = MALLOC(sizeof(Network_DataBuf_S));
					dataBuf->data = MALLOC(buf->len);
					
					dataBuf->len = buf->len;
					memcpy(dataBuf->data,buf->data,buf->len);
					xQueueSend(webTaskHandler->sendBufQueue, &dataBuf, 0);
				}break;
				case tWebsocket:{
					Network_WsSend(buf);
				}break;
				default:break;
			}
		break;
		
		case tUdp:
		break;
		
		case tTcp:{
			if(taskHandler->destConn != null)
				netconn_write(taskHandler->destConn, buf->data, buf->len, NETCONN_COPY);
		}
		break;
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
static err_t Network_EthernetInput(struct pbuf *p, struct netif *netif)
{
	int i = 0;
	Network_EthernetFrame_S *frame;
	EthHandler_S *ethHandler;
	TaskHandler_S *taskHandler;
	Network_EthernetTaskPara_S *taskPara;
	Network_Mac_S mac;
	
	if (p->len <= SIZEOF_ETH_HDR) {
		goto free_and_return;
	}
	
	frame = (Network_EthernetFrame_S *)p->payload;
	
	/* 检索网络句柄eth0是否添加ethernet网络任务 */
	if(EthHandler[eth0].taskHandler[tEthernet] != null){
		ethHandler = &EthHandler[eth0];
		taskHandler = ethHandler->taskHandler[tEthernet];
		taskPara = (Network_EthernetTaskPara_S *)taskHandler->para;
		
		/* 对比IP,MAC,Port */
		if(NETWORK_COMPARISON_MAC((&(frame->destMac)),(ethHandler->para->mac)) && \
			NETWORK_COMPARISON_ADDR(frame->destIp,ethHandler->para->ip) && \
			lwip_htons(frame->destPort) == taskPara->port){
			
			xQueueSendFromISR(taskHandler->recBufQueue, &p, 0);
			return ERR_OK;
		}
		else if(taskPara->multicastNum > 0){
			for(i=0;i<taskPara->multicastNum;i++){
				/* 根据组播IP计算出相应MAC地址 */
				NETWORK_SET_MAC((&mac),0x01,0x00,0x5E,taskPara->multiIp[i].addr1 & 0x7F,taskPara->multiIp[i].addr2,taskPara->multiIp[i].addr3);
				/* 对比MAC,Port */
				if(NETWORK_COMPARISON_MAC((&(frame->destMac)),(&mac)) && \
					lwip_htons(frame->destPort) == taskPara->multiPort[i])	{
					/*内容发送至队列*/
					xQueueSendFromISR(taskHandler->recBufQueue, &p, 0);
					return ERR_OK;
				}
			}
		}
	}
	/* 检索网络句柄eth1是否添加ethernet网络任务 */
//	if(EthHandler[eth1].taskHandler[ethernet] != null){
//		ethHandler = &EthHandler[eth1];
//		taskHandler = ethHandler->taskHandler[ethernet];
//		taskPara = taskHandler->para;
//		
//		/* 对比IP,MAC,Port */
//		if(NETWORK_COMPARISON_MAC((&(frame->destMac)),(ethHandler->para->mac)) && \
//			NETWORK_COMPARISON_ADDR(frame->destIp,ethHandler->para->ip) && \
//			lwip_htons(frame->destPort) == taskPara->port){
//			
//			xQueueSendFromISR(taskHandler->recBufQueue, &p, 0);
//			return ERR_OK;
//		}
//		else if(taskPara->multicastNum > 0){
//			for(i=0;i<taskPara->multicastNum;i++){
//				/* 根据组播IP计算出相应MAC地址 */
//				NETWORK_SET_MAC((&mac),0x01,0x00,0x5E,taskPara->multiIp[i].addr1 & 0x7F,taskPara->multiIp[i].addr2,taskPara->multiIp[i].addr3);
//				/* 对比MAC,Port */
//				if(NETWORK_COMPARISON_MAC((&(frame->destMac)),(&mac)) && \
//					lwip_htons(frame->destPort) == taskPara->multiPort[i])	{
//					
//					xQueueSendFromISR(taskHandler->recBufQueue, &p, 0);
//					return ERR_OK;
//				}
//			}
//		}
//	}
	
free_and_return:
  pbuf_free(p);
  return ERR_OK;
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
static void Network_EthernetOutput(struct netif *netif,struct pbuf *p){
	if(netif == null || p == null)
		return;
		
	netif->linkoutput(netif,p);
	pbuf_free(p);
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
static void Network_TaskEthernet(void *pvParameters){
	uint32_t count;
	struct netconn *conn;
	struct pbuf *p;
	
	TaskHandler_S *taskHandler = (TaskHandler_S *)pvParameters;
	Network_EthernetTaskPara_S *taskPara;
	EthHandler_S *ethHandler;
	ip4_addr_t multiAddr;
	
	if(taskHandler == null || taskHandler->para == null)
		vTaskDelete(null);
	
	debug("Network_TaskEthernet start!!\r\n");
	
	ethHandler = &EthHandler[taskHandler->index];
	taskPara = (Network_EthernetTaskPara_S *)taskHandler->para;
	
	conn = netconn_new(NETCONN_UDP);
	netconn_bind(conn,ethHandler->ip,taskPara->port);
		
	if(taskPara->multicastNum){
		for(count = 0;count < taskPara->multicastNum;count++){
			IP4_ADDR(&multiAddr, taskPara->multiIp[count].addr0, taskPara->multiIp[count].addr1, taskPara->multiIp[count].addr2, taskPara->multiIp[count].addr3);
			netconn_join_leave_group(conn, &multiAddr, ethHandler->ip, NETCONN_JOIN);
		}
	}
		
	while(1){
		if(xQueueReceive(taskHandler->sendBufQueue, &p, MAX_NUM)){
			if(p == null)
				continue;
			
//			debug("get sendBuf pbuf len = %d\r\n",p->len);
//			for(count = 0;count<p->len;count++)
//				debug("%X ",((uint8_t *)(p->payload))[count]);
//			debug("\r\n\r\n");
			Network_EthernetOutput(ethHandler->netif,p);
		}
	}
	
}


/**
* @Name  		Network_TaskTcp
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static void Network_TaskTcp(void *pvParameters){
	err_t err;
	struct netbuf *buf;
	void *data;
	uint16_t len;
	Network_DataBuf_S *dataBuf;
	
	TaskHandler_S *taskHandler = (TaskHandler_S *)pvParameters;
	Network_TcpTaskPara_S *taskPara;
	EthHandler_S *ethHandler;
	
	if(taskHandler == null || taskHandler->para == null)
		vTaskDelete(null);
		
	ethHandler = &EthHandler[taskHandler->index];
	taskPara = (Network_TcpTaskPara_S *)taskHandler->para;
	
	/* TCP 服务端 */
	if(taskPara->type == tServer){

		debug("Network tcp server task start!!\r\n");
		
		
		taskHandler->srcConn = netconn_new(NETCONN_TCP);
		netconn_bind(taskHandler->srcConn, ethHandler->ip, taskPara->port);
		/* Tell connection to go into listening mode. */
		netconn_listen(taskHandler->srcConn);

		while (1) {
			/* Grab new connection. */
			err = netconn_accept(taskHandler->srcConn, &taskHandler->destConn);
			/*printf("accepted new connection %p\n", newconn);*/
			/* Process the new connection. */
			if (err == ERR_OK) {
				taskPara->tcpListener(true);
				while ((err = netconn_recv(taskHandler->destConn, &buf)) == ERR_OK) {
					do {
						dataBuf = MALLOC(sizeof(Network_DataBuf_S));
						
						netbuf_data(buf, &data, &len);
						dataBuf->len = len;
						dataBuf->data = MALLOC(len);
						memcpy(dataBuf->data,data,len);
//						/*内容发送至队列*/
						xQueueSend(taskHandler->recBufQueue, &dataBuf, 0);
					} while (netbuf_next(buf) >= 0);
					netbuf_delete(buf);
				}
				taskPara->tcpListener(false);
				/* Close connection and discard connection identifier. */
				netconn_close(taskHandler->destConn);
				netconn_delete(taskHandler->destConn);
			}
		}
	}
	/* TCP 客户端 */
	else if(taskPara->type == tClient){
		ip4_addr_t destIp;
		NETWORK_PORT destPort;
		
		debug("Network tcp client task start!!\r\n");
		
		IP4_ADDR(&destIp,taskPara->destIp.addr0, taskPara->destIp.addr1, taskPara->destIp.addr2, taskPara->destIp.addr3);
		destPort = taskPara->destPort;
	
		while (1) {
			/* Grab new connection. 8000*/
			taskHandler->destConn = netconn_new(NETCONN_TCP);
			err = netconn_connect(taskHandler->destConn,&destIp,destPort);
			/* Process the new connection. */
			if (err == ERR_OK) 
			{
//				struct netbuf *buf;
//				void *data;
//				u16_t len;

				while ((err = netconn_recv(taskHandler->destConn, &buf)) == ERR_OK) {
				/*printf("Recved\n");*/
					do {
//						netbuf_data(buf, &data, &len);
//						err = netconn_write(taskHandler->destConn, data, len, NETCONN_COPY);
					} while (netbuf_next(buf) >= 0);
					netbuf_delete(buf);
				}
			}
			netconn_close(taskHandler->destConn);
			netconn_delete(taskHandler->destConn);
			DELAY(1000);
		}
	}
	/* TCP 类型非法传参 */
	else{
		debug("Tcp type is invalid, task will be suspend !!\r\n");
		vTaskDelete(null);
		while(1);
	}
}



/**
* @Name  		Network_CgiGetContent
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static uint32_t Network_CgiGetContent(char *src,const char *var_name, char **dst, uint32_t length)
{
    uint32_t headLen,contentLen = 0;

	(*dst) = src;
    headLen = strlen(var_name);
	if(headLen == 0)	return null;

    if (((*dst) = strstr(src, var_name)) != 0)
    {
        if ((*dst)[headLen] == '=')
        {
            (*dst) += headLen + 1;

            contentLen = strcspn((*dst), "&");
            if (contentLen >= length)
            {
                contentLen = length - 1;
            }
        }
    }
    return contentLen;
}



/**
* @Name  		Network_WebCgiProcess
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static int Network_WebCgiProcess(HTTPSRV_CGI_REQ_STRUCT *param)
{
    HTTPSRV_CGI_RES_STRUCT response = {0};

	const char emptyData[] = "null";
	Network_DataBuf_S *dataBuf;
	Network_HttpTaskPara_S *taskPara;

	ERR_CHECK(webTaskHandler != null, return null);

	response.ses_handle = param->ses_handle;
    response.status_code = HTTPSRV_CODE_OK;

	taskPara = (Network_HttpTaskPara_S *)webTaskHandler->para;

    if (param->request_method == HTTPSRV_REQ_GET)
    {
		uint8_t *buf, len;
	
        response.content_type = HTTPSRV_CONTENT_TYPE_PLAIN;
		if(xQueueReceive(webTaskHandler->sendBufQueue, &dataBuf, 0)){
			buf= MALLOC(dataBuf->len);
			memcpy(buf,dataBuf->data,dataBuf->len);
			response.data = (char *)buf;
        	response.data_length = dataBuf->len;
			FREE(dataBuf->data);
			FREE(dataBuf);
		}
        else{
			len = sizeof(emptyData) + 1;
			buf= MALLOC(len);
			memcpy(buf,emptyData,sizeof(emptyData));
			response.data = (char *)buf;
        	response.data_length = len;
		}
        response.content_length = response.data_length;
        HTTPSRV_cgi_write(&response);
		FREE(buf);
    }
    else if (param->request_method == HTTPSRV_REQ_POST)
    {
		char *buf,*content;
		uint32_t length;;

		do{
			length = param->content_length > WEB_CGI_DATA_LEN_MAX ? WEB_CGI_DATA_LEN_MAX : param->content_length;
			if(length == 0)	break;

			buf = MALLOC(length);
			HTTPSRV_cgi_read(param->ses_handle, buf, length);
			length = Network_CgiGetContent(buf,taskPara->cgi.postName,&content,length);
			if(length != 0){
				dataBuf = MALLOC(sizeof(Network_DataBuf_S));
				dataBuf->data = MALLOC(length);
				dataBuf->len = length;
				dataBuf->webType = tCgi;
				memcpy(dataBuf->data,content,length);
				/*内容发送至队列*/
				xQueueSend(webTaskHandler->recBufQueue, &dataBuf, 0);
			}
		}while(0);

        /* Write the response using chunked transmission coding. */
        response.content_type = HTTPSRV_CONTENT_TYPE_HTML;
        /* Set content length to -1 to indicate unknown content length. */
        response.content_length = -1;
        response.data = "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0 Transitional//EN\">";
        response.data_length = strlen(response.data);
        HTTPSRV_cgi_write(&response);
        response.data = "<html><head><title>POST successfull!</title>";
        response.data_length = strlen(response.data);
        HTTPSRV_cgi_write(&response);
        response.data = "<meta http-equiv=\"refresh\" content=\"0; url=cgi.html\"></head><body></body></html>";
        response.data_length = strlen(response.data);
        HTTPSRV_cgi_write(&response);
        response.data_length = 0;
        HTTPSRV_cgi_write(&response);
    }

    return (response.content_length);
}


#if HTTPSRV_CFG_WEBSOCKET_ENABLED
/**
* @Name  		Network_WsConnected
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static uint32_t Network_WsConnected(void *param, WS_USER_CONTEXT_STRUCT context)
{
	Network_HttpTaskPara_S *taskPara;
	
	ERR_CHECK(webTaskHandler != null, return null);
	webTaskHandler->wsHandler = context.handle;
	taskPara = (Network_HttpTaskPara_S *)webTaskHandler->para;
	taskPara->websocket.wsListener(true);
    return (0);
}

/**
* @Name  		Network_WsDisconnect
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static uint32_t Network_WsDisconnect(void *param, WS_USER_CONTEXT_STRUCT context)
{
	Network_HttpTaskPara_S *taskPara;
	
	ERR_CHECK(webTaskHandler != null, return null);
	webTaskHandler->wsHandler = null;
	taskPara = (Network_HttpTaskPara_S *)webTaskHandler->para;
	taskPara->websocket.wsListener(false);
    return (0);
}

/**
* @Name  		Network_WsError
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static uint32_t Network_WsError(void *param, WS_USER_CONTEXT_STRUCT context)
{
    debug("WebSocket error: 0x%X.\r\n", context.error);
    return (0);
}


/**
* @Name  		Network_WsReceive
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static uint32_t Network_WsReceive(void *param, WS_USER_CONTEXT_STRUCT context)
{
	Network_DataBuf_S *dataBuf;	

	ERR_CHECK(context.data.length != 0, return null);
	ERR_CHECK(webTaskHandler != null, return null);
	
	dataBuf = MALLOC(sizeof(Network_DataBuf_S));
	dataBuf->webType = tWebsocket;
	dataBuf->wsType = context.data.type;
	
    if (context.data.type == WS_DATA_TEXT){
        /* 接收文本消息，buf预留最后以为填'\0' */
		dataBuf->data = MALLOC(context.data.length + 1);
    }
    else if(context.data.type == WS_DATA_BINARY){
		dataBuf->data = MALLOC(context.data.length);
    }
	else{
		FREE(dataBuf);
		return null;
	}
	dataBuf->len = context.data.length;
	memcpy(dataBuf->data,context.data.data_ptr,context.data.length);
	/*内容发送至队列*/
	xQueueSend(webTaskHandler->recBufQueue, &dataBuf, 0);

    return (0);
}

/**
* @Name  		Network_WsSend
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static void Network_WsSend(Network_DataBuf_S *dataBuf)
{
	WS_USER_CONTEXT_STRUCT *context;

	ERR_CHECK(dataBuf != null, return);
	ERR_CHECK(dataBuf->data != null, return);
	ERR_CHECK(webTaskHandler != null, return);
	
	context = MALLOC(sizeof(WS_USER_CONTEXT_STRUCT));
	
	context->handle = webTaskHandler->wsHandler;
	context->fin_flag = true;
	context->free_flag = true;
	
	context->data.type = dataBuf->wsType;
	context->data.length = dataBuf->len;
	
	context->data.data_ptr = MALLOC(dataBuf->len);
	memcpy(context->data.data_ptr,dataBuf->data,dataBuf->len);
	
	WS_send(context);

	FREE(context);
}


#endif /* HTTPSRV_CFG_WEBSOCKET_ENABLED */



#ifndef ETH_PHY_RESET_OS_DELAY
static void Network_ResetDelay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 1000000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}
#endif


