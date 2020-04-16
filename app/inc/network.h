#ifndef __NETWORK_H__
#define __NETWORK_H__

#include "global_config.h"
#include "httpsrv.h"
#include "ethernetif.h"
#include "app.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define NETWORK_ETH_NUM								(2)

#define NETWORK_MAC_SIZE							(6)
#define NETWORK_IP_SIZE								(4)


#define NETWORK_IP									Network_Addr_S
#define NETWORK_GW									Network_Addr_S
#define NETWORK_MASK								Network_Addr_S
#define NETWORK_MAC									Network_Mac_S
#define NETWORK_PORT								uint16_t
#define NETWORK_TASK_FUNC							TaskFunction_t

#define NETWORK_ETH_FRAME_SIZE(dataSize)			(dataSize + 8 + 20 +14)

/* 网络类型 */
/** If set, the netif is an ethernet device. It might not use ARP or TCP/IP if it is used for PPPoE only.*/
#define NETWORK_TYPE_ETHERNET						(1U)	
/** If set, the netif is an ethernet device using ARP.Set by the netif driver in its init function.Used to check input packet types and use of DHCP. */
#define NETWORK_TYPE_TCPIP							(2U)		

#define NETWORK_SET_ADDR(addr,a0,a1,a2,a3)			{do{addr.addr0=a0;addr.addr1=a1;addr.addr2=a2;addr.addr3=a3;}while(0);}
#define NETWORK_SET_MAC(mac,m0,m1,m2,m3,m4,m5)		{if(mac!=0){do{mac->mac0=m0;mac->mac1=m1;mac->mac2=m2;mac->mac3=m3;mac->mac4=m4;mac->mac5=m5;}while(0);}}

#define NETWORK_COMPARISON_ADDR(ADDR1,ADDR2)		(ADDR1.addr0 == ADDR2.addr0 && \
													 ADDR1.addr1 == ADDR2.addr1 && \
													 ADDR1.addr2 == ADDR2.addr2 && \
													 ADDR1.addr3 == ADDR2.addr3)
													 
#define NETWORK_COMPARISON_MAC(MAC1,MAC2)			(MAC1 != null && \
													 MAC2 != null && \
													 MAC1->mac0 == MAC2->mac0 && \
													 MAC1->mac1 == MAC2->mac1 && \
													 MAC1->mac2 == MAC2->mac2 && \
													 MAC1->mac3 == MAC2->mac3 && \
													 MAC1->mac4 == MAC2->mac4 && \
													 MAC1->mac5 == MAC2->mac5)
/***************** 网络配置相关 *****************/
/* 网口状态监听函数 */
typedef void (*Network_EthStaListener)(bool);

/* 网络接口类型 */
//typedef enum {
//	eth0 = 0,
//	eth1 = 1,
//} Network_EthIndex_EN;
typedef enet_port_type Network_EthIndex_EN;

/* 网络IP地址数据结构 */
typedef struct {
	uint8_t addr0;
	uint8_t addr1;
	uint8_t addr2;
	uint8_t addr3;
} Network_Addr_S;

/* 网络MAC地址数据结构 */
typedef struct {
	uint8_t mac0;
	uint8_t mac1;
	uint8_t mac2;
	uint8_t mac3;
	uint8_t mac4;
	uint8_t mac5;
} Network_Mac_S;

/* 网络配置参数数据结构 */
typedef struct {
	Network_EthIndex_EN index;
	uint8_t			type;
	NETWORK_IP 		ip;
	NETWORK_GW 		gateway;
	NETWORK_MASK 	netmask;
	NETWORK_MAC		*mac;
	Network_EthStaListener ethStaListener;
} Network_EthPara_S; 
 
/* 以太网数据帧结构 */
typedef struct {
	Network_Mac_S 	destMac;
	Network_Mac_S 	srcMac;
	uint16_t 		type;
	uint8_t 		ver_len;
	uint8_t 		servField;
	uint16_t 		totalLen;
	uint16_t 		id;
	uint16_t		offset;
	uint8_t 		ttl;
	uint8_t 		prot;
	uint16_t 		headChecksum;
	Network_Addr_S	srcIp;
	Network_Addr_S	destIp;
	NETWORK_PORT	srcPort;
	NETWORK_PORT	destPort;
	uint16_t		length;
	uint16_t		checksum;
	uint8_t			dataHead;
}Network_EthernetFrame_S;
 
/***************** 网络任务相关 *****************/
/* WEB接口(websocket,cgi等)接收数据最大长度*/
#define WEB_CGI_DATA_LEN_MAX			(256)


/* 每个网口网络任务最大创建数量(根据任务类型数量) */
#define NETWORK_TASK_MAX_NUM		4


/* 网络任务类型 */
typedef enum{
	tEthernet, tUdp, tTcp, tHttp,
}Network_TaskType_EN;


/* 任务参数指针 */
typedef void * Network_TaskPara; 
 
/* 网络任务控制句柄 */
typedef void * Network_TaskHandler_S;

/* TCP类型 */
typedef enum {
	tServer,tClient
}Network_TcpType_EN;

/* Web(http)传输类型 */
typedef enum {
	tCgi,tWebsocket
}Network_WebType_EN;


/* 网络任务通讯数据包结构 */
typedef struct{
	/* 数据包源地址及目的地址(用于ethernet网络任务及wifi会议) */
	Network_Mac_S 	destMac;
	Network_Addr_S	destIp;
	NETWORK_PORT	destPort;
	
	Network_Mac_S 	srcMac;
	Network_Addr_S	srcIp;
	NETWORK_PORT	srcPort;

	/* WEB相关数据类型 */
	Network_WebType_EN webType;
	/* WebSocket 数据类型 */
	WS_DATA_TYPE wsType;

	/* 用于记录数据包最大长度 */
	uint32_t 		maxLen;

	/* 数据包长度及数据包指针 */
	uint32_t 		len;
	uint8_t 		*data;
}Network_DataBuf_S;

/* HTTP任务参数结构 */
typedef struct {
	/* 网页后台文件系统 */
	const HTTPSRV_FS_DIR_ENTRY *httpFsDir;
	/* 文件根目录 */
	const char *rootDir;		
	/* 主页 */
	const char *indexPage;
	
	/* 端口号(为0的话使用默认端口'80') */
	uint16_t port;

	/* cgi相关配置参数 */
	struct {
		bool enable;
		const char* postName;
	}cgi;

	/* websocket相关配置参数 */
	struct {
		bool enable;
		/* websocket连接监听函数 */
		void (*wsListener)(bool);
	}websocket;
}Network_HttpTaskPara_S;

/* UDP任务参数结构 */
typedef struct {
	uint8_t i;
}Network_UdpTaskPara_S;

/* TCP任务参数结构 */
typedef struct {
	Network_TcpType_EN type;
	
	/* 本地端口 */
	NETWORK_PORT port;
	
	/* tcp连接监听函数 */
	void (*tcpListener)(bool);
	
	/* 目的IP端口(用于tcp客户端) */
	NETWORK_IP	destIp;
	NETWORK_PORT destPort;
}Network_TcpTaskPara_S;

/* ETHERNET任务参数结构 */
typedef struct {
	/* 本地端口 */
	NETWORK_PORT port;
	
	/* 多播监听使能 地址 端口 */
	uint8_t multicastNum;
	NETWORK_IP	*multiIp;
	NETWORK_PORT *multiPort;

}Network_EthernetTaskPara_S;



/* 网络功能API数据结构 */
typedef struct {
//    void (*init)(void);
	AppLauncher_S *launcher;

	void (*ethConfig)(Network_EthPara_S *);
	Network_TaskHandler_S *(*creatTask)(Network_EthIndex_EN ,Network_TaskType_EN ,Network_TaskPara );
	void (*destoryTask)(Network_TaskHandler_S *netHandler);
	void (*receive)(Network_TaskHandler_S *, Network_DataBuf_S * ,uint32_t);
	void (*transmit)(Network_TaskHandler_S *,Network_DataBuf_S *);
} Network_S;




/*******************************************************************************
 * API
 ******************************************************************************/
extern Network_S Network;
extern void Network_AddUdpNoArpTask(Network_EthIndex_EN index);

#endif
