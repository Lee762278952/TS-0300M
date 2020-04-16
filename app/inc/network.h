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

/* �������� */
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
/***************** ����������� *****************/
/* ����״̬�������� */
typedef void (*Network_EthStaListener)(bool);

/* ����ӿ����� */
//typedef enum {
//	eth0 = 0,
//	eth1 = 1,
//} Network_EthIndex_EN;
typedef enet_port_type Network_EthIndex_EN;

/* ����IP��ַ���ݽṹ */
typedef struct {
	uint8_t addr0;
	uint8_t addr1;
	uint8_t addr2;
	uint8_t addr3;
} Network_Addr_S;

/* ����MAC��ַ���ݽṹ */
typedef struct {
	uint8_t mac0;
	uint8_t mac1;
	uint8_t mac2;
	uint8_t mac3;
	uint8_t mac4;
	uint8_t mac5;
} Network_Mac_S;

/* �������ò������ݽṹ */
typedef struct {
	Network_EthIndex_EN index;
	uint8_t			type;
	NETWORK_IP 		ip;
	NETWORK_GW 		gateway;
	NETWORK_MASK 	netmask;
	NETWORK_MAC		*mac;
	Network_EthStaListener ethStaListener;
} Network_EthPara_S; 
 
/* ��̫������֡�ṹ */
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
 
/***************** ����������� *****************/
/* WEB�ӿ�(websocket,cgi��)����������󳤶�*/
#define WEB_CGI_DATA_LEN_MAX			(256)


/* ÿ����������������󴴽�����(����������������) */
#define NETWORK_TASK_MAX_NUM		4


/* ������������ */
typedef enum{
	tEthernet, tUdp, tTcp, tHttp,
}Network_TaskType_EN;


/* �������ָ�� */
typedef void * Network_TaskPara; 
 
/* ����������ƾ�� */
typedef void * Network_TaskHandler_S;

/* TCP���� */
typedef enum {
	tServer,tClient
}Network_TcpType_EN;

/* Web(http)�������� */
typedef enum {
	tCgi,tWebsocket
}Network_WebType_EN;


/* ��������ͨѶ���ݰ��ṹ */
typedef struct{
	/* ���ݰ�Դ��ַ��Ŀ�ĵ�ַ(����ethernet��������wifi����) */
	Network_Mac_S 	destMac;
	Network_Addr_S	destIp;
	NETWORK_PORT	destPort;
	
	Network_Mac_S 	srcMac;
	Network_Addr_S	srcIp;
	NETWORK_PORT	srcPort;

	/* WEB����������� */
	Network_WebType_EN webType;
	/* WebSocket �������� */
	WS_DATA_TYPE wsType;

	/* ���ڼ�¼���ݰ���󳤶� */
	uint32_t 		maxLen;

	/* ���ݰ����ȼ����ݰ�ָ�� */
	uint32_t 		len;
	uint8_t 		*data;
}Network_DataBuf_S;

/* HTTP��������ṹ */
typedef struct {
	/* ��ҳ��̨�ļ�ϵͳ */
	const HTTPSRV_FS_DIR_ENTRY *httpFsDir;
	/* �ļ���Ŀ¼ */
	const char *rootDir;		
	/* ��ҳ */
	const char *indexPage;
	
	/* �˿ں�(Ϊ0�Ļ�ʹ��Ĭ�϶˿�'80') */
	uint16_t port;

	/* cgi������ò��� */
	struct {
		bool enable;
		const char* postName;
	}cgi;

	/* websocket������ò��� */
	struct {
		bool enable;
		/* websocket���Ӽ������� */
		void (*wsListener)(bool);
	}websocket;
}Network_HttpTaskPara_S;

/* UDP��������ṹ */
typedef struct {
	uint8_t i;
}Network_UdpTaskPara_S;

/* TCP��������ṹ */
typedef struct {
	Network_TcpType_EN type;
	
	/* ���ض˿� */
	NETWORK_PORT port;
	
	/* tcp���Ӽ������� */
	void (*tcpListener)(bool);
	
	/* Ŀ��IP�˿�(����tcp�ͻ���) */
	NETWORK_IP	destIp;
	NETWORK_PORT destPort;
}Network_TcpTaskPara_S;

/* ETHERNET��������ṹ */
typedef struct {
	/* ���ض˿� */
	NETWORK_PORT port;
	
	/* �ಥ����ʹ�� ��ַ �˿� */
	uint8_t multicastNum;
	NETWORK_IP	*multiIp;
	NETWORK_PORT *multiPort;

}Network_EthernetTaskPara_S;



/* ���繦��API���ݽṹ */
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
