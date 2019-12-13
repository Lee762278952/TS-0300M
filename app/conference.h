#ifndef __CONFERENCE_H__
#define __CONFERENCE_H__



#include "stdint.h"
#include "network.h"
#include "protocol.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
 /* ����ģʽ */
typedef enum {
	kMode_Conference = 0, 
	kMode_Sign, 
	kMode_Vote, 
	kMode_EditID, 
	kMode_DevSign, 
	kMode_DevVote,
}SysMode_EN;

/* ��Ͳģʽ */
typedef enum {
	kMode_Mic_Fifo = 1, 
	kMode_Mic_Normal, 
	kMode_Mic_VoiceCtrl, 
	kMode_Mic_Apply,
}MicMode_EN;
 

/********************** �����߳�֪ͨ������ݽṹ������ *****************************/  
 
/* ֪ͨԴ(���ߵ�Ԫ��WIFI��Ԫ��WEB����(HTTP)��PC�˿ڿ���(TCP)�����ڿ��ơ���Ļ����) */
typedef enum {
	kType_NotiSrc_WiredUnit 	= (1<<0), 
	kType_NotiSrc_WifiUnit  	= (1<<1),
	kType_NotiSrc_Web 			= (1<<2), 
	kType_NotiSrc_PC			= (1<<3), 
	kType_NotiSrc_UartCtrl		= (1<<4), 
	kType_NotiSrc_ScreenCtrl 	= (1<<5),
}NotifySrcType_EN;

typedef NotifySrcType_EN InstructDest_EN;

/* ֪ͨ��ʽ */
typedef struct {
	/* ֪ͨԴ */
	NotifySrcType_EN nSrc;
	
	/* ����֪ͨ���α��� */
	union{
		ConfProtocol_S conference;
		WifiUnitProtocol_S wifi;
		ScreenProtocol_S screen;
	}prot;

	uint16_t    exLen;
	uint8_t		exDataHead;
}Notify_S;

/********************** �����߳�ִ�в���������ݽṹ������ *****************************/  
/* ִ�в���Ŀ�����(���ߵ�Ԫ��WIFI��Ԫ��WEB���ơ�PC�˿ڿ��ơ����ڿ��ơ���Ļ����)
	ͨ�� NotifySrcType_EN ǿתshort�����ݣ�ͨ��λ�ж�ȷ������Ŀ��*/
#define EXE_DEST							uint16_t
#define ALL_DEST							(0xFFFF)
#define EX_CTRL_DEST						(kType_NotiSrc_PC|kType_NotiSrc_Web|kType_NotiSrc_UartCtrl)
#define ALL_UNIT							(kType_NotiSrc_WifiUnit|kType_NotiSrc_WiredUnit)


/********************** ��Ԫ�豸������ݽṹ������ *****************************/ 
/* ��Ԫ�������� */
#define UNIT_TYPE_NUM								(2)
 
/* ������ߵ�Ԫ�� */
#define WIRED_UNIT_MAX_ONLINE_NUM					(4096)

/* ���������Ͳ�������ߣ� */
#define WIRED_UNIT_MAX_ALLWO_OPEN					(8)

/* ���WIFI��Ԫ�� */
#define WIFI_UNIT_MAX_ONLINE_NUM					(300)

/* ���������Ͳ����WIFI�� */
#define WIFI_UNIT_MAX_ALLWO_OPEN					(6)

/* WIFI��Ԫ��ʼID */
#define WIFI_UNIT_START_ID							(0x3000)


 /* ��Ԫ���ͣ����ߵ�Ԫ��WIFI��Ԫ�� */
 typedef enum{
	tWired = 0, tWifi = 1
 }UnitType_EN;
 
/* ��Ԫ���ԣ���ϯ����Ԫ����Ա�� */
typedef enum {
	aRepresentative, aChairman, aInterpreter
}UnitAttr_EN;

/* ��Ԫ��Ͳ״̬ */
typedef enum {
	sClose,sOpen,sApply,sWait
}UnitMicSta_EN;

/* ϵͳ�������� */
typedef enum {
	Chinese = 1,
	English,
	Russian,
	French,
}Language_EN;

/* ���߻�Ͳ�� */
typedef struct {
	/* ������ϯ��Ԫ */
	uint16_t wiredChm;
	/* ���ߴ���Ԫ */
	uint16_t wiredRps;
	/* WIFI��ϯ��Ԫ */
	uint16_t wifiChm;
	/* WIFI����Ԫ */
	uint16_t wifiRps;
	/* ��Ա�� */
	uint16_t interpreter;
}UnitOnlineNum;





/**
* @Description	��Ԫ��Ϣ���ݽṹ
*
**/
#pragma pack(1)
typedef struct {

	/* ���߱�־ */
	bool online;

	/* ����ǩ�� */
	bool allowSign;
	/* ǩ����־ */
	bool sign;

	/* ����ͶƱ */
	bool allowVote;
	/* ͶƱ��� */
	uint8_t vote;
	
	/* ��ѯ�������õ�Ӧ�����գ�
	   ��������"POLLING_NO_REPLY_OFFLINE_COUNT"�ж����ߣ� */
	uint16_t pollCount;
	
	/* ��Ԫ���ͣ���ϯ��������Ա���� */
	UnitAttr_EN attr;
	
	/* ��ԪMAC,IP */
	NETWORK_MAC mac;
	NETWORK_IP ip;
	
	/* ��Ԫ��Ͳ״̬(�����ء����롢�ȴ�) */
	UnitMicSta_EN micSta;
	
	/* �򿪻�Ͳʱ��Ƶͨ����Ĭ��0�� */
	uint8_t channel;

	/* �������ź�ֵ(wifi) */
	uint8_t soc;
	uint8_t rssi;
}UnitInfo_S;
#pragma pack()


 /**
 * @Description  ����ϵͳ���ݽṹ
 *
 **/
 typedef struct {
	 /* ϵͳģʽ */
	 SysMode_EN sysMode;
 
	 /* ��Ͳģʽ */
	 MicMode_EN micMode;
	 
	/* ͶƱģʽ */
	VoteMode_EN voteMode;
 
	 /* ���ߵ�Ԫ��ǰID(��ID) */
	 uint16_t wiredCurEditID;

	  /* WIFI��Ԫ��ǰID(��ID) */
	 uint16_t wifiCurEditID;
 
	 /* ǩ�������� */
	 uint16_t totalSignNum;
	 
	 /* ��ǰǩ������ */
	 uint16_t currentSignNum;
 
	 /* ������Ͳ�� - ���ߵ�Ԫ */
	 uint8_t wiredAllowOpen;
	 
	 /* �ɵȴ���Ͳ�� - ���ߵ�Ԫ
	 (�ɵȴ�������������Ͳ�������) */
	 uint8_t wiredAllowWait;
 
	 /* ������Ͳ�� - ���ߵ�Ԫ */
	 uint8_t wifiAllowOpen;
	 
	 /* �ɵȴ���Ͳ�� - ���ߵ�Ԫ
	 (�ɵȴ�������������Ͳ�������) */
	 uint8_t wifiAllowWait;

	 /* ϵͳ���� */
	 Language_EN language;
 
 }ConfSysInfo_S;



 /* API struct */
typedef struct {
	void (*launch)(void);
	void (*notify)(Notify_S *);
	void (*getOnlineNum)(UnitOnlineNum *);
	SysMode_EN (*getSysMode)(void);
	uint16_t (*getCurrentEditId)(void);
	UnitInfo_S *(*wiredUnitInfo)(void);
	UnitInfo_S *(*wifiUnitInfo)(void);
	ConfSysInfo_S (*getConfSysInfo)(void);
}Conference_S;

/*******************************************************************************
 * API
 ******************************************************************************/
extern Conference_S Conference;




#endif
