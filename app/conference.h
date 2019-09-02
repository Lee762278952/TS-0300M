#ifndef __CONFERENCE_H__
#define __CONFERENCE_H__

/* �������,ֻ�ڱ� .h�ļ��ڶ���,�ļ�ĩβȡ������ */
#define LICENSE_ACCESS_UNIT_INFO 

#include "stdint.h"
#include "network.h"
#include "protocol.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
 /* ����״̬ */
typedef enum {
	mConference = 0, mEditID, mSign, mVote, mDevSign, mDevVote
}SysMode_EN;

/* ��Ͳģʽ */
typedef enum {
	mFifo = 1, mNormal, mVoiceCtrl, mApply
}MicMode_EN;
 

/********************** �����߳�֪ͨ������ݽṹ������ *****************************/  
 
 /* �ⲿ�̣߳�web,pc,unit,screen�ȣ� ֪ͨ�����߳�
	�ؼ���,ǰ׺(NK_  ==  Notify Keywork)*/
#define NOTIFY_KEYWORD						uint8_t

#define NK_CONFERENCE_MSG					(0x01)
#define NK_UNITINFO_UPDATE					(0x02)
#define NK_UNIT_OFFLINE						(0x03)

/* ֪ͨԴ(���ߵ�Ԫ��WIFI��Ԫ��WEB����(HTTP)��PC�˿ڿ���(TCP)�����ڿ��ơ���Ļ����) */
typedef enum {
	sWiredUnit 	= (1<<0), 
	sWifiUnit  	= (1<<1),
	sWeb 		= (1<<2), 
	sPC			= (1<<3), 
	sUartCtrl	= (1<<4), 
	sScreenCtrl = (1<<5),
}NotifySrc_EN;

/* ֪ͨ��ʽ */
typedef struct {
	/* ֪ͨԴ */
	NotifySrc_EN nSrc;
	
//	/* ֪ͨ�ؼ��� */
//	NOTIFY_KEYWORD kWord;
	
	/* ����֪ͨ���α��� */
	union{
		ConfProtocol_S conference;
		WifiUnitProtocol_S wifi;
	}prot;

	uint16_t    exLen;
	uint8_t		exDataHead;
}Notify_S;

/********************** �����߳�ִ�в���������ݽṹ������ *****************************/  
/* ִ�в���Ŀ�����(���ߵ�Ԫ��WIFI��Ԫ��WEB���ơ�PC�˿ڿ��ơ����ڿ��ơ���Ļ����)
	ͨ�� NotifySrc_EN ǿתshort�����ݣ�ͨ��λ�ж�ȷ������Ŀ��*/
#define EXE_DEST							uint16_t
#define ALL_DEST							(0xFFFF)
#define EX_CTRL_DEST						(sPC|sWeb|sUartCtrl)
#define ALL_UNIT							(sWifiUnit|sWiredUnit)


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
* @Description	��Ԫ��Ϣ���ݽṹ,��Ҫ����ꡰLICENSE_ACCESS_UNIT_INFO��
*
* @License 		LICENSE_ACCESS_UNIT_INFO
**/
#ifdef LICENSE_ACCESS_UNIT_INFO
#pragma pack(1)
typedef struct {

	/* ���߱�־ */
	bool online;

	/* ǩ����־ */
	bool sign;
	
	/* ��ѯ�������õ�Ӧ�����գ�
	   ��������"POLLING_NO_REPLY_OFFLINE_COUNT"�ж����ߣ� */
	uint16_t pollCount;
	
	/* ��Ԫ���ͣ���ϯ��������Ա���� */
	UnitAttr_EN attr;
	
	/* ��ԪMAC */
	NETWORK_MAC mac;
	
	/* ��Ԫ��Ͳ״̬(�����ء����롢�ȴ�) */
	UnitMicSta_EN micSta;
	
	/* �򿪻�Ͳʱ��Ƶͨ����Ĭ��0�� */
	uint8_t channel;
}UnitInfo_S;
#pragma pack()
#endif



 /* API struct */
typedef struct {
	void (*launch)(void);
	void (*notify)(Notify_S *);
	void (*getOnlineNum)(UnitOnlineNum *);
	SysMode_EN (*getSysMode)(void);
	uint16_t (*getCurrentEditId)(void);
#ifdef LICENSE_ACCESS_UNIT_INFO
	UnitInfo_S *(*wiredUnitInfo)(void);
#else
	void *priv;
#endif
	
}Conference_S;

/*******************************************************************************
 * API
 ******************************************************************************/
extern Conference_S Conference;



/* ȡ��������� */
#undef LICENSE_ACCESS_UNIT_INFO
#endif
