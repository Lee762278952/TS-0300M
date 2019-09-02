#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "stdint.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
 
/*******************************************************************************
 ***                        ȫ���ֻ���ϵͳͨѶЭ��                           ***
 *******************************************************************************/
/************ �㲥ID ****************/
#define WHOLE_BROADCAST_ID							(0xFFF0)			//�㲥ȫ�����
#define UNIT_BROADCAST_ID							(0xFFF1)			//���Ե�Ԫ���㲥��Ϣ
#define EX_HOST_BROADCAST_ID						(0xFFF2)			//������չ�����㲥��Ϣ
#define TRANS_BROADCAST_ID							(0xFFF3)			//������Ա���㲥��Ϣ
#define MODE_BROADCAST_ID							(0xFFE1)			//�㲥��ǰ����ģʽ
#define NUM_BROADCAST_ID							(0xFFE2)			//�㲥�᳡����������ǰ����Ͳ����
#define APPLY_OPEN_MIC_ID							(0xFFC1)			//����ϯ���㲥��ǰ���뿪��ͲID
#define REVOKE_APPLY_ID								(0xFFC2)			//����ϯ���㲥��ǰ�������뻰ͲID

/* �����Ϣ(0x83)���ID */
#define RESULT_MSG_ID(_id)							(0xFFD0 + _id)		 //�����ϢID
#define RES_SIGN_NUM								(0x01)				 //ǩ��������
#define RES_OPT_NUM(n)								(0x01 + n)			 //��n��ѡ������
 
/************ ����(type) ************/
#define BASIC_MSG									(0x80)				//������Ϣ
#define POLLING_MSG									(0x81)				//��ѯ��Ϣ
#define STATE_MSG									(0x82)				//״̬��Ϣ
#define RESULT_MSG									(0x83)				//�����Ϣ
#define APPLY_MSG									(0x84)				//������Ϣ
#define EXTEND_MSG									(0x85)				//��չͳ����Ϣ
#define PC_MSG										(0x86)				//PCͨѶ��Ϣ
#define WIRED_UNIT_NUM								(0x87)				//���ߵ�Ԫ����
#define WIFI_UNIT_NUM								(0x88)				//���ߵ�Ԫ����
#define NAMEPLATE_PARA								(0x90)				//�������Ʋ���
#define CONFERENCE_MSG								(0x96)				//������Ϣ���������ݡ�ժҪ�ȣ�
#define NAMEPLATE_COMP								(0x97)				//PC�·������������ݣ���˾���ƣ�
#define NAMEPLATE_POS								(0x98)				//PC�·������������ݣ�ְλ��
#define NAMEPLATE_CONTENT							(0x99)				//PC�·������������ݣ��ɵ�������Э�飬���ڼ��ݣ�
#define USER_LIST									(0x9A)				//�û��б�ID�Ͳλ��˵Ķ�Ӧ�б�
#define CONSTANT_NOTIFY								(0x9B)				//PC�·��Ĺ̶�֪ͨ��Ϣ������������xxxx��
#define NAMEPLATE_NAME								(0x9C)				//PC�·������������ݣ�������
#define NAMEPLATE_UPDATE							(0x9D)				//PC�������µ������ƣ��ɵ�������Э�飬���ڼ��ݣ�
#define HOST_REPLY_PC								(0x9E)				//�����ظ�PC���ͳɹ�

 
/************ ����ph(para high) ************/
/* ǰ���ֶ� 	BASIC_MSG */
#define CONFERENCE_MODE								(0x00)
#define SIGN_MODE									(0x01)
#define VOTE_MODE									(0x02)
#define EDIT_ID_MODE								(0x03)

/* ǰ���ֶ� 	PC_MSG */
#define HEARTBEAT									(0x00)
#define CFG_UART_1									(0x01)
#define CFG_UART_2									(0x02)
#define CFG_UART_3									(0x03)
#define CFG_UART_4									(0x04)
#define CFG_UART_5									(0x05)
#define CFG_MAC										(0x06)
#define CFG_IP										(0x07)
#define CFG_MASK									(0x08)
#define CFG_GW										(0x09)
#define QUERY_HOST									(0x0F)
#define SCAN_ONLINE_UNIT							(0x30)
#define ONLINE_UNIT									(0x31)
#define QUERY_PRIOR_SIGN							(0x33)
#define QUERY_PRIOR_VOTE							(0x34)
#define QUERY_PRIOR_SCAN							(0x35)

/* ǰ���ֶ� 	POLLING_MSG */
#define SIGN_POLLING								(0x06)
#define EDIT_ID_POLLING								(0x08)		

/************ ����pl(para low) ************/
/* ǰ���ֶ� 	BASIC_MSG  CONFERENCE_MODE */
#define START_CONFERENCE_MODE						(0x01)
#define ID_DUPICATE									(0x03)
#define RPS_OPEN_MIC								(0x05)
#define RPS_CLOSE_MIC								(0x06)
#define CHM_OPEN_MIC								(0x07)
#define CHM_CLOSE_MIC								(0x08)
#define CHM_PRIORITY								(0x0B)
#define MIC_WAIT									(0x0D)
#define MIC_DISWAIT									(0x0E)
#define QUERY_UNIT									(0x13)
#define AGREE_OPEN_MIC								(0x15)
#define RPS_REPLY_ONLINE							(0x16)
#define CHM_REPLY_ONLINE							(0x17)
#define MIC_CHANNEL_FULL							(0x25)
#define RPS_APPLY_ACCESS_SYS						(0x26)
#define CHM_APPLY_ACCESS_SYS						(0x27)
#define UNIT_ACCESS_SYS								(0x28)
#define UNIT_OFFLINE								(0x40)
#define DISAGREE_OPEN_MIC							(0x5C)

/* ǰ���ֶ� 	BASIC_MSG  SIGN_MODE */
#define START_SIGN_MODE								(0x01)
#define UNIT_SIGN_IN								(0x05)
#define END_SIGN_MODE								(0x06)
#define SIGN_ENABLE									(0x08)
#define SIGN_DISABLE								(0x09)
#define SUPPLEMENT_SIGN								(0x0A)
#define UNIT_SUPPLEMENT_SIGN						(0x0B)
#define DEV_START_SIGN_MODE							(0x0E)
#define DEV_END_SIGN_MODE							(0x0F)
#define CONTROL_UNIT_SIGN							(0x12)

/* ǰ���ֶ� 	BASIC_MSG  EDIT_ID_MODE */
#define START_EDIT_ID_MODE							(0x80)
#define UNIT_CONFIRM_ID								(0x86)

/* ǰ���ֶ� 	POLLING_MSG  SIGN_POLLING */
#define HOST_SIGN_POLLIGN							(0x52)

/* ǰ���ֶ� 	POLLING_MSG  EDIT_ID_POLLING */
#define REPEATED_ID									(0x60)
#define CURRENT_ID									(0x61)
#define CONFIRM_ID									(0x62)




/* ǰ���ֶ� 	PC_MSG  SCAN_ONLINE_UNIT */
#define SCAN_UNIT_END								(0x01)


/*******************************************************************************
 ***                        WIFI��ԪͨѶЭ��                           ***
 *******************************************************************************/





/******************************** Э�����ݽṹ ************************************/
/*  ȫ���ֻ���ϵͳͨѶЭ�����ݽṹ(�����ṹ) */
#pragma pack(1)	/* ���ֽڶ��� */
typedef struct {
	uint16_t id;
	uint8_t type;
	uint8_t ph;
	uint8_t pl;
	uint8_t ch; 
	uint16_t sec;
}ConfProtocol_S;
#pragma pack()

/*  WIFI��ԪͨѶЭ�����ݽṹ(�����ṹ) */
#pragma pack(1)	/* ���ֽڶ��� */
typedef struct {
	uint16_t id;
	uint8_t cmd;
	uint8_t ph;
	uint8_t pl;
}WifiUnitProtocol_S;
#pragma pack()


typedef struct {
	ConfProtocol_S *(*conference)(ConfProtocol_S *,uint16_t ,uint8_t ,uint8_t ,uint8_t ,uint8_t ,uint16_t );
	WifiUnitProtocol_S *(*wifiUnit)(WifiUnitProtocol_S *, uint16_t, uint8_t ,uint8_t ,uint8_t);
}Protocol_S;

extern Protocol_S Protocol;
#endif

