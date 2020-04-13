#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "stdint.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 ***                        ȫ���ֻ���ϵͳͨѶЭ��                           ***
 *******************************************************************************/

/* ȫ���ֻ���Э�����ݰ���С���� */
#define CONF_PROT_MIN_LEN							(13)

/************ �㲥ID ****************/
#define WHOLE_BROADCAST_ID							(0xFFF0)			//�㲥ȫ�����
#define UNIT_BROADCAST_ID							(0xFFF1)			//���Ե�Ԫ���㲥��Ϣ
#define EX_HOST_BROADCAST_ID						(0xFFF2)			//������չ�����㲥��Ϣ
#define TRANS_BROADCAST_ID							(0xFFF3)			//������Ա���㲥��Ϣ
#define MODE_BROADCAST_ID							(0xFFE1)			//�㲥��ǰ������Ͳģʽ�����ߵ�Ԫ��
#define NUM_BROADCAST_ID							(0xFFE2)			//�㲥�᳡����������ǰ����Ͳ����
#define NOW_DATE_TIME_ID							(0xFFEC)			//PC�㲥�·�ʱ��					
#define WIFI_MODE_BROADCAST_ID						(0xFFEA)			//�㲥��ǰ������Ͳģʽ��WIFI��Ԫ��
#define APPLY_OPEN_MIC_ID							(0xFFC1)			//����ϯ���㲥��ǰ���뿪��ͲID
#define REVOKE_APPLY_ID								(0xFFC2)			//����ϯ���㲥��ǰ�������뻰ͲID

/* �����Ϣ(0x83)���ID */
//#define RESULT_MSG_ID(_id)							(0xFFD0 + _id)		 //�����ϢID
//#define RES_SIGN_NUM								(0x01)				 //ǩ��������
//#define RES_OPT_NUM(n)								(0x01 + n)			 //��n��ѡ������

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
#define AUDIO_MATRIX								(0xB0)				//��Ƶ�����������
#define AUDIO_MATRIX_INQUIRE						(0xB1)              //��Ƶ���������ѯ


/************ ����ph(para high) ************/
/* ǰ���ֶ� 	BASIC_MSG */
#define CONFERENCE_MODE								(0x00)
#define SIGN_MODE									(0x01)
#define VOTE_MODE									(0x02)
#define EDIT_ID_MODE								(0x03)
#define UNIT_CTRL									(0x04)

/* ǰ���ֶ� RESULT_MSG */
#define SIGN_RESULT									(0x01)
#define VOTE_RESULT									(0x02)

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
#define CAMERA_POSITION								(0x10)
#define CAMERA_CONTROL								(0x14)
#define UNIT_LOGOUT									(0x20)
#define UNIT_LOGIN									(0x21)
#define CUSTOM_VOTE_ITEM							(0x24)
#define PC_EDIT_ID									(0x2F)
#define SCAN_ONLINE_UNIT							(0x30)
#define ONLINE_UNIT									(0x31)
#define QUERY_PRIOR_SIGN							(0x33)
#define QUERY_PRIOR_VOTE							(0x34)
#define QUERY_PRIOR_SCAN							(0x35)

/* ǰ���ֶ� 	POLLING_MSG */
#define SIGN_POLLING								(0x06)
#define VOTE_POLLING								(0x07)
#define EDIT_ID_POLLING								(0x08)

/* ǰ���ֶ� AUDIO_MATRIX �� AUDIO_MATRIX_INQUIRE */
#define DSP_MODE									(0x00)
#define DSP_UNIT_OUT_CFG							(0x01)
#define DSP_NOR_OUT_CFG								(0x02)
#define DSP_CHANNEL_OUT_CFG							(0x03)



/************ ����pl(para low) ************/
/* ǰ���ֶ� 	BASIC_MSG  CONFERENCE_MODE */
#define START_CONFERENCE_MODE						(0x01)
#define HOST_LAUNCH									(0x02)
#define ID_DUPICATE									(0x03)
#define ID_UNDUPICATE								(0x04)
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
#define FIRE_ALARM_SIGNAL							(0x21)
#define FIRE_ALARM_SIGNAL_CANCEL					(0x22)
#define MIC_CHANNEL_FULL							(0x25)
#define RPS_APPLY_ACCESS_SYS						(0x26)
#define CHM_APPLY_ACCESS_SYS						(0x27)
#define RECKON_TIME_ENABLE							(0x30)
#define RECKON_TIME_DISABLE							(0x31)
#define TIME_LIMIT_ENABLE							(0x32)
#define TIME_LIMIT_DISABLE							(0x33)
#define UNIT_ACCESS_SYS								(0x28)
#define UNIT_OFFLINE								(0x40)
#define SERVICE_TEA									(0x50)
#define SERVICE_PAPER_PEN							(0x51)
#define SERVICE_ARTIFICIAL							(0x52)
#define SERVICE_COFFEE								(0x53)
#define SERVICE_FLOWER								(0x54)
#define SERVICE_MICPHONE							(0x55)
#define SERVICE_RESERVED_1							(0x56)
#define SERVICE_RESERVED_2							(0x57)
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
#define END_EDIT_ID_MODE							(0x81)
#define UNIT_CONFIRM_ID								(0x86)

/* ǰ���ֶ� 	BASIC_MSG  VOTE_MODE */
#define PAUSE_VOTE									(0x07)				//��ͣ���
#define FINISH_VOTE									(0x08)				//�������
#define DEV_LAUNCH_VOTE								(0x2E)				//��ϯ��������
#define DEV_FINISH_VOTE								(0x2F)				//��ϯ���������
#define VOTE_ENABLE									(0x38)
#define VOTE_DISABLE								(0x39)

#define KEY3_FIRST_SIGN_VOTE 						(0x13)				// ͶƱ/���/����/����� ����
#define KEY3_FIRST_NOSIGN_VOTE   					(0x12)
#define KEY3_LAST_SIGN_VOTE     					(0x11)
#define KEY3_LAST_NOSIGN_VOTE						(0x10)

#define KEY5_FIRST_SIGN_SELECT						(0x17)
#define KEY5_FIRST_NOSIGN_SELECT					(0x16)
#define KEY5_LAST_SIGN_SELECT						(0x15)
#define KEY5_LAST_NOSIGN_SELECT						(0x14)

#define KEY5_FIRST_SIGN_RATE						(0x1B)
#define KEY5_FIRST_NOSIGN_RATE						(0x1A)
#define KEY5_LAST_SIGN_RATE							(0x19)
#define KEY5_LAST_NOSIGN_RATE						(0x18)

#define KEY2_FIRST_SIGN_CUSTOM						(0x1C)
#define KEY2_FIRST_NOSIGN_CUSTOM					(0x1D)
#define KEY2_LAST_SIGN_CUSTOM						(0x1E)
#define KEY2_LAST_NOSIGN_CUSTOM						(0x1F)

#define KEY3_FIRST_SIGN_CUSTOM						(0x20)
#define KEY3_FIRST_NOSIGN_CUSTOM					(0x21)
#define KEY3_LAST_SIGN_CUSTOM						(0x22)
#define KEY3_LAST_NOSIGN_CUSTOM						(0x23)

#define KEY4_FIRST_SIGN_CUSTOM						(0x30)
#define KEY4_FIRST_NOSIGN_CUSTOM					(0x31)
#define KEY4_LAST_SIGN_CUSTOM						(0x32)
#define KEY4_LAST_NOSIGN_CUSTOM						(0x33)

#define KEY5_FIRST_SIGN_CUSTOM						(0x34)
#define KEY5_FIRST_NOSIGN_CUSTOM					(0x35)
#define KEY5_LAST_SIGN_CUSTOM						(0x36)
#define KEY5_LAST_NOSIGN_CUSTOM						(0x37)

#define KEY3_FIRST_SIGN_SATISFACTION	  			(0x3D)
#define KEY3_FIRST_NOSIGN_SATISFACTION 				(0x3C)
#define KEY3_LAST_SIGN_SATISFACTION	  				(0x3B)
#define KEY3_LAST_NOSIGN_SATISFACTION  				(0x3A)

/* ǰ���ֶ� 	BASIC_MSG  UNIT_CTRL */
#define SET_UNTI_EQ									(0x40)
#define GAIN_UNIT_EQ								(0x41)
#define SET_UNIT_SENSITIVITY						(0x43)
#define GAIN_UNIT_SENSITIVITY						(0x44)
#define SET_VOICE_CTRL_SENSITIVITY					(0x53)			
#define SET_VOICE_CTRL_CLOSE_TIME					(0x54)
#define SET_LANGUAGE(_lan)							(0xF0 + _lan)

/* ǰ���ֶ� 	POLLING_MSG  SIGN_POLLING */
#define HOST_SIGN_POLLIGN							(0x52)

/* ǰ���ֶ� 	POLLING_MSG  EDIT_ID_POLLING */
#define REPEATED_ID									(0x60)
#define CURRENT_ID									(0x61)
#define CONFIRM_ID									(0x62)

/* ǰ���ֶ� 	POLLING_MSG  VOTE_POLLING  */
#define  POLLING_VOTE       						(0x53)               //�����ѯ�루������������һ����Ч������Ҫǩ���� 
#define  POLLING_VOTE_SIGN  						(0x54)               //�����ѯ�루������������һ����Ч����Ҫǩ����
#define  POLLING_VOTE_F     						(0x55)               //�����ѯ�� �������������һ����Ч������Ҫǩ���� 
#define  POLLING_VOTE_FSIGN 						(0x56)               //�����ѯ�� �������������һ����Ч����Ҫǩ����
#define  POLLING_ELECT								(0x57)               //�����ѯ�루���ѡ�٣����һ����Ч������Ҫǩ����
#define  POLLING_ELECT_SIGN							(0x58)               //�����ѯ�� �����ѡ�٣���һ����Ч����Ҫǩ����
#define  POLLING_ELECT_F							(0x59)               //�����ѯ�� �����ѡ�٣���һ����Ч������Ҫǩ����
#define  POLLING_ELECT_FSIGN 						(0x5A)               //�����ѯ�루���ѡ�٣����һ����Ч����Ҫǩ����
#define  POLLING_RATE								(0x5B)               //�����ѯ�루������������һ����Ч������Ҫǩ����
#define  POLLING_RATE_SIGN							(0x5C)               //�����ѯ�� �������������һ����Ч����Ҫǩ����
#define  POLLING_RATE_F								(0x5D)               //�����ѯ�� �������������һ����Ч������Ҫǩ����
#define  POLLING_RATE_FSIGN							(0x5E)               //�����ѯ�루������������һ����Ч����Ҫǩ����

#define  POLLING_VOTECUSTOM_F_2_S					(0x60)               //�����Զ���������һ����Ч����Ҫǩ��
#define  POLLING_VOTECUSTOM_F_2						(0x61)               //�����Զ���������һ����Ч������Ҫǩ��
#define  POLLING_VOTECUSTOM_L_2_S					(0x62)               //�����Զ����������һ����Ч����Ҫǩ��
#define  POLLING_VOTECUSTOM_L_2						(0x63)               //�����Զ����������һ����Ч������Ҫǩ��
#define  POLLING_VOTECUSTOM_F_3_S					(0x64)               //�����Զ���������һ����Ч����Ҫǩ��
#define  POLLING_VOTECUSTOM_F_3						(0x65)               //�����Զ���������һ����Ч������Ҫǩ��
#define  POLLING_VOTECUSTOM_L_3_S					(0x66)               //�����Զ����������һ����Ч����Ҫǩ��
#define  POLLING_VOTECUSTOM_L_3						(0x67)               //�����Զ����������һ����Ч������Ҫǩ��
#define  POLLING_VOTECUSTOM_F_4_S					(0x68)               //�ļ��Զ���������һ����Ч����Ҫǩ��
#define  POLLING_VOTECUSTOM_F_4						(0x69)               //�ļ��Զ���������һ����Ч������Ҫǩ��
#define  POLLING_VOTECUSTOM_L_4_S					(0x6A)               //�ļ��Զ����������һ����Ч����Ҫǩ��
#define  POLLING_VOTECUSTOM_L_4						(0x6B)               //�ļ��Զ����������һ����Ч������Ҫǩ��
#define  POLLING_VOTECUSTOM_F_5_S					(0x6C)               //����Զ���������һ����Ч����Ҫǩ��
#define  POLLING_VOTECUSTOM_F_5						(0x6D)               //����Զ���������һ����Ч������Ҫǩ��
#define  POLLING_VOTECUSTOM_L_5_S					(0x6E)               //����Զ����������һ����Ч����Ҫǩ��
#define  POLLING_VOTECUSTOM_L_5						(0x6F)               //����Զ����������һ����Ч������Ҫǩ��

#define  POLLING_SATISFACTION						(0x70)               //�����ѯ�루��������ȣ����һ����Ч������Ҫǩ����
#define  POLLING_SATISFACTION_SIGN					(0x71)               //�����ѯ�� ����������ȣ���һ����Ч����Ҫǩ����
#define  POLLING_SATISFACTION_F						(0x72)               //�����ѯ�� ����������ȣ���һ����Ч������Ҫǩ����
#define  POLLING_SATISFACTION_FSIGN					(0x73)               //�����ѯ�루��������ȣ����һ����Ч����Ҫǩ����


/* ǰ���ֶ�      PC_MSG PC_EDIT_ID */
#define PC_START_EDIT_ID							(0x01)
#define PC_STOP_EDIT_ID								(0x02)

/* ǰ���ֶ� 	PC_MSG  SCAN_ONLINE_UNIT */
#define SCAN_UNIT_END								(0x01)




/*******************************************************************************
 ***                        WIFI��ԪͨѶЭ��                           ***
 *******************************************************************************/
#define MasterStarUp_MtoU_G 						(0)  	//�����㲥����\�ػ� ��Ԫ���յ������ػ�ָ�����Զ��ػ�
#define ApplyEnterSystm_UtoM_D 						(1)  	//��Ԫ���������ϵͳ
#define RepApplyEnterSystm_MtoU_D 					(2)  	//�ظ���Ԫ���������ϵͳ
#define EnterMeetingMode_MtoU_G 					(3)  	//�����㲥����Ԫ���������ģʽ
#define ApplyOpenMic_UtoM_D 						(4)  	//��Ԫ�����뿪��Ͳָ��
#define RepApplyOpenMic_MtoU_D 						(5) 	//�ظ���Ԫ��������뿪��Ͳ
#define UnitCloseMic_UtoM_D 						(6)  	//��Ԫ���رջ�Ͳ
#define MasterOpenOrCloseMic_MtoU_D 				(7)  	//������\�رյ�Ԫ����Ͳ������ר�ã�
#define UnitGetoutWaitQuenen_UtoM_D 				(8)  	//��Ԫ���˳��ȴ�����
#define ChairExecutePriority_UtoM_D 				(9)  	//��ϯ��ִ������Ȩ����
#define RepChairExecutePriority_MtoU_D 				(10)  	//�ظ���ϯ��ִ������Ȩ����
#define RepresentApplyOpenMic_MtoU_G 				(11) 	//�����㲥����ϯ���д�������뿪��Ͳ
#define ChairAgreeOpenOrNot_UtoM_D 					(12) 	//��ϯ��ͬ��\��ͬ�⿪��Ͳ��  ��ϯ�����ݽ����05��ָ��������    
#define MasterEnableMicOrNot_MtoU_D 				(13)  	//����ʹ��\��ֹ��Ԫ���򿪻�Ͳ
#define AlarmOrNot_MtoU_G 							(14)  	//�����㲥����Ԫ������\��������
#define EnterSpeakWithTimeOrNot_MtoU 				(15)  	//����\�˳����Լ�ʱ
#define TimeLimitOrNot_MtoU 						(16)  	//�����㲥\��������Ԫ������\�˳�������ʱ
#define UnitApplyWater_UtoM_D 						(17)  	//��Ԫ����ˮ����   
#define TranslatorAskSlow_UtoM_D					(18)  	//��Ա��Ҫ���������  
#define EnterSignMode_MtoU_G						(19)  	//�����㲥����Ԫ������ǩ��ģʽ
#define EnableSignOrNot_MtoU_D						(20)  	//��ֹ\ʹ�ܵ�Ԫ��ǩ��
#define SupplementSign_MtoU_D 						(21)  	//�����ﵥԪ����ǩ��
#define RepSupplementSign_UtoM_D 					(22) 	//�ظ������ﵥԪ����ǩ��
#define ChairStarupOrEndSign_UtoM_D 				(23)  	//��ϯ����������ǩ��
#define RepChairStarupOrEndSign_MtoU_D				(24)  	//�ظ���ϯ������ǩ��
#define EnterVoteMode_MtoU_G 						(25)	//�����㲥����Ԫ������ǩ�����ģʽ
#define ChairStarupOrEndVote_UtoM_D 				(26)  	//��ϯ��������
#define RepChairStarupOrEndVote_MtoU_D 				(27)  	//�ظ���ϯ��������
#define ChairSuspendVote_UtoM_D 			        (28)  	//��ϯ����ͣ�������Ҫ�������㲥������
#define EnableVoteOrNot_MtoU_D			  	        (29)  	//��ֹ\ʹ�ܵ�Ԫ�����
#define SetLanguage_MtoU_G							(30)  	//���õ�Ԫ����ʾ����
#define SetUnitEQ_MtoU_G				            (31)  	//���ñ���EQ
#define SetandSaveUnitEQ_MtoU_D 		            (32)  	//���ñ���EQ
#define ReadUnitEQ_MtoU_D			               	(33)  	//������ȡEQ
#define RepReadUnitEQ_UtoM_D						(34)  	//�ظ�������ȡEQ
#define RepReadUnitEQ_MtoU_D 						(35)  	//���õ�Ԫ����ͨ�˲���
#define SetUnitMICSensitivity_MtoU_D				(36)  	//���õ�Ԫ��MIC������
#define RepReadUnitMICSensitivity_UtoM_D 			(37)  	//�ظ�PC��ȡMIC������
#define SetUnitMICEQ_MtoU_D 						(38)  	//PC���õ�Ԫ��EQ
#define SetUnitChannel_MtoU_G 						(39)  	//���õ�Ԫ��ͨ����
#define PollUnitl_MtoU_D 							(40)  	//������ѯ
#define RepPollUnitl_UtoM_D 						(41)  	//��ѯӦ�𣨻���ģʽpara1�߰��ֽ���Ч ��ǩ��ģʽpara1�װ��ֽ���Ч�����ģʽpara2��Ч ��
#define SetUnitPrintMsg_MtoU_G						(42)	//���õ�Ԫ����ʾ��\����Ϣ
#define SetUnitPrintFixMsg_MtoU_G			 		(43)  	//���õ�Ԫ����ʾ�̶���Ϣ 1���������㣬�뵽�������ſ�     2���������㣬�뵽����̨
#define EnterEditingIDMtoU_G						(44)	//�����IDģʽ
#define UnitGraspIDUtoM_D							(45)	//��Ԫ���Ե�ǰ�㲥��ID��Ϊ����ID
#define IDRepeatingMtoU_G							(46)	//�㲥��ID�ظ�
#define MasterAskforReenterSysMtoU_D				(47) 	//����Ҫ��Ԫ�����½���ϵͳ
#define UnitCapacityStrChangeUtoM_D					(48) 	//��Ԫ��֪ͨ��������ֻ��һ��
#define ControlSign_MtoU_D							(49) 	//��������ǩ��
#define ChangeMicManage_MtoU_G						(50)	//�����㲥����Ԫ���ı仰Ͳ����ģʽ
#define PCUpdateCustomTerm_MtoU_G					(51)	//PC�·�����Զ���ѡ��
#define ConfirmUnitIDMtoU_D							(52)  	//�յ���Ԫ���Ե�ǰ�㲥��ID��Ϊ����ID��ȷ���ֶ�
#define SetVoiceSensitivity_MtoU_D					(60)	//��������������
#define SetVoiceCloseTime_MtoU_D					(61)	//���������Զ��ر�ʱ��
#define ResetAllUnit_MtoU_G							(63)	//���������еĵ�Ԫ�� (20181019)	
#define RealTimeMtoU_G								(80) 	//�·���ʵʱ�䵽���ߵ�Ԫ 20190827
#define MasterTransmissionMtoU_D             		(250) 	//PC�·�͸��ָ��


/* ͶƱ/���/����/����� ���� */
typedef enum {
    Key3First_Sign_vote 	= 0,
    Key3First_NoSign_vote   = 1,
    Key3Last_Sign_vote     	= 2,
    Key3Last_NoSign_vote	= 3,

    Key5First_Sign_Select	= 4,
    Key5First_NoSign_Select	= 5,
    Key5Last_Sign_Select	= 6,
    Key5Last_NoSign_Select	= 7,

    Key5First_Sign_Rate		= 8,
    Key5First_NoSign_Rate	= 9,
    Key5Last_Sign_Rate		= 10,
    Key5Last_NoSign_Rate	= 11,

    Key2First_Sign_CustomTerm	= 12,
    Key2First_NoSign_CustomTerm	= 13,
    Key2Last_Sign_CustomTerm	= 14,
    Key2Last_NoSign_CustomTerm	= 15,

    Key3First_Sign_CustomTerm	= 16,
    Key3First_NoSign_CustomTerm	= 17,
    Key3Last_Sign_CustomTerm	= 18,
    Key3Last_NoSign_CustomTerm	= 19,

    Key4First_Sign_CustomTerm	= 20,
    Key4First_NoSign_CustomTerm	= 21,
    Key4Last_Sign_CustomTerm	= 22,
    Key4Last_NoSign_CustomTerm	= 23,

    Key5First_Sign_CustomTerm	= 24,
    Key5First_NoSign_CustomTerm	= 25,
    Key5Last_Sign_CustomTerm	= 26,
    Key5Last_NoSign_CustomTerm	= 27,

    Key3First_Sign_Satisfaction	  = 28,
    Key3First_NoSign_Satisfaction = 29,
    Key3Last_Sign_Satisfaction	  = 30,
    Key3Last_NoSign_Satisfaction  = 31,

	VotePause = 255,
} VoteMode_EN;

/*******************************************************************************
 ***                        �����Ļ�����ģ�ͨѶЭ��               			             ***
 *******************************************************************************/
/* ��Ļҳ�� Screen page */
#define SP_START_UP 										(00)               //1��������
#define SP_WELCOME											(01)               //2��ӭ����
#define SP_MAIN_MENU 										(02)               //������
#define SP_CONF_MODE										(03)               //3����ģʽ
#define SP_SET_ID 											(04)               //4��ʼ��ID
#define SP_SET_ID_END 										(05)               //5������ID
#define SP_ID_PEPEAT  										(06)               //6ID�ظ�
#define SP_RESET_ID	 										(07)               //ID�ظ����ID
//#define SP_EndSetID 										(08)               //������ID
//#define SP_Hard_SetID 									(09)               //Ӳ����ID
//#define SP_StartTranSetID 								(10)               //��Ա����ʼ��ID
//#define SP_EndTranSetID									(11)               //��Ա��������ID
#define SP_SYS_STATE 										(12)               //ϵͳ״̬
#define SP_SYS_SETTING 										(13)               //ϵͳ����
#define SP_SET_LANGUAGE										(14)               //ϵͳ����-��������
#define SP_SET_DISPLAY 										(15)               //ϵͳ����-��ʾ����
#define SP_SET_VOLUME 										(16)               //ϵͳ����-��������
#define	SP_RESTORE_DEF										(17)               //ϵͳ����-�ָ�Ĭ��
#define SP_LOCAL_IP 										(18)               //ϵͳ����-����IP
//#define SP_ID_MODE										(19)               //ϵͳ����-��IDģʽ
#define	SP_SET_WIFI_UNIT_OFF								(20)               //ϵͳ����-���ߵ�Ԫ�ػ�
#define SP_SET_DOWN_TRANS									(21)               //ϵͳ����-�´�����
//#define SP_AUD_PLAYER_PAUSE									(22)               //��ͣ
//#define SP_AUD_PLAYER_PLAY									(23)               //����
//#define SP_AUD_RECORD										(24)               //¼������
#define SP_AUD_RECORD_WAIT									(25)               //�ȴ�¼����������Ӧ
#define SP_ERROR											(26)               //�������
#define SP_EX_CTRL_CONNECT 									(27)               //�ⲿ����������
//#define SP_Hardware_Error 								(28)               //Ӳ���жϴ���
//#define SP_Restore_default_OK								(29)               //�ָ�Ĭ�ϳɹ�
//#define SP_EEPROM_Error 									(30)               //δ�����EEPROM
//#define SP_PT2315_Error 									(31)               //��Ƶͨ��������
//#define SP_FPGA_Error 									(32)               //FPGA�����쳣						
//#define SP_W5100_Error 									(33)               //5100�����쳣						
//#define SP_PT2315_Error									(34)               //
#define SP_SIGNING_IN 										(35)               //Ӳ��ǩ��������
#define SP_VOTING 											(36)               //Ӳ�����������
#define SP_FIRE_WARNING 									(37)               //��������
#define SP_BATTERY_ERR 										(38)               //���ûװ��
#define SP_TRIAL_EXPIRED									(39)               //�������ѵ�		
#define SP_USB_INIT											(40)               //U�̳�ʼ��
#define SP_UNIT_TYPE										(41)               //��������

#define SP_AUD_PLAY_OR_RECORD								(44)               //��Ƶ���Ż�¼��ѡ��
#define SP_AUD_PLAYER_PLAYING								(45)               //������ ���� ���ڲ���
#define SP_AUD_PLAY_STOPED									(46)               //������ ���� ��ֹͣ
#define SP_AUD_RECORD_STOPED								(47)               //¼�� ���� ��ֹͣ
#define SP_AUD_RECORDING									(48)               //¼�� ���� ����¼��



/* ��Ļ�����Ĵ��� Screen variable  register */
#define SVR_WIFI_OL_TOTAL_NUM										(0x0001)			//���߻�Ͳ��������
#define SVR_WIFI_OL_CHM_NUM											(0x0002)			//
#define SVR_WIFI_OL_RPS_NUM											(0x0003)

#define SVR_CONF_MODE												(0x0005)
#define SVR_SET_MIC_MODE											(0x0012)
#define SVR_SET_MIC_NUM												(0x0014)

#define SVR_START_ID												(0x0022)

#define SVR_WIRED_OL_TOTAL_NUM										(0x0034)
#define SVR_WIRED_OL_CHM_NUM										(0x0035)
#define SVR_WIRED_OL_RPS_NUM										(0x0036)
#define SVR_INTERP_OL_NUM											(0x0037)
#define SVR_DOWN_TRANSMIT											(0x0041)
#define SVR_BRIGHTNESS												(0x0044)

#define SVR_IP_ADDR													(0x0048)
#define SVR_GATEWAY_ADDR											(0x0058)
#define SVR_NETMASK_ADDR											(0x0068)
#define SVR_PORT													(0x0078)

#define SVR_SOFTWARE_VERSION_M										(0x0039)
#define SVR_SOFTWARE_VERSION_S										(0x0080)

#define SVR_LINEOUT1_VOL											(0x0092)
#define SVR_LINEOUT2_VOL											(0x0094)

#define SVR_AUD_PLAY_NAME											(0x03A0)
#define SVR_AUD_REC_ICO												(0x0505)
#define SVR_AUD_REC_PLAY_TIME										(0x06A0)


/* ��ĻͨѶ���� */
typedef enum {
	/* �л�ҳ�� */
	tType_Screen_Page = 0x80,
	/* ��ѯҳ�� */
	tType_Screen_Query = 0x81,
	/* ���üĴ�����ֵ */
	tType_Screen_CfgReg = 0x82,
	/* ��Ļ���淵��ֵ */
	tType_Screen_InterfaceRet = 0x83,
}ScreenProtType_EN;


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
} ConfProtocol_S;
#pragma pack()

/*  WIFI��ԪͨѶЭ�����ݽṹ(�����ṹ) */
#pragma pack(1)	/* ���ֽڶ��� */
typedef struct {
    uint16_t id;
    uint8_t cmd;
    uint8_t ph;
    uint8_t pl;
} WifiUnitProtocol_S;
#pragma pack()


/* �����ĻͨѶЭ�飨����Э�飩���ݽ�����������ṹ�� */
#pragma pack(1)	/* ���ֽڶ��� */
typedef struct {
    ScreenProtType_EN type;
	uint8_t para[5];
} ScreenProtocol_S;
#pragma pack()




typedef struct {
    ConfProtocol_S *(*conference)(ConfProtocol_S *,uint16_t,uint8_t,uint8_t,uint8_t,uint8_t,uint16_t );
    WifiUnitProtocol_S *(*wifiUnit)(WifiUnitProtocol_S *, uint16_t, uint8_t,uint8_t,uint8_t);
	ScreenProtocol_S *(*screen)(ScreenProtocol_S *prot,ScreenProtType_EN type,uint16_t reg);
} Protocol_S;

extern Protocol_S Protocol;
#endif



