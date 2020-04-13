#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "stdint.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 ***                        全数字会议系统通讯协议                           ***
 *******************************************************************************/

/* 全数字会议协议数据包最小长度 */
#define CONF_PROT_MIN_LEN							(13)

/************ 广播ID ****************/
#define WHOLE_BROADCAST_ID							(0xFFF0)			//广播全体机型
#define UNIT_BROADCAST_ID							(0xFFF1)			//仅对单元机广播消息
#define EX_HOST_BROADCAST_ID						(0xFFF2)			//仅对扩展主机广播消息
#define TRANS_BROADCAST_ID							(0xFFF3)			//仅对议员机广播消息
#define MODE_BROADCAST_ID							(0xFFE1)			//广播当前主机话筒模式（有线单元）
#define NUM_BROADCAST_ID							(0xFFE2)			//广播会场总数量、当前开话筒数量
#define NOW_DATE_TIME_ID							(0xFFEC)			//PC广播下发时间					
#define WIFI_MODE_BROADCAST_ID						(0xFFEA)			//广播当前主机话筒模式（WIFI单元）
#define APPLY_OPEN_MIC_ID							(0xFFC1)			//向主席机广播当前申请开话筒ID
#define REVOKE_APPLY_ID								(0xFFC2)			//向主席机广播当前撤销申请话筒ID

/* 结果消息(0x83)相关ID */
//#define RESULT_MSG_ID(_id)							(0xFFD0 + _id)		 //结果消息ID
//#define RES_SIGN_NUM								(0x01)				 //签到总人数
//#define RES_OPT_NUM(n)								(0x01 + n)			 //第n项选项人数

/************ 类型(type) ************/
#define BASIC_MSG									(0x80)				//基本消息
#define POLLING_MSG									(0x81)				//轮询消息
#define STATE_MSG									(0x82)				//状态消息
#define RESULT_MSG									(0x83)				//结果消息
#define APPLY_MSG									(0x84)				//申请消息
#define EXTEND_MSG									(0x85)				//扩展统计消息
#define PC_MSG										(0x86)				//PC通讯消息
#define WIRED_UNIT_NUM								(0x87)				//有线单元数量
#define WIFI_UNIT_NUM								(0x88)				//无线单元数量
#define NAMEPLATE_PARA								(0x90)				//电子铭牌参数
#define CONFERENCE_MSG								(0x96)				//会议信息（会议内容、摘要等）
#define NAMEPLATE_COMP								(0x97)				//PC下发电子铭牌内容（公司名称）
#define NAMEPLATE_POS								(0x98)				//PC下发电子铭牌内容（职位）
#define NAMEPLATE_CONTENT							(0x99)				//PC下发电子铭牌内容（旧电子铭牌协议，用于兼容）
#define USER_LIST									(0x9A)				//用户列表（ID和参会人的对应列表）
#define CONSTANT_NOTIFY								(0x9B)				//PC下发的固定通知消息（有事找您，xxxx）
#define NAMEPLATE_NAME								(0x9C)				//PC下发电子铭牌内容（姓名）
#define NAMEPLATE_UPDATE							(0x9D)				//PC单播更新电子铭牌（旧电子铭牌协议，用于兼容）
#define HOST_REPLY_PC								(0x9E)				//主机回复PC发送成功
#define AUDIO_MATRIX								(0xB0)				//音频矩阵参数配置
#define AUDIO_MATRIX_INQUIRE						(0xB1)              //音频矩阵参数查询


/************ 参数ph(para high) ************/
/* 前置字段 	BASIC_MSG */
#define CONFERENCE_MODE								(0x00)
#define SIGN_MODE									(0x01)
#define VOTE_MODE									(0x02)
#define EDIT_ID_MODE								(0x03)
#define UNIT_CTRL									(0x04)

/* 前置字段 RESULT_MSG */
#define SIGN_RESULT									(0x01)
#define VOTE_RESULT									(0x02)

/* 前置字段 	PC_MSG */
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

/* 前置字段 	POLLING_MSG */
#define SIGN_POLLING								(0x06)
#define VOTE_POLLING								(0x07)
#define EDIT_ID_POLLING								(0x08)

/* 前置字段 AUDIO_MATRIX 或 AUDIO_MATRIX_INQUIRE */
#define DSP_MODE									(0x00)
#define DSP_UNIT_OUT_CFG							(0x01)
#define DSP_NOR_OUT_CFG								(0x02)
#define DSP_CHANNEL_OUT_CFG							(0x03)



/************ 参数pl(para low) ************/
/* 前置字段 	BASIC_MSG  CONFERENCE_MODE */
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


/* 前置字段 	BASIC_MSG  SIGN_MODE */
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

/* 前置字段 	BASIC_MSG  EDIT_ID_MODE */
#define START_EDIT_ID_MODE							(0x80)
#define END_EDIT_ID_MODE							(0x81)
#define UNIT_CONFIRM_ID								(0x86)

/* 前置字段 	BASIC_MSG  VOTE_MODE */
#define PAUSE_VOTE									(0x07)				//暂停表决
#define FINISH_VOTE									(0x08)				//结束表决
#define DEV_LAUNCH_VOTE								(0x2E)				//主席机发起表决
#define DEV_FINISH_VOTE								(0x2F)				//主席机结束表决
#define VOTE_ENABLE									(0x38)
#define VOTE_DISABLE								(0x39)

#define KEY3_FIRST_SIGN_VOTE 						(0x13)				// 投票/表决/评级/满意度 类型
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

/* 前置字段 	BASIC_MSG  UNIT_CTRL */
#define SET_UNTI_EQ									(0x40)
#define GAIN_UNIT_EQ								(0x41)
#define SET_UNIT_SENSITIVITY						(0x43)
#define GAIN_UNIT_SENSITIVITY						(0x44)
#define SET_VOICE_CTRL_SENSITIVITY					(0x53)			
#define SET_VOICE_CTRL_CLOSE_TIME					(0x54)
#define SET_LANGUAGE(_lan)							(0xF0 + _lan)

/* 前置字段 	POLLING_MSG  SIGN_POLLING */
#define HOST_SIGN_POLLIGN							(0x52)

/* 前置字段 	POLLING_MSG  EDIT_ID_POLLING */
#define REPEATED_ID									(0x60)
#define CURRENT_ID									(0x61)
#define CONFIRM_ID									(0x62)

/* 前置字段 	POLLING_MSG  VOTE_POLLING  */
#define  POLLING_VOTE       						(0x53)               //表决轮询码（三键表决，最后一次有效，不需要签到） 
#define  POLLING_VOTE_SIGN  						(0x54)               //表决轮询码（三键表决，最后一次有效，需要签到）
#define  POLLING_VOTE_F     						(0x55)               //表决轮询码 （三键表决，第一次有效，不需要签到） 
#define  POLLING_VOTE_FSIGN 						(0x56)               //表决轮询码 （三键表决，第一次有效，需要签到）
#define  POLLING_ELECT								(0x57)               //表决轮询码（五键选举，最后一次有效，不需要签到）
#define  POLLING_ELECT_SIGN							(0x58)               //表决轮询码 （五键选举，第一次有效，需要签到）
#define  POLLING_ELECT_F							(0x59)               //表决轮询码 （五键选举，第一次有效，不需要签到）
#define  POLLING_ELECT_FSIGN 						(0x5A)               //表决轮询码（五键选举，最后一次有效，需要签到）
#define  POLLING_RATE								(0x5B)               //表决轮询码（五键评级，最后一次有效，不需要签到）
#define  POLLING_RATE_SIGN							(0x5C)               //表决轮询码 （五键评级，第一次有效，需要签到）
#define  POLLING_RATE_F								(0x5D)               //表决轮询码 （五键评级，第一次有效，不需要签到）
#define  POLLING_RATE_FSIGN							(0x5E)               //表决轮询码（五键评级，最后一次有效，需要签到）

#define  POLLING_VOTECUSTOM_F_2_S					(0x60)               //两键自定义表决，第一次有效，需要签到
#define  POLLING_VOTECUSTOM_F_2						(0x61)               //两键自定义表决，第一次有效，不需要签到
#define  POLLING_VOTECUSTOM_L_2_S					(0x62)               //两键自定义表决，最后一次有效，需要签到
#define  POLLING_VOTECUSTOM_L_2						(0x63)               //两键自定义表决，最后一次有效，不需要签到
#define  POLLING_VOTECUSTOM_F_3_S					(0x64)               //三键自定义表决，第一次有效，需要签到
#define  POLLING_VOTECUSTOM_F_3						(0x65)               //三键自定义表决，第一次有效，不需要签到
#define  POLLING_VOTECUSTOM_L_3_S					(0x66)               //三键自定义表决，最后一次有效，需要签到
#define  POLLING_VOTECUSTOM_L_3						(0x67)               //三键自定义表决，最后一次有效，不需要签到
#define  POLLING_VOTECUSTOM_F_4_S					(0x68)               //四键自定义表决，第一次有效，需要签到
#define  POLLING_VOTECUSTOM_F_4						(0x69)               //四键自定义表决，第一次有效，不需要签到
#define  POLLING_VOTECUSTOM_L_4_S					(0x6A)               //四键自定义表决，最后一次有效，需要签到
#define  POLLING_VOTECUSTOM_L_4						(0x6B)               //四键自定义表决，最后一次有效，不需要签到
#define  POLLING_VOTECUSTOM_F_5_S					(0x6C)               //五键自定义表决，第一次有效，需要签到
#define  POLLING_VOTECUSTOM_F_5						(0x6D)               //五键自定义表决，第一次有效，不需要签到
#define  POLLING_VOTECUSTOM_L_5_S					(0x6E)               //五键自定义表决，最后一次有效，需要签到
#define  POLLING_VOTECUSTOM_L_5						(0x6F)               //五键自定义表决，最后一次有效，不需要签到

#define  POLLING_SATISFACTION						(0x70)               //表决轮询码（三键满意度，最后一次有效，不需要签到）
#define  POLLING_SATISFACTION_SIGN					(0x71)               //表决轮询码 （三键满意度，第一次有效，需要签到）
#define  POLLING_SATISFACTION_F						(0x72)               //表决轮询码 （三键满意度，第一次有效，不需要签到）
#define  POLLING_SATISFACTION_FSIGN					(0x73)               //表决轮询码（三键满意度，最后一次有效，需要签到）


/* 前置字段      PC_MSG PC_EDIT_ID */
#define PC_START_EDIT_ID							(0x01)
#define PC_STOP_EDIT_ID								(0x02)

/* 前置字段 	PC_MSG  SCAN_ONLINE_UNIT */
#define SCAN_UNIT_END								(0x01)




/*******************************************************************************
 ***                        WIFI单元通讯协议                           ***
 *******************************************************************************/
#define MasterStarUp_MtoU_G 						(0)  	//主机广播开机\关机 单元机收到主机关机指令后会自动关机
#define ApplyEnterSystm_UtoM_D 						(1)  	//单元机申请进入系统
#define RepApplyEnterSystm_MtoU_D 					(2)  	//回复单元机申请进入系统
#define EnterMeetingMode_MtoU_G 					(3)  	//主机广播给单元机进入会议模式
#define ApplyOpenMic_UtoM_D 						(4)  	//单元机申请开话筒指令
#define RepApplyOpenMic_MtoU_D 						(5) 	//回复单元机申请进入开话筒
#define UnitCloseMic_UtoM_D 						(6)  	//单元机关闭话筒
#define MasterOpenOrCloseMic_MtoU_D 				(7)  	//主机打开\关闭单元机话筒（主机专用）
#define UnitGetoutWaitQuenen_UtoM_D 				(8)  	//单元机退出等待队列
#define ChairExecutePriority_UtoM_D 				(9)  	//主席机执行优先权命令
#define RepChairExecutePriority_MtoU_D 				(10)  	//回复主席机执行优先权命令
#define RepresentApplyOpenMic_MtoU_G 				(11) 	//主机广播给主席机有代表机申请开话筒
#define ChairAgreeOpenOrNot_UtoM_D 					(12) 	//主席机同意\不同意开话筒；  主席机根据结果发05号指令给代表机    
#define MasterEnableMicOrNot_MtoU_D 				(13)  	//主机使能\禁止单元机打开话筒
#define AlarmOrNot_MtoU_G 							(14)  	//主机广播给单元机报警\撤销报警
#define EnterSpeakWithTimeOrNot_MtoU 				(15)  	//进入\退出发言计时
#define TimeLimitOrNot_MtoU 						(16)  	//主机广播\单播给单元机进入\退出发言限时
#define UnitApplyWater_UtoM_D 						(17)  	//单元机茶水申请   
#define TranslatorAskSlow_UtoM_D					(18)  	//译员机要求放慢语速  
#define EnterSignMode_MtoU_G						(19)  	//主机广播给单元机进入签到模式
#define EnableSignOrNot_MtoU_D						(20)  	//禁止\使能单元机签到
#define SupplementSign_MtoU_D 						(21)  	//主机帮单元机补签到
#define RepSupplementSign_UtoM_D 					(22) 	//回复主机帮单元机补签到
#define ChairStarupOrEndSign_UtoM_D 				(23)  	//主席机发起或结束签到
#define RepChairStarupOrEndSign_MtoU_D				(24)  	//回复主席机发起签到
#define EnterVoteMode_MtoU_G 						(25)	//主机广播给单元机进入签到表决模式
#define ChairStarupOrEndVote_UtoM_D 				(26)  	//主席机发起表决
#define RepChairStarupOrEndVote_MtoU_D 				(27)  	//回复主席机发起表决
#define ChairSuspendVote_UtoM_D 			        (28)  	//主席机暂停表决，并要求主机广播表决结果
#define EnableVoteOrNot_MtoU_D			  	        (29)  	//禁止\使能单元机表决
#define SetLanguage_MtoU_G							(30)  	//设置单元机显示语言
#define SetUnitEQ_MtoU_G				            (31)  	//设置保存EQ
#define SetandSaveUnitEQ_MtoU_D 		            (32)  	//设置保存EQ
#define ReadUnitEQ_MtoU_D			               	(33)  	//主机读取EQ
#define RepReadUnitEQ_UtoM_D						(34)  	//回复主机读取EQ
#define RepReadUnitEQ_MtoU_D 						(35)  	//设置单元机高通滤波器
#define SetUnitMICSensitivity_MtoU_D				(36)  	//设置单元机MIC灵敏度
#define RepReadUnitMICSensitivity_UtoM_D 			(37)  	//回复PC读取MIC灵敏度
#define SetUnitMICEQ_MtoU_D 						(38)  	//PC设置单元机EQ
#define SetUnitChannel_MtoU_G 						(39)  	//设置单元机通道数
#define PollUnitl_MtoU_D 							(40)  	//主机轮询
#define RepPollUnitl_UtoM_D 						(41)  	//轮询应答（会议模式para1高半字节有效 ；签到模式para1底半字节有效；表决模式para2有效 ）
#define SetUnitPrintMsg_MtoU_G						(42)	//设置单元机显示短\长消息
#define SetUnitPrintFixMsg_MtoU_G			 		(43)  	//设置单元机显示固定消息 1：有事找你，请到会议室门口     2：有事找你，请到控制台
#define EnterEditingIDMtoU_G						(44)	//进入编ID模式
#define UnitGraspIDUtoM_D							(45)	//单元机以当前广播的ID作为本机ID
#define IDRepeatingMtoU_G							(46)	//广播有ID重复
#define MasterAskforReenterSysMtoU_D				(47) 	//主机要求单元机重新进入系统
#define UnitCapacityStrChangeUtoM_D					(48) 	//单元机通知主机电量只有一格
#define ControlSign_MtoU_D							(49) 	//主机控制签到
#define ChangeMicManage_MtoU_G						(50)	//主机广播给单元机改变话筒管理模式
#define PCUpdateCustomTerm_MtoU_G					(51)	//PC下发表决自定义选项
#define ConfirmUnitIDMtoU_D							(52)  	//收到单元机以当前广播的ID作为本机ID后确认字段
#define SetVoiceSensitivity_MtoU_D					(60)	//设置声控灵敏度
#define SetVoiceCloseTime_MtoU_D					(61)	//设置声控自动关闭时间
#define ResetAllUnit_MtoU_G							(63)	//主重置所有的单元机 (20181019)	
#define RealTimeMtoU_G								(80) 	//下发真实时间到无线单元 20190827
#define MasterTransmissionMtoU_D             		(250) 	//PC下发透传指令


/* 投票/表决/评级/满意度 类型 */
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
 ***                        大彩屏幕（迪文）通讯协议               			             ***
 *******************************************************************************/
/* 屏幕页面 Screen page */
#define SP_START_UP 										(00)               //1开机动画
#define SP_WELCOME											(01)               //2欢迎界面
#define SP_MAIN_MENU 										(02)               //主界面
#define SP_CONF_MODE										(03)               //3会议模式
#define SP_SET_ID 											(04)               //4开始编ID
#define SP_SET_ID_END 										(05)               //5结束编ID
#define SP_ID_PEPEAT  										(06)               //6ID重复
#define SP_RESET_ID	 										(07)               //ID重复后编ID
//#define SP_EndSetID 										(08)               //结束编ID
//#define SP_Hard_SetID 									(09)               //硬件编ID
//#define SP_StartTranSetID 								(10)               //译员机开始编ID
//#define SP_EndTranSetID									(11)               //译员机结束编ID
#define SP_SYS_STATE 										(12)               //系统状态
#define SP_SYS_SETTING 										(13)               //系统设置
#define SP_SET_LANGUAGE										(14)               //系统设置-语言设置
#define SP_SET_DISPLAY 										(15)               //系统设置-显示设置
#define SP_SET_VOLUME 										(16)               //系统设置-音量设置
#define	SP_RESTORE_DEF										(17)               //系统设置-恢复默认
#define SP_LOCAL_IP 										(18)               //系统设置-本地IP
//#define SP_ID_MODE										(19)               //系统设置-编ID模式
#define	SP_SET_WIFI_UNIT_OFF								(20)               //系统设置-无线单元关机
#define SP_SET_DOWN_TRANS									(21)               //系统设置-下传功能
//#define SP_AUD_PLAYER_PAUSE									(22)               //暂停
//#define SP_AUD_PLAYER_PLAY									(23)               //播放
//#define SP_AUD_RECORD										(24)               //录音播放
#define SP_AUD_RECORD_WAIT									(25)               //等待录音操作有响应
#define SP_ERROR											(26)               //输入错误
#define SP_EX_CTRL_CONNECT 									(27)               //外部控制连接中
//#define SP_Hardware_Error 								(28)               //硬件中断错误
//#define SP_Restore_default_OK								(29)               //恢复默认成功
//#define SP_EEPROM_Error 									(30)               //未检出到EEPROM
//#define SP_PT2315_Error 									(31)               //音频通道不正常
//#define SP_FPGA_Error 									(32)               //FPGA启动异常						
//#define SP_W5100_Error 									(33)               //5100启动异常						
//#define SP_PT2315_Error									(34)               //
#define SP_SIGNING_IN 										(35)               //硬件签到进行中
#define SP_VOTING 											(36)               //硬件表决进行中
#define SP_FIRE_WARNING 									(37)               //消防警告
#define SP_BATTERY_ERR 										(38)               //电池没装好
#define SP_TRIAL_EXPIRED									(39)               //试用期已到		
#define SP_USB_INIT											(40)               //U盘初始化
#define SP_UNIT_TYPE										(41)               //机器类型

#define SP_AUD_PLAY_OR_RECORD								(44)               //音频播放或录音选择
#define SP_AUD_PLAYER_PLAYING								(45)               //播放器 —— 正在播放
#define SP_AUD_PLAY_STOPED									(46)               //播放器 —— 已停止
#define SP_AUD_RECORD_STOPED								(47)               //录音 —— 已停止
#define SP_AUD_RECORDING									(48)               //录音 —— 正在录音



/* 屏幕变量寄存器 Screen variable  register */
#define SVR_WIFI_OL_TOTAL_NUM										(0x0001)			//无线话筒在线总数
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


/* 屏幕通讯类型 */
typedef enum {
	/* 切换页面 */
	tType_Screen_Page = 0x80,
	/* 查询页面 */
	tType_Screen_Query = 0x81,
	/* 配置寄存器的值 */
	tType_Screen_CfgReg = 0x82,
	/* 屏幕界面返回值 */
	tType_Screen_InterfaceRet = 0x83,
}ScreenProtType_EN;


/******************************** 协议数据结构 ************************************/
/*  全数字会议系统通讯协议数据结构(基本结构) */
#pragma pack(1)	/* 单字节对齐 */
typedef struct {
    uint16_t id;
    uint8_t type;
    uint8_t ph;
    uint8_t pl;
    uint8_t ch;
    uint16_t sec;
} ConfProtocol_S;
#pragma pack()

/*  WIFI单元通讯协议数据结构(基本结构) */
#pragma pack(1)	/* 单字节对齐 */
typedef struct {
    uint16_t id;
    uint8_t cmd;
    uint8_t ph;
    uint8_t pl;
} WifiUnitProtocol_S;
#pragma pack()


/* 大彩屏幕通讯协议（迪文协议）数据介机构（基本结构） */
#pragma pack(1)	/* 单字节对齐 */
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



