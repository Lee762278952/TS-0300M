#ifndef __CONFERENCE_H__
#define __CONFERENCE_H__



#include "stdint.h"
#include "network.h"
#include "protocol.h"
#include "database.h"
#include "dsp.h"
#include "app.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* 会议模式 */
typedef enum {
    kMode_Conference = 0,
    kMode_Sign,
    kMode_Vote,
    kMode_EditID,
    kMode_DevSign,
    kMode_DevVote,
} SysMode_EN;

/* 话筒模式 */
typedef enum {
    kMode_Mic_Fifo = 1,
    kMode_Mic_Normal,
    kMode_Mic_VoiceCtrl,
    kMode_Mic_Apply,
} MicMode_EN;


/********************** 会议线程通知相关数据结构及定义 *****************************/

/* 通知源(有线单元、WIFI单元、WEB控制(HTTP)、PC端口控制(TCP)、串口控制、屏幕控制) */
typedef enum {
    kType_NotiSrc_WiredUnit 	= (1<<0),
    kType_NotiSrc_WifiUnit  	= (1<<1),
    kType_NotiSrc_Web 			= (1<<2),
    kType_NotiSrc_PC			= (1<<3),
    kType_NotiSrc_UartCtrl		= (1<<4),
    kType_NotiSrc_ScreenCtrl 	= (1<<5),
    kType_NotiSrc_SlaveMcu      = (1<<6),		/* 从单片机 */
} NotifySrcType_EN;

typedef NotifySrcType_EN InstructDest_EN;

/* 通知格式 */
typedef struct {
    /* 通知源 */
    NotifySrcType_EN nSrc;

    /* 通讯协议 */
    union {
        ConfProtocol_S conference;
        WifiUnitProtocol_S wifi;
        ScreenProtocol_S screen;
    } prot;

    uint16_t    exLen;
    uint8_t		exDataHead;
} Notify_S;

/********************** 会议线程执行操作相关数据结构及定义 *****************************/
/* 执行操作目标对象(有线单元、WIFI单元、WEB控制、PC端口控制、串口控制、屏幕控制)
	通过 NotifySrcType_EN 强转short型数据，通过位判断确定操作目标*/
#define EXE_DEST							uint16_t
#define ALL_DEST							(0xFFFF)
#define EX_CTRL_DEST						(kType_NotiSrc_PC|kType_NotiSrc_Web/*|kType_NotiSrc_UartCtrl*/)
#define ALL_UNIT							(kType_NotiSrc_WifiUnit|kType_NotiSrc_WiredUnit)


/********************** 单元设备相关数据结构及定义 *****************************/
/* 单元类型数量 */
#define UNIT_TYPE_NUM								(2)


#ifndef WIRED_UNIT_MAX_ONLINE_NUM
/* 最大有线单元数 */
#define WIRED_UNIT_MAX_ONLINE_NUM					(4096)
#endif

#ifndef WIFI_UNIT_MAX_ONLINE_NUM
/* 最大WIFI单元数 */
#define WIFI_UNIT_MAX_ONLINE_NUM					(300)
#endif

#ifndef WIRED_UNIT_MAX_ALLWO_OPEN
/* 最大允许开话筒数（有线） */
#define WIRED_UNIT_MAX_ALLWO_OPEN					(8)
#endif

#ifndef WIFI_UNIT_MAX_ALLWO_OPEN
/* 最大允许开话筒数（WIFI） */
#define WIFI_UNIT_MAX_ALLWO_OPEN					(6)
#endif

#ifndef WIFI_UNIT_START_ID
/* WIFI单元起始ID */
#define WIFI_UNIT_START_ID							(0x3000)
#endif

#ifndef VOTE_CUSTOM_ITEM_LEN
/* 自定义表决项长度 */
#define VOTE_CUSTOM_ITEM_LEN						(20)
#endif




/* 单元类型（有线单元、WIFI单元） */
typedef enum {
//    kType_Unit_Wired = 0, kType_Unit_Wifi = 1
	kType_Unit_Wired = 1, 
	kType_Unit_Wifi = 2,
} UnitType_EN;

/* 单元属性（主席、单元、译员） */
typedef enum {
    aRepresentative, aChairman, aInterpreter
} UnitAttr_EN;

/* 单元话筒状态 */
typedef enum {
    kStatus_UnitMic_Close,
    kStatus_UnitMic_Open,
    kStatus_UnitMic_Apply,
    kStatus_UnitMic_Wait,
} UnitMicSta_EN;

/* 系统语言种类 */
typedef enum {
    Chinese = 0,
    English,
    Russian,
    French,
} Language_EN;

/* 在线话筒数 */
typedef struct {
    /* 有线主席单元 */
    uint16_t wiredChm;
    /* 有线代表单元 */
    uint16_t wiredRps;
    /* WIFI主席单元 */
    uint16_t wifiChm;
    /* WIFI代表单元 */
    uint16_t wifiRps;
    /* 译员机 */
    uint16_t interpreter;
} UnitOnlineNum;





/**
* @Description	单元信息数据结构
*
**/
#pragma pack(1)
typedef struct {

    /* 上线标志 */
    bool online;

    /* 允许签到 */
    bool allowSign;
    /* 签到标志 */
    bool sign;

    /* 允许投票 */
    bool allowVote;
    /* 投票结果 */
    uint8_t vote;

    /* 轮询计数（得到应答会清空，
       计数超过"POLLING_NO_REPLY_OFFLINE_COUNT"判断离线） */
    uint16_t pollCount;

    /* 单元类型（主席、代表、译员机） */
    UnitAttr_EN attr;

    /* 单元MAC,IP */
    NETWORK_MAC mac;
    NETWORK_IP ip;

    /* 单元话筒状态(开、关、申请、等待) */
    UnitMicSta_EN micSta;

    /* 打开话筒时音频通道（默认0） */
    uint8_t channel;

    /* 电量、信号值(wifi) */
    uint8_t soc;
    uint8_t rssi;

    /* 单元配置（需要连接到Database 用于保存） */
    UnitCfg_S *config;
} UnitInfo_S;
#pragma pack()



/**
* @Description  会议系统数据结构
*
**/
typedef struct {

    /* 系统状态 */
    struct {
        /* 系统模式 */
        SysMode_EN sysMode;

        /* 投票模式 */
        VoteMode_EN voteMode;

		/* ID重复标志 */
		bool idDupicate;
		/* 重复的ID */
		uint16_t dupId;
		/* ID重复(计数器，收到ID重复时重置为3，
		   定时任务每检测标志位一次减一，减到0
		   后认为ID重复设备已经被断开) */
		uint8_t idDupCnt;

        /* 有线单元当前ID(编ID) */
        uint16_t wiredCurEditID;

        /* WIFI单元当前ID(编ID) */
        uint16_t wifiCurEditID;

		/* 起始ID(编ID) */
		uint32_t startID;

		/* 发起签到或表决的设备ID */
		uint16_t initSignVoteId;

		/* 表决结果（各选项人数） */
		uint16_t voteResult[5];

        /* 签到总人数 */
        uint16_t totalSignNum;

        /* 当前签到人数 */
        uint16_t currentSignNum;

		/* 自定义表决内容 */
		uint8_t voteCustomItem[5][VOTE_CUSTOM_ITEM_LEN];

		/* WIFI音频下传 */
		bool wifiAudDownward;

		/* 火警信号 */
		bool fireAlarm;
		
    } state;

    /* 系统配置（需要连接到Database 用于保存） */
    SysCfg_S *config;
} ConfSysInfo_S;



/* API struct */
typedef struct {
	AppLauncher_S *launcher;
//    void (*launch)(void);

    void (*notify)(Notify_S *);
    void (*getOnlineNum)(UnitOnlineNum *);
    SysMode_EN (*getSysMode)(void);
    uint16_t (*getCurrentEditId)(void);
    UnitInfo_S *(*wiredUnitInfo)(void);
    UnitInfo_S *(*wifiUnitInfo)(void);
    ConfSysInfo_S *(*getConfSysInfo)(void);
} Conference_S;

/*******************************************************************************
 * API
 ******************************************************************************/
extern Conference_S Conference;




#endif
