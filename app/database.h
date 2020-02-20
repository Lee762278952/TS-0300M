#ifndef __DATABASE_H_
#define __DATABASE_H_

#include "stdint.h"
//#include "conference.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifndef WIRED_UNIT_MAX_ONLINE_NUM
/* 最大有线单元数 */
#define WIRED_UNIT_MAX_ONLINE_NUM					(4096)
#endif

#ifndef WIFI_UNIT_MAX_ONLINE_NUM
/* 最大WIFI单元数 */
#define WIFI_UNIT_MAX_ONLINE_NUM					(300)
#endif


typedef enum {
    kType_Database_SysCfg = 1 << 0,
    kType_Database_WifiCfg = 1 << 1,
    kType_Database_WiredCfg = 1 << 2,

    kType_Database_Whole = 0xFF,
} DatabaseType_EN;


/* DSP配置数据结构 */
typedef struct {
    uint8_t vol;
    uint8_t eqVol[10];
    uint8_t inputVol[7];
    uint8_t downTrans;
    uint8_t dly;
} DspCfg_S;

/* 系统配置数据结构 */
#pragma pack(1)	/* 单字节对齐 */
typedef struct {
    /* 数据库特征码 */
    uint8_t features[4];

    /* 本地IP地址（PC连接口） */
    uint8_t ip[4];
    uint8_t mask[4];
    uint8_t gateWay[4];
    uint16_t port;

    /* 话筒模式 */
    uint8_t micMode;

    /* DSP模式 */
    uint8_t dspMode;

    /* 摄像跟踪全景预置位及视频通道 */
    uint8_t panorPos;
    uint8_t panorCh;

    /* 系统语言 */
    uint8_t language;

    /* 允许开话筒数 - 有线单元 */
    uint8_t wiredAllowOpen;

    /* 可等待话筒数 - 有线单元
    (可等待数量和允许开话筒数量相等) */
    uint8_t wiredAllowWait;

    /* 允许开话筒数 - WIFI单元 */
    uint8_t wifiAllowOpen;

    /* 可等待话筒数 - WIFI单元
    (可等待数量和允许开话筒数量相等) */
    uint8_t wifiAllowWait;

    /* 屏幕亮度 */
    uint8_t brightness;

	/* 系统签到、表决功能是否开启	*/
	uint8_t signEn;
	uint8_t voteEn;

    /* DSP参数配置 */
    DspCfg_S dsp[22];
} SysCfg_S;
#pragma pack()

/* 单元配置数据结构 */
typedef struct {
    /* 16个分区通道音量 */
    uint8_t chVol[16];
    /* EQ频率选择 */
    uint8_t eqFreq[5];
    /* EQ音量 */
    uint8_t eqVol[5];
    /* 灵敏度 */
    uint8_t sensitivity;
    /* 摄像跟踪——摄像通道 */
    uint8_t camCh;
    /* 摄像跟踪——预置位*/
    uint8_t camPos;
    /* 保留 */
    uint8_t reserved[3];
} UnitCfg_S;


typedef struct {
    void (*init)(void);
    void (*save)(void);
    void (*saveSpecify)(uint8_t type, uint16_t id);
    uint8_t *(*getInstance)(DatabaseType_EN type);
    void (*restoreDef)(uint8_t type);
} Database_S;

/*******************************************************************************
 * API
 ******************************************************************************/
extern Database_S Database;


#endif



