#ifndef __DATABASE_H_
#define __DATABASE_H_

#include "stdint.h"
//#include "conference.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifndef WIRED_UNIT_MAX_ONLINE_NUM
/* ������ߵ�Ԫ�� */
#define WIRED_UNIT_MAX_ONLINE_NUM					(4096)
#endif

#ifndef WIFI_UNIT_MAX_ONLINE_NUM
/* ���WIFI��Ԫ�� */
#define WIFI_UNIT_MAX_ONLINE_NUM					(300)
#endif


typedef enum {
    kType_Database_SysCfg = 1 << 0,
    kType_Database_WifiCfg = 1 << 1,
    kType_Database_WiredCfg = 1 << 2,

    kType_Database_Whole = 0xFF,
} DatabaseType_EN;


/* DSP�������ݽṹ */
typedef struct {
    uint8_t vol;
    uint8_t eqVol[10];
    uint8_t inputVol[7];
    uint8_t downTrans;
    uint8_t dly;
} DspCfg_S;

/* ϵͳ�������ݽṹ */
#pragma pack(1)	/* ���ֽڶ��� */
typedef struct {
    /* ���ݿ������� */
    uint8_t features[4];

    /* ����IP��ַ��PC���ӿڣ� */
    uint8_t ip[4];
    uint8_t mask[4];
    uint8_t gateWay[4];
    uint16_t port;

    /* ��Ͳģʽ */
    uint8_t micMode;

    /* DSPģʽ */
    uint8_t dspMode;

    /* �������ȫ��Ԥ��λ����Ƶͨ�� */
    uint8_t panorPos;
    uint8_t panorCh;

    /* ϵͳ���� */
    uint8_t language;

    /* ������Ͳ�� - ���ߵ�Ԫ */
    uint8_t wiredAllowOpen;

    /* �ɵȴ���Ͳ�� - ���ߵ�Ԫ
    (�ɵȴ�������������Ͳ�������) */
    uint8_t wiredAllowWait;

    /* ������Ͳ�� - WIFI��Ԫ */
    uint8_t wifiAllowOpen;

    /* �ɵȴ���Ͳ�� - WIFI��Ԫ
    (�ɵȴ�������������Ͳ�������) */
    uint8_t wifiAllowWait;

    /* ��Ļ���� */
    uint8_t brightness;

    /* DSP�������� */
    DspCfg_S dsp[22];
} SysCfg_S;
#pragma pack()

/* ��Ԫ�������ݽṹ */
typedef struct {
    /* 16������ͨ������ */
    uint8_t chVol[16];
    /* EQƵ��ѡ�� */
    uint8_t eqFreq[5];
    /* EQ���� */
    uint8_t eqVol[5];
    /* ������ */
    uint8_t sensitivity;
    /* ������١�������ͨ�� */
    uint8_t camCh;
    /* ������١���Ԥ��λ*/
    uint8_t camPos;
    /* ���� */
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



