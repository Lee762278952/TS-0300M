/**
 *
 *	@name				database.c
 *	@author 			KT
 *	@date 				2019/12/11
 *  @include			database.h
 *
 *
 **/
/*******************************************************************************
 * includes
 ******************************************************************************/
/* OS */
#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"

/* HAL */
#include "hal_spi_flash.h"

/* API */
#include "ram.h"
#include "debug.h"

/* APP */
#include "conference.h"
#include "database.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FLASH_SECTOR_SIZE								(4096)

/* ���ڷ����ڴ���4096�ֽڶ��룬����FLASH�洢��ȡ */
#define SPACE_SIZE(_contantSize)						((_contantSize/FLASH_SECTOR_SIZE + (_contantSize % FLASH_SECTOR_SIZE ? 1 : 0)) * FLASH_SECTOR_SIZE)


#define SYS_DATA_STORAGE_ADDR							(0x1000)
#define SYS_DATA_SPACE_SIZE								SPACE_SIZE(sizeof(SysCfg_S))

#define WIFI_CFG_STORAGE_ADDR							(0x3000)
#define WIFI_CFG_SPACE_SIZE								SPACE_SIZE(sizeof(UnitCfg_S) * (WIFI_UNIT_MAX_ONLINE_NUM + 1))

#define WIRED_CFG_STORAGE_ADDR							(0x7000)
#define WIRED_CFG_SPACE_SIZE							SPACE_SIZE(sizeof(UnitCfg_S) * (WIRED_UNIT_MAX_ONLINE_NUM + 1))

/* �����ջ��С�����ȼ� */
#define DATABASE_TASK_STACK_SIZE						(1024)
#define DATABASE_TASK_PRIORITY							(6)

/* ÿ��SECTOR���Դ洢�ĵ�Ԫ�������� */
#define UNIT_EACH_SECTOR								(FLASH_SECTOR_SIZE / sizeof(UnitCfg_S))

/* ���㵥Ԫ���õ�ԪID��Ӧ�Ĵ洢SECTOR */
#define UNIT_CFG_ID_TO_SECTOR(_id)						(_id / UNIT_EACH_SECTOR + 1)


/* ���ݿ��첽�洢ʱ�� */
#define DATABASE_SAVE_TIME								(500)

/* DATABASE��ȡFLASH���������루4 byte�� */
#define DATQBASE_FLASH_FEATURE							"DB00"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* API */
static void Database_Init(void);
static void Database_Save(void);
static void Database_SaveSpecify(uint8_t type,uint16_t id);
static uint8_t *Database_GetInstance(DatabaseType_EN type);
static void Database_RestoreDefault(uint8_t type);

/* INTERNAL */
static void Database_Saver(uint8_t type);
static void Database_SaverTask(void *pvParameters);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint8_t SysDataSpace[SYS_DATA_SPACE_SIZE] 	__attribute__((section("OcramHeap")));
static uint8_t WifiCfgSpace[WIFI_CFG_SPACE_SIZE] 	__attribute__((section("OcramHeap")));
static uint8_t WiredCfgSpace[WIRED_CFG_SPACE_SIZE] 	__attribute__((section("OcramHeap")));

static SemaphoreHandle_t SaveSemap;

static uint8_t SaveType;
static uint8_t maxSector;

static SysCfg_S *SysCfg = (SysCfg_S *)SysDataSpace;
static UnitCfg_S *WifiCfg = (UnitCfg_S *)WifiCfgSpace;
static UnitCfg_S *WiredCfg = (UnitCfg_S *)WiredCfgSpace;


Database_S Database = {
    .init = Database_Init,
    .save = Database_Save,
    .saveSpecify = Database_SaveSpecify,
    .getInstance = Database_GetInstance,
    .restoreDef = Database_RestoreDefault,
};

/*******************************************************************************
 * Code
 ******************************************************************************/
static void Database_Init(void)
{
    uint8_t *txBuf,*rxBuf;
    uint16_t i;

    debug("Database init start!\r\n");
	
	if(FLASH_SECTOR_SIZE % sizeof(UnitCfg_S)){
		debug("Warnint:UnitCfg_S must Can be divisible by FLASH_SECTOR_SIZE!!");
	}

    txBuf = MALLOC(4200);
    rxBuf = MALLOC(4200);

    HAL_FlashBufferConfig(tW25Q64,txBuf,rxBuf);

    SaveSemap = xSemaphoreCreateBinary();

    taskENTER_CRITICAL();
    /* ��ȡFLASH */
    for(i = 0; i < (SYS_DATA_SPACE_SIZE / FLASH_SECTOR_SIZE); i++) {
        HAL_FlashReadSector(tW25Q64,SYS_DATA_STORAGE_ADDR + i * FLASH_SECTOR_SIZE,SysDataSpace + (i * FLASH_SECTOR_SIZE));
    }

    for(i = 0; i < (WIFI_CFG_SPACE_SIZE / FLASH_SECTOR_SIZE); i++) {
        HAL_FlashReadSector(tW25Q64,WIFI_CFG_STORAGE_ADDR + i * FLASH_SECTOR_SIZE,WifiCfgSpace + (i * FLASH_SECTOR_SIZE));
    }

    for(i = 0; i < (WIRED_CFG_SPACE_SIZE / FLASH_SECTOR_SIZE); i++) {
        HAL_FlashReadSector(tW25Q64,WIRED_CFG_STORAGE_ADDR + i * FLASH_SECTOR_SIZE,WiredCfgSpace + (i * FLASH_SECTOR_SIZE));
    }
    taskEXIT_CRITICAL();

    /* �״γ�ʼ�� */
    if(strncmp((char *)SysCfg->features,DATQBASE_FLASH_FEATURE,4) != 0) {

        memset(SysDataSpace,0,sizeof(SysDataSpace));
        memset(WifiCfgSpace,0,sizeof(WifiCfgSpace));
        memset(WiredCfgSpace,0,sizeof(WiredCfgSpace));

        /* ��¼���� */
        memcpy(SysCfg->features,DATQBASE_FLASH_FEATURE,4);

        Database_RestoreDefault(kType_Database_Whole);
        Database_Saver(kType_Database_Whole);
    }

    if (xTaskCreate(Database_SaverTask, "SaverTask", DATABASE_TASK_STACK_SIZE, null, DATABASE_TASK_PRIORITY, null) != pdPASS) {
        debug("save task error\r\n");
    }

//	debug("size = %d addr = 0x%X\r\n",sizeof(SysDataSpace),SysDataSpace);
//	debug("size = %d addr = 0x%X\r\n",sizeof(WiredCfgSpace),WiredCfgSpace);
//	debug("size = %d addr = 0x%X\r\n",sizeof(WifiCfgSpace),WifiCfgSpace);
}

static void Database_SaverTask(void *pvParameters)
{

    while(1) {
        xSemaphoreTake(SaveSemap, MAX_NUM);
        DELAY(DATABASE_SAVE_TIME);

        Database_Saver(SaveType);

        SaveType = 0;
		maxSector = 0;
    }
}

static void Database_SaveSpecify(uint8_t type,uint16_t id)
{
	uint8_t sector;
	
    SaveType |= type;
	
	/* �������������ߵ�Ԫ�����ݣ���ô��Ҫ����һ��ID��ͨ��IDֵ�ж�������Ҫ�洢��sector */
	if(type & kType_Database_WiredCfg){
		sector = UNIT_CFG_ID_TO_SECTOR(id);
		maxSector = sector > maxSector ? sector : maxSector;
	}
    xSemaphoreGive(SaveSemap);
}


static void Database_Save(void)
{
    Database_SaveSpecify(kType_Database_Whole,WIRED_UNIT_MAX_ONLINE_NUM);
}



static uint8_t *Database_GetInstance(DatabaseType_EN type)
{
    switch(type) {
    case kType_Database_SysCfg:
        return (uint8_t *)SysCfg;
    case kType_Database_WifiCfg:
        return (uint8_t *)WifiCfg;
    case kType_Database_WiredCfg:
        return (uint8_t *)WiredCfg;

    default:
        return null;
    }
}

static void Database_Saver(uint8_t type)
{
    uint16_t i;

    if(SaveType & kType_Database_SysCfg) {
        for(i = 0; i < (SYS_DATA_SPACE_SIZE / FLASH_SECTOR_SIZE); i++) {
            HAL_FlashEraseSector(tW25Q64,SYS_DATA_STORAGE_ADDR + i * FLASH_SECTOR_SIZE);
            while(HAL_FlashGetBusy(tW25Q64)) {
                DELAY(50);
            }
            taskENTER_CRITICAL();
            HAL_FlashWriteSector(tW25Q64,SYS_DATA_STORAGE_ADDR + i * FLASH_SECTOR_SIZE,SysDataSpace + (i * FLASH_SECTOR_SIZE));
            taskEXIT_CRITICAL();
        }
    }

    if(SaveType & kType_Database_WifiCfg) {
        for(i = 0; i < (WIFI_CFG_SPACE_SIZE / FLASH_SECTOR_SIZE); i++) {
            HAL_FlashEraseSector(tW25Q64,WIFI_CFG_STORAGE_ADDR + i * FLASH_SECTOR_SIZE);
            while(HAL_FlashGetBusy(tW25Q64)) {
                DELAY(50);
            }
            taskENTER_CRITICAL();
            HAL_FlashWriteSector(tW25Q64,WIFI_CFG_STORAGE_ADDR + i * FLASH_SECTOR_SIZE,WifiCfgSpace + (i * FLASH_SECTOR_SIZE));
            taskEXIT_CRITICAL();
        }
    }

    if(SaveType & kType_Database_WiredCfg) {

		/* ���ߵ�Ԫ���ݽṹ̫�󣬰������id���� */
		for(i = 0; i < maxSector; i++) {
            HAL_FlashEraseSector(tW25Q64,WIRED_CFG_STORAGE_ADDR + i * FLASH_SECTOR_SIZE);
            while(HAL_FlashGetBusy(tW25Q64)) {
                DELAY(50);
            }
            taskENTER_CRITICAL();
            HAL_FlashWriteSector(tW25Q64,WIRED_CFG_STORAGE_ADDR + i * FLASH_SECTOR_SIZE,WiredCfgSpace + (i * FLASH_SECTOR_SIZE));
            taskEXIT_CRITICAL();
        }
    }

    debug("Database has been saved!\r\n");
}

static void Database_RestoreDefault(uint8_t type)
{
    uint32_t i;
    uint8_t ip[] = EX_CTRL_LOCAL_IP_DEF;
    uint8_t gw[] = EX_CTRL_GATEWAY_DEF;
    uint8_t mask[] = EX_CTRL_NETMASK_DEF;

    /* ��������ָ�Ĭ�� */
    if(type & kType_Database_SysCfg) {
        memcpy(SysCfg->ip,ip,4);
        memcpy(SysCfg->gateWay,gw,4);
        memcpy(SysCfg->mask,mask,4);
        SysCfg->port = EX_CTRL_PORT_DEF;
        SysCfg->language = Chinese;
        SysCfg->micMode = kMode_Mic_Fifo;
        SysCfg->wiredAllowOpen = WIRED_UNIT_MAX_ALLWO_OPEN_DEF;
        SysCfg->wiredAllowWait = WIRED_UNIT_MAX_ALLWO_OPEN_DEF;
        SysCfg->wifiAllowOpen = WIFI_UNIT_MAX_ALLWO_OPEN_DEF;
        SysCfg->wifiAllowWait = WIFI_UNIT_MAX_ALLWO_OPEN_DEF;
        SysCfg->dspMode = DSP_MODE_WIRE;
        SysCfg->panorPos = 0;
        SysCfg->panorCh = 0;

        for(i = 0; i < 22; i++) {
            SysCfg->dsp[i].vol = 0x1F;
            SysCfg->dsp[i].downTrans = 0x00;
            SysCfg->dsp[i].dly = 0x00;
            memset(SysCfg->dsp[i].eqVol,0x0A,10);
            memset(SysCfg->dsp[i].inputVol,0x1F,7);
        }
    }

    if(type & kType_Database_WifiCfg) {
        for(i = 0; i <= WIFI_UNIT_MAX_ONLINE_NUM; i++) {
            WifiCfg[i].sensitivity = 0x04;
            memset(WifiCfg[i].chVol,0x1F,16);
            memset(WifiCfg[i].eqFreq,0x00,5);
            memset(WifiCfg[i].eqVol,0x1F,5);
        }
    }

    if(type & kType_Database_WiredCfg) {
        for(i = 0; i <= WIRED_UNIT_MAX_ONLINE_NUM; i++) {
            WiredCfg[i].sensitivity = 0x04;
            memset(WiredCfg[i].chVol,0x1F,16);
            memset(WiredCfg[i].eqFreq,0x00,5);
            memset(WiredCfg[i].eqVol,0x1F,5);
        }

    }

    debug("\r\nDatabase has been restore to default!!\r\n\r\n");
}



