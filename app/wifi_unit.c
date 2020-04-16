/**
 *
 *	@name				wifi_unit.c
 *	@author 			KT
 *	@date 				2019/07/23
 *	@brief
 *  @include			wifi_unit.h
 *
 *  @API
 *
 *  @description
 *
 **/

/*******************************************************************************
 * includes
 ******************************************************************************/
/* HAL */
#include "hal_mac_phy.h"
#include "hal_gpio.h"

/* OS */
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"

/* GLOBAL */
#include "log.h"

/* APP */
#include "wifi_unit.h"
#include "conference.h"
#include "network.h"
#include "ram.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MAC_PHY_IRQ_GPIO 										(GPIO3)
#define MAC_PHY_IRQ_GPIO_PIN 									(4)

/* �����ջ��С�����ȼ� */
#define WIFI_UNIT_TASK_STACK_SIZE								(1024)
#define WIFI_UNIT_TASK_PRIORITY									(14)


/* ����MAC��IP���˿ں� */
#define HOST_MAC_DEF   											{0x00, 0x0C, 0x29, 0x55, 0xDD, 0x41}
#define HOST_IP_DEF												{10,1,0,2}
#define HOST_PORT												(3000)

/* Ŀ�Ķ˿ں� */
#define DEST_PORT												(51288)

/* �ಥ��ַ */
#define MULTICAST_IP											{224,0,2,4}

/* �ಥ���ʹ��� */
#define MULTICAST_SEND_TIMES									(10)

/* WIFIָ���ʱ���� */
#define WIFI_TRANSMIT_TIME_INTERVAL								(25)



/* WIFI��Ԫ��ѯʱ���� */
#define WIFI_POLLING_TIME_MS									(100)

#if	(WIFI_POLLING_TIME_MS <= WIFI_TRANSMIT_TIME_INTERVAL)
#err "WIFI_POLLING_TIME_MS should not less tan WIFI_TRANSMIT_TIME_INTERVAL !!"
#endif

/* ��ѯ��Ӧ���ж�Ϊ���ߴ��� */
#if 0
/* ������ */
#define WIFI_POLLING_NO_REPLY_OFFLINE_COUNT						(1200)
#else
#define WIFI_POLLING_NO_REPLY_OFFLINE_COUNT						(4)
#endif




/* WIFI��Ԫ��������ͨѶЭ���ʽ */
typedef struct {
    uint8_t frameCnt;
    WifiUnitProtocol_S prot;
    uint16_t exLen;
    uint8_t exDataHead;
} WifiUnitData_S;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* LAUNCHER */
static void WifiUnit_NetDataProcessTask(void *pvParameters);
static void WifiUnit_NetDataTransmitTask(void *pvParameters);
static void WifiUnit_ConfigTask(void *pvParameters);

/* API���� */
//static void WifiUnit_Launch(void);
static void WifiUnit_NetDataTransmitWithExData(WifiSendMode_EN mode,WifiUnitProtocol_S *prot, uint16_t exLen, uint8_t *exData);
static void WifiUnit_NetDataTransmit(WifiSendMode_EN mode,WifiUnitProtocol_S *prot);

/* �ڲ����� */
static void WifiUnit_MacPhyIrqCallback(void *param);
static Network_DataBuf_S *WifiUnit_NetDataAnalysis(uint8_t *data);
static void WifiUnit_AccessSystem(uint16_t id,Network_Mac_S *unitSrcMac,Network_Addr_S *unitSrcIp,UnitAttr_EN attr);
static void WifiUnit_NotifyConferenceWithExData(WifiUnitProtocol_S *prot, uint16_t exLen, uint8_t *exData);
static void WifiUnit_NotifyConference(WifiUnitProtocol_S *prot);
static void WifiUnit_PollingTimer(TimerHandle_t xTimer);
static void WifiUnit_NetDataTransWithDstAddr(WifiSendMode_EN 		mode,
        WifiUnitProtocol_S 		*prot,
        uint16_t 				exLen,
        uint8_t 				*exData,
        Network_Mac_S 			*destMac,
        Network_Addr_S			*destIp,
        NETWORK_PORT			destPort);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* IO�ж����� */
static HAL_GpioHandler MacPhyIrq;

/* MAC��IPʵ�� */
static NETWORK_MAC HostMac = HOST_MAC_DEF;
static NETWORK_IP HostIP = HOST_IP_DEF;
static NETWORK_IP MultiCastIp = MULTICAST_IP;
static NETWORK_MAC MultiCastMac;

static const NETWORK_MAC nullMac = {0x00,0x00,0x00,0x00,0x00,0x00};


/* �����ź��� */
static SemaphoreHandle_t RecvSemaphore;

/* wifi��Ԫ��Ϣ */
static UnitInfo_S *unitInfo;

/* ���Ͷ��� */
static QueueHandle_t SendQueue;

/* ��ѯ��ʱ�� */
static TimerHandle_t wifiPollingTmr;
/* ��¼��ѯID */
static uint16_t pollingId = 0;


/*******************************************************************************
 * Task & API
 ******************************************************************************/

static AppTask_S ConfigTask = {
    .task = WifiUnit_ConfigTask,
    .name = "App.WifiUnit.Config",
    .stack = WIFI_UNIT_TASK_STACK_SIZE,
    .para = null,
    .prio = WIFI_UNIT_TASK_PRIORITY,
    .handle = null
};

static AppTask_S NetDataProcess = {
    .task = WifiUnit_NetDataProcessTask,
    .name = "App.WifiUnit.NetDataProcess",
    .stack = WIFI_UNIT_TASK_STACK_SIZE,
    .para = null,
    .prio = WIFI_UNIT_TASK_PRIORITY,
    .handle = null
};

static AppTask_S NetDataTransmit = {
    .task = WifiUnit_NetDataTransmitTask,
    .name = "App.WifiUnit.NetDataTransmit",
    .stack = WIFI_UNIT_TASK_STACK_SIZE,
    .para = null,
    .prio = WIFI_UNIT_TASK_PRIORITY,
    .handle = null
};

static AppTask_S *FuncTask[] = {&NetDataProcess,&NetDataTransmit};

static AppLauncher_S Launcher = {
    .init = null,
    .configTask = &ConfigTask,
    .funcNum  = 2,
    .funcTask = FuncTask,
};


/* API */
WifiUnit_S WifiUnit = {
    .launcher = &Launcher,

    .transmit = WifiUnit_NetDataTransmit,
    .transWithExData = WifiUnit_NetDataTransmitWithExData,
};
/*******************************************************************************
 * Code
 ******************************************************************************/


/**
* @Name  		WifiUnit_ConfigTask
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void WifiUnit_ConfigTask(void *pvParameters)
{
    MacPhyIrq = HAL_GpioInit(MAC_PHY_IRQ_GPIO, MAC_PHY_IRQ_GPIO_PIN, kGPIO_DigitalInput, 0, kGPIO_IntRisingEdge);
    HAL_SetIrqCallback(MacPhyIrq,WifiUnit_MacPhyIrqCallback,null);

    HAL_MacPhyInit(tDm9051,(uint8_t *)&HostMac,false);

    NETWORK_SET_MAC((&MultiCastMac),0x01,0x00,0x5E,(HostIP.addr1 & 0x7F),HostIP.addr2,HostIP.addr3);

    RecvSemaphore = xSemaphoreCreateCounting(64, 0);
    SendQueue = xQueueCreate(256, sizeof(void *));

    unitInfo = Conference.wifiUnitInfo();

    /* �ȴ���������Ӧ */
    while(HAL_MacPhyGetAutoNegotiation(tDm9051) != kStatus_Success) {
        DELAY(100);
    }

    /* ��ʼ����ѯ��ʱ�� */
    wifiPollingTmr = xTimerCreate("wifiPolling",WIFI_POLLING_TIME_MS,pdTRUE,null,WifiUnit_PollingTimer);

    Log.i("Wifi unit configuration finish ... \r\n");


    vTaskDelete(null);
}


/**
* @Name  		WifiUnit_Init
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void WifiUnit_NetDataProcessTask(void *pvParameters)
{
    Network_DataBuf_S *netBuf;
    WifiUnitProtocol_S prot;
    Network_Mac_S *unitMac;
    Network_Addr_S *unitIp;
    uint8_t exDataLen, *exData;
    uint8_t *recvBuffer, recvLength;
    ConfSysInfo_S *info;

    Log.i("Wifi unit data process task start!!\r\n");

    recvBuffer = MALLOC(100);
    exData = MALLOC(50);

    Protocol.wifiUnit(&prot,0,MasterStarUp_MtoU_G,0,0);
    WifiUnit_NetDataTransmit(kMode_Wifi_Multicast,&prot);

    xTimerStart(wifiPollingTmr,0);
    Log.i("Wifi unit polling timer start!!\r\n");

    /* �㲥ģʽ������ */
    info = Conference.getConfSysInfo();
    WifiUnit_NetDataTransmit(kMode_Wifi_Multicast,Protocol.wifiUnit(&prot,0,ChangeMicManage_MtoU_G,info->config->micMode,info->config->wifiAllowOpen));

    while(1) {
        xSemaphoreTake(RecvSemaphore,MAX_NUM);

        recvLength = HAL_MacPhyReceive(tDm9051, recvBuffer);

        if(recvLength < 64) {
            Log.e("wifi unit bad package\r\n");
            continue;
        }
        netBuf = WifiUnit_NetDataAnalysis(recvBuffer);

        if(netBuf == null)
            continue;

        unitMac = &netBuf->srcMac;
        unitIp = &netBuf->srcIp;


        /* ���Ʋ��� */
        memcpy(&prot,netBuf->data,sizeof(WifiUnitProtocol_S));
        prot.id = lwip_htons(prot.id);


        /* Э������������չ���� */
        if(netBuf->len > sizeof(WifiUnitProtocol_S)) {
            exDataLen = netBuf->len - sizeof(WifiUnitProtocol_S);
            memcpy(exData,netBuf->data + sizeof(WifiUnitProtocol_S),exDataLen);
        }

        if(prot.id > WIFI_UNIT_START_ID && prot.id <= (WIFI_UNIT_START_ID + WIFI_UNIT_MAX_ONLINE_NUM)) {
            prot.id = prot.id - WIFI_UNIT_START_ID;

            /* ��Ԫ�������ϵͳ */
            if(prot.cmd == ApplyEnterSystm_UtoM_D && prot.pl == 0) {
                goto access_sys;
            }

            /* ��Ԫȷ�ϵ�ǰID(��ID) */
            else if(prot.cmd == UnitGraspIDUtoM_D) {
                uint8_t *macIp;
                uint16_t confirmID = (uint16_t)(prot.ph << 8) | prot.pl;

                if(confirmID == info->state.wifiCurEditID) {
                    /* ֪ͨ�������߳� */
                    macIp = MALLOC(NETWORK_IP_SIZE + NETWORK_MAC_SIZE);
                    memcpy(&macIp[0],unitMac,NETWORK_MAC_SIZE);
                    memcpy(&macIp[6],unitIp,NETWORK_IP_SIZE);
                    WifiUnit_NotifyConferenceWithExData(&prot, NETWORK_IP_SIZE + NETWORK_MAC_SIZE, macIp);
                    FREE(macIp);

                    /* �ظ���Ԫȷ�� */
                    WifiUnit_NetDataTransWithDstAddr(kMode_Wifi_Unitcast,  \
                                                     Protocol.wifiUnit(&prot,prot.id,ConfirmUnitIDMtoU_D,(confirmID >> 8),(confirmID & 0xFF)),
                                                     null,null, \
                                                     unitMac,unitIp,DEST_PORT);

                    info->state.wifiCurEditID = (info->state.wifiCurEditID + 1) > WIFI_ID(WIFI_UNIT_MAX_ONLINE_NUM) ? WIFI_ID(1) : info->state.wifiCurEditID + 1;

                    /* �㲥���µ�ǰID */
                    WifiUnit_NetDataTransmit(kMode_Wifi_Multicast,  \
                                             Protocol.wifiUnit(&prot,0,EnterEditingIDMtoU_G,(info->state.wifiCurEditID >> 8),(info->state.wifiCurEditID & 0xFF)));

                }

                goto clear_buf;
            }

            else {

                /* ��鵥Ԫ���߱�־λ */

                /* ��Ԫ״̬Ϊ���� */
                if(unitInfo[prot.id].online) {
                    unitInfo[prot.id].pollCount = 0;


                    /* ��Ԫ�ظ����� */
                    if(prot.cmd == RepPollUnitl_UtoM_D) {
                        SysMode_EN mode = Conference.getSysMode();

                        unitInfo[prot.id].soc = exData[0]&0x0F;
                        unitInfo[prot.id].rssi = exData[1];

                        unitInfo[prot.id].allowSign = (prot.pl>>3)&0x01;
                        unitInfo[prot.id].allowVote = (prot.pl>>4)&0x01;

                        /* ǩ��ģʽ�£���Ԫ��״̬Ϊ��ǩ������������¼״̬Ϊ�� */
                        if((mode == kMode_Sign || mode == kMode_DevSign) && (prot.ph & 0x01) && !unitInfo[prot.id].sign) {
                            WifiUnit_NotifyConference(&prot);
                        }

                        /* ���ģʽ�£���Ԫ�صı��״̬��������¼״̬��һ�� */
                        else if((mode == kMode_Vote || mode == kMode_DevVote) && (prot.pl != unitInfo[prot.id].vote)) {
                            WifiUnit_NotifyConference(&prot);
                        }
                        goto clear_buf;
                    }

                    /* ����������Ϣ���͸����������� */
                    else {
                        WifiUnit_NotifyConferenceWithExData(&prot,exDataLen,exData);
                    }

                    goto clear_buf;
                }

                /* ��Ԫ״̬Ϊ���� */
                else {
                    /* ���Ϊ������״̬���Ҳ����������ϵͳ����ID��������ǰ���ݣ�ֱ�ӷ��ظ���Ԫ��Ҫ�����½���ϵͳ */
                    WifiUnit_NetDataTransWithDstAddr(kMode_Wifi_Unitcast,&prot,null,null,unitMac,unitIp,DEST_PORT);

                    goto clear_buf;
                }
            }

access_sys:
            /* ��Ԫ����ϵͳ */
            if(prot.ph == 0)
                WifiUnit_AccessSystem(prot.id,unitMac,unitIp,aRepresentative);
            else if(prot.ph == 1)
                WifiUnit_AccessSystem(prot.id,unitMac,unitIp,aChairman);

        }

        /* ��ջ��� */
clear_buf:
        FREE(netBuf->data);
        FREE(netBuf);
        memset(exData,0,50);
        exDataLen = 0;
    }
}


/**
* @Name  		WifiUnit_AccessSystem
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void WifiUnit_AccessSystem(uint16_t id,Network_Mac_S *unitSrcMac,Network_Addr_S *unitSrcIp,UnitAttr_EN attr)
{
    ConfSysInfo_S *info;
    uint8_t content[20],contLen = 0;
    WifiUnitProtocol_S prot;
    UnitInfo_S *devInfo;

    ERR_CHECK(unitSrcMac != null && unitSrcIp != null,return);
    ERR_CHECK(id > 0 && id <= WIFI_UNIT_MAX_ONLINE_NUM, return);
    ERR_CHECK(!NETWORK_COMPARISON_MAC(unitSrcMac,(&nullMac)), return);

    devInfo = &unitInfo[id];

    /* ��ȡϵͳ״̬���·� */
    info = Conference.getConfSysInfo();

    switch(info->state.sysMode) {
    case kMode_Conference:
    default:
        content[6] = kMode_Conference;
        content[7] = info->config->micMode;
        content[8] = info->config->wifiAllowOpen;
        contLen = 9;
        break;
    case kMode_Sign:
    case kMode_DevSign:
        content[6] = kMode_Sign;
        content[7] = (uint8_t)info->state.totalSignNum >> 8;
        content[8] = (uint8_t)info->state.totalSignNum;
        content[9] = (uint8_t)info->state.currentSignNum >> 8;
        content[10] = (uint8_t)info->state.currentSignNum;
        content[11] = devInfo->sign;
        content[12] = info->config->micMode;
        content[13] = info->config->wifiAllowOpen;
        contLen = 14;
        break;
    case kMode_Vote:
    case kMode_DevVote:
        content[6] = kMode_Vote;
        content[7] = info->state.voteMode;
        content[8] = info->config->micMode;
        content[9] = info->config->wifiAllowOpen;
        content[10] = devInfo->vote;
        content[11] = devInfo->sign;
        contLen = 12;
        break;
    case kMode_EditID:
        content[6] = kMode_EditID;
        contLen = 7;
        break;
    }

    /* �豸ID�����ߣ���MAC�������ϵͳMAC��ͬ */
    if(devInfo->online && !NETWORK_COMPARISON_MAC((&devInfo->mac),unitSrcMac)) {
        /* ͬ�����ϵͳ��֪ͨ��Ͳģʽ */
		Protocol.wifiUnit(&prot,id, RepApplyEnterSystm_MtoU_D, 0, 0);
//        WifiUnit_NetDataTransmitWithExData(kMode_Wifi_Unitcast,&prot,contLen,content);
		WifiUnit_NetDataTransWithDstAddr(kMode_Wifi_Unitcast,&prot,null,null,unitSrcMac,unitSrcIp,DEST_PORT);

        /* ����������߳��ظ�ID */
        Protocol.wifiUnit(&prot,0, IDRepeatingMtoU_G, id >> 8, id & 0xFF);
        WifiUnit_NotifyConference(&prot);
    }
    /* �豸ID������,�������ߵ�MAC��ͬ,�ظ�ע��ɹ����ظ�����״̬ */
    else {
        Protocol.wifiUnit(&prot,id, RepApplyEnterSystm_MtoU_D, 0, 0);

        /* ���浥Ԫ��Ϣ */
        devInfo->attr = attr;
        devInfo->pollCount = 0;
        NETWORK_SET_MAC((&devInfo->mac),unitSrcMac->mac0,unitSrcMac->mac1,unitSrcMac->mac2,unitSrcMac->mac3,unitSrcMac->mac4,unitSrcMac->mac5);
        NETWORK_SET_ADDR((devInfo->ip),unitSrcIp->addr0,unitSrcIp->addr1,unitSrcIp->addr2,unitSrcIp->addr3);

        /* ͬ�����ϵͳ��֪ͨ��Ͳģʽ */
        WifiUnit_NetDataTransmitWithExData(kMode_Wifi_Unitcast,&prot,contLen,content);

        WifiUnit_NotifyConference(&prot);

    }
}



/**
* @Name  		WifiUnit_NetDataTransmitTask
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void WifiUnit_NetDataTransmitTask(void *pvParameters)
{
    Network_DataBuf_S *netBuf;
    Network_EthernetFrame_S *frame;
    uint16_t frameId = 0;
    uint8_t *data;

    data = MALLOC(100);
    frame = (Network_EthernetFrame_S *)data;

    Log.i("Wifi unit data transmit task start!!\r\n");

    while(1) {
        xQueueReceive(SendQueue, &netBuf, MAX_NUM);
        ERR_CHECK_DBG(netBuf != null,"netBuf == null!!\r\n",continue);

        memcpy(&frame->destMac,&netBuf->destMac,NETWORK_MAC_SIZE);
        memcpy(&frame->destIp,&netBuf->destIp,NETWORK_IP_SIZE);
        frame->destPort = lwip_htons(netBuf->destPort);

        memcpy(&frame->srcMac,&netBuf->srcMac,NETWORK_MAC_SIZE);
        memcpy(&frame->srcIp,&netBuf->srcIp,NETWORK_IP_SIZE);
        frame->srcPort = lwip_htons(netBuf->srcPort);

        /* ��װ������*/
        frame->type = lwip_htons(0x0800);
        frame->ver_len = 0x45;
        frame->totalLen = lwip_htons(netBuf->len+8+20);
        frame->id = lwip_htons(frameId++);
        frame->ttl = 128;
        frame->prot = 17;
        frame->length = lwip_htons(netBuf->len+8);
        memcpy(&(frame->dataHead),netBuf->data,netBuf->len);

        HAL_MacPhySend(tDm9051, data, netBuf->len + 42);

        FREE(netBuf->data);
        FREE(netBuf);
        /* ������ʱ25ms */
        DELAY(25);
    }
}


/**
* @Name  		WifiUnit_PollingTimer
* @Author  		KT
* @para
*
*
* @return
* @Description:

   **WIFI��ѯ���� ����ѯ��ʱ��"wifiPollingTmr"ÿ"WIFI_POLLING_TIME_MS"���봥��һ
                    �Σ�ÿ�δ���ѡ��һ�����ߵ�WIFI��Ԫ��������IP������ѯ��������
                    ���ͺ��¼��ǰID��"pollingId",�´δ�"pollingId + 1"��ʼ����
                    ���ߵ�Ԫ;
*/
static void WifiUnit_PollingTimer(TimerHandle_t xTimer)
{
    uint16_t id, i ;
    WifiUnitProtocol_S prot;
    ConfSysInfo_S *info;
    static uint8_t editIdPollingCnt;

    memset(&prot,0,sizeof(WifiUnitProtocol_S));

    info = Conference.getConfSysInfo();

    switch(info->state.sysMode) {
	    case kMode_Conference:
	    case kMode_DevSign:
	    case kMode_Sign:
	    case kMode_DevVote:
	    case kMode_Vote:
	    default: {
	        for(i = 0,id = pollingId + 1; i < WIFI_UNIT_MAX_ONLINE_NUM; i++,id++) {
	            if(id >= WIFI_UNIT_MAX_ONLINE_NUM)
	                id = 0;

	            if(unitInfo[id].online) {
	                if(unitInfo[id].pollCount < WIFI_POLLING_NO_REPLY_OFFLINE_COUNT) {
	                    Protocol.wifiUnit(&prot,id,PollUnitl_MtoU_D,info->state.sysMode,0);
	                    unitInfo[id].pollCount++;
	                } else {
	                    Protocol.wifiUnit(&prot,id,MasterAskforReenterSysMtoU_D,0,0);
	                    unitInfo[id].online = false;
	                    WifiUnit_NotifyConference(&prot);
	                }
	                WifiUnit_NetDataTransmit(kMode_Wifi_Unitcast,&prot);
	                pollingId = id;
	                break;
	            }
	        }

	    }
	    break;

	    case kMode_EditID:
	        if(++ editIdPollingCnt >= 10) {
	            Protocol.wifiUnit(&prot,0,EnterEditingIDMtoU_G,(uint8_t)(info->state.wifiCurEditID >> 8),(uint8_t)(info->state.wifiCurEditID));
	            WifiUnit_NetDataTransmit(kMode_Wifi_Multicast,&prot);
	            editIdPollingCnt = 0;
	        }

	        break;
    }

}


/**
* @Name  		WifiUnit_NotifyConferenceWithExData
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void WifiUnit_NotifyConferenceWithExData(WifiUnitProtocol_S *prot, uint16_t exLen, uint8_t *exData)
{
    Notify_S *notify;

    if(prot == null || (exLen != 0 && exData == null))
        return;

    notify = MALLOC(sizeof(Notify_S) + exLen);

    notify->nSrc = kType_NotiSrc_WifiUnit;

    memcpy(&notify->prot.wifi,prot,sizeof(WifiUnitProtocol_S));
    notify->exLen = exLen;
    if(exLen != 0) {
        memcpy(&notify->exDataHead,exData,exLen);
    }

    Conference.notify(notify);
}

/**
* @Name  		WifiUnit_NotifyConference
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void WifiUnit_NotifyConference(WifiUnitProtocol_S *prot)
{
    WifiUnit_NotifyConferenceWithExData(prot, null, null);
}


/**
* @Name  		WifiUnit_NetDataTransWithDstAddr
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void WifiUnit_NetDataTransWithDstAddr(	WifiSendMode_EN 		mode,
												WifiUnitProtocol_S 		*prot,
												uint16_t 				exLen,
												uint8_t 				*exData,
												Network_Mac_S 			*destMac,
												Network_Addr_S			*destIp,
												NETWORK_PORT			destPort)
{
    static uint8_t framecnt;
    uint8_t length,i;

    /* wifiϵͳ����Э�鳤�ȼ�ȥID�����ֽ�ʣ��3���ֽ�,�鲥����ǰ��һ���ֽ�framecnt */
    length = 3 + exLen + ((mode == kMode_Wifi_Multicast || mode == kMode_Wifi_UrgMulticast) ? 1 : 0);


    /* ���� */
    if(mode == kMode_Wifi_Unitcast) {
        Network_DataBuf_S *netBuf;

        netBuf = MALLOC(sizeof(Network_DataBuf_S));
        netBuf->data = MALLOC(length);

        memcpy(&netBuf->destIp,destIp,NETWORK_IP_SIZE);
        memcpy(&netBuf->destMac,destMac,NETWORK_MAC_SIZE);
        netBuf->destPort = destPort;

        memcpy(&netBuf->srcIp,&HostIP,NETWORK_IP_SIZE);
        memcpy(&netBuf->srcMac,&HostMac,NETWORK_MAC_SIZE);
        netBuf->srcPort = HOST_PORT;

        netBuf->len = length;

        netBuf->data[0] = prot->cmd;
        netBuf->data[1] = prot->ph;
        netBuf->data[2] = prot->pl;

        if(exLen != 0) {
            memcpy(&netBuf->data[3],exData,exLen);
        }

        /* ��ǰ�� */
        xQueueSendToFront(SendQueue, &netBuf, MAX_NUM);
    }

    /* �ಥ(�ಥʱ�������Ŀ�ĵ�ַ��Ч��ʹ�öಥ��ַ) */
    else if(mode == kMode_Wifi_Multicast || mode == kMode_Wifi_UrgMulticast) {
        Network_DataBuf_S *netBuf[MULTICAST_SEND_TIMES];

        for(i = 0; i < MULTICAST_SEND_TIMES; i++) {
            netBuf[i] = MALLOC(sizeof(Network_DataBuf_S));
            netBuf[i]->data = MALLOC(length);

            memcpy(&netBuf[i]->destIp,&MultiCastIp,NETWORK_IP_SIZE);
            memcpy(&netBuf[i]->srcIp,&HostIP,NETWORK_IP_SIZE);
            memcpy(&netBuf[i]->destMac,&MultiCastMac,NETWORK_MAC_SIZE);
            memcpy(&netBuf[i]->srcMac,&HostMac,NETWORK_MAC_SIZE);
            netBuf[i]->destPort = DEST_PORT;
            netBuf[i]->srcPort = HOST_PORT;

            netBuf[i]->len = length;

            netBuf[i]->data[0] = framecnt;
            netBuf[i]->data[1] = prot->cmd;
            netBuf[i]->data[2] = prot->ph;
            netBuf[i]->data[3] = prot->pl;

            if(exLen != 0) {
                memcpy(&netBuf[i]->data[4],exData,exLen);
            }

			if(mode == kMode_Wifi_Multicast){
	            /* �����10�� */
	            xQueueSendToBack(SendQueue, &(netBuf[i]), MAX_NUM);
			}
			else if(mode == kMode_Wifi_UrgMulticast){
				/* �����鲥��ǰ��10�� */
				 xQueueSendToFront(SendQueue, &(netBuf[i]), MAX_NUM);
			}
        }

        framecnt ++;
    }


}


/**
* @Name  		WifiUnit_NetDataTransmitWithExData
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void  WifiUnit_NetDataTransmitWithExData(WifiSendMode_EN mode,WifiUnitProtocol_S *prot, uint16_t exLen, uint8_t *exData)
{
//    uint8_t length,i;
//    static uint8_t framecnt;

    /* Э����ָ��Ϊ�� ; ��չ�ֽڳ��Ȳ�Ϊ0 ����չ�ֽ�ָ��Ϊ�� */
    ERR_CHECK((prot != null) && !(exLen != 0 && exData == null), return);
    /* idΪ0������ģʽ��Ϊ�鲥 */
    ERR_CHECK(!(prot->id == 0 && (mode != kMode_Wifi_Multicast && mode != kMode_Wifi_UrgMulticast)), return );

#if 1
    /* ID����(Ϊ�˷�ֹ����ĳЩ�ط����ܴ��ڵ�IDд�����)��
       �������idΪ0x3000 ~ 0x312C�Զ���ȥ0x3000 */
    if(prot->id >= 0x3000 && prot->id <= 0x312C)
        prot->id -= WIFI_UNIT_START_ID;

#endif

    /* �ж�ID��Χ�Ƿ�Ϸ� */
    ERR_CHECK(prot->id >= 0 && prot->id <= WIFI_UNIT_MAX_ONLINE_NUM, return );


    WifiUnit_NetDataTransWithDstAddr(mode,prot,exLen,exData,&unitInfo[prot->id].mac,&unitInfo[prot->id].ip,DEST_PORT);
#if 0
    /* wifiϵͳ����Э�鳤�ȼ�ȥID�����ֽ�ʣ��3���ֽ�,�鲥����ǰ��һ���ֽ�framecnt */
    length = 3 + exLen + (mode == kMode_Wifi_Multicast ? 1 : 0);


    /* ���� */
    if(mode == kMode_Wifi_Unitcast) {
        Network_DataBuf_S *netBuf;

        netBuf = MALLOC(sizeof(Network_DataBuf_S));
        netBuf->data = MALLOC(length);

        memcpy(&netBuf->destIp,&unitInfo[prot->id].ip,NETWORK_IP_SIZE);
        memcpy(&netBuf->srcIp,&HostIP,NETWORK_IP_SIZE);
        memcpy(&netBuf->destMac,&unitInfo[prot->id].mac,NETWORK_MAC_SIZE);
        memcpy(&netBuf->srcMac,&HostMac,NETWORK_MAC_SIZE);
        netBuf->destPort = DEST_PORT;
        netBuf->srcPort = HOST_PORT;
        netBuf->len = length;

        netBuf->data[0] = prot->cmd;
        netBuf->data[1] = prot->ph;
        netBuf->data[2] = prot->pl;

        if(exLen != 0) {
            memcpy(&netBuf->data[3],exData,exLen);
        }

        /* ��ǰ�� */
        xQueueSendToFront(SendQueue, &netBuf, MAX_NUM);
    }

    /* �ಥ */
    else if(mode == kMode_Wifi_Multicast) {
        Network_DataBuf_S *netBuf[MULTICAST_SEND_TIMES];

        for(i = 0; i < MULTICAST_SEND_TIMES; i++) {
            netBuf[i] = MALLOC(sizeof(Network_DataBuf_S));
            netBuf[i]->data = MALLOC(length);

            memcpy(&netBuf[i]->destIp,&MultiCastIp,NETWORK_IP_SIZE);
            memcpy(&netBuf[i]->srcIp,&HostIP,NETWORK_IP_SIZE);
            memcpy(&netBuf[i]->destMac,&MultiCastMac,NETWORK_MAC_SIZE);
            memcpy(&netBuf[i]->srcMac,&HostMac,NETWORK_MAC_SIZE);
            netBuf[i]->destPort = DEST_PORT;
            netBuf[i]->srcPort = HOST_PORT;
            netBuf[i]->len = length;

            netBuf[i]->data[0] = framecnt;
            netBuf[i]->data[1] = prot->cmd;
            netBuf[i]->data[2] = prot->ph;
            netBuf[i]->data[3] = prot->pl;

            if(exLen != 0) {
                memcpy(&netBuf[i]->data[4],exData,exLen);
            }

            /* �����10�� */
            xQueueSendToBack(SendQueue, &(netBuf[i]), MAX_NUM);
        }

        framecnt ++;
    }
#endif


}


/**
* @Name  		WifiUnit_NetDataTransmit
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void WifiUnit_NetDataTransmit(WifiSendMode_EN mode,WifiUnitProtocol_S *prot)
{
    WifiUnit_NetDataTransmitWithExData(mode,prot,null,null);
}


/**
* @Name  		WifiUnit_MacPhyIrqCallback
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static Network_DataBuf_S *WifiUnit_NetDataAnalysis(uint8_t *data)
{
    Network_DataBuf_S *buf;
    Network_EthernetFrame_S *frame;

    ERR_CHECK(data != null,return null);

    buf = MALLOC(sizeof(Network_DataBuf_S));

    frame = (Network_EthernetFrame_S *)data;
    memcpy(&buf->destMac,&frame->destMac,6);
    memcpy(&buf->srcMac,&frame->srcMac,6);
    memcpy(&buf->destIp,&frame->destIp,4);
    memcpy(&buf->srcIp,&frame->srcIp,4);
    buf->destPort = lwip_htons(frame->destPort);
    buf->srcPort = lwip_htons(frame->srcPort);

    if(lwip_htons(frame->length) > 8) {
        buf->len = (lwip_htons(frame->length) - 8);
        buf->data = MALLOC(buf->len);
        memcpy(buf->data,&frame->dataHead,buf->len);
    }

    return buf;

}

/**
* @Name  		WifiUnit_MacPhyIrqCallback
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void WifiUnit_MacPhyIrqCallback(void *param)
{
    uint8_t irqSta;
    bool lnk;
    portBASE_TYPE taskToWake = pdFALSE;

    irqSta = HAL_MacPhyGetIrqSta(tDm9051);


    if(irqSta & MAC_PHY_DM9051_ISR_ROO) {
        Log.e(" Receive Overflow Counter Overflow!\r\n");
    }

    if(irqSta & MAC_PHY_DM9051_ISR_ROS) {
        Log.e(" Receive Overflow\r\n");
    }



    if(irqSta & MAC_PHY_DM9051_ISR_LINK) {
        HAL_MacPhyGetLink(tDm9051, &lnk);

        if(lnk)
            Log.d("Wifi port linked\r\n");
        else
            Log.d("Wifi port unlinked\r\n");
    }

    if(irqSta & MAC_PHY_DM9051_ISR_PR) {
        xSemaphoreGiveFromISR(RecvSemaphore, &taskToWake);
    }

}



