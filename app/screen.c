/**
 *
 *	@name				screen.c
 *	@author 			KT
 *	@date 				2019/11/18
 *	@brief
 *  @include			screen.h
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
#include "hal_uart.h"

/* OS */
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"
#include "queue.h"

/* API */
#include "ram.h"

/* APP */
#include "screen.h"
#include "conference.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CTRL_UART_BASE									(LPUART8)

#define CTRL_UART_BAUDRATE								(115200U)

/* 任务堆栈大小及优先级 */
#define SCREEN_TASK_STACK_SIZE							(1024)
#define SCREEN_TASK_PRIORITY							(12)

/* 接收静态缓存BUF大小*/
#define UART_RECV_BUF_SIZE								(32)

/* 数据头长度 */
#define DATA_HEAD_LENGTH								(3)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* LAUNCHER */
static void Screen_Init(void);
static void Screen_UartDataProcessTask(void *pvParameters);

/* API */
//static void Screen_Launch(void);
static void Screen_TransmitWithExData(ScreenProtocol_S *prot,uint16_t exLen, uint8_t *exData);
static void Screen_Transmit(ScreenProtocol_S *prot);
static void Screen_SetBacklight(uint8_t brightness);
static void Screen_TogglePage(uint8_t page);
static void Screen_SetVariable(uint16_t reg,uint8_t len,uint8_t *var);
static void Screen_Lock(uint8_t page);
static void Screen_Unlock(void);
static bool Screen_IsLock(void);
static uint8_t Screen_CurrentPage(void);
static void Screen_SetLanguage(uint8_t language);



/* Internal */
static void Screen_CtrlUartCallback(uint8_t data,void *para);
static void Screen_NotifyConferenceWithExData(ScreenProtocol_S *prot, uint16_t exLen, uint8_t *exData);
static void Screen_NotifyConference(ScreenProtocol_S *prot);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* 记录当前显示页面 */
static uint8_t CurrentPage = SP_WELCOME;

/* 记录锁屏页面 */
static uint8_t LockPage = null;

/* 记录当前屏幕是否处于锁屏状态 */
static bool IsLock = false;

/* 屏幕通讯协议 */
ScreenProtocol_S screenProt;


/* 串口控制句柄 */
static HAL_UartHandler_S CtrlUartHandler;

static uint8_t RecvCount = 0;
static uint8_t RecvBuf[UART_RECV_BUF_SIZE];

/* 数据头(2字节数据头+1字节长度) */
static const uint8_t DataHead[DATA_HEAD_LENGTH] = {0x5A,0xA5,0};

/* 修改屏幕语言指令 */
static const uint8_t SetLanguage[] = {0x5A,0xA5,0x03,0x80,0x71,0x00};

/*******************************************************************************
 * Task & API
 ******************************************************************************/

static AppTask_S UartDataProcess = {
    .task = Screen_UartDataProcessTask,
    .name = "Screen.UartDataProcess",
    .stack = SCREEN_TASK_STACK_SIZE,
    .para = null,
    .prio = SCREEN_TASK_PRIORITY,
    .handle = null
};


static AppTask_S *FuncTask[] = {&UartDataProcess};

static AppLauncher_S Launcher = {
    .init = Screen_Init,
    .configTask = null,
    .funcNum  = 1,
    .funcTask = FuncTask,
};


/* API */
Screen_S Screen = {
    .launcher = &Launcher,

    .togglePage = Screen_TogglePage,
    .lock = Screen_Lock,
    .unlock = Screen_Unlock,
    .isLock = Screen_IsLock,
    .currentPage = Screen_CurrentPage,
    .setVariable = Screen_SetVariable,
    .transmit = Screen_Transmit,
    .transWithExData = Screen_TransmitWithExData,
    .backlight = Screen_SetBacklight,
    .setLanguage = Screen_SetLanguage,
};
/*******************************************************************************
 * Code
 ******************************************************************************/
/**
* @Name 		Screen_Init
* @Author		KT
* @Description
* @para
*
*
* @return
*/
static void Screen_Init(void)
{
    HAL_UartConfig_S *config;

    config = MALLOC(sizeof(HAL_UartConfig_S));
    CtrlUartHandler = MALLOC(HAL_UART_HANDLER_SIZE);

    config->base = CTRL_UART_BASE;
    config->baudRate = CTRL_UART_BAUDRATE;
    config->enableRx = true;
    config->enableTx = true;
    config->rxIrq = true;
    config->txIrq = false;

    HAL_UartInit(CtrlUartHandler, config);
    HAL_UartSetCallback(CtrlUartHandler, RecvBuf, UART_RECV_BUF_SIZE, Screen_CtrlUartCallback, null);
}



/**
* @Name  		Screen_TogglePage
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Screen_TogglePage(uint8_t page)
{
    if(IsLock)
        return;

    CurrentPage = page;
    Screen_Transmit(Protocol.screen(&screenProt,tType_Screen_Page,CurrentPage));
}

/**
* @Name  		Screen_SetVariable
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Screen_SetVariable(uint16_t reg,uint8_t len,uint8_t *var)
{
    ERR_CHECK(var != null, return);

    Screen_TransmitWithExData(Protocol.screen(&screenProt,tType_Screen_CfgReg,reg),len,var);
}


/**
* @Name  		Screen_Lock
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Screen_Lock(uint8_t page)
{
    IsLock = true;

    if(LockPage != page) {
        LockPage = page;
        Screen_Transmit(Protocol.screen(&screenProt,tType_Screen_Page,LockPage));
    }
}

/**
* @Name  		Screen_Unlock
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Screen_Unlock(void)
{
    if(!IsLock)
        return;

    IsLock = false;
    LockPage = null;
    Screen_Transmit(Protocol.screen(&screenProt,tType_Screen_Page,CurrentPage));
}

/**
* @Name  		Screen_IsLock
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static bool Screen_IsLock(void)
{
    return IsLock;
}



/**
* @Name  		Screen_CurrentPage
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static uint8_t Screen_CurrentPage(void)
{
    return CurrentPage;
}


/**
* @Name  		Screen_UartDataProcessTask
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Screen_UartDataProcessTask(void *pvParameters)
{
    ScreenProtocol_S prot;
    uint8_t exData[15],exLen;
    uint8_t *date;
    uint16_t year,mon,day;

    Log.i("Screen data process task start!!\r\n");

    date = MALLOC(20);
    APP_GetBuildDate(&year, &mon, &day);

    /* 更新显示当前系统版本号 */
    sprintf((char *)&date[0],"M_%s_%04d%02d%02d",APP_VERSION,year,mon,day);
    Screen.setVariable(SVR_SOFTWARE_VERSION_M,16,date);

    /* 更新显示代码编译日期 */
//  sprintf((char *)&exData[0],"S_%s_%d%d%d",APP_VERSION,year,mon,day);
//  Screen.setVariable(SVR_SOFTWARE_VERSION_S,16,exData);

    FREE(date);

    while(1) {
        DELAY(100);
        if(RecvCount < 7)
            continue;

        if(RecvBuf[0] == 0x5A && RecvBuf[1] == 0xA5 && RecvBuf[2] > 4 && RecvBuf[2] <= 6) {
            prot.type = (ScreenProtType_EN)RecvBuf[3];
            memcpy(&prot.para[0],&RecvBuf[4],RecvBuf[2] - 1);
            Screen_NotifyConference(&prot);
        } else if(RecvBuf[0] == 0x5A && RecvBuf[1] == 0xA5 && RecvBuf[2] > 6) {
            prot.type = (ScreenProtType_EN)RecvBuf[3];
            memcpy(&prot.para[0],&RecvBuf[4],5);

            exLen = RecvBuf[2] - 6;
            memcpy(exData,&RecvBuf[9],exLen);
            Screen_NotifyConferenceWithExData(&prot,exLen,exData);
        }

        RecvCount = 0;
        HAL_UartClearCount(CtrlUartHandler);
    }
}

/**
* @Name  		Screen_NotifyConferenceWithExData
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Screen_NotifyConferenceWithExData(ScreenProtocol_S *prot, uint16_t exLen, uint8_t *exData)
{
    Notify_S *notify;

    if(prot == null || (exLen != 0 && exData == null))
        return;

    notify = MALLOC(sizeof(Notify_S) + exLen);

    notify->nSrc = kType_NotiSrc_ScreenCtrl;

    memcpy(&notify->prot.screen,prot,sizeof(ScreenProtocol_S));
    notify->exLen = exLen;
    if(exLen != 0) {
        memcpy(&notify->exDataHead,exData,exLen);
    }

    Conference.notify(notify);
}

/**
* @Name  		Screen_NotifyConference
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Screen_NotifyConference(ScreenProtocol_S *prot)
{
    Screen_NotifyConferenceWithExData(prot, null, null);
}

/**
* @Name  		Screen_TransmitWithExData
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Screen_TransmitWithExData(ScreenProtocol_S *prot,uint16_t exLen, uint8_t *exData)
{
    uint8_t *data,dataLen,protLen;

    ERR_CHECK(prot != null,return);
    ERR_CHECK(!(exLen != 0 && exData == null),return);

    protLen = (prot->type == tType_Screen_Page) ? 4 : 3;

    dataLen = DATA_HEAD_LENGTH + protLen + exLen ;

    data = MALLOC(dataLen);

    memcpy(&data[0],DataHead,DATA_HEAD_LENGTH);
    memcpy(&data[DATA_HEAD_LENGTH],prot,protLen);
    if(exLen != 0)
        memcpy(&data[DATA_HEAD_LENGTH + protLen],exData,exLen);

    data[2] = dataLen - DATA_HEAD_LENGTH;

    HAL_UartSend(CtrlUartHandler, data, dataLen);

    FREE(data);
}


/**
* @Name  		Screen_SetLanguage
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Screen_SetLanguage(uint8_t language)
{
    uint8_t *cmd;

    cmd = MALLOC(sizeof(SetLanguage));

    memcpy(cmd,SetLanguage,sizeof(SetLanguage));

    cmd[5] = language;

    HAL_UartSend(CtrlUartHandler, cmd, 6);

    FREE(cmd);
}


/**
* @Name  		Screen_Transmit
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Screen_Transmit(ScreenProtocol_S *prot)
{
    Screen_TransmitWithExData(prot,null,null);
}

/**
* @Name  		Screen_SetBacklight
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Screen_SetBacklight(uint8_t brightness)
{
    uint8_t cmd[10] = {0x5A, 0xA5, 0x07, 0x80, 0x70, 0x02, 0, 30, 100, 0};
    cmd[8] = brightness;

    HAL_UartSend(CtrlUartHandler, cmd, 10);
}



/**
* @Name  		Screen_CtrlUartCallback
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void Screen_CtrlUartCallback(uint8_t count,void *para)
{
    RecvCount = count;
}


