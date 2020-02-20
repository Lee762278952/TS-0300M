/**
 *
 *	@FileName								usbDisk.c
 *	@Author									KT
 *	@CreationTime						2019/03/20
 *	@Description						�ṩUSB�豸��д����API���Զ�����ʽ�������ָ��ɶ��߳�ͬʱ���Ͳ���ָ�
 *													��������������dataStream��ͨ��������д��/��ȡ����,���֧��4����������
 *													ͨ���� USB_INTERFACE ����USBӲ���ӿڣ�
 *
 * 	@Include								usb_disk.h
 *
 *	@API										UsbDisk
 *
 **/

/*******************************************************************************
 * includes
 ******************************************************************************/
#include "string.h"
#include "board.h"
#include "usb_host_config.h"
#include "usb_host.h"
#include "usb_host_msd.h"
#include "usb_phy.h"
#include "usb_disk.h"
#include "ff.h"
#include "diskio.h"
#include "host_msd_fatfs.h"
#include "ram.h"
#include "log.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define USB_HOST_BUS									kUSB_ControllerEhci1
#define USB_OTG_BUS										kUSB_ControllerEhci0

#define USB_INTERFACE									USB_HOST_BUS

#if (USB_INTERFACE != USB_HOST_BUS) && (USB_INTERFACE != USB_OTG_BUS)
#error "Macro definition \"USB_INTERFACE\" configuration error"
#endif

#if ((defined USB_HOST_CONFIG_EHCI) && (USB_HOST_CONFIG_EHCI))
#define CONTROLLER_ID 									USB_INTERFACE
#endif

/* USB�ж����ȼ� */
#define USB_HOST_INTERRUPT_PRIORITY 					(6U)

/* �ж���Ϣ�����������ȼ�����ջ */
#define USB_HOST_EHCI_TASK_PRIO							(configMAX_PRIORITIES - 1)
#define USB_HOST_EHCI_TASK_STACK						(1024U)

/* �ļ�ϵͳ�����������ȼ�����ջ */
#define USB_FS_MOUNT_TASK_PRIO							(8U)
#define USB_FS_MOUNT_TASK_STACK							(1024U)

/* USBָ����������ȼ�����ջ */
#define USB_CMD_PROCESS_TASK_PRIO						(9U)
#define USB_CMD_PROCESS_TASK_STACK						(1024U)

/* USB�����������������ȼ�����ջ */
#define USB_STREAM_TASK_PRIO							(14)
#define USB_STREAM_TASK_STACK							(1024U)

#if (defined(USB_STREAM_ENABLE) && USB_STREAM_ENABLE)
/* USB�������������С */
#define USB_STREAM_BUF_SIZE								(2*1024U)
/* ���������ͨ������ */
#define USB_MAX_DATA_STREAM_NUM							(2)
#endif

/* U�̿��Ʋ������г��� */
#define USB_CMD_QUEUE_LEN      	 						(30)
#define USB_CMD_ITEM_SIZE    							(sizeof(UsbCmd_S))

/* �ļ��б���󳤶� */
#define USB_FILEINFO_STACK_SIZE							(60)

/* �����ڴ漰�����ڴ� */
#define MEM_ALLOC_COPY(dest,src,size)					{if(src!=NULL&&size!=0){do{dest=MALLOC(size);memset(dest,'\0',size);memcpy(dest,src,size);}while(0);}}

/* �����ڴ漰�����ַ��� */
#define STR_ALLOC_COPY(dest,src)						{if(src!=NULL){do{sLen=strlen(src);dest=MALLOC(sLen+1);memset(dest,'\0',sLen+1);memcpy(dest,src,sLen);}while(0);}}

/* �����ȴ�USB������������ź��� */
#define CREAT_RETURN_SEM()								xSemaphoreCreateBinary()

/* �ȴ���ɾ��USB������������ź��� */
#define GET_RETURN_SEM(sem,time)						{do{xSemaphoreTake(sem,time);vSemaphoreDelete(sem);sem=NULL;}while(0);}


/* USB������������ */
typedef enum {
    tNone = 0,
    tMkdir,
    tMkfs,
    tOpen,
    tClose,
    tWrite,
    tRead,
    tOpenDir,
    tCloseDir,
    tReadDir,
    tGetFree,
    tWaitConnect,
    tIsExist,
    tFileInfo,
    tFileList,
    tRename,
} UsbCmdType_EN;


/* USB��������ṹ�� */
typedef struct {
    UsbCmdType_EN type;
    char *path;
    char *newPath;
    uint8_t *wData;
    uint8_t *rData;
    uint32_t dataLen;
    SemaphoreHandle_t *sem;
    FRESULT *result;
    BYTE mode;
    FIL *fp;
    FILINFO *fInfo;
} UsbCmd_S;

#if (defined(USB_STREAM_ENABLE) && USB_STREAM_ENABLE)
/* USB���������ƽṹ�� */
typedef struct {
    StreamType_EN type;
    char *filePath;
    FIL *fp;
    DataStreamHandler_S *dataStream;
	uint8_t *buffer;
    int8_t timeCount;
    bool enable;
    SemaphoreHandle_t *sem;
    TaskHandle_t taskHandler;
} UsbDataStream_S;
#endif
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void USB_CmdProcessTask(void *param);
static void USB_HostClockInit(void);
static void USB_HostIsrEnable(void);
static void USB_DiskCtrlCmdQueueSend(UsbCmd_S *cmd);
static void USB_DiskInit(void);
static bool USB_DiskDeviceConnected(void);
static void USB_DiskWaitDevConnect(void);
static bool USB_DiskFileIsExist(const char *path);
static uint32_t USB_DiskFileSize(const char *path);
static void USB_DiskRename(const char *path,const char *newPath);
static void USB_HostEhciTask(void *param);
static void USB_TaskLauncher(void);
static void USB_DiskFsMountTask(void *param);
static bool USB_CmdQueueInit(void);
static FRESULT USB_Mkdir(const char *path,bool waitResul);
static FRESULT USB_Open(FIL *fp,const char *path,BYTE mode,bool waitResult);
static FRESULT USB_Write(FIL *fp,const uint8_t* data, uint32_t dLen,bool waitResult);
static FRESULT USB_Close(FIL *fp,bool waitResult);
static FRESULT USB_Read(FIL *fp,uint8_t* data, uint32_t dLen,bool waitResult);
static size_t USB_FreeSize(void);
static size_t USB_TotalSize(void);
static uint8_t USB_GetFileList(const TCHAR *path,FILINFO **fList);
static void USB_SetStaListener(USB_StaListener listener);
static usb_status_t USB_HostEvent(usb_device_handle deviceHandle,
                                  usb_host_configuration_handle configurationHandle,
                                  uint32_t eventCode);

#if (defined(USB_STREAM_ENABLE) && USB_STREAM_ENABLE)
static void USB_DataStreamProcessTask(void *pvParameters);
static uint8_t USB_setOutputStream(const char *path,DataStreamHandler_S *dataStreamHandler);
static uint8_t USB_setInputStream(const char *path,DataStreamHandler_S *dataStreamHandler);
static void USB_closeDataStream(uint8_t index);
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* �ļ�ϵͳ��ر��� */
static FATFS 	Fatfs;
static size_t 	FreeSize,TotalSize;
//static BYTE 	FsType;
static FILINFO 	FileList[USB_FILEINFO_STACK_SIZE];
static uint32_t FileListSize;

/* ״̬�������� */
static USB_StaListener Listener = null;

/* USB����������� */
static QueueHandle_t UsbDiskCmdQueue;

/* usb�豸�����־ */
static bool UsbDeviceConnected = false;

/* USB�ļ�ϵͳʵ�� */
extern usb_host_msd_fatfs_instance_t g_MsdFatfsInstance;

/* USB���ƾ�� */
static usb_host_handle UsbHostHandle;

static TaskHandle_t CmdProcessTaskHandler;

/* Usb��������ͬ���� */
static SemaphoreHandle_t UsbDiskOpearatSync;


#if (defined(USB_STREAM_ENABLE) && USB_STREAM_ENABLE)

/* ���롢������������ָ������(���ݺ궨���������) */
static UsbDataStream_S *usbDataStream[USB_MAX_DATA_STREAM_NUM] = {NULL};
static uint8_t currentStreamNum = 0;

/* Usb���������ݻ��� */
static uint8_t Streambuff[USB_MAX_DATA_STREAM_NUM][USB_STREAM_BUF_SIZE];

#endif

/** usbDisk API **/
UsbDisk_S UsbDisk = {
    .init = USB_DiskInit,
    .launch = USB_TaskLauncher,
    .devConnected = USB_DiskDeviceConnected,
    .waitDevConnect = USB_DiskWaitDevConnect,
    .isExist = USB_DiskFileIsExist,
    .fileSize = USB_DiskFileSize,
    .rename = USB_DiskRename,
    .mkdir = USB_Mkdir,
    .open = USB_Open,
    .write = USB_Write,
    .close = USB_Close,
    .read = USB_Read,
    .freeSize = USB_FreeSize,
    .totalSize = USB_TotalSize,
    .getFileList = USB_GetFileList,
    .setListener = USB_SetStaListener,
    
#if (defined(USB_STREAM_ENABLE) && USB_STREAM_ENABLE)
    .setOutputStream = USB_setOutputStream,
    .setInputStream = USB_setInputStream,
    .closeDataStream = USB_closeDataStream,
#endif
};
/*******************************************************************************
 * Code
 ******************************************************************************/
	/**
	*	@Name				USB_DiskInit
	*	@Author 			KT
	* @Description	USB���ܳ�ʼ��
	*	@Para
	* @Return
	*
	*/
	static void USB_SetStaListener(USB_StaListener listener){
		Listener = listener;
	}


/**
*	@Name  				USB_DiskInit
*	@Author  			KT
* @Description 	USB���ܳ�ʼ��
*	@Para
* @Return
*
*/
static void USB_DiskInit(void)
{
    usb_status_t status = kStatus_USB_Success;

    USB_HostClockInit();

#if ((defined FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT))
    SYSMPU_Enable(SYSMPU, 0);
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

    status = USB_HostInit(CONTROLLER_ID, &UsbHostHandle, USB_HostEvent);
    if (status != kStatus_USB_Success) {
        Log.e("host init error\r\n");
        return;
    }
    USB_HostIsrEnable();

    UsbDiskOpearatSync = xSemaphoreCreateMutex();
    xSemaphoreGive(UsbDiskOpearatSync);

    if(!USB_CmdQueueInit())
        Log.e("usb cmd queue int fail!\r\n");

    Log.d("USB disk init finish!!\r\n");

//    USB_TaskLauncher();
}


/**
* @Name  				USB_DiskDeviceConnected
* @Author  			KT
* @Description 	��ȡusb�豸����״̬
* @Para
* @Return				true	:	����USB�����豸����
*								false	:	��USB�����豸����
*
*/
static bool USB_DiskDeviceConnected(void)
{
    return UsbDeviceConnected;
}

/**
*	@Name  				USB_DiskWaitDevConnect
*	@Author  			KT
* @Description 	�ȴ�usb�豸����
*	@Para
* @Return
*
*/
static void USB_DiskWaitDevConnect(void)
{
    UsbCmd_S cmd = {(UsbCmdType_EN)0};

    if(UsbDeviceConnected)
        return;

    cmd.type = tWaitConnect;
    cmd.sem = CREAT_RETURN_SEM();
    USB_DiskCtrlCmdQueueSend(&cmd);
    GET_RETURN_SEM(cmd.sem,RES_MAX_WAIT);
}

/**
*	@Name  				USB_DiskFileIsExist
*	@Author  			KT
* @Description 	�����Ƿ����ָ���ļ�
*	@Para    			path ���ļ�·�����ļ���
* @Return				bool ���Ƿ����
*
*/
static bool USB_DiskFileIsExist(const char *path)
{
    UsbCmd_S cmd = {(UsbCmdType_EN)0};
    FRESULT fResult;
    FIL *fp;
    uint16_t sLen;

    if(!UsbDeviceConnected || path == NULL)
        return false;

    fp = MALLOC(sizeof(FIL));

    cmd.type = tIsExist;
    cmd.fp = fp;
    cmd.result = MALLOC(sizeof(FRESULT));
    *cmd.result = (FRESULT)0xFF;

    STR_ALLOC_COPY(cmd.path,path);

    cmd.sem = CREAT_RETURN_SEM();

    USB_DiskCtrlCmdQueueSend(&cmd);

    GET_RETURN_SEM(cmd.sem,RES_MAX_WAIT);

    fResult = *cmd.result;
    FREE(cmd.result);
    FREE(fp);

    return (bool)(fResult == FR_OK);
}

/**
*	@Name  				USB_DiskFileSize
*	@Author  			KT
* @Description 	�����Ƿ����ָ���ļ�
*	@Para    			path ���ļ�·�����ļ���
* @Return				bool ���Ƿ����
*
*/
static uint32_t USB_DiskFileSize(const char *path)
{
    FRESULT fResult;
    UsbCmd_S cmd = {(UsbCmdType_EN)0};
    uint16_t sLen;
    uint32_t fSize = 0;

    if(path == NULL)
        return fSize;

    if(USB_DiskFileIsExist(path)) {

        cmd.type = tFileInfo;
        cmd.fp = MALLOC(sizeof(FIL));
        cmd.fInfo = MALLOC(sizeof(FILINFO));
        cmd.result = MALLOC(sizeof(FRESULT));
        *cmd.result = (FRESULT)0xFF;

        STR_ALLOC_COPY(cmd.path,path);

        cmd.sem = CREAT_RETURN_SEM();
        USB_DiskCtrlCmdQueueSend(&cmd);
        GET_RETURN_SEM(cmd.sem,RES_MAX_WAIT);

        fResult = *cmd.result;
        FREE(cmd.result);
        FREE(cmd.fp);

        if(fResult == FR_OK)
            fSize = cmd.fInfo->fsize;

        FREE(cmd.fInfo);
    }
    return fSize;
}


/**
*	@Name  				USB_DiskRename
*	@Author  			KT
* @Description 	�������ļ�
*	@Para    			path 		��Ŀ���ļ�·�����ļ���
*								newPath �����ļ���
*
*/
static void USB_DiskRename(const char *path,const char *newPath)
{
//    FRESULT fResult;
    UsbCmd_S cmd = {(UsbCmdType_EN)0};
    uint16_t sLen;

    if(path == NULL || newPath == NULL)
        return;


    if(USB_DiskFileIsExist(path)) {

        cmd.type = tRename;
        cmd.fp = MALLOC(sizeof(FIL));
        cmd.result = MALLOC(sizeof(FRESULT));
        *cmd.result = (FRESULT)0xFF;

        STR_ALLOC_COPY(cmd.path,path);
        STR_ALLOC_COPY(cmd.newPath,newPath);

        cmd.sem = CREAT_RETURN_SEM();
        USB_DiskCtrlCmdQueueSend(&cmd);
        GET_RETURN_SEM(cmd.sem,RES_MAX_WAIT);

        DELAY(200);

//        fResult = *cmd.result;
        FREE(cmd.result);
        FREE(cmd.fp);
        FREE(cmd.fInfo);
    }
}

/**
*	@Name  				USB_Mkdir
*	@Author  			KT
* @Description 	�½��ļ���
*	@para    			*path 			:	·��
*								*waitResult ���Ƿ�ȴ������̷߳��ؽ��
*/
static FRESULT USB_Mkdir(const char *path,bool waitResult)
{
    FRESULT fResult;
    UsbCmd_S cmd = {(UsbCmdType_EN)0};
    uint16_t sLen;

    if(path == NULL) return (FRESULT)0xFF;

    cmd.type = tMkdir;
    cmd.result = MALLOC(sizeof(FRESULT));
    *cmd.result = (FRESULT)0xFF;

    STR_ALLOC_COPY(cmd.path,path);

    if(waitResult) cmd.sem = CREAT_RETURN_SEM();

    USB_DiskCtrlCmdQueueSend(&cmd);

    if(waitResult) GET_RETURN_SEM(cmd.sem,RES_MAX_WAIT);

    fResult = *cmd.result;
    FREE(cmd.result);

    return fResult;
}

/**
*	@Name  				USB_DiskCtrlCmdQueueSend
*	@Author  			KT
* @Description 	usbdisk������з���
*	@para    			*cmd 			:	���ƾ��
*/
static void USB_DiskCtrlCmdQueueSend(UsbCmd_S *cmd)
{
    if(xQueueSend(UsbDiskCmdQueue, cmd, RES_MAX_WAIT) != pdPASS)
        Log.e("queue send fail!\r\n");
}



/**
*	@Name  				USB_Open
*	@Author  			KT
* @Description 	���ļ�
*	@para    			*fp 			:	�ļ�ָ��
*								*path			:	�ļ�·��
*								mode			:	��ģʽ
*								waitResult: �Ƿ�ȴ�USB�������Ӧ��
*/
static FRESULT USB_Open(FIL *fp,const char *path,BYTE mode,bool waitResult)
{
    FRESULT fResult;
    UsbCmd_S cmd = {(UsbCmdType_EN)0};
    uint16_t sLen;

    if(path == NULL || fp == NULL)
        return (FRESULT)0xFF;



    cmd.type = tOpen;
    cmd.mode = mode;
    cmd.fp = fp;
    cmd.result = MALLOC(sizeof(FRESULT));
    *cmd.result = (FRESULT)0xFF;

    STR_ALLOC_COPY(cmd.path,path);

    if(waitResult) cmd.sem = CREAT_RETURN_SEM();

    USB_DiskCtrlCmdQueueSend(&cmd);

    if(waitResult) GET_RETURN_SEM(cmd.sem,RES_MAX_WAIT);

    fResult = *cmd.result;
    FREE(cmd.result);

    return fResult;
}

/**
*	@Name  				USB_Close
*	@Author  			KT
* @Description 	�ر��ļ�
*	@para    			*fp 			:	�ļ�ָ��
*								waitResult: �Ƿ�ȴ�USB�������Ӧ��
*/
static FRESULT USB_Close(FIL *fp,bool waitResult)
{
    FRESULT fResult;
    UsbCmd_S cmd = {(UsbCmdType_EN)0};

    if(fp == NULL)
        return (FRESULT)0xFF;

    cmd.type = tClose;
    cmd.fp = fp;
    cmd.result = MALLOC(sizeof(FRESULT));
    *cmd.result = (FRESULT)0xFF;

    if(waitResult) cmd.sem = CREAT_RETURN_SEM();

    USB_DiskCtrlCmdQueueSend(&cmd);

    if(waitResult) GET_RETURN_SEM(cmd.sem,RES_MAX_WAIT);

    fResult = *cmd.result;
    FREE(cmd.result);

    return fResult;
}

/**
*	@Name  				USB_Write
*	@Author  			KT
* @Description 	���ļ�д������
*	@para    			*fp 			:	�ļ�ָ��
*								*data			:	����ָ��
*								dLen			:	���ݳ���
*								waitResult: �Ƿ�ȴ�Ӧ��
*/
static FRESULT USB_Write(FIL *fp,const uint8_t* data, uint32_t dLen,bool waitResult)
{
    FRESULT fResult;
    UsbCmd_S cmd = {(UsbCmdType_EN)0};

    if(fp == NULL)
        return (FRESULT)0xFF;

    cmd.type = tWrite;
    cmd.dataLen = dLen;
    cmd.fp = fp;
    cmd.result = MALLOC(sizeof(FRESULT));
    *cmd.result = (FRESULT)0xFF;

    MEM_ALLOC_COPY(cmd.wData,data,dLen);

    if(waitResult) cmd.sem = CREAT_RETURN_SEM();

    USB_DiskCtrlCmdQueueSend(&cmd);

    if(waitResult) GET_RETURN_SEM(cmd.sem,RES_MAX_WAIT);

    fResult = *cmd.result;
    FREE(cmd.result);

    return fResult;
}

/**
*	@Name  				USB_Read
*	@Author  			KT
* @Description 	��ȡ�ļ�����
*	@para    			*fp 			:	�ļ�ָ��
*								*data			:	����ָ��
*								dLen			:	���ݳ���
*								waitResult: �Ƿ�ȴ�Ӧ��
*/
static FRESULT USB_Read(FIL *fp,uint8_t* data, uint32_t dLen,bool waitResult)
{
    FRESULT fResult;
    UsbCmd_S cmd = {(UsbCmdType_EN)0};

    if(fp == NULL || dLen == 0)
        return (FRESULT)0xFF;


    cmd.type = tRead;
    cmd.dataLen = dLen;
    cmd.fp = fp;
    cmd.result = MALLOC(sizeof(FRESULT));
    *cmd.result = (FRESULT)0xFF;

    if(data == NULL)
        data = MALLOC(dLen);

    cmd.rData = data;

    if(waitResult) cmd.sem = CREAT_RETURN_SEM();

    USB_DiskCtrlCmdQueueSend(&cmd);

    if(waitResult) GET_RETURN_SEM(cmd.sem,RES_MAX_WAIT);

    fResult = *cmd.result;
    FREE(cmd.result);

    return fResult;
}

/*!
 * @func U��ʣ��ռ�(��λ:KB)
 */
static size_t USB_FreeSize(void)
{
    return FreeSize;
}

/*!
 * @func U���ܿռ�(��λ:KB)
 */
static size_t USB_TotalSize(void)
{
    return TotalSize;
}


/**
*	@Name  				USB_GetFileList
*	@Author  			KT
* @Description 	��ȡ��ַ���ļ�/�ļ����б�
*	@para    			*path 		:	�ļ�·��
*								**fList		:	�ļ��б�ָ���ַ
*/
static uint8_t USB_GetFileList(const TCHAR *path,FILINFO **fList)
{
    FRESULT fResult = FR_OK;
    UsbCmd_S cmd = {(UsbCmdType_EN)0};
    uint16_t sLen;

    if(path == NULL)
        return (FRESULT)0xFF;

    cmd.type = tFileList;
    cmd.result = MALLOC(sizeof(FRESULT));
    *cmd.result = (FRESULT)0xFF;

    STR_ALLOC_COPY(cmd.path,path);

    cmd.sem = CREAT_RETURN_SEM();
    USB_DiskCtrlCmdQueueSend(&cmd);
    GET_RETURN_SEM(cmd.sem,RES_MAX_WAIT);

    fResult = *cmd.result;
    FREE(cmd.result);

    if(fResult == FR_OK) {
        *fList = FileList;
        return FileListSize;
    }

    return 0;
}



/*!
 * @func ����U����Ϣ
 */
static void USB_GetUsbDiskInfo()
{
    char *operaField;
    FATFS *fs;
    DWORD freeClusterNumber;
    FRESULT fResult;

    operaField = MALLOC(10);
    sprintf(operaField, "%s", USBDISK_ROOT);
    fResult = f_getfree((char const *)operaField, (DWORD *)&freeClusterNumber, &fs);
    FREE(operaField);
    if(fResult) {
        Log.e("get usb disk info error: %d \r\n",fResult);
        return;
    }
    FreeSize = (freeClusterNumber * (fs->csize) / 2);
    TotalSize = ((fs->n_fatent - 2) * (fs->csize) / 2);
//    FsType = fs->fs_type;

//    debug("USB Disk \'%s\' FAT type = %s\r\n",USBDISK_ROOT,(fs->fs_type == FS_FAT12 ? "FAT12" : (fs->fs_type == FS_FAT16 ? "FAT16" : "FAT32")));
//    debug("The free size: %dKB, the total size:%dKB\r\n",FreeSize,TotalSize);
}

/*!
 * @func �����ļ�ϵͳ
 */
static bool USB_FatfsMount()
{
    char *operaField;
    FRESULT fResult;

    operaField = MALLOC(10);
//    Log.d("Fatfs mount as logiacal driver %d......", USBDISK);
    sprintf(operaField, "%s", USBDISK_ROOT);
    fResult = f_mount(&Fatfs, (char const *)operaField, 0);
    FREE(operaField);
    if (fResult) {
        Log.e("error: %d \r\n",fResult);
        return false;
    }
//    Log.d("success\r\n");
    return true;
}

/*!
 * @func ж���ļ�ϵͳ
 */
static void USB_FatfsUnmount()
{
    char *operaField;
    FRESULT fResult;

    operaField = MALLOC(10);
//    Log.d("Fatfs unmount driver %d......", USBDISK);
    sprintf(operaField, "%s", USBDISK_ROOT);
    fResult = f_unmount((char const *)operaField);
    FREE(operaField);
    if (fResult) {
        Log.e("error: %d \r\n",fResult);
        return ;
    }
//    Log.d("success\r\n");
    return ;
}


/**
*	 @func ���������߳�
*/
static void USB_TaskLauncher(void)
{
    Log.d("usb task launcher!!\r\n");

    if (xTaskCreate(USB_HostEhciTask, "USB_HostEhciTask", USB_HOST_EHCI_TASK_STACK, UsbHostHandle, USB_HOST_EHCI_TASK_PRIO, NULL) != pdPASS) {
        Log.e("create host task error\r\n");
    }
}


static void USB_HostEhciTask(void *pvParameters)
{
    while (1) {
        USB_HostEhciTaskFunction((usb_host_handle)pvParameters);
    }
}


/**
*	 @func ��ʼ���������
*/
static bool USB_CmdQueueInit(void)
{
    UsbDiskCmdQueue = xQueueCreate(USB_CMD_QUEUE_LEN,USB_CMD_ITEM_SIZE);
//    Log.d("usb disk cmd queue init, queue len = %d,item size = %d\r\n",USB_CMD_QUEUE_LEN,USB_CMD_ITEM_SIZE);
    return (bool)(UsbDiskCmdQueue != NULL);
}

void USB_OTG2_IRQHandler(void)
{
    USB_HostEhciIsrFunction(UsbHostHandle);
}

static void USB_HostClockInit(void)
{

    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL, BOARD_USB_PHY_TXCAL45DP, BOARD_USB_PHY_TXCAL45DM,
    };


    if (CONTROLLER_ID == kUSB_ControllerEhci0) {
        CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
        CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, 480000000U);
    } else {
        CLOCK_EnableUsbhs1PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
        CLOCK_EnableUsbhs1Clock(kCLOCK_Usb480M, 480000000U);
    }
    USB_EhciPhyInit(CONTROLLER_ID, BOARD_XTAL0_CLK_HZ, &phyConfig);

}

static void USB_HostIsrEnable(void)
{
    uint8_t irqNumber;

    uint8_t usbHOSTEhciIrq[] = USBHS_IRQS;
    irqNumber = usbHOSTEhciIrq[CONTROLLER_ID - kUSB_ControllerEhci0];
    /* USB_HOST_CONFIG_EHCI */

    /* Install isr, set priority, and enable IRQ. */
#if defined(__GIC_PRIO_BITS)
    GIC_SetPriority((IRQn_Type)irqNumber, USB_HOST_INTERRUPT_PRIORITY);
#else
    NVIC_SetPriority((IRQn_Type)irqNumber, USB_HOST_INTERRUPT_PRIORITY);
#endif
    EnableIRQ((IRQn_Type)irqNumber);
}


static usb_status_t USB_HostEvent(usb_device_handle deviceHandle,
                                  usb_host_configuration_handle configurationHandle,
                                  uint32_t eventCode)
{
    usb_status_t status = kStatus_USB_Success;
    switch (eventCode) {
    case kUSB_HostEventAttach:
        status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
//        Log.d("\r\nUSB_HostEvent:USB Host Event Attach.\r\n");
        if (xTaskCreate(USB_DiskFsMountTask, "USB_DiskFsMountTask", USB_FS_MOUNT_TASK_STACK, &g_MsdFatfsInstance, USB_FS_MOUNT_TASK_PRIO, NULL) != pdPASS) {
            Log.e("create host task error\r\n");
        }
		if(Listener != null)
			Listener(kStatus_DEV_Attached);
        break;

    case kUSB_HostEventNotSupported:
//        Log.d("USB_HostEvent:device not supported.\r\n");
        break;

    case kUSB_HostEventEnumerationDone:
        status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
//        Log.d("USB_HostEvent:USB Host Event Enumeration Done.\r\n");
        break;

    case kUSB_HostEventDetach:
        status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
//        Log.d("USB_HostEvent:USB Host Event Detach.\r\n");
		if(Listener != null)
				Listener(kStatus_DEV_Detached);
        break;

    default:
        break;
    }
    return status;
}



static void USB_MsdTransferCallback(void *param, uint8_t *data, uint32_t dataLength, usb_status_t status)
{
    usb_host_msd_fatfs_instance_t *msdFatfsInstance = (usb_host_msd_fatfs_instance_t *)param;
    if (msdFatfsInstance->runWaitState == kUSB_HostMsdRunWaitSetInterface) { /* set interface finish */
        msdFatfsInstance->runWaitState = kUSB_HostMsdRunIdle;
        msdFatfsInstance->runState = kUSB_HostMsdRunFSMount;
    }
}

static void USB_DiskFsMountTask(void *pvParameters)
{
    while (1) {
        usb_status_t status;
        usb_host_msd_fatfs_instance_t *msdFatfsInstance = (usb_host_msd_fatfs_instance_t *)pvParameters;

        switch (msdFatfsInstance->runState) {
        case kUSB_HostMsdRunSetInterface: /* set msd interface */
            msdFatfsInstance->runState = kUSB_HostMsdRunIdle;
            msdFatfsInstance->runWaitState = kUSB_HostMsdRunWaitSetInterface;
            status = USB_HostMsdSetInterface(msdFatfsInstance->classHandle, msdFatfsInstance->interfaceHandle, 0,
                                             USB_MsdTransferCallback, msdFatfsInstance);
            if (status != kStatus_USB_Success) {
                Log.e("set interface fail\r\n");
            }
            break;

        case kUSB_HostMsdRunFSMount: /* set interface succeed */
            if(USB_FatfsMount()) {
                USB_GetUsbDiskInfo();
//                Log.d("USB Disk \'%s\' FAT type = %s\r\n",USBDISK_ROOT,	(FsType == FS_FAT12 ? "FAT12" :
//                        (FsType == FS_FAT16 ? "FAT16" :
//                         (FsType == FS_FAT32 ? "FAT32" :
//                          (FsType == FS_EXFAT ? "EXFAT" :
//                           "unknow type")))));
//                Log.d("The free size: %dKB, the total size:%dKB\r\n",FreeSize,TotalSize);
                msdFatfsInstance->runState = kUSB_HostMsdRunStorageOperation;

                if (xTaskCreate(USB_CmdProcessTask, "USB_CmdProcessTask", USB_CMD_PROCESS_TASK_STACK,
                                null, USB_CMD_PROCESS_TASK_PRIO, &CmdProcessTaskHandler) != pdPASS) {

                    Log.e("create host task error\r\n");
                }

				if(Listener != null)
					Listener(kStatus_FS_Mounted);

                UsbDeviceConnected = true;
            }
            break;

        case kUSB_HostMsdRunFSUnmount:
            UsbDeviceConnected = false;
            vTaskDelete(CmdProcessTaskHandler);
            USB_FatfsUnmount();
            msdFatfsInstance->runState = kUSB_HostMsdRunIdle;
			if(Listener != null)
					Listener(kStatus_DEV_Idle);
            vTaskDelete(null);
            break;

        case kUSB_HostMsdRunStorageOperation:
            break;

        default:
            break;
        }
        DELAY(100);
    }
}

#if (defined(USB_STREAM_ENABLE) && USB_STREAM_ENABLE)
/**
*	@Name  				USB_setOutputStream
*	@Author  			KT
* @Description 	����USB���������
*	@para    			*path 		:	�ļ�·��
*								*dataStreamHandler		:	���������ƾ��
*
* @return				���ر����������������������ڹر�������ʹ��;
*/
static uint8_t USB_setOutputStream(const char *path,DataStreamHandler_S *dataStreamHandler)
{
    uint8_t index = 0xFF,pathLen = 0;

    if(currentStreamNum >= USB_MAX_DATA_STREAM_NUM || dataStreamHandler == NULL || path == NULL) {
        return index;
    }

    taskENTER_CRITICAL();
    for(index = 0; index < USB_MAX_DATA_STREAM_NUM; index++) {
        if(usbDataStream[index] == NULL) {
            usbDataStream[index] = MALLOC(sizeof(UsbDataStream_S));

            usbDataStream[index]->type = output;
            usbDataStream[index]->fp = NULL;

            pathLen = strlen(path);
            usbDataStream[index]->filePath = MALLOC(pathLen+1);
            memset(usbDataStream[index]->filePath,'\0',pathLen+1);
            memcpy(usbDataStream[index]->filePath,path,pathLen);

            usbDataStream[index]->dataStream = dataStreamHandler;

            currentStreamNum += 1;
			
			usbDataStream[index]->buffer = Streambuff[index];
            usbDataStream[index]->enable = true;

            if (xTaskCreate(USB_DataStreamProcessTask, "USBStreamProcess", USB_STREAM_TASK_STACK,
                            usbDataStream[index], USB_STREAM_TASK_PRIO, &usbDataStream[index]->taskHandler) != pdPASS) {

                Log.e("create USBStreamProcess error\r\n");
            }

            Log.d("set Output Data Stream path = \"%s\" \r\n",path);
            break;
        }
    }

    index = (index >= USB_MAX_DATA_STREAM_NUM) ? 0xFF : index;
    taskEXIT_CRITICAL();

    return index;
}


/**
* @Name  			USB_setInputStream
* @Author  			KT
* @Description 		����USB����������
* @para    			*path :�ļ�·��
*					dataStreamHandler:���������ƾ��
*
* @return			���ر����������������������ڹر�������ʹ��;
*/
static uint8_t USB_setInputStream(const char *path,DataStreamHandler_S *dataStreamHandler)
{
    uint8_t index = 0xFF,pathLen = 0;

    if(currentStreamNum >= USB_MAX_DATA_STREAM_NUM || dataStreamHandler == NULL || path == NULL) {
        return index;
    }

    taskENTER_CRITICAL();
    for(index = 0; index < USB_MAX_DATA_STREAM_NUM; index++) {
        if(usbDataStream[index] == NULL) {
            usbDataStream[index] = MALLOC(sizeof(UsbDataStream_S));

            usbDataStream[index]->type = input;
            usbDataStream[index]->fp = NULL;

            pathLen = strlen(path);
            usbDataStream[index]->filePath = MALLOC(pathLen+1);
            memset(usbDataStream[index]->filePath,'\0',pathLen+1);
            memcpy(usbDataStream[index]->filePath,path,pathLen);

            usbDataStream[index]->dataStream = dataStreamHandler;

            currentStreamNum += 1;
			usbDataStream[index]->buffer = Streambuff[index];
            usbDataStream[index]->enable = true;

            if (xTaskCreate(USB_DataStreamProcessTask, "USBStreamProcess", USB_STREAM_TASK_STACK,
                            usbDataStream[index], USB_STREAM_TASK_PRIO, &usbDataStream[index]->taskHandler) != pdPASS) {

                Log.e("create USBStreamProcess error\r\n");
            }

            Log.d("set Iutput Data Stream path = \"%s\" \r\n",path);
            break;
        }
    }

    index = (index >= USB_MAX_DATA_STREAM_NUM) ? 0xFF : index;
    taskEXIT_CRITICAL();

    return index;
}


/**
* @Name  		USB_closeDataStream
* @Author  		KT
* @Description 	ɾ��USB���������
* @para    		index 	: ������������������
*
* @return
*/
static void USB_closeDataStream(uint8_t index)
{
    if(usbDataStream[index] == NULL)
        return;

    /* �����ź������ȴ��źŷ��ر�ʾ����������رղ��� */
    usbDataStream[index]->sem = CREAT_RETURN_SEM();
    usbDataStream[index]->enable = false;
    GET_RETURN_SEM(usbDataStream[index]->sem,RES_MAX_WAIT);

    Log.d("finish data stream %d close\r\n",index);

    vTaskDelete(usbDataStream[index]->taskHandler);

    FREE(usbDataStream[index]);
    usbDataStream[index] = NULL;
    currentStreamNum = currentStreamNum > 0 ? currentStreamNum - 1 : 0;
}



static void USB_DataStreamProcessTask(void *pvParameters)
{
    UsbDataStream_S *usbStream;

    /* �ļ�������ر��� */
    FRESULT fResult;
    FILINFO fInfo;

    /* �����������ر��� */
    DataStreamHandler_S *stream;
	uint32_t size;

    ERR_CHECK_DBG(pvParameters != null,"pvParameters is null!!\r\n",vTaskDelete(null));

    usbStream = (UsbDataStream_S *)pvParameters;
    stream = usbStream->dataStream;

    Log.d("Usb DataStream Task launche \r\n\r\n");

    /* �����(������ݵ�UsbDisk) */
    if(usbStream->type == output) {
    	UINT bw;
	
        if(usbStream->fp == NULL && usbStream->filePath != NULL) {
			
            usbStream->fp = MALLOC(sizeof(FIL));
			
			xSemaphoreTake(UsbDiskOpearatSync,MAX_NUM);
		
            fResult = f_open(usbStream->fp,usbStream->filePath,FA_WRITE|FA_OPEN_EXISTING);

            /* Ŀ���ļ������ڣ��½��ļ� */
            if(fResult) {
                fResult = f_open(usbStream->fp,usbStream->filePath,FA_WRITE|FA_CREATE_NEW);
                Log.d("open file path = \"%s\" not exist! creat new file ....",usbStream->filePath);

                if(fResult) {
                    FREE(usbStream->fp);
                    Log.e("creat fail!\r\n");
					xSemaphoreGive(UsbDiskOpearatSync);
                    vTaskDelete(null);
                }

                Log.d("success!  \r\n\r\n");
            }

            /* Ŀ���ļ����ڣ����ļ������ļ�ָ��ָ����� */
            else {
                f_stat(usbStream->filePath,&fInfo);
                f_lseek(usbStream->fp,fInfo.fsize);
                Log.d("open file path = \"%s\" exist! file size = %d \r\n",usbStream->filePath,fInfo.fsize);
            }

			xSemaphoreGive(UsbDiskOpearatSync);
        }

        while(1) {

            xSemaphoreTake(UsbDiskOpearatSync,MAX_NUM);


            /* ������д�����������ݴ�Сռ�ܴ�С�ٷֱȣ�����50% ���� ��ʱ����10����λ*/
//            if( (stream->filledSize * 100 / stream->totalSize) >= 50 ||  usbStream->timeCount > 100)
            if(stream->filledSize > USB_STREAM_BUF_SIZE) {
//                Log.d("stream fill precent %d\%  timeconut = %d\r\n",(stream->filledSize * 100 / stream->totalSize),usbStream->timeCount);

                if(stream->filledSize == 0)
                    continue;

                size = DataStream.read(stream,usbStream->buffer,USB_STREAM_BUF_SIZE);

                fResult = f_write(usbStream->fp,usbStream->buffer,size,&bw);
                f_sync(usbStream->fp);

                usbStream->timeCount = 0;
            }

            else {
//										Log.d("usb output stream:empty size = %d  fill size = %d\r\n",stream->emptySize,stream->filledSize);
                usbStream->timeCount ++;
            }

            if(usbStream->enable == false) {
                Log.d("close data stream ");

                if(usbStream->fp != NULL) {
                    fResult = f_close(usbStream->fp);
                    FREE(usbStream->fp);
                }
                if(usbStream->filePath != NULL) {
                    Log.d(" path = \"%s\"",usbStream->filePath);
                    FREE(usbStream->filePath);
                }

                if(usbStream->sem != NULL) {
                    xSemaphoreGive(usbStream->sem);
                }
				printf("\r\n");

            }

            xSemaphoreGive(UsbDiskOpearatSync);
            DELAY(5);
        }
    }

	 /* ������ */
     else if(usbStream->type == input) {
	 	UINT br;
		 
		if(usbStream->fp == NULL && usbStream->filePath != NULL) {
			
            usbStream->fp = MALLOC(sizeof(FIL));

			xSemaphoreTake(UsbDiskOpearatSync,MAX_NUM);
		
            fResult = f_open(usbStream->fp,usbStream->filePath,FA_READ|FA_OPEN_EXISTING);

			xSemaphoreGive(UsbDiskOpearatSync);
            /* Ŀ���ļ������ڣ��½��ļ� */
            if(fResult) {
				Log.e("file is not exist!!\r\n");
				vTaskDelete(null);
			}

        }

        while(1) {
			xSemaphoreTake(UsbDiskOpearatSync,MAX_NUM);
			
			while(stream->emptySize == 0){
				DELAY(10);
				Log.d("stream full\r\n");
			}
		
			f_read(usbStream->fp,usbStream->buffer, USB_STREAM_BUF_SIZE, &br);
			if(br == 0){
				usbStream->enable = false;
			}
			DataStream.write(stream,usbStream->buffer,br);

			if(usbStream->enable == false) {
                Log.d("close data stream ");

                if(usbStream->fp != NULL) {
                    fResult = f_close(usbStream->fp);
                    FREE(usbStream->fp);
                }
                if(usbStream->filePath != NULL) {
                    Log.d(" path = \"%s\"",usbStream->filePath);
                    FREE(usbStream->filePath);
                }

                if(usbStream->sem != NULL) {
                    xSemaphoreGive(usbStream->sem);
                }

                printf("\r\n");
            }

			xSemaphoreGive(UsbDiskOpearatSync);
        	DELAY(5);
		}
		
	 	
               
     }
}
#endif


static void USB_CmdProcessTask(void *pvParameters)
{

    FRESULT fResult;
    UINT bw;
    UsbCmd_S cmd;
    DIR dir;
    FILINFO fileInfo;

    ERR_CHECK_DBG(UsbDiskCmdQueue != null, "UsbDiskCmdQueue is null!!\r\n",vTaskSuspend(null));

//	Log.d("\r\nUsb cmd process task launcher..\r\n");

    /* USB������д��� */
    while(1) {
        xQueueReceive(UsbDiskCmdQueue, &cmd, MAX_NUM);

        xSemaphoreTake(UsbDiskOpearatSync,MAX_NUM);

        switch(cmd.type) {
        case tNone:
            break;
        case tOpen:
            fResult = f_open(cmd.fp,_T(cmd.path),cmd.mode);
            FREE(cmd.path);
            if(cmd.result != NULL)	*cmd.result = fResult;
            if(cmd.sem != NULL)	xSemaphoreGive(cmd.sem);
            break;
        case tMkdir:
            fResult = f_mkdir(_T(cmd.path));
            FREE(cmd.path);
            if(cmd.result != NULL)	*cmd.result = fResult;
            if(cmd.sem != NULL)	xSemaphoreGive(cmd.sem);
            break;
        case tWrite:
            fResult = f_write(cmd.fp,cmd.wData,cmd.dataLen,&bw);
            FREE(cmd.wData);
            if(cmd.result != NULL)	*cmd.result = fResult;
            if(cmd.sem != NULL)	xSemaphoreGive(cmd.sem);
            break;
        case tClose:
            fResult = f_close(cmd.fp);
            if(cmd.result != NULL)	*cmd.result = fResult;
            if(cmd.sem != NULL)	xSemaphoreGive(cmd.sem);
            break;
        case tRead:
            fResult = f_read (cmd.fp, cmd.rData, cmd.dataLen,&bw);
            if(cmd.result != NULL)	*cmd.result = fResult;
            if(cmd.sem != NULL)	xSemaphoreGive(cmd.sem);
            break;
        case tWaitConnect:
            if(cmd.result != NULL)	*cmd.result = fResult;
            if(cmd.sem != NULL)	xSemaphoreGive(cmd.sem);
            break;
        case tIsExist:
            fResult = f_open(cmd.fp,_T(cmd.path),FA_READ|FA_OPEN_EXISTING);
            f_close(cmd.fp);
            if(cmd.result != NULL)	*cmd.result = fResult;
            if(cmd.sem != NULL)	xSemaphoreGive(cmd.sem);
            break;
        case tFileInfo:
            fResult = f_stat(cmd.path,cmd.fInfo);
            if(cmd.result != NULL)	*cmd.result = fResult;
            if(cmd.sem != NULL)	xSemaphoreGive(cmd.sem);
            break;
        case tFileList:
            memset(FileList,0,sizeof(FileList));
			FileListSize = 0;
            fResult = f_opendir(&dir, cmd.path);
            if(fResult && cmd.result != NULL) {
                *cmd.result = fResult;
                if(cmd.sem != NULL)	xSemaphoreGive(cmd.sem);
                break;
            }
            while (1) {
                fResult = f_readdir(&dir, &fileInfo);
                if ((fResult) || (!fileInfo.fname[0]) || FileListSize >= USB_FILEINFO_STACK_SIZE)
                    break;

                memcpy(&FileList[FileListSize],&fileInfo,sizeof(FILINFO));
                FileListSize++;
            }
            if(cmd.result != NULL)	*cmd.result = FR_OK;
            if(cmd.sem != NULL)	xSemaphoreGive(cmd.sem);
            break;

        case tRename:
            fResult = f_rename(cmd.path,cmd.newPath);
            if(cmd.result != NULL)	*cmd.result = FR_OK;
            if(cmd.sem != NULL)	xSemaphoreGive(cmd.sem);
            break;
        default:
            Log.e("wrong cmd type:%d\r\n",cmd.type);
            break;
        }

        xSemaphoreGive(UsbDiskOpearatSync);
    }

}




