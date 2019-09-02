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
#include "debug.h"
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
#define CONTROLLER_ID 								USB_INTERFACE
#endif

/* �������ȼ��Ͷ�ջ��С */
#define USB_HOST_INTERRUPT_PRIORITY 	(3U)
#define USB_TASK_STACK_DEPTH					(2000L / sizeof(portSTACK_TYPE))
#define USB_TASK_PRIORITY							(3U)

/* U�̿��Ʋ������г��� */
#define USB_CMD_QUEUE_LEN      	 			30
#define USB_CMD_ITEM_SIZE    					(sizeof(CtrlCmd))

/* �ļ��б���󳤶� */
#define USB_FILEINFO_STACK_SIZE				60

/* �����ڴ漰�����ڴ� */
#define MEM_ALLOC_COPY(dest,src,size)	{if(src!=NULL&&size!=0){do{dest=MALLOC(size);memset(dest,'\0',size);memcpy(dest,src,size);}while(0);}}

/* �����ڴ漰�����ַ��� */
#define STR_ALLOC_COPY(dest,src)			{if(src!=NULL){do{sLen=strlen(src);dest=MALLOC(sLen+1);memset(dest,'\0',sLen+1);memcpy(dest,src,sLen);}while(0);}}

/* �����ȴ�USB������������ź��� */
#define CREAT_RETURN_SEM()							xSemaphoreCreateBinary()

/* �ȴ���ɾ��USB������������ź��� */
#define GET_RETURN_SEM(sem,time)				{do{xSemaphoreTake(sem,time);vSemaphoreDelete(sem);sem=NULL;}while(0);}

/* USB��������ṹ�� */
typedef struct {
    usb_host_msd_fatfs_instance_t* msdFatfsInstance;
    usb_host_handle hostHandle;
} UsbFuncPara;

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
} CmdType;

/* USB��������ṹ�� */
typedef struct
{
    CmdType type;
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
} CtrlCmd;

/* USB���������ƽṹ�� */
typedef struct
{
    StreamType_EN type;
    char *filePath;
    FIL *fp;
    DataStreamHandler_S *dataStream;
    int8_t timeCount;
    bool enable;
    SemaphoreHandle_t *sem;
} StreamHandler;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void USB_DiskApp(void *param);
static void USB_HostClockInit(void);
static void USB_HostIsrEnable(void);
static void USB_DiskCtrlCmdQueueSend(CtrlCmd *cmd);
static void USB_DiskInit(void);
static bool USB_DiskDeviceConnected(void);
static void USB_DiskWaitDevConnect(void);
static bool USB_DiskFileIsExist(const char *path);
static uint32_t USB_DiskFileSize(const char *path);
static void USB_DiskRename(const char *path,const char *newPath);
static void USB_HostEhciTask(void *param);
static void USB_TaskLauncher(void);
static void USB_ApplicationTask(void *param);
static bool USB_CmdQueueInit(void);
static FRESULT USB_Mkdir(const char *path,bool waitResul);
static FRESULT USB_Open(FIL *fp,const char *path,BYTE mode,bool waitResult);
static FRESULT USB_Write(FIL *fp,const uint8_t* data, uint32_t dLen,bool waitResult);
static FRESULT USB_Close(FIL *fp,bool waitResult);
static FRESULT USB_Read(FIL *fp,uint8_t* data, uint32_t dLen,bool waitResult);
static uint8_t USB_setOutputStream(const char *path,DataStreamHandler_S *dataStreamHandler);
static void USB_closeDataStream(uint8_t index);
static size_t USB_FreeSize(void);
static size_t USB_TotalSize(void);
static uint8_t USB_GetFileList(const TCHAR *path,FILINFO **fList);
static usb_status_t USB_HostEvent(usb_device_handle deviceHandle,
                                  usb_host_configuration_handle configurationHandle,
                                  uint32_t eventCode);

//static FRESULT USB_DiskCtrlCmd(CmdType 	type,
//                               FIL 				*fp,
//                               const char *path,
//                               uint8_t 		*wData,
//                               uint8_t		*rData,
//                               uint32_t 	dataLen,
//                               BYTE 			mode,
//                               bool 		waitResult);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* �ļ�ϵͳ��ر��� */
static FATFS 		fatfs;
static size_t 	freeSize,totalSize;
static BYTE 		fsType;
static FILINFO 	fileList[USB_FILEINFO_STACK_SIZE];
static uint32_t fileListSize;

/* USB����������� */
static QueueHandle_t usbDiskCmdQueue;

/* ���롢������������ָ������(���ݺ궨���������) */
static StreamHandler *usbDataStream[USB_MAX_DATA_STREAM_NUM] = {NULL};
static uint8_t currentStreamNum = 0;

/* usb�豸�����־ */
static bool usbDeviceConnected = false;

extern usb_host_msd_fatfs_instance_t g_MsdFatfsInstance;
usb_host_handle g_HostHandle;
UsbFuncPara* g_usbFuncPara;



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
    .setOutputStream = USB_setOutputStream,
    .closeDataStream = USB_closeDataStream,
};
/*******************************************************************************
 * Code
 ******************************************************************************/
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

    status = USB_HostInit(CONTROLLER_ID, &g_HostHandle, USB_HostEvent);
    if (status != kStatus_USB_Success)
    {
        debug("host init error\r\n");
        return;
    }
    USB_HostIsrEnable();

    g_usbFuncPara->msdFatfsInstance = &g_MsdFatfsInstance;
    g_usbFuncPara->hostHandle = g_HostHandle;

    if(!USB_CmdQueueInit())
        debug("usb cmd queue int fail!\r\n");

    debug("USB disk init finish!!\r\n");

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
static bool USB_DiskDeviceConnected(void) {
    return usbDeviceConnected;
}

/**
*	@Name  				USB_DiskWaitDevConnect
*	@Author  			KT
* @Description 	�ȴ�usb�豸����
*	@Para
* @Return
*
*/
static void USB_DiskWaitDevConnect(void) {
    CtrlCmd cmd = {(CmdType)0};

    if(usbDeviceConnected)
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
static bool USB_DiskFileIsExist(const char *path) {
    CtrlCmd cmd = {(CmdType)0};
    FRESULT fResult;
    FIL *fp;
    uint16_t sLen;

    if(!usbDeviceConnected || path == NULL)
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
static uint32_t USB_DiskFileSize(const char *path) {
    FRESULT fResult;
    CtrlCmd cmd = {(CmdType)0};
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
static void USB_DiskRename(const char *path,const char *newPath) {
    FRESULT fResult;
    CtrlCmd cmd = {(CmdType)0};
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

        fResult = *cmd.result;
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
static FRESULT USB_Mkdir(const char *path,bool waitResult) {
    FRESULT fResult;
    CtrlCmd cmd = {(CmdType)0};
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
static void USB_DiskCtrlCmdQueueSend(CtrlCmd *cmd) {
    if(xQueueSend(usbDiskCmdQueue, cmd, RES_MAX_WAIT) != pdPASS)
        debug("queue send fail!\r\n");
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
static FRESULT USB_Open(FIL *fp,const char *path,BYTE mode,bool waitResult) {
    FRESULT fResult;
    CtrlCmd cmd = {(CmdType)0};
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
static FRESULT USB_Close(FIL *fp,bool waitResult) {
    FRESULT fResult;
    CtrlCmd cmd = {(CmdType)0};

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
static FRESULT USB_Write(FIL *fp,const uint8_t* data, uint32_t dLen,bool waitResult) {
    FRESULT fResult;
    CtrlCmd cmd = {(CmdType)0};

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
static FRESULT USB_Read(FIL *fp,uint8_t* data, uint32_t dLen,bool waitResult) {
    FRESULT fResult;
    CtrlCmd cmd = {(CmdType)0};

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
static size_t USB_FreeSize(void) {
    return freeSize;
}

/*!
 * @func U���ܿռ�(��λ:KB)
 */
static size_t USB_TotalSize(void) {
    return totalSize;
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
    CtrlCmd cmd = {(CmdType)0};
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

		if(fResult == FR_OK){
			*fList = fileList;
			return fileListSize;
		}
		
		return 0;
}


/**
*	@Name  				USB_setOutputStream
*	@Author  			KT
* @Description 	����USB���������
*	@para    			*path 		:	�ļ�·��
*								*dataStreamHandler		:	���������ƾ��
*
* @return				���ر����������������������ڹر�������ʹ��;
*/
static uint8_t USB_setOutputStream(const char *path,DataStreamHandler_S *dataStreamHandler) {
    uint8_t index = 0xFF,pathLen = 0;

    if(currentStreamNum >= USB_MAX_DATA_STREAM_NUM || dataStreamHandler == NULL || path == NULL){
        return index;
				}

    taskENTER_CRITICAL();
    for(index = 0; index < USB_MAX_DATA_STREAM_NUM; index++) {
        if(usbDataStream[index] == NULL) {
            usbDataStream[index] = MALLOC(sizeof(StreamHandler));

            usbDataStream[index]->type = output;
            usbDataStream[index]->fp = NULL;

            pathLen = strlen(path);
            usbDataStream[index]->filePath = MALLOC(pathLen+1);
            memset(usbDataStream[index]->filePath,'\0',pathLen+1);
            memcpy(usbDataStream[index]->filePath,path,pathLen);

            usbDataStream[index]->dataStream = dataStreamHandler;

            currentStreamNum += 1;

            usbDataStream[index]->enable = true;
            debug("set Output Data Stream path = \"%s\" \r\n",path);
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
static void USB_closeDataStream(uint8_t index) {

    if(usbDataStream[index] == NULL)
        return;

    /* �����ź������ȴ��źŷ��ر�ʾ����������رղ��� */
    usbDataStream[index]->sem = CREAT_RETURN_SEM();
    usbDataStream[index]->enable = false;
    GET_RETURN_SEM(usbDataStream[index]->sem,RES_MAX_WAIT);

    debug("finish data stream %d close\r\n",index);

    FREE(usbDataStream[index]);
    usbDataStream[index] = NULL;
    currentStreamNum = currentStreamNum > 0 ? currentStreamNum - 1 : 0;

}



/*!
 * @func ����U����Ϣ
 */
static void USB_GetUsbDiskInfo() {
    char *operaField;
    FATFS *fs;
    DWORD freeClusterNumber;
    FRESULT fResult;

    operaField = MALLOC(10);
    sprintf(operaField, "%s", USBDISK_ROOT);
    fResult = f_getfree((char const *)operaField, (DWORD *)&freeClusterNumber, &fs);
    FREE(operaField);
    if(fResult) {
        debug("get usb disk info error: %d \r\n",fResult);
        return;
    }
    freeSize = (freeClusterNumber * (fs->csize) / 2);
    totalSize = ((fs->n_fatent - 2) * (fs->csize) / 2);
    fsType = fs->fs_type;

//    debug("USB Disk \'%s\' FAT type = %s\r\n",USBDISK_ROOT,(fs->fs_type == FS_FAT12 ? "FAT12" : (fs->fs_type == FS_FAT16 ? "FAT16" : "FAT32")));
//    debug("The free size: %dKB, the total size:%dKB\r\n",freeSize,totalSize);
}

/*!
 * @func �����ļ�ϵͳ
 */
static bool USB_FatfsMount() {
    char *operaField;
    FRESULT fResult;

    operaField = MALLOC(10);
    debug("Fatfs mount as logiacal driver %d......", USBDISK);
    sprintf(operaField, "%s", USBDISK_ROOT);
    fResult = f_mount(&fatfs, (char const *)operaField, 0);
    FREE(operaField);
    if (fResult) {
        debug("error: %d \r\n",fResult);
        return false;
    }
    debug("success\r\n");
    return true;
}

/*!
 * @func ж���ļ�ϵͳ
 */
static void USB_FatfsUnmount() {
    char *operaField;
    FRESULT fResult;

    operaField = MALLOC(10);
    debug("Fatfs unmount driver %d......", USBDISK);
    sprintf(operaField, "%s", USBDISK_ROOT);
    fResult = f_unmount((char const *)operaField);
    FREE(operaField);
    if (fResult) {
        debug("error: %d \r\n",fResult);
        return ;
    }
    debug("success\r\n");
    return ;
}


/**
*	 @func ���������߳�
*/
static void USB_TaskLauncher(void) {
	debug("usb task launcher!!\r\n");
    //*****USB_HostEhciTask���ȼ������USB_ApplicationTask��******/
    if (xTaskCreate(USB_HostEhciTask, "USB_HostEhciTask", USB_TASK_STACK_DEPTH, g_usbFuncPara, USB_TASK_PRIORITY + 1, NULL) != pdPASS)
    {
        debug("create host task error\r\n");
    }
    if (xTaskCreate(USB_ApplicationTask, "USB_ApplicationTask", USB_TASK_STACK_DEPTH, g_usbFuncPara, USB_TASK_PRIORITY, NULL) != pdPASS)
    {
        debug("create host task error\r\n");
    }
}

/**
*	 @func ��ʼ���������
*/
static bool USB_CmdQueueInit(void) {
    usbDiskCmdQueue = xQueueCreate(USB_CMD_QUEUE_LEN,USB_CMD_ITEM_SIZE);
//    debug("usb disk cmd queue init, queue len = %d,item size = %d\r\n",USB_CMD_QUEUE_LEN,USB_CMD_ITEM_SIZE);
    return (bool)(usbDiskCmdQueue != NULL);
}

void USB_OTG2_IRQHandler(void)
{
    USB_HostEhciIsrFunction(g_HostHandle);
}

static void USB_HostClockInit(void)
{

    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL, BOARD_USB_PHY_TXCAL45DP, BOARD_USB_PHY_TXCAL45DM,
    };


    if (CONTROLLER_ID == kUSB_ControllerEhci0)
    {
        CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
        CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, 480000000U);
    }
    else
    {
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
    switch (eventCode)
    {
    case kUSB_HostEventAttach:
        status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
        debug("\r\nUSB_HostEvent:USB Host Event Attach.\r\n");
        break;

    case kUSB_HostEventNotSupported:
        debug("USB_HostEvent:device not supported.\r\n");
        break;

    case kUSB_HostEventEnumerationDone:
        status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
        debug("USB_HostEvent:USB Host Event Enumeration Done.\r\n");
        break;

    case kUSB_HostEventDetach:
        status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
        debug("USB_HostEvent:USB Host Event Detach.\r\n");
        break;

    default:
        break;
    }
    return status;
}



static void USB_HostEhciTask(void *param)
{
    while (1)
    {
        USB_HostEhciTaskFunction(((UsbFuncPara *)param)->hostHandle);
//				debug("USB_HostEhciTaskFunction\r\n");
    }
}

static void USB_ApplicationTask(void *param) {
    while (1)
    {
        USB_DiskApp(((UsbFuncPara *)param)->msdFatfsInstance);
        DELAY(20);
    }
}

void USB_AppCallback(void *param, uint8_t *data, uint32_t dataLength, usb_status_t status)
{
    usb_host_msd_fatfs_instance_t *msdFatfsInstance = (usb_host_msd_fatfs_instance_t *)param;

//	debug("void USB_HostMsdControlCallback");
    if (msdFatfsInstance->runWaitState == kUSB_HostMsdRunWaitSetInterface) /* set interface finish */
    {
        msdFatfsInstance->runWaitState = kUSB_HostMsdRunIdle;
        msdFatfsInstance->runState = kUSB_HostMsdRunFSMount;
    }
//    controlIng = 0;
//    controlStatus = status;
}

/**/
static void USB_DataStreamProcess(void) {
    uint8_t index;


    /* �ļ�������ر��� */
    UINT bw;
    FRESULT fResult;
    FILINFO fInfo;

    /* �����������ر��� */
    DataStreamHandler_S *stream;
    uint8_t *head,*end;
    uint32_t dLen,fillSize;

    if(currentStreamNum != 0) {
        for(index = 0; index < USB_MAX_DATA_STREAM_NUM; index++) {

            if(usbDataStream[index] == NULL || usbDataStream[index]->dataStream == NULL) {
                continue;
            }

            if(usbDataStream[index]->fp == NULL && usbDataStream[index]->filePath != NULL) {
                usbDataStream[index]->fp = MALLOC(sizeof(FIL));

                fResult = f_open(usbDataStream[index]->fp,usbDataStream[index]->filePath,FA_WRITE|FA_OPEN_EXISTING);

                /* Ŀ���ļ������ڣ��½��ļ� */
                if(fResult) {
                    fResult = f_open(usbDataStream[index]->fp,usbDataStream[index]->filePath,FA_WRITE|FA_CREATE_NEW);
                    debug("open file path = \"%s\" not exist! creat new file ....",usbDataStream[index]->filePath);

                    if(fResult) {
                        FREE(usbDataStream[index]->fp);
                        debug("creat fail!\r\n");
                        continue;
                    }

                    debug("success!  \r\n\r\n");
                }

                /* Ŀ���ļ����ڣ����ļ������ļ�ָ��ָ����� */
                else {
                    f_stat(usbDataStream[index]->filePath,&fInfo);
                    f_lseek(usbDataStream[index]->fp,fInfo.fsize);
                    debug("open file path = \"%s\" exist! file size = %d \r\n",usbDataStream[index]->filePath,fInfo.fsize);
                }
            }

//						taskENTER_CRITICAL();
            stream = usbDataStream[index]->dataStream;


            /* �����(������ݵ�UsbDisk) */
            if(usbDataStream[index]->type == output) {


                /* ������д�����������ݴ�Сռ�ܴ�С�ٷֱȣ�����50% ���� ��ʱ����50����λ����20ms������ʱ���� ��Լ1S��*/
                if( (stream->filledSize * 100 / stream->totalSize) >= 50 ||  usbDataStream[index]->timeCount > 50)	{
                    /* �����ʱ�� */
                    usbDataStream[index]->timeCount = 0;

                    /* ����ָ�������ֵ�������ڴ�������б��ı�����������е�ָ������� */
                    taskENTER_CRITICAL();
                    head = stream->head;
                    end = stream->end;
                    fillSize = stream->filledSize;
                    taskEXIT_CRITICAL();

                    if(stream->filledSize != 0 && head < end) {
                        dLen = (end - head >= fillSize) ? fillSize : end - head;

//                        debug("###write data len = %d to file \"%s\" ... ",dLen,usbDataStream[index]->filePath);
                        fResult = f_write(usbDataStream[index]->fp,head,dLen,&bw);
//                        debug("##done res=%d ##\r\n",fResult);
                        f_sync(usbDataStream[index]->fp);

                        stream->head += dLen;
                        stream->filledSize = (stream->filledSize >= dLen) ? stream->filledSize - dLen : 0;
                        stream->emptySize = (stream->emptySize + dLen <= stream->totalSize) ? stream->emptySize + dLen : stream->totalSize;
                    }

                    else if(stream->filledSize != 0 && head >= end ) {
                        /* ������ͷ��Ϊ�ڴ�����ͷ�� */
                        if(head == stream->data && end == stream->data) {
                            dLen = stream->totalSize;

//                            debug("@@@write data len = %d to file \"%s\"\r\n",dLen,usbDataStream[index]->filePath);
                            fResult = f_write(usbDataStream[index]->fp,head,dLen,&bw);
//                            debug("@@done res=%d @@\r\n",fResult);
                            f_sync(usbDataStream[index]->fp);
                            stream->filledSize = (stream->filledSize >= dLen) ? stream->filledSize - dLen : 0;
                            stream->emptySize = (stream->emptySize + dLen <= stream->totalSize) ? stream->emptySize + dLen : stream->totalSize;


                        }

                        /* ������ͷ����Ϊ�ڴ�����ͷ�����ֿ������ֲ��� */
                        else {
                            dLen = stream->data + stream->totalSize - head;
                            if(dLen) {

//                                debug("$$$write data len = %d to file \"%s\"\r\n",dLen,usbDataStream[index]->filePath);
                                fResult = f_write(usbDataStream[index]->fp,head,dLen,&bw);
//                                debug("$$done res=%d $$\r\n",fResult);
                                f_sync(usbDataStream[index]->fp);
                                stream->filledSize = (stream->filledSize >= dLen) ? stream->filledSize - dLen : 0;
                                stream->emptySize = (stream->emptySize + dLen <= stream->totalSize) ? stream->emptySize + dLen : stream->totalSize;


                            }

                            dLen = end - stream->data;
                            if(dLen) {

//                                debug("***write data len = %d to file \"%s\"\r\n",dLen,usbDataStream[index]->filePath);
                                fResult = f_write(usbDataStream[index]->fp,stream->data,dLen,&bw);
//                                debug("**done res=%d **\r\n",fResult);
                                f_sync(usbDataStream[index]->fp);

                                stream->filledSize = (stream->filledSize >= dLen) ? stream->filledSize - dLen : 0;
                                stream->emptySize = (stream->emptySize + dLen <= stream->totalSize) ? stream->emptySize + dLen : stream->totalSize;
                            }
                            stream->head = end;
                        }
                    }

                    if(usbDataStream[index]->enable == false) {
                        debug("close data stream ");

                        if(usbDataStream[index]->fp != NULL) {
                            fResult = f_close(usbDataStream[index]->fp);
                            FREE(usbDataStream[index]->fp);
                        }
                        if(usbDataStream[index]->filePath != NULL) {
                            debug(" path = \"%s\"",usbDataStream[index]->filePath);
                            FREE(usbDataStream[index]->filePath);
                        }

                        if(usbDataStream[index]->sem != NULL) {
                            xSemaphoreGive(usbDataStream[index]->sem);
                        }

                        debug("\r\n\r\n");
                    }


//										debug("usb output stream:empty size = %d  fill size = %d\r\n",stream->emptySize,stream->filledSize);
                } else {
//										debug("usb output stream:empty size = %d  fill size = %d\r\n",stream->emptySize,stream->filledSize);
                    usbDataStream[index]->timeCount ++;
                }


            }

            /* ������ */
            else if(usbDataStream[index]->type == input) {
            }


//						taskEXIT_CRITICAL();
        }
    }
}

static void USB_DiskApp(void *param) {
    usb_status_t status;
    usb_host_msd_fatfs_instance_t *msdFatfsInstance = (usb_host_msd_fatfs_instance_t *)param;

    FRESULT fResult;
    UINT bw;
    CtrlCmd cmd;
    DIR dir;
    FILINFO fileInfo;

    switch (msdFatfsInstance->runState)
    {
    case kUSB_HostMsdRunIdle:
        break;

    case kUSB_HostMsdRunSetInterface: /* set msd interface */
        msdFatfsInstance->runState = kUSB_HostMsdRunIdle;
        msdFatfsInstance->runWaitState = kUSB_HostMsdRunWaitSetInterface;
        status = USB_HostMsdSetInterface(msdFatfsInstance->classHandle, msdFatfsInstance->interfaceHandle, 0,
                                         USB_AppCallback, msdFatfsInstance);
        if (status != kStatus_USB_Success)
        {
            debug("set interface fail\r\n");
        }
        break;

    case kUSB_HostMsdRunFSMount: /* set interface succeed */
#if ((defined MSD_FATFS_THROUGHPUT_TEST_ENABLE) && (MSD_FATFS_THROUGHPUT_TEST_ENABLE))
        USB_HostMsdFatfsThroughputTest(msdFatfsInstance); /* test throughput */
#else
        if(USB_FatfsMount()) {
            USB_GetUsbDiskInfo();
            debug("USB Disk \'%s\' FAT type = %s\r\n",USBDISK_ROOT,	(fsType == FS_FAT12 ? "FAT12" :
                    (fsType == FS_FAT16 ? "FAT16" :
                     (fsType == FS_FAT32 ? "FAT32" :
                      (fsType == FS_EXFAT ? "EXFAT" :
                       "unknow type")))));
            debug("The free size: %dKB, the total size:%dKB\r\n",freeSize,totalSize);
            msdFatfsInstance->runState = kUSB_HostMsdRunStorageOperation;
            usbDeviceConnected = true;
        }
        break;

    case kUSB_HostMsdRunFSUnmount:
        usbDeviceConnected = false;
        USB_FatfsUnmount();
        msdFatfsInstance->runState = kUSB_HostMsdRunIdle;
        break;

    case kUSB_HostMsdRunStorageOperation:

        /* USB������д��� */
        while(xQueueReceive(usbDiskCmdQueue, &cmd, 0))
        {
//						debug("usbDiskCmdQueue process\r\n");
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
                memset(fileList,0,sizeof(fileList));
                fResult = f_opendir(&dir, cmd.path);
                if(fResult && cmd.result != NULL) {
                    *cmd.result = fResult;
                    if(cmd.sem != NULL)	xSemaphoreGive(cmd.sem);
                    break;
                }
                while (1)
                {
                    fResult = f_readdir(&dir, &fileInfo);
                    if ((fResult) || (!fileInfo.fname[0]) || fileListSize >= USB_FILEINFO_STACK_SIZE)
                        break;
												
                    memcpy(&fileList[fileListSize],&fileInfo,sizeof(FILINFO));
                    fileListSize++;
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
                debug("wrong cmd type:%d\r\n",cmd.type);
                break;
            }
        }

        /* ���������� */
        USB_DataStreamProcess();
//				if(currentStreamNum != 0){
//					for(index = 0;index < USB_MAX_DATA_STREAM_NUM;index++){
//						if(dataStream[index] == NULL)
//							continue;
//
//							if(dataStream[index]->fp == NULL && dataStream[index]->filePath != NULL){
//								dataStream[index]->fp = MALLOC(sizeof(FIL));
//								fResult = f_open(dataStream[index]->fp,_T(dataStream[index]->filePath),FA_READ|FA_WRITE|FA_CREATE_NEW);
//							}
//
//							if()
//					}
//				}


#endif /* MSD_FATFS_THROUGHPUT_TEST_ENABLE */
//        msdFatfsInstance->runState = kUSB_HostMsdRunIdle;
        break;

    default:
        break;
    }
}




#if 0
//static FRESULT USB_DiskCtrlCmd(CmdType 	type,
//                               FIL 				*fp,
//                               const char *path,
//                               uint8_t 		*wData,
//                               uint8_t		*rData,
//                               uint32_t 	dataLen,
//                               BYTE 			mode,
//                               bool 		waitResult)
//{
//    FRESULT fResult;
//    CtrlCmd cmd = {(CmdType)0};
//    uint16_t sLen;

//    cmd.type = type;
//    cmd.mode = mode;
//    cmd.dataLen = dataLen;
//    cmd.fp = (fp == NULL) ? NULL : fp;
//    cmd.result = MALLOC(sizeof(FRESULT));
//    *cmd.result = (FRESULT)0xFF;

//    STR_ALLOC_COPY(cmd.path,path);

//    MEM_ALLOC_COPY(cmd.wData,wData,dataLen);

//    if(type == tRead) {
//        if(cmd.rData == NULL && dataLen != 0)	cmd.rData = MALLOC(dataLen);
//        cmd.rData = rData;
//    }

//    if(waitResult) cmd.sem = CREAT_RETURN_SEM();

//    USB_DiskCtrlCmdQueueSend(&cmd);

//    if(waitResult) GET_RETURN_SEM(cmd.sem,RES_MAX_WAIT);

//    fResult = *cmd.result;
//    FREE(cmd.result);

//    return fResult;
//}
#endif


