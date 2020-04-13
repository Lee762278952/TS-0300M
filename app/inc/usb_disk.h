#ifndef __USBDISKAPP_H__
#define __USBDISKAPP_H__

#include "stdint.h"
#include "ff.h"
#include "global_config.h"
#include "data_stream.h"
#include "app.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define USBDISK_ROOT									"1:"
#define RES_MAX_WAIT									MAX_NUM

typedef enum{
	kMode_Usb_Read = FA_READ,
	kMode_Usb_Write = FA_WRITE,
	kMode_Usb_OpenExist = FA_OPEN_EXISTING,
	kMode_Usb_CreateNew = FA_CREATE_NEW,
	kMode_Usb_CreateAlways = FA_CREATE_ALWAYS,
	kMode_Usb_OpenAlways = FA_OPEN_ALWAYS,
	kMode_Usb_OpenAppend = FA_OPEN_APPEND,
}UsbOpenMode_EN;

/* USBÁ¬½Ó×´Ì¬¼àÌýº¯Êý */
typedef void(*USB_StaListener)(status_t sta);


typedef struct {
//    void (*init)(void);
//	void (*launch)(void);
	AppLauncher_S *launcher;

	bool (*devConnected)(void);
	bool (*isExist)(const char *);
	void (*waitDevConnect)(void);
	void (*rename)(const char *,const char *);
	void (*setFileDateTime)(const char *path,uint8_t year,uint8_t month,uint8_t day,uint8_t hour,uint8_t min);
	FRESULT (*fileInfo)(const char *path,FILINFO *filInfo);
    FRESULT (*mkdir)(const char *, bool );
    FRESULT (*open)(FIL *,const char *,BYTE,bool );
    FRESULT (*write)(FIL *,const uint8_t*, uint32_t,bool );
    FRESULT (*close)(FIL *,bool );
	FRESULT (*read)(FIL *,uint8_t*, uint32_t,bool );
	size_t  (*freeSize)(void);
	size_t  (*totalSize)(void);
	uint8_t (*getFileList)(const TCHAR *,FILINFO **);
	void (*setListener)(USB_StaListener listener);
	
#if (defined(USB_STREAM_ENABLE) && USB_STREAM_ENABLE)	
	uint8_t (*setOutputStream)(const char *,DataStreamHandler_S *);
	uint8_t (*setInputStream)(const char *,DataStreamHandler_S *);
	void (*closeDataStream)(uint8_t );
#endif
} UsbDisk_S;

/*! @brief host app device attach/detach status */
typedef enum {
    kStatus_DEV_Idle = 0, /*!< there is no device attach/detach */
    kStatus_DEV_Attached, /*!< device is attached */
    kStatus_FS_Mounted,    /*!< File System is mounted */
    kStatus_DEV_Detached, /*!< device is detached */
} UsbState_EN;

#if (defined(USB_STREAM_ENABLE) && USB_STREAM_ENABLE)
typedef enum {
	input = 0,
	output,
} StreamType_EN;
#endif




/*******************************************************************************
 * API
 ******************************************************************************/
extern UsbDisk_S UsbDisk;

#endif
