#ifndef __WT2000A_H_
#define __WT2000A_H_

#include "drivers_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* 数据应答回调函数 */
typedef void(*Wt2000_AckCallback)(uint8_t data);


typedef enum{
	/* 配置 */
	kType_WtCmd_Disk = 0xD2,
	kType_WtCmd_NeedReturn = 0xBA,
	kType_WtCmd_RecordRate = 0xD4,
	kType_WtCmd_SetVolume = 0xAE,
	kType_WtCmd_SetPlayMode = 0xAF,
	
	/* 播放 */
	kType_WtCmd_PlayFile = 0xA9,
	kType_WtCmd_PlayDirIndex = 0xA4,
	kType_WtCmd_PlayOrPause = 0xAA,
	kType_WtCmd_Stop = 0xAB,
	kType_WtCmd_Next = 0xAC,
	kType_WtCmd_Previous = 0xAD,
	
	/* 录音 */
	kType_WtCmd_RecStart = 0xD8,
	kType_WtCmd_RecStop = 0xD9,

	/* 查询 */
	kType_WtCmd_Volume = 0xC1,
	kType_WtCmd_State = 0xC2,
	kType_WtCmd_FileNum = 0xC5,
	kType_WtCmd_DirFileNum = 0xC6,
	kType_WtCmd_CurrPlayFile = 0xC9,
	kType_WtCmd_LinkSta = 0xCA,
	kType_WtCmd_Space = 0xCE,

}WtCmdType_EN;

typedef enum{
	r128KBPS=0,
	r96KBPS,
	r64KBPS,
	r32KBPS,
}WtMp3BitRate_EN;


typedef enum {
	kStatus_WtPlay_Playing = 1,
	kStatus_WtPlay_Stop = 2,
	kStatus_WtPlay_Pause = 3,
	kStatus_WtPlay_Recording = 4,
}WtPlaySta_EN;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void Wt2000_SetRecvEnable(bool enable);
status_t Wt2000_SetAckCallback(Wt2000_AckCallback callback);
status_t Wt2000_SendCommand(WtCmdType_EN cmd,const char *dir,const char *file,uint8_t *para,uint8_t pLen);
status_t Wt2000_SendConfig(WtCmdType_EN cmd,uint8_t cfg);


#endif

