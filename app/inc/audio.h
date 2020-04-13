#ifndef _AUDIO_H_
#define _AUDIO_H_

#include "stdint.h"
#include "app.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
	/* 录音/播放文件指定目录文件名 */
#define AUDIO_DEFAULT_DIR								"AUDIO"


/* 音频播放或录音状态 */
typedef enum {
	kStatus_Aud_Idle = 0,
	kStatus_Aud_Recording,
	kStatus_Aud_Playing,
}AudStaType_EN;

/* 音频播放或录音事件 */
typedef enum {
	kEvent_Aud_None,
	kEvent_Aud_UsbConnect,
	kEvent_Aud_UsbDisconnect,
	kEvent_Aud_RecordStar,
	kEvent_Aud_PlayStar,
	kEvent_Aud_RecordStop,
	kEvent_Aud_PlayStop,
	kEvent_Aud_RecordErr,
	kEvent_Aud_PlayErr,
	kEvent_Aud_UsbFileUpdata,
}AudEvent_EN;


typedef struct {
	/* 是否已连接U盘 */
	bool usbConnect;
	/* 音频播放或录音状态 */
	AudStaType_EN state;
	/* 播放计时 */
	uint32_t runTime;
	/* 播放索引 */
	uint8_t playIndex;
	/* 录音文件名 */
	uint8_t recFile[25];
}AudInfo_S;

/* 录音/播放状态监听函数 */
typedef void(*Audio_StaListener)(AudEvent_EN event,AudInfo_S *info);


/* API */
typedef struct
{
	AppLauncher_S *launcher;
	
	void (*playPause)(void);
	void (*next)(void);
	void (*previous)(void);
	void (*record)(const char *fileName);
	void (*stopRec)(void);
	void (*stopPlay)(void);
	void (*setListener)(Audio_StaListener listener);
} Audio_S;
/*******************************************************************************
 * API
 ******************************************************************************/
extern Audio_S Audio;

#endif

