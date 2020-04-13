#ifndef _AUDIO_H_
#define _AUDIO_H_

#include "stdint.h"
#include "app.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
	/* ¼��/�����ļ�ָ��Ŀ¼�ļ��� */
#define AUDIO_DEFAULT_DIR								"AUDIO"


/* ��Ƶ���Ż�¼��״̬ */
typedef enum {
	kStatus_Aud_Idle = 0,
	kStatus_Aud_Recording,
	kStatus_Aud_Playing,
}AudStaType_EN;

/* ��Ƶ���Ż�¼���¼� */
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
	/* �Ƿ�������U�� */
	bool usbConnect;
	/* ��Ƶ���Ż�¼��״̬ */
	AudStaType_EN state;
	/* ���ż�ʱ */
	uint32_t runTime;
	/* �������� */
	uint8_t playIndex;
	/* ¼���ļ��� */
	uint8_t recFile[25];
}AudInfo_S;

/* ¼��/����״̬�������� */
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

