/**
 *
 *	@FileName								audioRecord.c
 *	@Author									KT
 *	@CreationTime						2019/05/15
 *	@Description						����¼�����ܣ���ѡ��
 *
 * 	@Include								audio_record.h	
 *
 *	@API										 
 *
 **/
/*******************************************************************************
 * includes
 ******************************************************************************/
#include "string.h"
#include "stdio.h"
#include "debug.h"
#include "ram.h"
#include "audio_sai.h"
#include "audio_record.h"
#include "semphr.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define AUDIO_RECORD_DATA_STREAM_SIZE			(32U*1024U)
#define NUM_SAI_RECORD_CHANNEL_MAX				(1)
#define NUM_NET_RECORD_CHANNEL_MAX				(3)

#define NUM_TOTAL_RECORD_CHANNEL_MAX			(NUM_SAI_RECORD_CHANNEL_MAX + NUM_NET_RECORD_CHANNEL_MAX)

#define AUDIO_FILE_SUFFIX						".wav"
#define AUDIO_TEMP_FILE_SUFFIX					".wavtmp"
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void audioRecord_TimerCallback(TimerHandle_t xTimer);
static void audioRecord_WavHeader(uint8_t *header, uint32_t sampleRate, uint32_t bitsPerFrame, uint32_t fileSize);

static AudioRecordHandler_S *audioRecordCreate(const char *path,AudioType_EN type);
static void audioRecordSaiStart(AudioRecordHandler_S *audioRecordHandler);
static void audioRecordSaiStop(AudioRecordHandler_S *audioRecordHandler);
static void audioRecordDismiss(AudioRecordHandler_S *audioRecordHandler);
static void audioRecordNetData(AudioRecordHandler_S *handler, uint8_t *data,uint32_t size);
/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint8_t numSaiRecordChannel = 0;
static uint8_t numNetRecordChannel = 0;

static bool audioSaiInitialized = false;

/* �߳�ͬ�������ź��� */
static SemaphoreHandle_t taskSynchronization = NULL;

static AudioRecordHandler_S *audioRecordHandler[NUM_TOTAL_RECORD_CHANNEL_MAX] = {NULL};


AudioRecord_S AudioRecord = {
	.creat = audioRecordCreate,
	.saiStart = audioRecordSaiStart,
	.saiStop = audioRecordSaiStop,
	.netData = audioRecordNetData,
	.dismiss = audioRecordDismiss,
};
/*******************************************************************************
 * Code
 ******************************************************************************/
/**
* @Name  		audioRecordCreate
* @Author  		KT
* @Description 	����¼��
* @para    		path	: 	¼���ļ������޺�׺��
*				type	:	¼�����ͣ�sai ͨ��sai�ӿ�����¼��  net ¼�������ɴ��봫�루������Ƶ����
*
* @return		AudioRecordHandler_S * : ¼�����ܿ��ƾ��
*/
static AudioRecordHandler_S *audioRecordCreate(const char *path,AudioType_EN type){
	uint8_t i,ch = 0,len, *wavHeader;
	FRESULT fResult;
	char *wavPath, *tmpPath;

	if(	!UsbDisk.devConnected || 
			(type == saiAudio && numSaiRecordChannel >= NUM_SAI_RECORD_CHANNEL_MAX)  ||
			(type == netAudio && numNetRecordChannel >= NUM_NET_RECORD_CHANNEL_MAX) 	)
			return NULL;
			
			if(taskSynchronization == NULL)
					taskSynchronization = xSemaphoreCreateMutex();
					
		/* Ѱ��ָ�������еĿ���λ�� */
		for(ch = 0;ch < NUM_TOTAL_RECORD_CHANNEL_MAX;ch++){
			/* �ҵ�δʹ�õĿ��ƾ������ʼ�����ڴ漰��ֵ */
			if(audioRecordHandler[ch] == NULL){
			
				xSemaphoreTake(taskSynchronization, MAX_NUM);
				
				/* �����ڴ浽���ƾ�� */
				audioRecordHandler[ch] = MALLOC(sizeof(AudioRecordHandler_S));
				if(audioRecordHandler[ch] == NULL){
					xSemaphoreGive(taskSynchronization);
					return NULL;
				}
				
				/* ���������� */
				audioRecordHandler[ch]->index = ch;
				/* �������� */
				audioRecordHandler[ch]->type = type;
				
				/* ���������������ʧ�ܣ��ͷ�֮ǰ������ڴ棬��������NULL */
				audioRecordHandler[ch]->dataStreamHandler = DataStream.creat(AUDIO_RECORD_DATA_STREAM_SIZE);
				if(audioRecordHandler[ch]->dataStreamHandler == NULL){
					FREE(audioRecordHandler[ch]);
					xSemaphoreGive(taskSynchronization);
					return NULL;
				}
				
				/* ������ʱ��������¼����ʱ */
				audioRecordHandler[ch]->timeCount = 0;
				audioRecordHandler[ch]->timer = xTimerCreate("Audio Record Timer",MsToTick(1000),true,audioRecordHandler[ch],audioRecord_TimerCallback);
				
				
				/* ����·�����Ƽ��Ϻ�׺����׺Ϊ.wav��.wavtmp�� */
				len = strlen(path);
				wavPath = MALLOC(len+15);
				tmpPath = MALLOC(len+15);
				
				memset(wavPath,0,len+15);
				memset(tmpPath,0,len+15);
				
				sprintf(wavPath,"%s%s",path,AUDIO_FILE_SUFFIX);
				sprintf(tmpPath,"%s%s",path,AUDIO_TEMP_FILE_SUFFIX);
				
				/* ��鴫���ļ�·�����ƣ���׺Ϊ.wav��.wavtmp���Ƿ��Ѿ�����,������޸�·������,��Ӻ��(1~9) */
				if(UsbDisk.isExist(wavPath) || UsbDisk.isExist(tmpPath)){
				for(i = 0;i < 10;i++){
						sprintf(wavPath,"%s(%d)%s",path,i,AUDIO_FILE_SUFFIX);
						sprintf(tmpPath,"%s(%d)%s",path,i,AUDIO_TEMP_FILE_SUFFIX);
						if(!UsbDisk.isExist(wavPath) && !UsbDisk.isExist(tmpPath)){
							/* ��Ӻ������ļ���������ڣ����ֺ�׺�¶�û����ͬ�ļ���������·���� */
							audioRecordHandler[ch]->path = MALLOC(len+5);
							memset(audioRecordHandler[ch]->path,'\0',len+5);
							sprintf(audioRecordHandler[ch]->path,"%s(%d)",path,i);
							break;
						}
							
						/* �����Ӻ�궼�޷����㣬����NULL */	
						if(i >= 9) {
							FREE(wavPath);
							FREE(tmpPath);
							xSemaphoreGive(taskSynchronization);
							return NULL;
						}
					}
				}
				
				/* �����ڣ�ֱ�ӱ��洫���ļ�·������ */
				else{
						audioRecordHandler[ch]->path = MALLOC(len+1);
						memset(audioRecordHandler[ch]->path,'\0',len+1);
						strcpy(audioRecordHandler[ch]->path,path);
				}
				FREE(wavPath);
				
				debug("audioRecord file path = %s\r\n",audioRecordHandler[ch]->path);
				
				/* �½��ļ� */
				audioRecordHandler[ch]->fp = MALLOC(sizeof(FIL));
				sprintf(tmpPath,"%s%s",audioRecordHandler[ch]->path,AUDIO_TEMP_FILE_SUFFIX);
				fResult = UsbDisk.open(audioRecordHandler[ch]->fp,tmpPath,FA_READ|FA_WRITE|FA_CREATE_NEW,true);
				
				if(fResult != FR_OK){
					debug("creat new file %s fail!! res = %d\r\n",tmpPath,fResult);
					FREE(tmpPath);
					xSemaphoreGive(taskSynchronization);
					return NULL;
				}
				/* ��д��wav����ͷ,Ȼ��ر��ļ�������������������д�� */
				wavHeader = MALLOC(44);
				audioRecord_WavHeader(wavHeader,48000U,16U,0);
				UsbDisk.write(audioRecordHandler[ch]->fp,wavHeader,44,true);
				UsbDisk.close(audioRecordHandler[ch]->fp,true);
				FREE(wavHeader);
				
				/* ����usb����� */
				audioRecordHandler[ch]->dataStreamIndex = UsbDisk.setOutputStream(tmpPath,audioRecordHandler[ch]->dataStreamHandler);
				if(audioRecordHandler[ch]->dataStreamIndex >= USB_MAX_DATA_STREAM_NUM){
					debug("set usb output stream fail! index = %d\r\n",audioRecordHandler[ch]->dataStreamIndex);
					FREE(tmpPath);
					xSemaphoreGive(taskSynchronization);
					return NULL;
				}
				
				FREE(tmpPath);
				
				if(audioRecordHandler[ch]->type == saiAudio){
					/* ��ʼ����Ƶģ��,���������� */
					if(!audioSaiInitialized){
							AudioSai.init();
							audioSaiInitialized = true;
					}
//					AudioSai.setInputStream(audioRecordHandler[ch]->dataStreamHandler);
				}
				
				/* �������ɹ�����¼��ǰ�ѷ����¼��Ƶ������ */
				if(type == saiAudio)
					numSaiRecordChannel++;
				else if(type == netAudio)
					numNetRecordChannel++;
				
				xSemaphoreGive(taskSynchronization);
				return audioRecordHandler[ch];
			}
		}
		xSemaphoreGive(taskSynchronization);
		return NULL;
}


/**
*	@Name  				audioRecord_SaiRecordStart
*	@Author  			KT
* @Description 	��ʼSAI����¼��
*	@para    			AudioRecordHandler_S * : ¼�����ܿ��ƾ��
*
*/
static void audioRecordSaiStart(AudioRecordHandler_S *handler){
		if(handler == NULL || handler->type != saiAudio)
			return;
			
			AudioSai.setInputStream(handler->dataStreamHandler);
			AudioSai.recStart(stream);
			xTimerStart(handler->timer,0);
			
			DELAY(250);
}

/**
*	@Name  				audioRecordSaiStop
*	@Author  			KT
* @Description 	��ͣSAI����¼�����ɻָ���
*	@para    			AudioRecordHandler_S * : ¼�����ܿ��ƾ��
*
*/
static void audioRecordSaiStop(AudioRecordHandler_S *handler){
			if(handler == NULL || handler->type != saiAudio)
				return;

			AudioSai.recStop();
			xTimerStop(handler->timer,0);
			
			DELAY(250);
}


/**
*	@Name  				audioRecordDismiss
*	@Author  			KT
* @Description 	ֹͣ���ر�¼��
*	@para    			AudioRecordHandler_S * : ¼�����ܿ��ƾ��
*
*/
static void audioRecordDismiss(AudioRecordHandler_S *handler){
			uint8_t *wavHeader;
			uint32_t fSize;
			char *tmpPath ,*wavPath;

			if(handler == NULL)
				return;
			
			/* �ر�SAI */
//			if(handler->type == saiAudio)
//				AudioSai.deinit();
			
			/* �ر�USB������ */
			UsbDisk.closeDataStream(handler->dataStreamIndex);
			
			/* ɾ�������� */
			DataStream.dismiss(handler->dataStreamHandler);
			
			/* ����wav����ͷ */
			tmpPath = MALLOC(strlen(handler->path) + 10);
			sprintf(tmpPath,"%s%s",handler->path,AUDIO_TEMP_FILE_SUFFIX);
			fSize = UsbDisk.fileSize(tmpPath);
			
			wavHeader = MALLOC(44);
			audioRecord_WavHeader(wavHeader,48000U,16U,fSize);
			
			UsbDisk.open(handler->fp,tmpPath,FA_WRITE|FA_OPEN_EXISTING,true);
			UsbDisk.write(handler->fp,wavHeader,44,true);
			UsbDisk.close(handler->fp,true);
			FREE(wavHeader);
			
			/* �޸��ļ���������ʱ�ļ�(.wavtmp)��Ϊ(.wav),���¼�� */
			wavPath = MALLOC(strlen(handler->path) + 10);
			sprintf(wavPath,"%s%s",handler->path,AUDIO_FILE_SUFFIX);
			UsbDisk.rename(tmpPath,wavPath);
			
			debug("\"%s\" audio record completed!!\r\n",wavPath);
			
			/* �ͷ��ڴ� */
			FREE(tmpPath);
			FREE(wavPath);
			FREE(handler->path);
			FREE(handler->fp);
			xTimerDelete(handler->timer,0);
			FREE(audioRecordHandler[handler->index]);
			
			/* ��ƵƵ������ */
				if(handler->type == saiAudio)
					numSaiRecordChannel--;
				else if(handler->type == netAudio)
					numNetRecordChannel--;
			
}

/**
*	@Name  				audioRecordNetData
*	@Author  			KT
* @Description 	������Ƶ����ӿ�
*	@para    			handler : ¼�����ܿ��ƾ��
*								data		:	����ָ��
*								size		�����ݴ�С
*
*/
static void audioRecordNetData(AudioRecordHandler_S *handler, uint8_t *data,uint32_t size){
//			uint8_t *wavHeader;
//			uint32_t fSize;
//			char *tmpPath ,*wavPath;

			if(handler == NULL || handler->type == saiAudio || data == NULL)
				return;
			
			DataStream.write(handler->dataStreamHandler,data,size);
}


/* ��ʱ���ص�������1S�ص�һ�Σ�ʱ������� +1 */
static void audioRecord_TimerCallback(TimerHandle_t xTimer) {
		AudioRecordHandler_S *audioRecordHandler;
		
		audioRecordHandler = (AudioRecordHandler_S *)pvTimerGetTimerID(xTimer);
		
		audioRecordHandler->timeCount++;
		
		debug("audio file \'%s\' was record by %d second\r\n",audioRecordHandler->path,audioRecordHandler->timeCount);
}

static void audioRecord_WavHeader(uint8_t *header, uint32_t sampleRate, uint32_t bitsPerFrame, uint32_t fileSize)
{
    uint32_t totalDataLen = fileSize - 8U;
    uint32_t audioDataLen = fileSize - 44U;
    uint32_t byteRate = sampleRate * (bitsPerFrame / 8U);
    header[0] = 'R';
    header[1] = 'I';
    header[2] = 'F';
    header[3] = 'F';
    header[4] = (totalDataLen & 0xff); /* file-size (equals file-size - 8) */
    header[5] = ((totalDataLen >> 8U) & 0xff);
    header[6] = ((totalDataLen >> 16U) & 0xff);
    header[7] = ((totalDataLen >> 24U) & 0xff);
    header[8] = 'W'; /* Mark it as type "WAVE" */
    header[9] = 'A';
    header[10] = 'V';
    header[11] = 'E';
    header[12] = 'f'; /* Mark the format section 'fmt ' chunk */
    header[13] = 'm';
    header[14] = 't';
    header[15] = ' ';
    header[16] = 16; /* 4 bytes: size of 'fmt ' chunk, Length of format data.  Always 16 */
    header[17] = 0;
    header[18] = 0;
    header[19] = 0;
    header[20] = 1; /* format = 1 ,Wave type PCM */
    header[21] = 0;
    header[22] = 1; /* channels */
    header[23] = 0;
    header[24] = (sampleRate & 0xff);
    header[25] = ((sampleRate >> 8U) & 0xff);
    header[26] = ((sampleRate >> 16U) & 0xff);
    header[27] = ((sampleRate >> 24U) & 0xff);
    header[28] = (byteRate & 0xff);
    header[29] = ((byteRate >> 8U) & 0xff);
    header[30] = ((byteRate >> 16U) & 0xff);
    header[31] = ((byteRate >> 24U) & 0xff);
    header[32] = (bitsPerFrame / 8); /* block align */
    header[33] = 0;
    header[34] = bitsPerFrame; /* bits per sample */
    header[35] = 0;
    header[36] = 'd'; /*"data" marker */
    header[37] = 'a';
    header[38] = 't';
    header[39] = 'a';
    header[40] = (audioDataLen & 0xff); /* data-size (equals file-size - 44).*/
    header[41] = ((audioDataLen >> 8) & 0xff);
    header[42] = ((audioDataLen >> 16) & 0xff);
    header[43] = ((audioDataLen >> 24) & 0xff);
}


