#ifndef __HAL_SAI_H_
#define __HAL_SAI_H_

#include "stdint.h"
#include "fsl_sai.h"
#include "hal.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#if SAI1_ENABLE || SAI2_ENABLE || SAI3_ENABLE


/* ��ʹ��SAI�ӿ����� */
#define SAI_ENABLE_NUM											(SAI1_ENABLE + SAI2_ENABLE + SAI3_ENABLE)

/* DMA�Ƿ�ʹ�þ�̬���� */
#ifndef SAI_DMA_STATIC_MEM
#define SAI_DMA_STATIC_MEM										(1)
#endif


/* SAI �ӿ�����ѡ�� */
typedef enum{

#if SAI1_ENABLE
	tSai1,
#endif

#if SAI2_ENABLE
	tSai2,
#endif

#if SAI3_ENABLE
	tSai3,
#endif

}HAL_SaiType_S;

/* SAI������ƾ�� */
typedef void * HAL_SaiHandle_S;

/* ��Ƶ�շ��жϻص����� */
typedef void(*HAL_SaiCallback)(HAL_SaiHandle_S * saiHandler,uint8_t *data,uint32_t dataLen,void *param);


/* ��Ƶ���ò������ݽṹ */
typedef struct {
	/* ��Ƶͨ�� */
	HAL_SaiType_S type;
	/* ������ */
	sai_sample_rate_t sampleRate;
	/* λ�� */
	sai_word_width_t bitWidth;
	/* ������/˫���� */
	sai_mono_stereo_t monoStereo;
	/* I2S���� */
	sai_master_slave_t masterSlave;
	/* I2SЭ�� */
	sai_protocol_t protocol;
	/* �Զ������ָ�� */
	void *param;
} HAL_SaiPara_S; 

/*******************************************************************************
 * API
 ******************************************************************************/
	HAL_SaiHandle_S *HAL_SaiInit(HAL_SaiPara_S *para);
	void HAL_SaiDisconfig(HAL_SaiHandle_S handler);
#if defined(SAI_DMA_STATIC_MEM) && SAI_DMA_STATIC_MEM
	void HAL_SaiStartReceive(HAL_SaiHandle_S *handler,HAL_SaiCallback callback);
	void HAL_SaiStartSend(HAL_SaiHandle_S *handler,HAL_SaiCallback callback);
#else
	void HAL_SaiStartReceive(HAL_SaiHandle_S *handler,uint8_t *data,uint32_t bufSize,HAL_SaiCallback callback);
	void HAL_SaiStartSend(HAL_SaiHandle_S *handler,uint8_t *data,uint32_t bufSize,HAL_SaiCallback callback);
#endif
	void HAL_SaiStopReceive(HAL_SaiHandle_S *handler);
	void HAL_SaiStopSend(HAL_SaiHandle_S *handler);
#endif

	
#endif

