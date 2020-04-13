#ifndef __HAL_AUD_CODEC_H_
#define __HAL_AUD_CODEC_H_

#include "fsl_common.h"
#include "drivers_common.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
#ifndef AUD_CODEC_NAU88C22_W_ENABLE
#define AUD_CODEC_NAU88C22_W_ENABLE							ENABLE
#endif

#ifndef AUD_CODEC_NAU88C22_D_ENABLE
#define AUD_CODEC_NAU88C22_D_ENABLE							ENABLE
#endif

	
#define AUD_CODEC_DRIVERS_TOTAL_NUM							(AUD_CODEC_NAU88C22_W_ENABLE + AUD_CODEC_NAU88C22_D_ENABLE)
	
	
typedef enum {
#if	AUD_CODEC_NAU88C22_W_ENABLE
		tNau88c22_W,
#endif

#if	AUD_CODEC_NAU88C22_D_ENABLE
		tNau88c22_D,
#endif

} HAL_AudCodecType_EN;


/*******************************************************************************
 * API
 ******************************************************************************/
status_t HAL_AudCodecInit(HAL_AudCodecType_EN type);
status_t HAL_AudCodeWriteReg(HAL_AudCodecType_EN type,uint8_t reg,uint16_t para);



#endif

