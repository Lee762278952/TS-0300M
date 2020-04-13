#ifndef __DSP_H_
#define __DSP_H_

#include "i2c_adau1466.h"

#include "app.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

//typedef DspSysMode_E DspMode_EN;
//
//typedef DspOutput_E DspOutput_EN;

 typedef struct {
// 	void (*launch)(void);
	AppLauncher_S *launcher;
	
	status_t (*setMode)(DspSysMode_E mod);
	status_t (*inputSrc)(DspOutput_E out,DspInputSrc_E src,DspVolume_E vol);
	status_t (*inputSrcRst)(void);
	status_t (*norOutput)(DspOutput_E out,DspOutputType_E src,DspEqPart_E part,uint8_t uchVol);
	status_t (*norOutputRst)(void);
	status_t (*chOutput)(DspOutput_E out,DspOutputType_E src,DspEqPart_E part,uint8_t uchVol);
	status_t (*chOutputRst)(void);
	status_t (*chInputSrc)(DspOutput_E out,DspInputSrc_E src,DspVolume_E vol);
	status_t (*chInputSrcRst)(void);
	status_t (*unitCtrl)(DspUintSrc_E src,uint8_t *out,uint8_t *ovol);
 }Dsp_S;

/*******************************************************************************
 * API
 ******************************************************************************/
extern Dsp_S Dsp;
#endif

