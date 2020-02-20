/**
 *
 *	@name				dsp.c
 *	@author 			KT
 *	@date 				2019/12/16
 *	@brief				
 *  @include			dsp.h
 *
 *
 *
 **/
/*******************************************************************************
 * includes
 ******************************************************************************/
 /* OS */
#include "FreeRTOS.h"
#include "task.h"

/* HAL */
#include "hal.h"

/* API */
#include "dsp.h"

/* APP */
#include "database.h"
#include "ram.h"

#include "log.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void Dsp_Init(void);
//static void Dsp_SetMode(DspSysMode_E mod);

/*******************************************************************************
 * Variables
 ******************************************************************************/
Dsp_S Dsp = {
	.init = Dsp_Init,
	.setMode = ADAU1466_SetMode,
	
	.inputSrc = ADAU1466_InputSource,
	.inputSrcRst = ADAU1466_InputSourceReset,
	.norOutput = ADAU1466_NormalOutputHandle,
	.norOutputRst = ADAU1466_NormalOutputReset,
	.chOutput = ADAU1466_ChannelOutputHandle,
	.chOutputRst = ADAU1466_ChannelOutputReset,
	.chInputSrc = ADAU1466_ChannelInputSource,
	.chInputSrcRst = ADAU1466_ChannelInputSourceReset,
	.unitCtrl = ADAU1466_UnitCtrl,
};

/*******************************************************************************
 * Code
 ******************************************************************************/

static void Dsp_Init(void){
	uint8_t dspOut,eq,dspIn;
	SysCfg_S *sysCfg;

	Log.d("Dsp init start!\r\n");

	I2C_ADAU1466_Init();

	sysCfg = (SysCfg_S *)Database.getInstance(kType_Database_SysCfg);

	ADAU1466_SetMode((DspSysMode_E)sysCfg->dspMode);

	/* ����LineOut1    ~ LineOut16 */
	for(dspOut = 0; dspOut < 16;dspOut++){
		/* �������� */
		Dsp.chOutput((DspOutput_E)dspOut,DSP_OUTPUT_VOLUME,(DspEqPart_E)null,(uint8_t)(31 - sysCfg->dsp[dspOut].vol));

		/* ������ʱ */
		Dsp.chOutput((DspOutput_E)dspOut,DSP_OUTPUT_DELAY,(DspEqPart_E)null,sysCfg->dsp[dspOut].dly);

		/* ����10��EQ���� */
		for(eq = 0;eq < 10;eq++)
			Dsp.chOutput((DspOutput_E)dspOut,DSP_OUTPUT_EQ,(DspEqPart_E)eq,(DspEqValue_E)sysCfg->dsp[dspOut].eqVol[eq]);

		/* ���ö�Ӧ����ͨ������ */
		for(dspIn = 0;dspIn < 7;dspIn++)
			Dsp.chInputSrc((DspOutput_E)dspOut,(DspInputSrc_E)dspIn,(DspVolume_E)(31 - sysCfg->dsp[dspOut].inputVol[dspIn]));
	}

	/* ������ͨ��� */
	for(dspOut = 16; dspOut < 22;dspOut++){
		/* �������� */
		Dsp.norOutput((DspOutput_E)dspOut,DSP_OUTPUT_VOLUME,(DspEqPart_E)null,(uint8_t)(31 - sysCfg->dsp[dspOut].vol));

		/* ����10��EQ���� */
		for(eq = 0;eq < 10;eq++)
			Dsp.norOutput((DspOutput_E)dspOut,DSP_OUTPUT_EQ,(DspEqPart_E)eq,(DspEqValue_E)sysCfg->dsp[dspOut].eqVol[eq]);

		/* ���ö�Ӧ����ͨ������ */
		for(dspIn = 0;dspIn < 7;dspIn++)
			Dsp.chInputSrc((DspOutput_E)dspOut,(DspInputSrc_E)dspIn,(DspVolume_E)(31 - sysCfg->dsp[dspOut].inputVol[dspIn]));
	}

}

//static void Dsp_SetMode(DspSysMode_E mod){
//	uint8_t dspOut,eq,dspIn;
//	SysCfg_S *sysCfg;
//
//	sysCfg = (SysCfg_S *)Database.getInstance(kType_Database_SysCfg);
//
//	ADAU1466_SetMode(mod);
//	
//
//}


