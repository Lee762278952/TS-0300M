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
/* DSP配置线程堆栈大小、优先级 */
#define DSP_CONFIG_TASK_STACK_SIZE						(1024)
#define DSP_CONFIG_TASK_PRIORITY						(12)

/* DSP监控线程堆栈大小、优先级 */
#define DSP_MONITOR_TASK_STACK_SIZE						(256)
#define DSP_MONITOR_TASK_PRIORITY						(3)


/* DSP */
#define DSP_1											(0x01)
#define DSP_2											(0x02)

/* 配置延时(ms) */
#define DSP_CFG_DELAY									(200)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
 /* LAUNCHER */
 static void Dsp_MonitorTask(void *pvParameters);
static void Dsp_ConfigTask(void *pvParameters);

/* INTERNAL */
static void Dsp_Config(uint8_t dsp);
static void Dsp_SetPara(void);


/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Task & API
 ******************************************************************************/

static AppTask_S ConfigTask = {
	.task = Dsp_ConfigTask,
	.name = "Dsp.Config",	
	.stack = DSP_CONFIG_TASK_STACK_SIZE,
	.para = null,
	.prio = DSP_CONFIG_TASK_PRIORITY,
	.handle = null
};

static AppTask_S Monitor = {
	.task = Dsp_MonitorTask,
	.name = "Dsp.Monitor",	
	.stack = DSP_MONITOR_TASK_STACK_SIZE,
	.para = null,
	.prio = DSP_MONITOR_TASK_PRIORITY,
	.handle = null
};


static AppTask_S *FuncTask[] = {&Monitor};


static AppLauncher_S Launcher = {
	.init = null,
	.configTask = &ConfigTask,
	.funcNum  = 1,
	.funcTask = FuncTask,
};


Dsp_S Dsp = {
	.launcher = &Launcher,
		
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

/**
* @Name  		Dsp_MonitorTask
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static void Dsp_MonitorTask(void *pvParameters){
	bool reset = false;

	Log.i("Dsp monitor task start!!\r\n");

	while(1){
#if DSP_FUNCTION_ENABLE
		DELAY(1000);
		if(ADAU1466_1_AliveDetect()){
			Log.e("Dsp_1 has been reset \r\n");
			Dsp_Config(DSP_1);
			reset = true;
		}

		if(ADAU1466_2_AliveDetect()){
			Log.e("Dsp_2 has been reset \r\n");
			Dsp_Config(DSP_2);
			reset = true;
		}

		if(reset){
			reset = false;
			Dsp_SetPara();
		}
#else
		vTaskSuspend(null);
		DELAY(10000);
#endif
		
		
	}
}


/**
* @Name  		Dsp_SetPara
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static void Dsp_SetPara(void){
	uint8_t dspOut,eq,dspIn;
	SysCfg_S *sysCfg;

	
	sysCfg = (SysCfg_S *)Database.getInstance(kType_Database_SysCfg);

	ADAU1466_SetMode((DspSysMode_E)sysCfg->dspMode);

	/* 配置LineOut1    ~ LineOut16 */
	for(dspOut = 0; dspOut < 16;dspOut++){
		/* 配置音量 */
		Dsp.chOutput((DspOutput_E)dspOut,DSP_OUTPUT_VOLUME,(DspEqPart_E)null,(uint8_t)(31 - sysCfg->dsp[dspOut].vol));

		/* 配置延时 */
		Dsp.chOutput((DspOutput_E)dspOut,DSP_OUTPUT_DELAY,(DspEqPart_E)null,sysCfg->dsp[dspOut].dly);

		/* 配置10段EQ音量 */
		for(eq = 0;eq < 10;eq++){
			Dsp.chOutput((DspOutput_E)dspOut,DSP_OUTPUT_EQ,(DspEqPart_E)eq,(DspEqValue_E)sysCfg->dsp[dspOut].eqVol[eq]);
		}

		/* 配置对应输入通道音量 */
		for(dspIn = 0;dspIn < 7;dspIn++){
			Dsp.chInputSrc((DspOutput_E)dspOut,(DspInputSrc_E)dspIn,(DspVolume_E)(31 - sysCfg->dsp[dspOut].inputVol[dspIn]));
		}
	}

	/* 配置普通输出 */
	for(dspOut = 16; dspOut < 22;dspOut++){
		/* 配置音量 */
		Dsp.norOutput((DspOutput_E)dspOut,DSP_OUTPUT_VOLUME,(DspEqPart_E)null,(uint8_t)(31 - sysCfg->dsp[dspOut].vol));

		/* 配置10段EQ音量 */
		for(eq = 0;eq < 10;eq++){
			Dsp.norOutput((DspOutput_E)dspOut,DSP_OUTPUT_EQ,(DspEqPart_E)eq,(DspEqValue_E)sysCfg->dsp[dspOut].eqVol[eq]);
		}

		/* 配置对应输入通道音量 */
		for(dspIn = 0;dspIn < 7;dspIn++){
			Dsp.chInputSrc((DspOutput_E)dspOut,(DspInputSrc_E)dspIn,(DspVolume_E)(31 - sysCfg->dsp[dspOut].inputVol[dspIn]));
		}
	}
}


/**
* @Name  		Dsp_ConfigTask
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static void Dsp_ConfigTask(void *pvParameters){

#if DSP_FUNCTION_ENABLE

	Dsp_Config(DSP_1 | DSP_2);

	Dsp_SetPara();
	
//	DELAY(3000);

	Log.i("DSP configuration finish ... \r\n");

#else
	Log.e("DSP function disable ...");
#endif


	vTaskDelete(null);
}

/**
* @Name  		Dsp_Config
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static void Dsp_Config(uint8_t dsp){
	status_t sta;
	uint8_t section;

	if(dsp & DSP_1){
		for(section = 0;section < 5;section ++){
			sta = ADAU1466_1_Init(section);
			
			if(sta != kStatus_Success)
				Log.e("DSP_1 init err (section = %d) , sta = %d ... \r\n",section,sta);
			
			DELAY(DSP_CFG_DELAY);
		}
	}

	if(dsp & DSP_2){
		for(section = 0;section < 5;section ++){
			sta = ADAU1466_2_Init(section);
			
			if(sta != kStatus_Success)
				Log.e("DSP_2 init err (section = %d) , sta = %d ... \r\n",section,sta);
			
			DELAY(DSP_CFG_DELAY);
		}
	}
	
	
}




