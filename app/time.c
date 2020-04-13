/**
 *
 *	@name				time.c
 *	@author 			KT
 *	@date 				2019/12/11
 *  @include			time.h
 *
 *	@description		用于获取设备系统时间，设备注册功能；
 *
 **/

#include "time.h"
/*******************************************************************************
 * includes
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* LAUNCHER */
static void Time_Init(void);

/* API */
static bool Time_GetRstFlag(void);
static void Time_GetNow(TimePara_S *para);
static void Time_PrintNow(char *str);
static void Time_SetNow(TimePara_S *para);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Task & API
 ******************************************************************************/

static AppLauncher_S Launcher = {
	.init = Time_Init,
	.configTask = null,
	.funcNum  = 0,
	.funcTask = null,
};


Time_S Time = {
//	.init = Time_Init,
	.launcher = &Launcher,

	.getRst = Time_GetRstFlag,
	.getNow = Time_GetNow,
	.pirnNow = Time_PrintNow,
	.setNow = Time_SetNow,
};


/*******************************************************************************
 * Code
 ******************************************************************************/
static void Time_Init(void){
	TimePara_S para;
	char timePrin[25];

	if(Time_GetRstFlag()){
		HAL_RtcGetDateTime(tPcf8563, (HAL_RtcPara_S *)&para);
		Time_PrintNow(timePrin);
		Log.d("Device time now is %s\r\n", timePrin);
	}
	else{
		Log.e("RTC has been reset,please check the battery!!\r\n");
	}
}

static void Time_GetNow(TimePara_S *para){
	if(para == null)
		return;

	HAL_RtcGetDateTime(tPcf8563, para);
}

static void Time_PrintNow(char *str){
	TimePara_S para;

	if(str == null)
		return;

	HAL_RtcGetDateTime(tPcf8563, (HAL_RtcPara_S *)&para);
	sprintf(str,"20%02d-%02d-%02d %02d:%02d:%02d",  \
		para.year,para.month,para.day,para.hour,para.min,para.sec);
}

static bool Time_GetRstFlag(void){
	return !(HAL_RtcGetRstFlag(tPcf8563));
}

static void Time_SetNow(TimePara_S *para){
	if(para == null)
		return;

	/* 设置时间 */
	HAL_RtcSetDateTime(tPcf8563, para);
	/* 复位时间重置标志位 */
	HAL_RtcSetRstFlag(tPcf8563);
}


