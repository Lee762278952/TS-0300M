#ifndef _APP_COMMON_H_
#define _APP_COMMON_H_

/*******************************************************************************
 * Device model & Application version
 ******************************************************************************/
#include "model_version.h"
/**************************** �����ͺ� *******************************/
#define DEVICE_MODEL									TS_0300M
/*********************************************************************/

/**************************** ����汾 *******************************/
#define APP_VERSION										"V1.0"
/*********************************************************************/

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"
#include "debug.h"
#include "stdbool.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* �����ֵ */
#define MAX_NUM  										(0xffffffffUL)	

/* ΢����ϵͳTickת�� */
#define MsToTick(ms) 									((1000L + ((uint32_t)configTICK_RATE_HZ * (uint32_t)(ms - 1U))) / 1000L)
#define TickToMs(tick) 									((tick)*1000uL / (uint32_t)configTICK_RATE_HZ)

/* OS��ʱ���� */
#define DELAY(ms)										vTaskDelay(MsToTick(ms))

/* nullֵ */
#define null 											0

/* ���������ж� */
#define ERR_CHECK(condition, implement) 				do { if (!(condition)) {implement;}} while(0)
/* ��debug���������ж� */
#define ERR_CHECK_DBG(condition, dbg, implement) 		do { if (!(condition)) {debug(dbg); implement;}} while(0)

#define APP_BUILD_TIME									APP_BuildTime()



#endif
