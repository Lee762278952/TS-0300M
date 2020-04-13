#ifndef _APP_H_
#define _APP_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
/* OS */
#include "FreeRTOS.h"
#include "task.h"

/* api */
//#include "log.h"
//#include "ram.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* 创建任务参数结构 */
typedef struct{
	TaskFunction_t task;
	const char * name;		
	uint16_t stack;
	void **para;
	UBaseType_t prio;
	TaskHandle_t *handle;
}AppTask_S;




typedef struct {
	/* 应用初始化函数 */
	void (*init)(void);
	
	/* 应用配置任务 */
	AppTask_S *configTask;
	
	/* 功能任务数量 */
	uint8_t funcNum;
	
	/* 应用功能任务 */
	AppTask_S **funcTask;
}AppLauncher_S;





#endif
