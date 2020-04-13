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

/* ������������ṹ */
typedef struct{
	TaskFunction_t task;
	const char * name;		
	uint16_t stack;
	void **para;
	UBaseType_t prio;
	TaskHandle_t *handle;
}AppTask_S;




typedef struct {
	/* Ӧ�ó�ʼ������ */
	void (*init)(void);
	
	/* Ӧ���������� */
	AppTask_S *configTask;
	
	/* ������������ */
	uint8_t funcNum;
	
	/* Ӧ�ù������� */
	AppTask_S **funcTask;
}AppLauncher_S;





#endif
