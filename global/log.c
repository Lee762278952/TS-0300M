/**
 *
 *	@name				debug.c
 *	@author 			KT
 *	@date 				2019/08/19
 *	@brief				
 *
 *  @API					
 *
 *  @description   
 *
 **/
/*******************************************************************************
 * includes
 ******************************************************************************/
 /* OS */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
 
/* GLOBAL */
#include "global_config.h"
#include "log.h"

#include "stdarg.h"

/* SDK */
//#include "fsl_debug_console.h"
//#include "fsl_debug_console_conf.h"
//#include "fsl_str.h"
#include "fsl_lpuart.h"

#include <stdio.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEBUG_TASK_PRIORITY 				(3)
#define DEBUG_TASK_STACK_SIZE  				(1024)

#define DEBUG_CMD_MAX_LEN					16

#define CMD_TOTAL_NUM						5

#define CMD_DEBUG_MODE						"debug"
#define CMD_MEM								"mem"

#define DEBUG_TICK_COUNT					(xTaskGetTickCount()/1000)

typedef enum {
	tDebug = 0,
	tMem,
}CommandType_EN;

typedef enum {
	dbgMode , cmdMode
}DebugMode_EN;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void Log_ShellTask(void *pvParameters);

static void Log_Debug(const char *fmt_s, ...);
static void Log_Information(const char *fmt_s, ...);
static void Log_Error(const char *fmt_s, ...);

static void Log_Init(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
//static SemaphoreHandle_t logLock;
//static portBASE_TYPE TaskWoken = pdFALSE;

static DebugMode_EN mode = dbgMode;

static const char topic[] = "TS-0300M:/ # ";
static char cmd[DEBUG_CMD_MAX_LEN + 1] = {0};
static char back[DEBUG_CMD_MAX_LEN+sizeof(topic)];
static uint8_t index = 0;

static const char command[CMD_TOTAL_NUM][10] = 
{
	CMD_DEBUG_MODE,CMD_MEM

};


Log_S Log = {
	.init = Log_Init,
	.d = Log_Debug,
	.i = Log_Information,
	.e = Log_Error,
};

/*******************************************************************************
 * Code
 ******************************************************************************/
/**
* @Name  		fputc
* @Author  		KT
* @Description	重定向printf函数
*
*/
int fputc(int c,FILE *stream){
	uint8_t data = c;
	LPUART_WriteBlocking(LOG_SERIAL_PORT,&data,1);
	return c;
}

/**
* @Name  		fgetc
* @Author  		KT
* @Description	重定向getchar函数
*
*/
int getc(FILE *stream){
	uint8_t data;
	LPUART_ReadBlocking(LOG_SERIAL_PORT,&data,1);
	return data;
}


//void debug(const char *fmt_s, ...)  
//{  
//	va_list args;       
//	va_start(args,fmt_s); 
//	vprintf(fmt_s,args); 
//	va_end(args);      
//}


static void Log_Init(void){
//	if (xTaskCreate(Log_ShellTask, "shell", DEBUG_TASK_STACK_SIZE, null, DEBUG_TASK_PRIORITY, NULL) != pdPASS)
//    {
//        Log_Error("shell launch error\r\n");
//    }
}

static void Log_Debug(const char *fmt_s, ...){
	printf("[%d] D/ ",DEBUG_TICK_COUNT);
	va_list args;       
	va_start(args,fmt_s); 
	vprintf(fmt_s,args); 
	va_end(args);
}

static void Log_Information(const char *fmt_s, ...){
	taskENTER_CRITICAL();
	printf("[%d] I/ ",DEBUG_TICK_COUNT);
	va_list args;       
	va_start(args,fmt_s); 
	vprintf(fmt_s,args); 
	va_end(args);
	taskEXIT_CRITICAL();
}

static void Log_Error(const char *fmt_s, ...){
	printf("\r\n[%d] E/ ",DEBUG_TICK_COUNT);
	va_list args;       
	va_start(args,fmt_s); 
	vprintf(fmt_s,args); 
	va_end(args);
	printf("\r\n");
}


static char getCharToCmd(char c){
	
	if(c == '\b' && index >= 0){
		index = (index == 0) ? 0 : index - 1;
		cmd[index] = 0;
	}
	else if(c >= 0x20 && c <= 0x7E && index < DEBUG_CMD_MAX_LEN){
		cmd[index] = c;
		index++;
	}
	
	return c;
}

static void analysis_cmd(char *cmd){
	uint8_t i;
	CommandType_EN type;
	
	for(i = 0;i<CMD_TOTAL_NUM;i++){
		if(strstr(cmd,command[i]) != null){
			type = (CommandType_EN)i;
			break;
		}
		
		if(i == CMD_TOTAL_NUM -1){
			PRINTF("command invalid!!\r\n");
			return;
		}
	}
	
//	PRINTF("cmd :%s\r\n",command[type]);
	switch(type){
		case tDebug:{
			mode = dbgMode;
		}break;
		case tMem:break;
	}
}

static void Log_ShellTask(void *pvParameters)
{
	char c;
	
	memset(back,'\b',DEBUG_CMD_MAX_LEN+sizeof(topic));
	
	Log_Debug("Log shell start...\r\n");
	
	while(1){
//		c = getchar();
//		printf("%c",c);
		switch(mode){
			case dbgMode:{
				if(c == '\r' || c == '\n'){
					c = getchar();
					if(c == '\r' || c == '\n'){
						mode = cmdMode;
						printf("\r\n%s",topic);
					}
				}
			}break;
			case cmdMode:{
				if(c == '\r' || c == '\n'){
					printf("\r\n");
					analysis_cmd(cmd);
					index = 0;
					memset(cmd,0,DEBUG_CMD_MAX_LEN + 1);
				}
				else{
					getCharToCmd(c);
					if(c == '\b') printf("\b \b");
				}
				
				if(mode == cmdMode)
					printf("%s%s%s",back,topic,cmd);
			}break;
			default:break;
		}
	}
}

