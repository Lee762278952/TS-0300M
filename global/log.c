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
/* CLIB */
#include "stdarg.h"
#include "stdio.h"

/* OS */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* GLOBAL */
#include "global_config.h"
#include "log.h"

/* SDK */
#include "fsl_lpuart.h"

/* APP */
#include "ram.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Shell任务堆栈大小及优先级 */
#define SHELL_TASK_PRIORITY 				(3)
#define SHELL_TASK_STACK_SIZE  				(256)

/* Shell指令最大长度 */
#define SHELL_CMD_MAX_LEN					(16)

/* Shell自动补全功能(tap) */
#define SHELL_AUTO_COMPLETE					(false)

/* Shell特殊字符 */
#define KEY_ESC 							(0x1BU)
#define KET_DEL 							(0x7FU)

/* Shell标题 */
#define COMPLETE_TOPIC(_dev,_topic)			_dev # _topic
#define SHELL_TOPIC							COMPLETE_TOPIC(DEVICE_MODEL,:/#)
#define TOPIC_LEN							(strlen(SHELL_TOPIC))

#define PRINTF_TOPIC()						printf("\r\n%s ",ShTopic);

/* 打印系统运行时间 */
#define LOG_TICK_COUNT						(xTaskGetTickCount()/1000)

/* 允许输入的字符 */
#define IS_PERMIT_CHAR(_ch)					((_ch >= 'A' && _ch <= 'Z') || (_ch >= 'a' && _ch <= 'z') || (_ch >= '0' && _ch <= '9')   \
	                                          || (_ch == '-') || (_ch == '"') || (_ch == ' '))


/* Shell命令定义 */
#define SHELL_CMD(_cmd) 					Sh_##_cmd

#define SHELL_CMD_DEFINE(_cmd, _descrip, _operate) \
static ShellCmd_S SHELL_CMD(_cmd) = {                             \
	(#_cmd), (_descrip), (_operate),         \
}



/* Shell命令结构 */
typedef struct {
	char *cmd;
	char *description;
	uint32_t (*operate)(const char *para);
}ShellCmd_S;


/* Shell输入状态 */
typedef enum {
    kStatus_Shell_Normal 	= 0U,   /*!< Normal key */
    kStatus_Shell_Special 	= 1U,  /*!< Special key */
    kStatus_Shell_Func 		= 2U, /*!< Function key */
} ShellStatus_EN;
	

/* Log打印类型 */
typedef enum {
    kType_Log_Debug,
    kType_Log_Info,
    kType_Log_Error,
} LogType_EN;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* LAUNCHER */
static void Log_ShellTask(void *pvParameters);

/* INTERNAL */
static void Log_Printf(LogType_EN type,const char *str);

/* SHELL */
static uint32_t Shell_Exit(const char *para);
static uint32_t Shell_Help(const char *para);
static uint32_t Shell_Task(const char *para);
static uint32_t Shell_Mem(const char *para);



/* API */
static void Log_Debug(const char *fmt_s, ...);
static void Log_Information(const char *fmt_s, ...);
static void Log_Error(const char *fmt_s, ...);
/*******************************************************************************
 * Variables
 ******************************************************************************/
static const char TypeTag[] = {'D','I','E'};
static const char ShTopic[] = SHELL_TOPIC;

static char ShContent[SHELL_CMD_MAX_LEN + 1] = {0};
static uint8_t ShLen = 0;

static bool isShelling = false;

static char LogTemp[1024] = {0};


/* Shell命令定义 */
SHELL_CMD_DEFINE(exit,"Exit shell mode.",Shell_Exit);
SHELL_CMD_DEFINE(help,"Describe all shell instructions.",Shell_Help);
SHELL_CMD_DEFINE(task,"Printf all tasks.",Shell_Task);
SHELL_CMD_DEFINE(mem,"Query memory remaining space.",Shell_Mem);

/* Shell命令集 */
ShellCmd_S *ShellCmds[] = {
	&SHELL_CMD(exit),
	&SHELL_CMD(help),
	&SHELL_CMD(task),
	&SHELL_CMD(mem),
	null,
};


/*******************************************************************************
 * Task & API
 ******************************************************************************/
static AppTask_S ShellTask = {
    .task = Log_ShellTask,
    .name = "Log.Shell",
    .stack = SHELL_TASK_STACK_SIZE,
    .para = null,
    .prio = SHELL_TASK_PRIORITY,
    .handle = null
};


static AppTask_S *FuncTask[] = {&ShellTask};

static AppLauncher_S Launcher = {
    .init = null,
    .configTask = null,
    .funcNum  = 1,
    .funcTask = FuncTask,
};


Log_S Log = {
    .launcher = &Launcher,

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
int fputc(int c,FILE *stream)
{
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
int getc(FILE *stream)
{
    uint8_t data;
    LPUART_ReadBlocking(LOG_SERIAL_PORT,&data,1);
    return data;
}

/**
* @Name  		Log_Printf
* @Author  		KT
* @Description	
*
*/
static void Log_Printf(LogType_EN type,const char *str)
{
	if(isShelling){
		printf("\r%s[%d] %c/ %s          %s",type == kType_Log_Error ? "\r\n" : "",LOG_TICK_COUNT,TypeTag[type],str,type == kType_Log_Error ? "\r\n" : "");
		printf("\r\n%s %s",ShTopic,ShContent);
	}
	else{
		printf("%s[%d] %c/ %s%s",type == kType_Log_Error ? "\r\n" : "",LOG_TICK_COUNT,TypeTag[type],str,type == kType_Log_Error ? "\r\n" : "");
	}
}


static void Log_Debug(const char *fmt_s, ...)
{
    va_list args;
    va_start(args,fmt_s);
    vsprintf(LogTemp,fmt_s,args);
    va_end(args);

    Log_Printf(kType_Log_Debug, LogTemp);
}

static void Log_Information(const char *fmt_s, ...)
{
    va_list args;
    va_start(args,fmt_s);
    vsprintf(LogTemp,fmt_s,args);
    va_end(args);

    taskENTER_CRITICAL();
    Log_Printf(kType_Log_Info, LogTemp);
    taskEXIT_CRITICAL();
}

static void Log_Error(const char *fmt_s, ...)
{
    va_list args;
    va_start(args,fmt_s);
    vsprintf(LogTemp,fmt_s,args);
    va_end(args);

    Log_Printf(kType_Log_Error, LogTemp);
}

static void Log_ShellInput(uint8_t ch){
	if(ShLen < SHELL_CMD_MAX_LEN  && IS_PERMIT_CHAR(ch)){
		ShContent[ShLen++] = ch;
		printf("%c",ch);
	}
}


static void Log_ShellDel(void){
	if(ShLen > 0){
			ShContent[ShLen--] = 0;
			printf("\b \b");
	}
}

static void Log_ShellEnter(void){
	uint8_t i,cmdLen;
	char *para;
	ShellCmd_S *sh;
	
	printf("\r\n");

	if(ShLen == 0){
		PRINTF_TOPIC();
		return;
	}
	
	for(i = 0; ;i++){
		if(ShellCmds[i] != null){
			sh = ShellCmds[i];
			
			if(strstr(ShContent,sh->cmd) == ShContent){
				cmdLen = strlen(sh->cmd);

				if(ShLen == cmdLen){
					sh->operate(null);
				}
				else if(ShLen > cmdLen && ShContent[cmdLen] == ' '){
					para = strchr(ShContent + cmdLen,'-');
					if(para != null)
						sh->operate(para);
					else
						continue;
				}
				else
					continue;

				if(isShelling)
						PRINTF_TOPIC();

				break;
			}
		}
		else{
			printf("Unknow shell command .. \r\n");
			PRINTF_TOPIC();
			break;
		}
	}
	
	memset(ShContent,0,TOPIC_LEN);
	ShLen = 0;
}



static void Log_ShellTask(void *pvParameters)
{
    int ch;
	static ShellStatus_EN shStatus = kStatus_Shell_Normal;

    Log.i("Log shell task start...\r\n");

    while(1) {
        ch = getchar();
		
        /* 回车进入Shell模式 */
        if(!isShelling) {
            if(ch == '\r' || ch == '\n'){
                isShelling = true;
				PRINTF_TOPIC();
			}
            continue;
        }

		switch(shStatus){
			case kStatus_Shell_Normal:{
				/* Special key */
		        if (ch == KEY_ESC) {
		            shStatus = kStatus_Shell_Special;
		        }
				/* Handle tab key */
		        else if (ch == '\t') {
#if SHELL_AUTO_COMPLETE
		            printf("Tap\r\n");
#endif
		        }
		        /* Handle backspace key */
		        else if ((ch == KET_DEL) || (ch == '\b')) {
		            Log_ShellDel();
		        } 
				/* Enter */
				else if ((ch == '\r') || (ch == '\n')){
					Log_ShellEnter();
				}
				/* Ctrl + C */
				else if(ch == 0x03){
					Shell_Exit(null);
				}
				/* Normal key */
				else{
					Log_ShellInput(ch);
		        }
			}break;

			case kStatus_Shell_Special:{
				/* Function key */
	            if (ch == '[') {
	                shStatus = kStatus_Shell_Func;
	                break;
	            }
				
	            shStatus = kStatus_Shell_Normal;
			}break;

			case kStatus_Shell_Func:{
				shStatus = kStatus_Shell_Normal;

				switch ((uint8_t)ch) {
	            /* History operation here */
	            case 'A': /* Up key */
	                break;
	            case 'B': /* Down key */
	                break;
	            case 'D': /* Left key */
	                break;
	            case 'C': /* Right key */
	                break;
	            default:
	                break;
	            }
			}break;
			
			default:
				break;
		}
       
	}
}


/*********** shell命令相关执行函数 *************/
static uint32_t Shell_Help(const char *para){
	uint8_t i;
	ShellCmd_S *sh;
	
	printf("\r\nShell command description\r\n\r\n");
	for(i = 0; ;i++){
		if(ShellCmds[i] != null){
			sh = ShellCmds[i];
			printf(" %s :	%s\r\n",sh->cmd,sh->description);
		}
		else
			break;
	}
	printf("\r\n");
	return 0;
}


static uint32_t Shell_Exit(const char *para){
	printf("Shell mode exit\r\n\r\n");
	memset(ShContent,0,TOPIC_LEN);
	ShLen = 0;

	isShelling = false;
	return 0;
}


static uint32_t Shell_Task(const char *para){
	uint8_t *buf;
	
	buf = MALLOC(1024);
	vTaskList((char *)buf);
	printf("TaskList : \r\nNAME                            STA     PRIO    STACK   ID\r\n");
	printf("%s\r\n",buf);
	
	FREE(buf);
	return 0;
}

static uint32_t Shell_Mem(const char *para){
	size_t size;

	size = xPortGetFreeHeapSize();
	printf("Memery heap free size = %d\r\n",size);
	size = xPortGetMinimumEverFreeHeapSize();
	printf("Memery heap minimum free size = %d\r\n",size);
	
	return 0;
} 
