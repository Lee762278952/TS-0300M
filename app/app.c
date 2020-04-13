/**
 *
 *	@name				app.c
 *	@author 			KT
 *	@date 				2019/07/26
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
/* APP */
#include "wired_unit.h"
#include "conference.h"
#include "external_ctrl.h"
#include "wifi_unit.h"
#include "screen.h"
#include "camera.h"
#include "slave_mcu.h"
#include "network.h"
#include "usb_disk.h"
#include "audio.h"
#include "time.h"
#include "database.h"
#include "dsp.h"
#include "ram.h"

/* GLOBAL */
#include "global_config.h"



/* OS */
#include "FreeRTOS.h"
#include "task.h"

/* HAL */
#include "hal.h"

/* SDK */
#include "board.h"
#include "pin_mux.h"
#include "fsl_iomuxc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Task priorities. */
#define LAUNCHER_PRIORITY 		(configMAX_PRIORITIES - 1)
#define LAUNCHER_STACK_SIZE  	(1024)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void App_LauncherTask(void *pvParameters);
/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Task
 ******************************************************************************/
/**
  * @Description   1.所有功能相关初始化及任务启动建议使用“AppLauncher_S”结构封装并
                   加入到“AppLauncher”数组进行初始化和任务启动;
				   2.初始化（init）串行调用，配置任务（configTask）并行调用，功能任
				   务(funcTask)并行调用;
				   3.App_LauncherTask会先完成所有初始化（init），然后完成所有配置任
				   务（configTask），然后再启动功能任务(funcTask);
				   
    @AppLauncher_S结构说明 
					
				   .init 初始化函数指针，可用于优先级最高的初始化任务，例如接口初始化
				         等。“App_LauncherTask”任务会首先调用所有“AppLauncher”的“init”
						 函数并顺序运行；
	
				   .configTask 任务配置结构指针，应指向一个"AppTask_S"结构体,
							   “App_LauncherTask”任务在完成所有初始化后会启动所有
							   “AppLauncher”中的“configTask”(configTask中的任务函数应为
							   一次性任务，任务函数最后应调用“vTaskDelete(null)”)，然后
							   持续检测这些任务是否已经完成(eCurrentState == eDeleted);
							   
				   .funcNum 需要启动的功能任务数量;
				   
				   .funcTask 功能任务结构指针数组,应指向一个"AppTask_S"结构体指针数组,
				             数组元素数量应该与“funcNum”对应，当“App_LauncherTask”完成
							 初始化后会调用并启动这些任务
							 
	@实例演示 
	
					 *函数声明* 
					 static Demo_init(void);
					 static Demo_ConfigTask(void *pvParameters);
					 static Demo_Func1Task(void *pvParameters);
					 static Demo_Func2Task(void *pvParameters);
					 
					 *变量*
					 static uint8_t arg;
					 static TaskHandle_t handle
					 
					 *配置任务*
					 static AppTask_S ConfigTask = {
						.task = Demo_ConfigTask,	//函数指针
						.name = "Demo.Config",		//任务名称
						.stack = 1024,       		//任务堆栈
						.para = &arg,				//任务传参(可以为空)
						.prio = 10,					//任务优先级
						.handle = &handle,			//任务控制句柄(可以为空)
					};

					*功能任务1*
					static AppTask_S Func1 = {
						.task = Demo_Func1Task,
						.name = "Demo.Func1",	
						.stack = 1024,
						.para = null,
						.prio = 11,
						.handle = null
					};
					*功能任务2*
					static AppTask_S Func2 = {
						.task = Demo_Func1Task,
						.name = "Demo.Func2",	
						.stack = 1024,
						.para = null,
						.prio = 12,
						.handle = null
					};

					*功能任务指针数组*
					static AppTask_S *FuncTask[] = {&Func1,&Func2};

					*任务启动结构*
					static AppLauncher_S Launcher = {
						.init = Demo_init,
						.configTask = &ConfigTask,
						.funcNum  = 2,
						.funcTask = FuncTask,
					};
				   
***/



static AppLauncher_S **AppLauncher[] = {
    &Database.launcher,
    &Network.launcher,
    &UsbDisk.launcher,
    &Screen.launcher,
    &Camera.launcher,
    &Time.launcher,
    &SlaveMcu.launcher,
    &Conference.launcher,
    &WiredUnit.launcher,
    &WifiUnit.launcher,
    &ExternalCtrl.launcher,
    &Dsp.launcher,
    &Audio.launcher,
};

const uint8_t NumOfApp = (sizeof(AppLauncher) / sizeof(void *));


/*******************************************************************************
 * Code
 ******************************************************************************/

/**
* @Name  		main
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
int main(void)
{
    HAL_Init();

    /* 打印软件设备信息 */
    APP_PRINT_DEV_MSG();

    /* 开启任务线程 */
    if (xTaskCreate(App_LauncherTask, "Launcher", LAUNCHER_STACK_SIZE, null, LAUNCHER_PRIORITY, NULL) != pdPASS) {
        printf("create Launcher task error\r\n");
    }

    vTaskStartScheduler();
    while(1);
}


/**
* @Name  		App_Init
* @Author  		KT
* @Description  调用所有AppLauncher中的init函数
* @para
*
*
* @return
*/
static void App_Init(void){
	AppLauncher_S *launcher;
	uint8_t i;

	for(i = 0; i < NumOfApp; i ++) {
        launcher = *AppLauncher[i];

        if(launcher->init != null){
            launcher->init();
        }
    }
}

/**
* @Name  		App_Config
* @Author  		KT
* @Description  启动所有AppLauncher中的configTask的配置任务，并持续检测任务是否结束
* @para
*
*
* @return
*/
static void App_Config(void){
	AppTask_S *configTask;
	TaskStatus_t taskStatus;
	TaskHandle_t taskHandle[NumOfApp];
	uint8_t i,cfgTaskCnt = 0,finishCnt = 0;;

	 for(i = 0; i < NumOfApp; i ++) {
        configTask = (*AppLauncher[i])->configTask;

        if(configTask != null) {
            if (xTaskCreate(configTask->task,
                            configTask->name,
                            configTask->stack,
                            (*configTask->para),
                            configTask->prio,
                            &taskHandle[cfgTaskCnt]) != pdPASS) 
            {
                Log.e("Config task \"%s\" creat error\r\n",configTask->name);
            }
//			else{
//				Log.d("Config task \"%s\" has been launched\r\n",configTask->name);
//			}

			cfgTaskCnt += 1;
        }
    }

	do{
		finishCnt = 0;
		for(i = 0;i < cfgTaskCnt;i++){
			vTaskGetInfo(taskHandle[i],&taskStatus,pdFALSE,eInvalid);
			if(taskStatus.eCurrentState == eDeleted)
				finishCnt += 1;
		}	
		DELAY(500);
	}while(finishCnt < cfgTaskCnt);
}


/**
* @Name  		App_LaunchFunc
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void App_LaunchFunc(void){
	uint8_t i,j,funcNum;
	AppTask_S **funcTask;

	for(i = 0; i < NumOfApp; i ++) {
		funcNum = (*AppLauncher[i])->funcNum;
        funcTask = (*AppLauncher[i])->funcTask;
	

        if(funcNum > 0 && funcTask != null) {
			for(j = 0;j < funcNum;j++){
				if (xTaskCreate(funcTask[j]->task,
                            funcTask[j]->name,
                            funcTask[j]->stack,
                            (*funcTask[j]->para),
                            funcTask[j]->prio,
                            funcTask[j]->handle) != pdPASS) 
	            {
	                Log.e("Function task \"%s\" creat error\r\n", funcTask[j]->name);
	            }
//				else{
//					Log.d("Function task \"%s\" has been launched \r\n",funcTask[j]->name);
//				}
			}
		}
	}
}


/**
* @Name  		App_LauncherTask
* @Author  		KT
* @Description
* @para
*
*
* @return
*/
static void App_LauncherTask(void *pvParameters)
{

//	taskENTER_CRITICAL();

    Log.init();
	Log.i("%s start to Initialization ... \r\n\r\n",DEVICE_MODEL);


    App_Init();
	Log.i("Initialization finish ... \r\n\r\n");


    App_Config();
	Log.i("Configuration finish ... \r\n\r\n");


	App_LaunchFunc();
	Log.i("Application launch finish ... \r\n\r\n");

    vTaskDelete(null);
    while(1);
}



void HardFault_Handler(void)
{
    Log.e("HardFault Handler!!!\r\n\r\n");
    while(1);
}

void MemManage_Handler(void)
{
    Log.e("MemManage Handler!!!\r\n\r\n");
    while(1);
}

void UsageFault_Handler(void)
{
    Log.e("UsageFault Handler!!!\r\n\r\n");
    while(1);
}


void BusFault_Handler(void)
{
    Log.e("BusFault HandlerHandler!!!\r\n\r\n");
    while(1);
}


