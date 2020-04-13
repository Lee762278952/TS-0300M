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
  * @Description   1.���й�����س�ʼ����������������ʹ�á�AppLauncher_S���ṹ��װ��
                   ���뵽��AppLauncher��������г�ʼ������������;
				   2.��ʼ����init�����е��ã���������configTask�����е��ã�������
				   ��(funcTask)���е���;
				   3.App_LauncherTask����������г�ʼ����init����Ȼ���������������
				   ��configTask����Ȼ����������������(funcTask);
				   
    @AppLauncher_S�ṹ˵�� 
					
				   .init ��ʼ������ָ�룬���������ȼ���ߵĳ�ʼ����������ӿڳ�ʼ��
				         �ȡ���App_LauncherTask����������ȵ������С�AppLauncher���ġ�init��
						 ������˳�����У�
	
				   .configTask �������ýṹָ�룬Ӧָ��һ��"AppTask_S"�ṹ��,
							   ��App_LauncherTask��������������г�ʼ�������������
							   ��AppLauncher���еġ�configTask��(configTask�е�������ӦΪ
							   һ�����������������Ӧ���á�vTaskDelete(null)��)��Ȼ��
							   ���������Щ�����Ƿ��Ѿ����(eCurrentState == eDeleted);
							   
				   .funcNum ��Ҫ�����Ĺ�����������;
				   
				   .funcTask ��������ṹָ������,Ӧָ��һ��"AppTask_S"�ṹ��ָ������,
				             ����Ԫ������Ӧ���롰funcNum����Ӧ������App_LauncherTask�����
							 ��ʼ�������ò�������Щ����
							 
	@ʵ����ʾ 
	
					 *��������* 
					 static Demo_init(void);
					 static Demo_ConfigTask(void *pvParameters);
					 static Demo_Func1Task(void *pvParameters);
					 static Demo_Func2Task(void *pvParameters);
					 
					 *����*
					 static uint8_t arg;
					 static TaskHandle_t handle
					 
					 *��������*
					 static AppTask_S ConfigTask = {
						.task = Demo_ConfigTask,	//����ָ��
						.name = "Demo.Config",		//��������
						.stack = 1024,       		//�����ջ
						.para = &arg,				//���񴫲�(����Ϊ��)
						.prio = 10,					//�������ȼ�
						.handle = &handle,			//������ƾ��(����Ϊ��)
					};

					*��������1*
					static AppTask_S Func1 = {
						.task = Demo_Func1Task,
						.name = "Demo.Func1",	
						.stack = 1024,
						.para = null,
						.prio = 11,
						.handle = null
					};
					*��������2*
					static AppTask_S Func2 = {
						.task = Demo_Func1Task,
						.name = "Demo.Func2",	
						.stack = 1024,
						.para = null,
						.prio = 12,
						.handle = null
					};

					*��������ָ������*
					static AppTask_S *FuncTask[] = {&Func1,&Func2};

					*���������ṹ*
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

    /* ��ӡ����豸��Ϣ */
    APP_PRINT_DEV_MSG();

    /* ���������߳� */
    if (xTaskCreate(App_LauncherTask, "Launcher", LAUNCHER_STACK_SIZE, null, LAUNCHER_PRIORITY, NULL) != pdPASS) {
        printf("create Launcher task error\r\n");
    }

    vTaskStartScheduler();
    while(1);
}


/**
* @Name  		App_Init
* @Author  		KT
* @Description  ��������AppLauncher�е�init����
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
* @Description  ��������AppLauncher�е�configTask���������񣬲�������������Ƿ����
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


