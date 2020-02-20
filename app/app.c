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

/* GLOBAL */
#include "global_config.h"

/* API */
#include "network.h"
#include "usb_disk.h"
#include "audio.h"
#include "time.h"
#include "database.h"
#include "dsp.h"

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
	taskENTER_CRITICAL();	
	
	Log.init();

	Database.init();

	Dsp.init();
	
	Network.init();
	
    UsbDisk.init();

	Screen.init();
	
    Camera.init();

	Time.init();
	
	taskEXIT_CRITICAL();	

    Log.d("Tasks is being launch.....\r\n");
	
	Conference.launch();
	
  	ExternalCtrl.launch();

	WiredUnit.launch();

	WifiUnit.launch();

	UsbDisk.launch();

	Audio.launch();

	Screen.launch();

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

void UsageFault_Handler(void){
	Log.e("UsageFault Handler!!!\r\n\r\n");
    while(1);
}


void BusFault_Handler(void){
	Log.e("BusFault HandlerHandler!!!\r\n\r\n");
    while(1);
}


