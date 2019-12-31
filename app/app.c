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
#include "app.h"
#include "wired_unit.h"
#include "conference.h"
#include "external_ctrl.h"
#include "wifi_unit.h"
#include "screen.h"
#include "camera.h"

/* OS */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/* API */
#include "network.h"
#include "usb_disk.h"
#include "audio.h"
#include "time.h"
#include "database.h"
#include "dsp.h"

/* HAL */
#include "hal.h"
#include "hal_gpio.h"
#include "hal_rtc.h"
#include "hal_spi_flash.h"

/* LIB */
#include "stdio.h"

/* SDK */
#include "board.h"
#include "pin_mux.h"
#include "fsl_iomuxc.h"

/* DRIVER */
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

	/* ��ӡ����豸��Ϣ */
	APP_PRINT_DEV_MSG();

	/* ���������߳� */
    if (xTaskCreate(App_LauncherTask, "Launcher", LAUNCHER_STACK_SIZE, null, LAUNCHER_PRIORITY, NULL) != pdPASS) {
        debug("create host task error\r\n");
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
//	taskENTER_CRITICAL();	

	Database.init();

	Dsp.init();
	
	Network.init();
	
    UsbDisk.init();
	
	Time.init();

    Camera.init();
	
//	taskEXIT_CRITICAL();	

    debug("\r\nTasks is being launch.....\r\n");
	
	Conference.launch();
	
    ExternalCtrl.launch();

	WiredUnit.launch();

	WifiUnit.launch();

	UsbDisk.launch();

	Audio.launch();

	Screen.launch();

//	debug_init();
    vTaskDelete(null);
    while(1);
}



void HardFault_Handler(void)
{
    debug("HardFault Handler!!!\r\n\r\n");
    while(1);
}

void MemManage_Handler(void)
{
    debug("MemManage Handler!!!\r\n\r\n");
    while(1);
}

void UsageFault_Handler(void){
	debug("UsageFault Handler!!!\r\n\r\n");
    while(1);
}


void BusFault_Handler(void){
	debug("BusFault HandlerHandler!!!\r\n\r\n");
    while(1);
}


