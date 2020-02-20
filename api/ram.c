/**
 *
 *	@name					ramManage.c
 *	@author 			KT
 *	@date 				2019/03/27
 *	@brief				初始化及配置OCRAM、DTIM、ITIM
 *  @include			ram.h
 *
 *  @API					.init()    初始化
 *
 *  @description   配合功能修改了FreeRTOSConfig.h（增加heap_4.c中使用的宏定义）修改heap_4.c（修改ucHeap为指针并指向FreeRTOSConfig.h中FLEXRAM定义的地址）
 *
 **/
/*******************************************************************************
 * includes
 ******************************************************************************/
#include "log.h"
#include "board.h"
#include "fsl_flexram.h"
#include "ram.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define APP_FLEXRAM FLEXRAM
#define APP_FLEXRAM_IRQ FLEXRAM_IRQn
#define APP_FLEXRAM_IRQ_HANDLER FLEXRAM_IRQHandler

#define APP_DSB() __DSB()
#define APP_ISB() __ISB()
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static status_t OCRAM_Reallocate(void);

static void Ram_Init(void);
static void *RAM_ALLOC(size_t size,uint32_t waitTime,const char *funcName);
static void RAM_FREE(void **ram);
/*******************************************************************************
 * Variables
 ******************************************************************************/
Ram_S Ram = {
    .init = Ram_Init,
		.malloc = RAM_ALLOC,
		.free = RAM_FREE,
};

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_FLEXRAM_IRQ_HANDLER(void)
{
    uint32_t status = 0;

    status = FLEXRAM_GetInterruptStatus(APP_FLEXRAM);
    Log.d("APP_FLEXRAM_IRQ_HANDLER: status = %d\r\n",status);

}

static void Ram_Init(void) {
    taskENTER_CRITICAL();

#if defined(__DCACHE_PRESENT) && __DCACHE_PRESENT
    SCB_DisableDCache();
#endif
    /* enable IRQ */
    EnableIRQ(APP_FLEXRAM_IRQ);
    /* reallocate ram */
    OCRAM_Reallocate();
    /* init flexram */
    FLEXRAM_Init(APP_FLEXRAM);

    taskEXIT_CRITICAL();
}


static status_t OCRAM_Reallocate(void)
{
    flexram_allocate_ram_t ramAllocate = {
        .ocramBankNum = configOCRAM_ALLOCATE_BANK_NUM,
        .dtcmBankNum = configDTCM_ALLOCATE_BANK_NUM,
        .itcmBankNum = configITCM_ALLOCATE_BANK_NUM,
    };

    if (FLEXRAM_AllocateRam(&ramAllocate) != kStatus_Success)
    {
        Log.e("Allocate on-chip ram fail!\r\n");
        return kStatus_Fail;
    }
    else
    {
        Log.d("Allocate on-chip ram success:ocram=%dKB dtcm=%dKB itcm=%dKB\r\n",
              configOCRAM_ALLOCATE_BANK_NUM*32,configDTCM_ALLOCATE_BANK_NUM*32,configITCM_ALLOCATE_BANK_NUM*32);
    }

    return kStatus_Success;
}

static void *RAM_ALLOC(size_t size,uint32_t waitTime,const char *funcName) {
    void *ram = NULL;
    uint32_t t = 0;

	if(!size) return null;

	do {
//		vTaskDelay(t&1);
		if(t)	DELAY(5);
		ram = pvPortMalloc(size);
		if(t>=1) Log.e("wait ram alloc\r\n");
		//Log.d("func : %s ram alloc size = %d,ram addr = 0x%x\r\n",funcName,size,ram);
	} while((++t >= waitTime) || !ram );
	memset(ram,0,size);
    return ram;
}

static void RAM_FREE(void **ram) {
    if(*ram == NULL)
        return;
    vPortFree(*ram);
    *ram = NULL;
}
