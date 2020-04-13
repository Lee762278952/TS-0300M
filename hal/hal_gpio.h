#ifndef __HAL_GPIO_H_
#define __HAL_GPIO_H_

#include "fsl_common.h"
#include "fsl_gpio.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef uint8_t HAL_GpioHandler;

/* IO中断回调函数 */
typedef void(*GpioIrqCallback)(void *param);

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/
HAL_GpioHandler HAL_GpioInit(GPIO_Type *base, uint8_t pin, gpio_pin_direction_t dirc, uint8_t logic, gpio_interrupt_mode_t mode);
status_t HAL_SetGpioLevel(HAL_GpioHandler index,uint8_t level);
uint8_t HAL_GetGpioLevel(HAL_GpioHandler index);
status_t HAL_SetIrqCallback(HAL_GpioHandler index,GpioIrqCallback callback,void *param);


#endif
