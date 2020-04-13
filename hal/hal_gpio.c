/*
 *	 @FileName					 hal_gpio.c
 *	 @Author					 KT
 *	 @CreationTime				 2019/10/17
 *	 @Description
 *
 *	 @Include					 hal_gpio.h
 *
 *
 **/

#include "hal_gpio.h"
#include "fsl_iomuxc.h"
#include "hal.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifndef GPIO_HAL_INSTANCE_NUM
#define GPIO_HAL_INSTANCE_NUM							(16)
#endif

#define GPIO_INTERRUPT_PRIORITY							(4)


typedef struct {
	GPIO_Type *base;
	uint32_t pin;
	GpioIrqCallback callback;
	void *param;
	gpio_pin_config_t config;
}HAL_Gpio_S;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void HAL_GpioIrqEnable(HAL_Gpio_S *gpio);
/*******************************************************************************
 * Variables
 ******************************************************************************/
 /* GPIO硬件抽象层特例 特例共 GPIO_HAL_INSTANCE_NUM 个 */
static HAL_Gpio_S Gpio_Instance[GPIO_HAL_INSTANCE_NUM] = {null};

/*******************************************************************************
 * Code
 ******************************************************************************/
HAL_GpioHandler HAL_GpioInit(GPIO_Type *base, uint8_t pin, gpio_pin_direction_t dirc, uint8_t logic, gpio_interrupt_mode_t mode){
	uint8_t index;
	HAL_Gpio_S *gpio = null;

	for(index = 0;index < GPIO_HAL_INSTANCE_NUM;index ++){
		if(Gpio_Instance[index].base != null)
			continue;

		gpio = &Gpio_Instance[index];

		gpio->base = base;
		gpio->pin = pin;
		gpio->config.direction = dirc;
		gpio->config.outputLogic = logic;
		gpio->config.interruptMode = mode;

		GPIO_PinInit(gpio->base, gpio->pin, &gpio->config);

		if(dirc == kGPIO_DigitalInput && mode != kGPIO_NoIntmode){
			HAL_GpioIrqEnable(gpio);
		}
		return index;
	}

	return 0xFF;
 }


status_t HAL_SetGpioLevel(HAL_GpioHandler index,uint8_t level){
	HAL_Gpio_S *gpio;
	ERR_CHECK(index >=0 && index < GPIO_HAL_INSTANCE_NUM, return kStatus_Fail);
	
	gpio = &Gpio_Instance[index];
	ERR_CHECK(gpio->base != null, return kStatus_Fail);
	GPIO_WritePinOutput(gpio->base, gpio->pin, level);	
	return kStatus_Success;
}

uint8_t HAL_GetGpioLevel(HAL_GpioHandler index){
	HAL_Gpio_S *gpio;
	ERR_CHECK(index >=0 && index < GPIO_HAL_INSTANCE_NUM, return kStatus_Fail);
	
	gpio = &Gpio_Instance[index];
	ERR_CHECK(gpio->base != null, return kStatus_Fail);

	return (uint8_t)GPIO_ReadPinInput(gpio->base, gpio->pin);

}

status_t HAL_SetIrqCallback(HAL_GpioHandler index,GpioIrqCallback callback,void *param){
	HAL_Gpio_S *gpio;

	ERR_CHECK(index >=0 && index < GPIO_HAL_INSTANCE_NUM, return kStatus_Fail);
	ERR_CHECK(callback != null, return kStatus_Fail);
	
	gpio = &Gpio_Instance[index];
	gpio->callback = callback;
	gpio->param = param;
	return kStatus_Success;
}

static void HAL_GpioIrqEnable(HAL_Gpio_S *gpio){
	IRQn_Type type;

	if(gpio->base == GPIO1){
		type = (gpio->pin > 15) ? GPIO1_Combined_16_31_IRQn : GPIO1_Combined_0_15_IRQn;
	}
	
	else if(gpio->base == GPIO2){
		type = (gpio->pin > 15) ? GPIO2_Combined_16_31_IRQn : GPIO2_Combined_0_15_IRQn;
	}
	
	else if(gpio->base == GPIO3){
		type = (gpio->pin > 15) ? GPIO3_Combined_16_31_IRQn : GPIO3_Combined_0_15_IRQn;
	}
	
	else if(gpio->base == GPIO4){
		type = (gpio->pin > 15) ? GPIO4_Combined_16_31_IRQn : GPIO4_Combined_0_15_IRQn;
	}
	
	else if(gpio->base == GPIO5){
		type = (gpio->pin > 15) ? GPIO5_Combined_16_31_IRQn : GPIO5_Combined_0_15_IRQn;
	}
	
	else if(gpio->base == GPIO6 || gpio->base == GPIO7 || gpio->base == GPIO8 || gpio->base == GPIO9){
		type = (gpio->pin > 15) ? GPIO5_Combined_16_31_IRQn : GPIO5_Combined_0_15_IRQn;
	}

	EnableIRQ(type);
	NVIC_SetPriority(type, GPIO_INTERRUPT_PRIORITY);
	GPIO_PortEnableInterrupts(gpio->base,1 << gpio->pin);
}

/*******  IRQ  ********/
static inline uint8_t HAL_GpioGetpinNum(uint32_t pinFlag){
	for(int i=0;i < 32;i++){
		if((pinFlag & (1 << i)) != 0)
			return i;
	}
	return 0xFF;
}

static void HAL_GpioIrqCommon(GPIO_Type *base, uint32_t pinFlag){
	uint8_t index,pin;

	GPIO_PortClearInterruptFlags(base, pinFlag);

	pin = HAL_GpioGetpinNum(pinFlag);

	for(index = 0;index < GPIO_HAL_INSTANCE_NUM;index++){
		if(Gpio_Instance[index].base == base && Gpio_Instance[index].pin == pin && Gpio_Instance[index].callback != null){
			Gpio_Instance[index].callback(Gpio_Instance[index].param);
			return;
		}
	}
}

void GPIO1_Combined_0_15_IRQHandler(void){
	HAL_GpioIrqCommon(GPIO1,GPIO_GetPinsInterruptFlags(GPIO1));
}

void GPIO1_Combined_16_31_IRQHandler(void){
	HAL_GpioIrqCommon(GPIO1,GPIO_GetPinsInterruptFlags(GPIO1));
}
void GPIO2_Combined_0_15_IRQHandler(void){
	HAL_GpioIrqCommon(GPIO2,GPIO_GetPinsInterruptFlags(GPIO2));
}
void GPIO2_Combined_16_31_IRQHandler(void){
	HAL_GpioIrqCommon(GPIO2,GPIO_GetPinsInterruptFlags(GPIO2));
}
void GPIO3_Combined_0_15_IRQHandler(void){
	HAL_GpioIrqCommon(GPIO3,GPIO_GetPinsInterruptFlags(GPIO3));
}
void GPIO3_Combined_16_31_IRQHandler(void){
	HAL_GpioIrqCommon(GPIO3,GPIO_GetPinsInterruptFlags(GPIO3));
}
void GPIO4_Combined_0_15_IRQHandler(void){
	HAL_GpioIrqCommon(GPIO4,GPIO_GetPinsInterruptFlags(GPIO4));
}
void GPIO4_Combined_16_31_IRQHandler(void){
	HAL_GpioIrqCommon(GPIO4,GPIO_GetPinsInterruptFlags(GPIO4));
}
void GPIO5_Combined_0_15_IRQHandler(void){
	HAL_GpioIrqCommon(GPIO5,GPIO_GetPinsInterruptFlags(GPIO5));
}
void GPIO5_Combined_16_31_IRQHandler(void){
	HAL_GpioIrqCommon(GPIO5,GPIO_GetPinsInterruptFlags(GPIO5));
}	

//void GPIO6_7_8_9_IRQHandler(void){
//	HAL_GpioIrqCommon(GPIO5,GPIO_GetPinsInterruptFlags(GPIO5));
//}

