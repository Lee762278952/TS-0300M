/**
 *
 *	@name				hal_aud_codec.c
 *	@author 			KT
 *	@date 				2019/11/30
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
#include "hal_aud_codec.h"
#include "aud-codec/nau88c22.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#if	AUD_CODEC_NAU88C22_W_ENABLE

#ifndef AUD_CODEC_NAU88C22_W_I2C_TYPE
#define AUD_CODEC_NAU88C22_W_I2C_TYPE					LPI2C2
#endif

#ifndef AUD_CODEC_NAU88C22_W_ADDR
#define AUD_CODEC_NAU88C22_W_ADDR						(0x34)
#endif

#endif

#if	AUD_CODEC_NAU88C22_D_ENABLE

#ifndef AUD_CODEC_NAU88C22_D_I2C_TYPE
#define AUD_CODEC_NAU88C22_D_I2C_TYPE					LPI2C1
#endif

#ifndef AUD_CODEC_NAU88C22_D_ADDR
#define AUD_CODEC_NAU88C22_D_ADDR						(0x34)
#endif

#endif



typedef struct {

    /* @brief DSP control & config LPI2C_Type */
    LPI2C_Type *base;

    /* @brief I2C address */
    uint8_t addr;

    status_t (*write)(LPI2C_Type *base,uint16_t slaveAddr,uint32_t addr,uint8_t *data,uint16_t len);
    status_t (*read)(LPI2C_Type *base,uint16_t slaveAddr,uint32_t addr,uint8_t *data,uint16_t len);

} HAL_AudCodec_S;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*******************************************************************************
 * Variables
 ******************************************************************************/
#if	AUD_CODEC_NAU88C22_W_ENABLE
HAL_AudCodec_S Nau88c22_W_driver = {
    .base = AUD_CODEC_NAU88C22_W_I2C_TYPE,
    .addr = AUD_CODEC_NAU88C22_W_ADDR,
    .write = Nau88c22_Write,
    .read = Nau88c22_Read,
};
#endif

#if	AUD_CODEC_NAU88C22_D_ENABLE
HAL_AudCodec_S Nau88c22_D_driver = {
    .base = AUD_CODEC_NAU88C22_D_I2C_TYPE,
    .addr = AUD_CODEC_NAU88C22_D_ADDR,
    .write = Nau88c22_Write,
    .read = Nau88c22_Read,
};
#endif




static HAL_AudCodec_S *AudCodec_drivers[AUD_CODEC_DRIVERS_TOTAL_NUM] = {
#if	AUD_CODEC_NAU88C22_W_ENABLE
    &Nau88c22_W_driver,
#endif

#if	AUD_CODEC_NAU88C22_D_ENABLE
    &Nau88c22_D_driver,
#endif

};

/*******************************************************************************
 * Code
 ******************************************************************************/
static void HAL_AudCodecWrite(HAL_AudCodec_S *audCodec,uint8_t reg,uint16_t para)
{
    uint8_t data[2];
    data[0] = (uint8_t)(para >> 8);
    data[1] = (uint8_t)(para & 0xFF);
    audCodec->write(audCodec->base,audCodec->addr,reg,data,2);
}

#if 0
static void HAL_AudCodecRead(HAL_AudCodec_S *audCodec,uint8_t reg,uint16_t* para)
{
    uint8_t data[2];
    audCodec->read(audCodec->base,audCodec->addr,reg,data,2);
    *para = (data[0] << 8) | data[1];
}
#endif

status_t HAL_AudCodeWriteReg(HAL_AudCodecType_EN type,uint8_t reg,uint16_t para){
	HAL_AudCodec_S *audCodec;

    ERR_CHECK(((uint8_t)type >=0 && (uint8_t)type < AUD_CODEC_DRIVERS_TOTAL_NUM), return kStatus_Fail);
    audCodec = AudCodec_drivers[type];

	HAL_AudCodecWrite(audCodec,reg, para);

	return kStatus_Success;
}


status_t HAL_AudCodecInit(HAL_AudCodecType_EN type)
{
    HAL_AudCodec_S *audCodec;

    ERR_CHECK(((uint8_t)type >=0 && (uint8_t)type < AUD_CODEC_DRIVERS_TOTAL_NUM), return kStatus_Fail);
    audCodec = AudCodec_drivers[type];

    switch(type) {
#if	AUD_CODEC_NAU88C22_W_ENABLE

    case tNau88c22_W: {
        HAL_AudCodecWrite(audCodec,0x00, 0x055);
        DRIVER_DELAY(5);
        HAL_AudCodecWrite(audCodec,0x01, 0x0ED);
        HAL_AudCodecWrite(audCodec,0x02, 0x03F);
        HAL_AudCodecWrite(audCodec,0x03, 0x183);
        HAL_AudCodecWrite(audCodec,0x04, 0x014);
        HAL_AudCodecWrite(audCodec,0x06, 0x009);
        HAL_AudCodecWrite(audCodec,0x0A, 0x008);
        HAL_AudCodecWrite(audCodec,0x0B, 0x0FF);
        HAL_AudCodecWrite(audCodec,0x0C, 0x1FF);
        HAL_AudCodecWrite(audCodec,0x0E, 0x008);
        HAL_AudCodecWrite(audCodec,0x0F, 0x0FF);
        HAL_AudCodecWrite(audCodec,0x10, 0x1FF);
        HAL_AudCodecWrite(audCodec,0x2C, 0x030);
		HAL_AudCodecWrite(audCodec,0x2D, 0x110); //模拟音量放大
        HAL_AudCodecWrite(audCodec,0x2F, 0x050);
        HAL_AudCodecWrite(audCodec,0x30, 0x050);
        HAL_AudCodecWrite(audCodec,0x31, 0x002);
        HAL_AudCodecWrite(audCodec,0x38, 0x001);
        HAL_AudCodecWrite(audCodec,0x39, 0x001);
    }
    break;
#endif

#if	AUD_CODEC_NAU88C22_D_ENABLE

    case tNau88c22_D: {
        HAL_AudCodecWrite(audCodec,0x00, 0x055);
        DRIVER_DELAY(5);
        HAL_AudCodecWrite(audCodec,0x01, 0x0AD);
        HAL_AudCodecWrite(audCodec,0x02, 0x03F);
        HAL_AudCodecWrite(audCodec,0x03, 0x103);
        HAL_AudCodecWrite(audCodec,0x04, 0x010);
        HAL_AudCodecWrite(audCodec,0x06, 0x008);
        HAL_AudCodecWrite(audCodec,0x0A, 0x008);
        HAL_AudCodecWrite(audCodec,0x0B, 0x1FF);  
		HAL_AudCodecWrite(audCodec,0x0C, 0x1FF);  
        HAL_AudCodecWrite(audCodec,0x0E, 0x008);
        HAL_AudCodecWrite(audCodec,0x0F, 0x1FF);
        HAL_AudCodecWrite(audCodec,0x2C, 0x003);
        HAL_AudCodecWrite(audCodec,0x2F, 0x050);
        HAL_AudCodecWrite(audCodec,0x30, 0x000);
        HAL_AudCodecWrite(audCodec,0x31, 0x002);
        HAL_AudCodecWrite(audCodec,0x38, 0x040);
        HAL_AudCodecWrite(audCodec,0x39, 0x008);
    }
    break;
#endif

    }

    return kStatus_Success;
}




