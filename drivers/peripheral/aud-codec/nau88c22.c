/**
 *
 *	@name				nau88c22.c
 *	@modify 			KT
 *	@date 				2019/11/29
 *	@brief				
 *
 *  @API				
 *
 *  @description   		
 *
 **/

#include "aud-codec/nau88c22.h"
#include "fsl_lpi2c.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* �Ĵ�����ַ���� */
#define NAU88c22_SUB_ADDR_SIZE					(1U)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*******************************************************************************
 * Variables
 ******************************************************************************/
/*******************************************************************************
 * Code
 ******************************************************************************/
status_t Nau88c22_Write(LPI2C_Type *base,uint16_t slaveAddr,uint32_t addr,uint8_t *data,uint16_t len)
{
	if(data == null || len == 0)
		return kStatus_Fail;

	/* ����Nau88c22ʱ��Ҫ�� */
	addr = (addr << 1) | (data[0] & 0x01);

	I2C_Write(base,slaveAddr,addr,NAU88c22_SUB_ADDR_SIZE,&data[1],1);

	return kStatus_Success;
}


status_t Nau88c22_Read(LPI2C_Type *base,uint16_t slaveAddr,uint32_t addr,uint8_t *data,uint16_t len)
{
	if(data == null || len == 0)
		return kStatus_Fail;

	/* ����Nau88c22ʱ��Ҫ�� */
	addr = (addr << 1) ;

	I2C_Read(base,slaveAddr,addr,NAU88c22_SUB_ADDR_SIZE,data,2);

	return kStatus_Success;
}




