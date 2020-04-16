#ifndef _DRIVERS_COMMON_
#define _DRIVERS_COMMON_

#include "MIMXRT1062.h"
#include "pin_mux.h"
//#include "log.h"
#include "global_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* 最大数值 */
#ifndef	MAX_NUM
#define MAX_NUM  										(0xffffffffUL)	
#endif

/* null值 */
#ifndef	null
#define null 											(0U)
#endif

/* 错误条件判断 */
#ifndef	ERR_CHECK
#define ERR_CHECK(condition, implement) 				do { if (!(condition)) {implement;}} while(0)
#endif

void DRIVER_DELAY(uint32_t dly);
/*******************************************************************************
 * 	PHY
 ******************************************************************************/
#include "fsl_enet.h"

/*! @brief Defines the timeout macro. */
#define PHY_TIMEOUT_COUNT (0x0FFFFU)

/*! @brief Defines the PHY status. */
enum _phy_status {
    kStatus_PHY_SMIVisitTimeout = MAKE_STATUS(kStatusGroup_PHY, 1),  /*!< ENET PHY SMI visit timeout. */
    kStatus_PHY_AutoNegotiateFail = MAKE_STATUS(kStatusGroup_PHY, 2) /*!< ENET PHY AutoNegotiate Fail. */
};

/*! @brief Defines the PHY link speed. This is align with the speed for ENET MAC. */
typedef enum _phy_speed {
    kPHY_Speed10M = 0U, /*!< ENET PHY 10M speed. */
    kPHY_Speed100M      /*!< ENET PHY 100M speed. */
} phy_speed_t;

/*! @brief Defines the PHY link duplex. */
typedef enum _phy_duplex {
    kPHY_HalfDuplex = 0U, /*!< ENET PHY half duplex. */
    kPHY_FullDuplex       /*!< ENET PHY full duplex. */
} phy_duplex_t;

/*! @brief Defines the PHY loopback mode. */
typedef enum _phy_loop {
    kPHY_LocalLoop = 0U, /*!< ENET PHY local loopback. */
    kPHY_RemoteLoop      /*!< ENET PHY remote loopback. */
} phy_loop_t;


status_t ENET_Write(ENET_Type *base, uint32_t phyAddr, uint32_t phyReg, uint32_t data);
status_t ENET_Read(ENET_Type *base, uint32_t phyAddr, uint32_t phyReg, uint32_t *dataPtr);


/*******************************************************************************
 * 	I2C
 ******************************************************************************/
#include "fsl_lpi2c.h"

status_t I2C_Write(LPI2C_Type *base,uint16_t slaveAddr,uint32_t subAdd,uint8_t subAddrSize,uint8_t *dataBuff,uint16_t dataLen);
status_t I2C_Read(LPI2C_Type *base,uint16_t slaveAddr,uint32_t subAdd,uint8_t subAddrSize,uint8_t* dataBuffer, uint16_t dataLen);

/*******************************************************************************
 * 	SPI
 ******************************************************************************/
#include "fsl_lpspi.h"

//status_t LPSPI_MasterRead(LPSPI_Type *base,uint32_t pcs,uint8_t reg,uint8_t *data,uint16_t len);
//status_t LPSPI_MasterWrite(LPSPI_Type *base,uint32_t pcs,uint8_t reg,uint8_t *data,uint16_t len);
status_t LPSPI_MasterRead(LPSPI_Type *base,uint32_t pcs,uint8_t *txData,uint8_t *rxData,uint16_t len);
status_t LPSPI_MasterWrite(LPSPI_Type *base,uint32_t pcs,uint8_t *txData,uint16_t len);

status_t LPSPI_MasterWriteSingleByte(LPSPI_Type *base,uint32_t pcs,uint8_t data);



#endif

