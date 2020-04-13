
/**
 *
 *	@name				hal_common.c
 *	@modify 			KT
 *	@date 				2019/10/25
 *	@brief
 *
 *  @API
 *
 *  @description
 *
 **/
#include "hal.h"

#include "board.h"
#include "pin_mux.h"
#include "fsl_iomuxc.h"
#include "fsl_dmamux.h"
#include "fsl_sai_edma.h"
#include "fsl_lpspi.h"
#include "fsl_lpi2c.h"
#include "fsl_enet.h"
#include "fsl_lpuart.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/*! @brief Pointers to enet clocks for each instance. */
extern clock_ip_name_t s_enetClock[FSL_FEATURE_SOC_ENET_COUNT];
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
/*******************************************************************************
 * Variables
 ******************************************************************************/
static void HAL_PinConfig(void){
	/* RTL8306 enable control*/
	IOMUXC_SetPinMux(IOMUXC_GPIO_B1_12_GPIO2_IO28,0U);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_12_GPIO2_IO28, 0xB0A9u);	
	
	/* RTL8306 enable control (temp test) */
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_11_GPIO1_IO11,0U);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_11_GPIO1_IO11, 0xB0A9u);	

	/* DM9051 INT input */
	IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_04_GPIO3_IO04,0U);                                   
	IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_04_GPIO3_IO04,0x10B0u);

	/* USB-WT2000 switch relay control */
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_11_GPIO1_IO11,0U);                                  
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_11_GPIO1_IO11, 0xB0A9u);   	//V4版本硬件
//	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_01_GPIO1_IO01,0U);                                  
//	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_01_GPIO1_IO01, 0xB0A9u);		//V3版本硬件

	/* USB Power control */
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_09_GPIO1_IO25,0U);                                  
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_09_GPIO1_IO25, 0xB0A9u);

	/* Lineout control H13 */
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_08_GPIO1_IO24,0U);                                  
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_08_GPIO1_IO24, 0xB0A9u);

	/* AFC control D14 */
	IOMUXC_SetPinMux(IOMUXC_GPIO_B1_13_GPIO2_IO29,0U);                                  
	IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_13_GPIO2_IO29, 0xB0A9u);

	/* Partition Out Ctrl L10   */
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_15_GPIO1_IO15,0U);                                  
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_15_GPIO1_IO15, 0xB0A9u);

	/* Device running signal (L12) */
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_04_GPIO1_IO20,0U);                                  
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_04_GPIO1_IO20, 0xB0A9u);

	/* 485 & 232 Power control */
	IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_03_GPIO3_IO03,0U);                                  
	IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_03_GPIO3_IO03, 0xB0A9u);

	/* 485 Transceiver control */
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_05_GPIO1_IO21,0U);                                  
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_05_GPIO1_IO21, 0xB0A9u);

	/* Fire alarm signal(H14) */
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_14_GPIO1_IO14,0U);                                  
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_14_GPIO1_IO14, 0xB0A9u);

#if (defined(LPI2C1_ENABLE) && LPI2C1_ENABLE)
	/* I2C1 */
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_00_LPI2C1_SCL, 1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_01_LPI2C1_SDA, 1U);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_00_LPI2C1_SCL,0xD8B0u); 
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_01_LPI2C1_SDA,0xD8B0u); 
#endif
	
#if (defined(LPI2C2_ENABLE) && LPI2C2_ENABLE)
	/* I2C2 */
	IOMUXC_SetPinMux(IOMUXC_GPIO_B0_04_LPI2C2_SCL, 1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_05_LPI2C2_SDA, 1U);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_04_LPI2C2_SCL,0xD8B0u); 
	IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_05_LPI2C2_SDA,0xD8B0u); 
#endif
	
#if (defined(LPI2C3_ENABLE) && LPI2C3_ENABLE)
	/* I2C3 */
//	IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_22_LPI2C3_SCL, 1U);
//    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_21_LPI2C3_SDA, 1U);
//	IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_22_LPI2C3_SCL,0xD8B0u); 
//	IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_21_LPI2C3_SDA,0xD8B0u); 

	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_07_LPI2C3_SCL, 1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_06_LPI2C3_SDA, 1U);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_07_LPI2C3_SCL,0xD8B0u); 
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_06_LPI2C3_SDA,0xD8B0u); 

#endif

#if (defined(LPSPI1_ENABLE) && LPSPI1_ENABLE)
	/* SPI1 */
	IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_00_LPSPI1_SCK,0U);
  	IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_01_LPSPI1_PCS0,0U);
  	IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_02_LPSPI1_SDO,0U);
  	IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_03_LPSPI1_SDI,0U);      
	IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_00_LPSPI1_SCK,  0x10B0u); 							  
	IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_01_LPSPI1_PCS0,  0x10B0u); 							  
	IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_02_LPSPI1_SDO,  0x10B0u); 							 
	IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_03_LPSPI1_SDI, 0x10B0u); 
#endif

#if (defined(LPSPI2_ENABLE) && LPSPI2_ENABLE)
	/* SPI2 */
#endif


#if (defined(LPSPI3_ENABLE) && LPSPI3_ENABLE)
	/* SPI3 */
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_15_LPSPI3_SCK,  0U); 								   
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_14_LPSPI3_SDO,  0U); 								   
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_13_LPSPI3_SDI,  0U); 								   
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_12_LPSPI3_PCS0, 0U); 								   
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_15_LPSPI3_SCK,  0x10B0u); 							  
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_14_LPSPI3_SDO,  0x10B0u); 							  
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_13_LPSPI3_SDI,  0x10B0u); 							 
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_12_LPSPI3_PCS0, 0x10B0u); 
#endif

#if (defined(ENET_ENABLE) && ENET_ENABLE)
	/* ENET */
	IOMUXC_SetPinMux(IOMUXC_GPIO_B1_04_ENET_RX_DATA00,0U);									
	IOMUXC_SetPinMux(IOMUXC_GPIO_B1_05_ENET_RX_DATA01,0U);									
	IOMUXC_SetPinMux(IOMUXC_GPIO_B1_06_ENET_RX_EN,0U);									
	IOMUXC_SetPinMux(IOMUXC_GPIO_B1_07_ENET_TX_DATA00,0U);									
	IOMUXC_SetPinMux(IOMUXC_GPIO_B1_08_ENET_TX_DATA01,0U);									
	IOMUXC_SetPinMux(IOMUXC_GPIO_B1_09_ENET_TX_EN,0U);									
	IOMUXC_SetPinMux(IOMUXC_GPIO_B1_10_ENET_REF_CLK,1U);									
	IOMUXC_SetPinMux(IOMUXC_GPIO_B1_11_ENET_RX_ER,0U);		

	IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_04_ENET_RX_DATA00, 0xB0E9u);                               
  	IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_05_ENET_RX_DATA01,0xB0E9u);                               
  	IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_06_ENET_RX_EN, 0xB0E9u);                               
 	IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_07_ENET_TX_DATA00, 0xB0E9u);                             
  	IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_08_ENET_TX_DATA01, 0xB0E9u);                               
  	IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_09_ENET_TX_EN, 0xB0E9u);                               
  	IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_10_ENET_REF_CLK,0x31u);                                 
  	IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_11_ENET_RX_ER, 0xB0E9u);  
#endif

#if (defined(SMI_ENABLE) && SMI_ENABLE)
	/* SMI */
	IOMUXC_SetPinMux(IOMUXC_GPIO_B1_14_ENET_MDC,0U);									
	IOMUXC_SetPinMux(IOMUXC_GPIO_B1_15_ENET_MDIO,0U);	
	IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_14_ENET_MDC,0xB0E9u);								
	IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_15_ENET_MDIO,0xB829u);	
#endif

#if (defined(ENET2_ENABLE) && ENET2_ENABLE)
  	/* ENET2 */
	IOMUXC_SetPinMux(IOMUXC_GPIO_B1_01_ENET2_RDATA00,0U);									
	IOMUXC_SetPinMux(IOMUXC_GPIO_B1_02_ENET2_RDATA01,0U);									
	IOMUXC_SetPinMux(IOMUXC_GPIO_B1_03_ENET2_RX_EN,0U);									
	IOMUXC_SetPinMux(IOMUXC_GPIO_B0_12_ENET2_TDATA00,0U);									
	IOMUXC_SetPinMux(IOMUXC_GPIO_B0_13_ENET2_TDATA01,0U);									
	IOMUXC_SetPinMux(IOMUXC_GPIO_B0_14_ENET2_TX_EN,0U);									
	IOMUXC_SetPinMux(IOMUXC_GPIO_B0_15_ENET2_REF_CLK2,1U);									
	IOMUXC_SetPinMux(IOMUXC_GPIO_B1_00_ENET2_RX_ER,0U);		

	IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_01_ENET2_RDATA00, 0xB0E9u); 
  	IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_02_ENET2_RDATA01,0xB0E9u); 
 	IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_03_ENET2_RX_EN, 0xB0E9u); 
  	IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_12_ENET2_TDATA00,  0xB0E9u); 
 	IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_13_ENET2_TDATA01, 0xB0E9u);  
  	IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_14_ENET2_TX_EN, 0xB0E9u); 
  	IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_15_ENET2_REF_CLK2, 0x31u);  
  	IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_00_ENET2_RX_ER,0xB0E9u);     
#endif

#if (defined(SMI2_ENABLE) && SMI2_ENABLE)
	/* SMI2 */
	IOMUXC_SetPinMux(IOMUXC_GPIO_B0_00_ENET2_MDC,0U);								
	IOMUXC_SetPinMux(IOMUXC_GPIO_B0_01_ENET2_MDIO,0U);	
	IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_00_ENET2_MDC,0xB0E9u);                               
  	IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_01_ENET2_MDIO,0xB829u); 
#endif

#if (defined(UART1_ENABLE) && UART1_ENABLE)
	/* UART1 */
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_12_LPUART1_TX,0U);									
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_13_LPUART1_RX,0U);   
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_12_LPUART1_TX,0x10B0u);								
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_13_LPUART1_RX,0x10B0u);								
#endif

#if (defined(UART2_ENABLE) && UART2_ENABLE)
	/* UART2 */
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_02_LPUART2_TX,0U);									
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_03_LPUART2_RX,0U);	
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_02_LPUART2_TX,0x10B0u);								
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_03_LPUART2_RX,0x10B0u);
#endif

#if (defined(UART3_ENABLE) && UART3_ENABLE)
	/* UART3 */
	IOMUXC_SetPinMux(IOMUXC_GPIO_B0_08_LPUART3_TX,0U);									
	IOMUXC_SetPinMux(IOMUXC_GPIO_B0_09_LPUART3_RX,0U);   
	IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_08_LPUART3_TX,0x10B0u);								
	IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_09_LPUART3_RX,0x10B0u);
#endif

#if (defined(UART4_ENABLE) && UART4_ENABLE)
	/* UART4 */
#endif

#if (defined(UART5_ENABLE) && UART5_ENABLE)
	/* UART5 */
	IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_23_LPUART5_TX,0U);									
	IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_24_LPUART5_RX,0U);   
	IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_23_LPUART5_TX,0x10B0u);								
	IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_24_LPUART5_RX,0x10B0u);	
#endif

#if (defined(UART6_ENABLE) && UART6_ENABLE)
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_02_LPUART6_TX,0U);									
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_03_LPUART6_RX,0U);   
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_02_LPUART6_TX,0x10B0u);								
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_03_LPUART6_RX,0x10B0u);
#endif

#if (defined(UART7_ENABLE) && UART7_ENABLE)
#endif

#if (defined(UART8_ENABLE) && UART8_ENABLE)
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_10_LPUART8_TX,0U);									
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_11_LPUART8_RX,0U);   
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_10_LPUART8_TX,0x10B0u);								
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_11_LPUART8_RX,0x10B0u);
#endif

#if (defined(SAI1_ENABLE) && SAI1_ENABLE)
		/* SAI1 */
		IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_12_SAI1_RX_DATA00, 1U);
		IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_13_SAI1_TX_DATA00, 1U);
		IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_14_SAI1_TX_BCLK, 1U);
		IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_15_SAI1_TX_SYNC, 1U);
		IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_09_SAI1_MCLK, 1U);
	
		IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_12_SAI1_RX_DATA00,0x10B0u);							  
		IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_13_SAI1_TX_DATA00,0x10B0u);							   
		IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_14_SAI1_TX_BCLK, 0x10B0u); 
		IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_15_SAI1_TX_SYNC,0x10B0u); 
		IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_09_SAI1_MCLK,0x10B0u); 
#endif
	
#if (defined(SAI2_ENABLE) && SAI2_ENABLE)
		/* SAI2 */
		IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_04_SAI2_TX_DATA, 1U);
		IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_05_SAI2_TX_SYNC, 1U);
		IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_06_SAI2_TX_BCLK, 1U);
		IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_08_SAI2_RX_DATA, 1U);
		IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_07_SAI2_MCLK, 1U);
	
		IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_04_SAI2_TX_DATA,0x10B0u);							  
		IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_05_SAI2_TX_SYNC,0x10B0u);							   
		IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_06_SAI2_TX_BCLK, 0x10B0u); 
		IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_08_SAI2_RX_DATA,0x10B0u); 
		IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_07_SAI2_MCLK,0x10B0u);
#endif
	
#if (defined(SAI3_ENABLE) && SAI3_ENABLE)
		/* SAI3 */
		IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_36_SAI3_TX_DATA, 1U);
		IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_39_SAI3_TX_SYNC, 1U);
		IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_38_SAI3_TX_BCLK, 1U);
		IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_33_SAI3_RX_DATA, 1U);
		IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_37_SAI3_MCLK, 1U);
	
		IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_36_SAI3_TX_DATA,0x10B0u);							  
		IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_39_SAI3_TX_SYNC,0x10B0u);							   
		IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_38_SAI3_TX_BCLK, 0x10B0u); 
		IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_33_SAI3_RX_DATA,0x10B0u); 
		IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_37_SAI3_MCLK,0x10B0u); 
#endif


}

static void HAL_SaiConfig(void)
{

#if SAI1_ENABLE || SAI2_ENABLE || SAI3_ENABLE

    edma_config_t dmaConfig;

	clock_audio_pll_config_t audioPllCfg = {
	    .loopDivider	= 	AUDIO_PLL_LOOP_DIVIDER,		/* PLL loop divider. Valid range for DIV_SELECT divider value: 27~54. */
	    .postDivider	= 	AUDIO_PLL_POST_DIVIDER,		/* Divider after the PLL, should only be 1, 2, 4, 8, 16. */
	    .numerator		= 	AUDIO_PLL_NUMERATOR,		/* 30 bit numerator of fractional loop divider. */
	    .denominator	= 	AUDIO_PLL_DENOMINATOR, 	/* 30 bit denominator of fractional loop divider */
	};

    /* 初始化时钟 */
//    CLOCK_InitAudioPll(&audioPllCfg);
	

    /* DMA初始化 */
//    EDMA_GetDefaultConfig(&dmaConfig);
//    EDMA_Init(SAI_DMA, &dmaConfig);

//    DMAMUX_Init(SAI_DMAMUX);
#endif
}

static void HAL_I2cMasterConfig(void){

	lpi2c_master_config_t masterConfig = {0};
	
	LPI2C_MasterGetDefaultConfig(&masterConfig);
#if defined(LPI2C1_ENABLE) && LPI2C1_ENABLE

	/* Change the default baudrate configuration */
    masterConfig.baudRate_Hz = LPI2C1_BAUDRATE;

    /* Initialize the LPI2C master peripheral */
    LPI2C_MasterInit(LPI2C1, &masterConfig, I2C_CLOCK_FREQ);
#endif

#if defined(LPI2C2_ENABLE) && LPI2C2_ENABLE
	
	/* Change the default baudrate configuration */
    masterConfig.baudRate_Hz = LPI2C2_BAUDRATE;

    /* Initialize the LPI2C master peripheral */
    LPI2C_MasterInit(LPI2C2, &masterConfig, I2C_CLOCK_FREQ);
#endif

#if defined(LPI2C3_ENABLE) && LPI2C3_ENABLE
	
	/* Change the default baudrate configuration */
    masterConfig.baudRate_Hz = LPI2C3_BAUDRATE;

    /* Initialize the LPI2C master peripheral */
    LPI2C_MasterInit(LPI2C3, &masterConfig, I2C_CLOCK_FREQ);
#endif

}

static void HAL_EnetConfig(void){
	uint32_t instance, sysClock;

	sysClock = CLOCK_GetFreq(SMI_CLOCK_NAME);

#if (defined(ENET_ENABLE) && ENET_ENABLE)
	instance = ENET_GetInstance(ENET);

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Set SMI first. */
    CLOCK_EnableClock(s_enetClock[instance]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

#if (defined(SMI_ENABLE) && SMI_ENABLE)
    ENET_SetSMI(ENET, sysClock, false);
#endif

#endif

#if (defined(ENET2_ENABLE) && ENET2_ENABLE)
	instance = ENET_GetInstance(ENET2);

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Set SMI first. */
    CLOCK_EnableClock(s_enetClock[instance]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

#if (defined(SMI2_ENABLE) && SMI2_ENABLE)
    ENET_SetSMI(ENET2, sysClock, false);
#endif

#endif

}

static void HAL_SpiMasterConfig(void){
	lpspi_master_config_t masterConfig;
	LPSPI_Type *base;

	/*Set clock source for LPSPI*/
	CLOCK_SetMux(kCLOCK_LpspiMux, LPSPI_CLOCK_SOURCE_SELECT);
	CLOCK_SetDiv(kCLOCK_LpspiDiv, LPSPI_CLOCK_SOURCE_DIVIDER);

#if defined(LPSPI1_ENABLE) && LPSPI1_ENABLE
	base = LPSPI1;

	LPSPI_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate = LPSPI1_BAUDRATE;
	masterConfig.pcsToSckDelayInNanoSec = LPSPI1_DELAY_NANO_SEC;
    masterConfig.lastSckToPcsDelayInNanoSec = LPSPI1_DELAY_NANO_SEC;
    masterConfig.betweenTransferDelayInNanoSec = LPSPI1_DELAY_NANO_SEC;
	masterConfig.whichPcs = LPSPI1_MASTER_PCS;
	
    LPSPI_MasterInit(base, &masterConfig, LPSPI_MASTER_CLK_FREQ);
	
#if (defined(LPSPI_USE_IRQ) && LPSPI_USE_IRQ)
	LPSPI_MasterTransferCreateHandle(base,&SpiMasterHandler,MacPhy_SpiTransCallback,0);
#endif
	
    LPSPI_Enable(base, true);

#if (defined(LPSPI_USE_IRQ) && LPSPI_USE_IRQ)
    /* Enable the NVIC for LPSPI peripheral. Note that below code is useless if the LPSPI interrupt is in INTMUX ,
    * and you should also enable the INTMUX interupt in your application.
    */
    EnableIRQ(LPSPI1_IRQn);
	LPSPI_EnableInterrupts(base, kLPSPI_RxInterruptEnable);
	LPSPI_EnableInterrupts(base, kLPSPI_TxInterruptEnable);
#endif
    /*TCR is also shared the FIFO , so wait for TCR written.*/
    while (LPSPI_GetTxFifoCount(base) != 0)
    {
    }
#endif

#if defined(LPSPI2_ENABLE) && LPSPI2_ENABLE
	base = LPSPI2;

	LPSPI_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate = LPSPI2_BAUDRATE;
	masterConfig.pcsToSckDelayInNanoSec = LPSPI2_DELAY_NANO_SEC;
    masterConfig.lastSckToPcsDelayInNanoSec = LPSPI2_DELAY_NANO_SEC;
    masterConfig.betweenTransferDelayInNanoSec = LPSPI2_DELAY_NANO_SEC;
	masterConfig.whichPcs = LPSPI2_MASTER_PCS;
	
    LPSPI_MasterInit(base, &masterConfig, LPSPI_MASTER_CLK_FREQ);
	
#if (defined(LPSPI_USE_IRQ) && LPSPI_USE_IRQ)
	LPSPI_MasterTransferCreateHandle(base,&SpiMasterHandler,MacPhy_SpiTransCallback,0);
#endif
	
    LPSPI_Enable(base, true);

#if (defined(LPSPI_USE_IRQ) && LPSPI_USE_IRQ)
    /* Enable the NVIC for LPSPI peripheral. Note that below code is useless if the LPSPI interrupt is in INTMUX ,
    * and you should also enable the INTMUX interupt in your application.
    */
    EnableIRQ(LPSPI2_IRQn);
	LPSPI_EnableInterrupts(base, kLPSPI_RxInterruptEnable);
	LPSPI_EnableInterrupts(base, kLPSPI_TxInterruptEnable);
#endif
    /*TCR is also shared the FIFO , so wait for TCR written.*/
    while (LPSPI_GetTxFifoCount(base) != 0)
    {
    }
#endif

#if defined(LPSPI3_ENABLE) && LPSPI3_ENABLE
	base = LPSPI3;

	LPSPI_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate = LPSPI3_BAUDRATE;
	masterConfig.pcsToSckDelayInNanoSec = LPSPI3_DELAY_NANO_SEC;
    masterConfig.lastSckToPcsDelayInNanoSec = LPSPI3_DELAY_NANO_SEC;
    masterConfig.betweenTransferDelayInNanoSec = LPSPI3_DELAY_NANO_SEC;
	masterConfig.whichPcs = LPSPI3_MASTER_PCS;
	
    LPSPI_MasterInit(base, &masterConfig, LPSPI_MASTER_CLK_FREQ);
	
#if (defined(LPSPI_USE_IRQ) && LPSPI_USE_IRQ)
	LPSPI_MasterTransferCreateHandle(base,&SpiMasterHandler,MacPhy_SpiTransCallback,0);
#endif
    LPSPI_Enable(base, true);

#if (defined(LPSPI_USE_IRQ) && LPSPI_USE_IRQ)
    /* Enable the NVIC for LPSPI peripheral. Note that below code is useless if the LPSPI interrupt is in INTMUX ,
    * and you should also enable the INTMUX interupt in your application.
    */
    EnableIRQ(LPSPI3_IRQn);
	LPSPI_EnableInterrupts(base, kLPSPI_RxInterruptEnable);
	LPSPI_EnableInterrupts(base, kLPSPI_TxInterruptEnable);
#endif
    /*TCR is also shared the FIFO , so wait for TCR written.*/
    while (LPSPI_GetTxFifoCount(base) != 0)
    {
    }
#endif
}

static void HAL_UartConfig(void)
{
	LPUART_Type *base;
	lpuart_config_t lpuartConfig;
	uint32_t srcClock_Hz = BOARD_DebugConsoleSrcFreq();
	
#if defined(LOG_SERIAL_PORT) && defined(LOG_BAUDRATE)
	base = LOG_SERIAL_PORT;

	LPUART_GetDefaultConfig(&lpuartConfig);
	lpuartConfig.baudRate_Bps = LOG_BAUDRATE;
    lpuartConfig.enableRx = true;
    lpuartConfig.enableTx = true;

	LPUART_Init(base, &lpuartConfig, srcClock_Hz);
#endif

#if defined(UART6_ENABLE) && UART6_ENABLE && defined(UART6_USE_IN_DRIVER)
	base = LPUART6;

	LPUART_GetDefaultConfig(&lpuartConfig);
	lpuartConfig.baudRate_Bps = UART6_BAUDRATE;
    lpuartConfig.enableRx = true;
    lpuartConfig.enableTx = true;

	LPUART_Init(base, &lpuartConfig, srcClock_Hz);

	
	NVIC_SetPriority(LPUART6_IRQn, UART_DEF_ISR_PRIORITY);
	LPUART_EnableInterrupts(base, kLPUART_RxDataRegFullInterruptEnable);
	EnableIRQ(LPUART6_IRQn);
#endif

}

void HAL_Init(void){
	/* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitModuleClock();

	/* HAL */
	HAL_PinConfig();
	HAL_EnetConfig();
	HAL_I2cMasterConfig();
//	HAL_SaiInit();
	HAL_UartConfig();
	HAL_SpiMasterConfig();
}




