/**
 *
 *	@name				dm9051.c
 *	@modify 			KT
 *	@date 				2019/10/17
 *	@brief
 *
 *  @API
 *
 *  @description
 *
 **/
#include "dm9051.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* DM9051 PHY address  */
#define DM9051_PHY_ADDR          					(0x01)
#define DM9051_PHY_SHIFT(x)							(x << 6)
#define DM9051_PHY_MASK								DM9051_PHY_SHIFT(DM9051_PHY_ADDR)


#define DM9051_ID           						(0x90510A46)		/* DM9051A ID */
#define DM9051_PKT_MAX     							(1536)          /* Received packet max size */
//#define DM9051_PKT_RDY      (0x01)          /* Packet ready to receive */

/*  Network Control Register (00H) */
#define DM9051_NCR          						(0x00)
#define DM9051_REG_RESET    						(0x01)
#define DM9051_NCR_DEFAULT							(0x0)

/* Network Status Register (01H) */
#define DM9051_NSR          						(0x01)
#define DM9051_NSR_SPEED           					(1 << 7)
#define DM9051_NSR_LINKST          					(1 << 6)
#define DM9051_NSR_WAKEST          					(1 << 5)
#define DM9051_NSR_TX2END          					(1 << 3)
#define DM9051_NSR_TX1END         					(1 << 2)
#define DM9051_NSR_RXOV            					(1 << 1)
#define DM9051_NSR_RXRDY            				(1 << 0)
#define DM9051_NSR_CLR_STATUS						(DM9051_NSR_WAKEST | DM9051_NSR_TX2END | DM9051_NSR_TX1END)

/* TX Control Register (02H) */
#define DM9051_TCR          						(0x02)
#define DM9051_TCR_TJDIS           					(1 << 6)
#define DM9051_TCR_EXCECM          					(1 << 5)
#define DM9051_TCR_PAD_DIS2        					(1 << 4)
#define DM9051_TCR_CRC_DIS2        					(1 << 3)
#define DM9051_TCR_PAD_DIS1        					(1 << 2)
#define DM9051_TCR_CRC_DIS1        					(1 << 1)
#define DM9051_TCR_TXREQ           					(1 << 0)
#define DM9051_TCR_DEFAULT							(0x0)


#define DM9051_TSR1         						(0x03)
#define DM9051_TSR2         						(0x04)

/* RX Control Register (05H) */
#define DM9051_RCR          						(0x05)
#define DM9051_RCR_WTDIS       					    (1 << 6)
#define DM9051_RCR_DIS_LONG    					    (1 << 5)
#define DM9051_RCR_DIS_CRC     					    (1 << 4)
#define DM9051_RCR_ALL         					    (1 << 3)
#define DM9051_RCR_RUNT        					    (1 << 2)
#define DM9051_RCR_PRMSC        					(1 << 1)
#define DM9051_RCR_RXEN								(1 << 0)
#define DM9051_RCR_DEFAULT							(DM9051_RCR_DIS_LONG | DM9051_RCR_DIS_CRC)
#define DM9051_RCR_SET      						(0x31)


#define DM9051_RSR          						(0x06)
#define DM9051_ROCR         						(0x07)

/* Back Pressure Threshold Register (08H) */
#define DM9051_BPTR         						(0x08)
#define DM9051_BPTR_DEFAULT							(0x3f)

/* Flow Control Threshold Register (09H) */
#define DM9051_FCTR         						(0x09)
#define DM9051_FCTR_DEFAULT							(0x3A)

/*  RX/TX Flow Control Register (0AH) */
#define DM9051_FCR          						(0x0A)
#define DM9051_FCR_DEFAULT							(0xFF)

/* EEPROM & PHY Control Register (0BH) */
#define DM9051_EPCR         						(0x0B)
#define DM9051_EPCR_RD_PHY							(0x0C)
#define DM9051_EPCR_WR_PHY							(0x0A)
#define DM9051_EPCR_RD_EEP							(0x04)
#define DM9051_EPCR_WR_EEP							(0x02)

/* EEPROM & PHY Address Register (0CH) */
#define DM9051_EPAR         						(0x0C)
/* EEPROM & PHY Data Register (EE_PHY_L:0DH EE_PHY_H:0EH) */
#define DM9051_EPDRL        						(0x0D)
#define DM9051_EPDRH        						(0x0E)

#define DM9051_WCR          						(0x0F)

/*  Physical Address Register (10H~15H) */
#define DM9051_PAR          						(0x10)
/* Multicast Address Hash Table Register (16H~1DH) */
#define DM9051_MAR          						(0x16)

/* General Purpose Control Register (1EH) */
#define DM9051_GPCR         						(0x1E)
#define DM9051_GPCR_GEP_CNTL       					(1 << 0)

/* General Purpose Register (1FH) */
#define DM9051_GPR          						(0x1f)

#define DM9051_TRPAL        						(0x22)
#define DM9051_TRPAH        						(0x23)
#define DM9051_RWPAL        						(0x24)
#define DM9051_RWPAH        						(0x25)

/*  Vendor ID Register (28H~29H) */
#define DM9051_VIDL         						(0x28)
#define DM9051_VIDH         						(0x29)
/* Product ID Register (2AH~2BH) */
#define DM9051_PIDL         						(0x2A)
#define DM9051_PIDH         						(0x2B)

#define DM9051_CHIPR        						(0x2C)

/* Transmit Control Register 2 (2DH) */
#define DM9051_TCR2         						(0x2D)
#define DM9051_TCR2_SET     						(0x90)	/* set one packet */


#define DM9051_OTCR         						(0x2E)

/*  Set Memory Conttrol Register */
#define DM9051_SMCR         						(0x2F)
#define DM9051_SMCR_DEFAULT							(0x0)

#define DM9051_ETCR         						(0x30)    /* early transmit control/status register  */

/* Transmit Check Sum Control Register (31H) */
#define DM9051_TCSCR         						(0x31)
#define DM9051_TCSCR_DEF							(0xFF)

#define DM9051_RCSSR        						(0x32)    /* receive check sum status register                                  */
#define DM9051_PBCR									(0x38)

/* INT Pin Control Register (39H) */
#define DM9051_INTR									(0x39)

/*  Memory Pointer Control Register (55H) */
#define DM9051_MPCR									(0x55)

/* Memory Data Pre-Fetch Read Command without Address Increment Register (70H) */
#define DM9051_MRCMDX       						(0x70)

#define DM9051_MRCMD        						(0x72)

#define DM9051_MRRL         						(0x74)
#define DM9051_MRRH         						(0x75)
#define DM9051_MWCMDX       						(0x76)
#define DM9051_MWCMD        						(0x78)
#define DM9051_MWRL         						(0x7A)
#define DM9051_MWRH         						(0x7B)

/*TX Packet Length Register (7CH~7DH)*/
#define DM9051_TXPLL        						(0x7C)
#define DM9051_TXPLH        						(0x7D)

/* Interrupt Status Register (7EH) */
#define DM9051_ISR          						(0x7E)
#define DM9051_ISR_LNKCHG          					(1 << 5)
#define DM9051_ISR_ROOS            					(1 << 3)
#define DM9051_ISR_ROS             					(1 << 2)
#define DM9051_ISR_PTS             					(1 << 1)
#define DM9051_ISR_PRS             					(1 << 0)
#define DM9051_ISR_CLR_STATUS      					(0x80 | 0x3F)

/* Interrupt Mask Register (7FH) */
#define DM9051_IMR          						(0x7F)
#define DM9051_IMR_PAR             					(1 << 7)
#define DM9051_IMR_LNKCHGI        					(1 << 5)
#define DM9051_IMR_UDRUN							(1 << 4)
#define DM9051_IMR_ROOM            					(1 << 3)
#define DM9051_IMR_ROM             					(1 << 2)
#define DM9051_IMR_PTM             					(1 << 1)
#define DM9051_IMR_PRM             					(1 << 0)
#define DM9051_IMR_FULL 							(DM9051_IMR_PAR | DM9051_IMR_LNKCHGI | DM9051_IMR_UDRUN |  \
													 DM9051_IMR_ROOM | DM9051_IMR_ROM | DM9051_IMR_PTM | DM9051_IMR_PRM)
#define DM9051_IMR_OFF								(DM9051_IMR_PAR)
//#define DM9051_IMR_DEFAULT							(DM9051_IMR_PAR | DM9051_IMR_PRM | DM9051_IMR_LNKCHGI)
#define DM9051_IMR_DEFAULT							(DM9051_IMR_FULL)

/* Memory Serial Write */
#define DM9051_MEM_WR_CONTINUOUS					(0xF8)
#define DM9051_MEM_RD_CONTINUOUS					(0x72)


/*! @brief Defines the PHY registers. */
#define DM9051_BASICCONTROL_REG						(0x00U)			/*!< The PHY basic control register. */
#define DM9051_BASICSTATUS_REG						(0x01U)			/*!< The PHY basic status register. */
#define DM9051_ID1_REG								(0x02U)			/*!< The PHY ID one register. */
#define DM9051_ID2_REG								(0x03U)			/*!< The PHY ID two register. */
#define DM9051_AUTONEG_ADVERTISE_REG				(0x04U)			/*!< The PHY auto-negotiate advertise register. */
#define DM9051_SPECIAL_CONTROL_STATUS_REG			(0x11U)			/*!< The PHY special control/status register. */

/*! @brief Defines the mask flag in basic status register. */
#define DM9051_BSTATUS_LINKSTATUS_MASK				(0x0004U)		/*!< The PHY link status mask. */
#define DM9051_BSTATUS_AUTONEGABLE_MASK				(0x0008U)		/*!< The PHY auto-negotiation ability mask. */
#define DM9051_BSTATUS_AUTONEGCOMP_MASK				(0x0020U)		/*!< The PHY auto-negotiation complete mask. */

/*! @brief Defines the mask flag in PHY special control/status register. */
#define DM9051_10HALFDUPLEX_MASK					(0x1000U) 		/*!< The PHY 10M half duplex mask. */
#define DM9051_100HALFDUPLEX_MASK					(0x4000U) 		/*!< The PHY 100M half duplex mask. */
#define DM9051_10FULLDUPLEX_MASK					(0x2000U)  		/*!< The PHY 10M full duplex mask. */
#define DM9051_100FULLDUPLEX_MASK					(0x8000U) 		/*!< The PHY 100M full duplex mask. */
#define DM9051_SPEEDUPLX_MASK						(0xF000U)		/*!< The PHY speed and duplex mask. */


typedef enum {
    DM9051_10MHD   = 0,
    DM9051_100MHD  = 1,
    DM9051_10MFD   = 4,
    DM9051_100MFD  = 5,
    DM9051_10M     = 6,
    DM9051_AUTO    = 8,
    DM9051_1M_HPNA = 0x10
} DM9051_PHY_mode;


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*******************************************************************************
 * Variables
 ******************************************************************************/
/*******************************************************************************
 * Code
 ******************************************************************************/

static uint8_t Dm9051_SpiRead(LPSPI_Type *base,uint32_t pcs,uint8_t reg)
{
    uint8_t txData[2] = {0},rxData[2] = {0};
	
	txData[0] = reg;
    LPSPI_MasterRead(base,pcs,txData,rxData,2);

    return rxData[1];
}

static void Dm9051_SpiWrite(LPSPI_Type *base,uint32_t pcs,uint8_t reg,uint8_t data)
{
	uint8_t txData[2];
	
    txData[0] = reg | 0x80;
	txData[1] = data;
    LPSPI_MasterWrite(base,pcs,txData,2);
}

static void Dm9051_SpiReadBuf(LPSPI_Type *base,uint32_t pcs,uint8_t reg,uint8_t *buf,uint16_t len)
{
    uint8_t txData[len + 1],rxData[len + 1];
	
	memset(txData,0,len + 1);
	memset(rxData,0,len + 1);
	
	txData[0] = reg;
    LPSPI_MasterRead(base,pcs,txData,rxData,len + 1);
	memcpy(buf,&rxData[1],len);
}

static void Dm9051_SpiWriteBuf(LPSPI_Type *base,uint32_t pcs,uint8_t reg,uint8_t *buf,uint16_t len)
{
	uint8_t txData[len + 1];
	
    txData[0] = reg;
	memcpy(&txData[1],buf,len);
    LPSPI_MasterWrite(base,pcs,txData,len + 1);
}



static status_t Dm9051_PhyRead(LPSPI_Type *base,uint32_t pcs,uint8_t reg,uint32_t *data)
{
	uint16_t cnt = 1000;
    ERR_CHECK((base != null) && (data != null), return kStatus_Fail);

    Dm9051_SpiWrite(base,pcs,DM9051_EPAR, DM9051_PHY_MASK | reg);
    Dm9051_SpiWrite(base,pcs,DM9051_EPCR, DM9051_EPCR_RD_PHY);
    while((Dm9051_SpiRead(base,pcs,DM9051_EPCR) & 0x1)  &&  cnt--) {
        DRIVER_DELAY(1);
    }
    Dm9051_SpiWrite(base,pcs,DM9051_EPCR, 0x0);
    *data = (Dm9051_SpiRead(base,pcs,DM9051_EPDRH) << 8) | Dm9051_SpiRead(base,pcs,DM9051_EPDRL);

    return kStatus_Success;
}

static status_t Dm9051_PhyWrite(LPSPI_Type *base,uint32_t pcs,uint8_t reg, uint16_t data)
{
	uint16_t cnt = 1000;
    ERR_CHECK((base != null), return kStatus_Fail);

    Dm9051_SpiWrite(base,pcs,DM9051_EPAR, DM9051_PHY_MASK | reg);
    Dm9051_SpiWrite(base,pcs,DM9051_EPDRL, (data & 0xff));
    Dm9051_SpiWrite(base,pcs,DM9051_EPDRH, ((data >> 8) & 0xff));
    Dm9051_SpiWrite(base,pcs,DM9051_EPCR, DM9051_EPCR_WR_PHY);
    while((Dm9051_SpiRead(base,pcs,DM9051_EPCR) & 0x1)  &&  cnt--)  {
        DRIVER_DELAY(1);
    }
    Dm9051_SpiWrite(base,pcs,DM9051_EPCR, 0x0);

    return kStatus_Success;
}


static void Dm9051_SetPhyMode(LPSPI_Type *base,uint32_t pcs,uint32_t uMediaMode)
{
    uint16_t phy_reg4 = 0x01e1, phy_reg0 = 0x1000;

    if (!(uMediaMode & DM9051_AUTO) ) {
        switch (uMediaMode) {
        case DM9051_10MHD: {
            phy_reg4 = 0x21;
            phy_reg0 = 0x0000;
            break;
        }
        case DM9051_10MFD: {
            phy_reg4 = 0x41;
            phy_reg0 = 0x1100;
            break;
        }
        case DM9051_100MHD: {
            phy_reg4 = 0x81;
            phy_reg0 = 0x2000;
            break;
        }
        case DM9051_100MFD: {
            phy_reg4 = 0x101;
            phy_reg0 = 0x3100;
            break;
        }
        case DM9051_10M: {
            phy_reg4 = 0x61;
            phy_reg0 = 0x1200;
            break;
        }
        }

        /* Set PHY media mode */
        Dm9051_PhyWrite(base,pcs,4, phy_reg4);
        /* Write rphy_reg0 to Tmp */
        Dm9051_PhyWrite(base,pcs,0, phy_reg0);
    }
}


status_t Dm9051_Loopback(LPSPI_Type *base,uint32_t pcs, phy_loop_t mode, phy_speed_t speed, bool enable)
{
    return kStatus_Fail;
}

status_t Dm9051_GetLink(LPSPI_Type *base,uint32_t pcs, bool *status)
{
    status_t result = kStatus_Success;
    uint32_t data;

    ERR_CHECK(status != null, return kStatus_Fail);
    data = Dm9051_SpiRead(base,pcs,DM9051_NSR);
    if (!(data & DM9051_NSR_LINKST)) {
        /* link down. */
        *status = false;
    } else {
        /* link up. */
        *status = true;
    }
    return result;
}

status_t Dm9051_GetSpeedDuplex(LPSPI_Type *base,uint32_t pcs, phy_speed_t *speed, phy_duplex_t *duplex)
{
    status_t result = kStatus_Success;
    uint32_t data, ctlReg;

    ERR_CHECK((speed != null) && (duplex != null), return kStatus_Fail);

    /* Read the control two register. */
    result = Dm9051_PhyRead(base, pcs, DM9051_SPECIAL_CONTROL_STATUS_REG, &ctlReg);
    if (result == kStatus_Success) {
        data = ctlReg & DM9051_SPEEDUPLX_MASK;
        if ((DM9051_10FULLDUPLEX_MASK == data) || (DM9051_100FULLDUPLEX_MASK == data)) {
            /* Full duplex. */
            *duplex = kPHY_FullDuplex;
        } else {
            /* Half duplex. */
            *duplex = kPHY_HalfDuplex;
        }

        if ((DM9051_100HALFDUPLEX_MASK == data) || (DM9051_100FULLDUPLEX_MASK == data)) {
            /* 100M speed. */
            *speed = kPHY_Speed100M;
        } else {
            /* 10M speed. */
            *speed = kPHY_Speed10M;
        }
    }

    return result;
}

status_t Dm9051_GetAutoNegotiation(LPSPI_Type *base,uint32_t pcs)
{
    uint32_t bssReg;

    if(Dm9051_PhyRead(base, pcs, DM9051_BASICSTATUS_REG, &bssReg) == kStatus_Success) {
        if((bssReg & DM9051_BSTATUS_AUTONEGCOMP_MASK) != 0) {
            return kStatus_Success;
        }
    }
    return kStatus_PHY_AutoNegotiateFail;
}




status_t Dm9051_Init(LPSPI_Type *base,uint32_t pcs,uint8_t *mac,bool broadcast)
{
    int i, oft;
    uint32_t id = 0;
    DM9051_PHY_mode mode;

    ERR_CHECK((base != null), return kStatus_Fail);

    id = (uint32_t)(Dm9051_SpiRead(base,pcs,DM9051_VIDL) |	\
                    Dm9051_SpiRead(base,pcs,DM9051_VIDH) << 8 | 	\
                    Dm9051_SpiRead(base,pcs,DM9051_PIDL) << 16 | 	\
                    Dm9051_SpiRead(base,pcs,DM9051_PIDH) << 24);

    if(id != 0x90510a46) {
        printf("\r\nDM9051 id error: 0x%x \r\n", id);
		return kStatus_Fail;
    }
    /* Setting Auto Mode */
    mode = DM9051_AUTO;

    /* RESET DM9051 device */
    Dm9051_SpiWrite(base,pcs,DM9051_NCR, DM9051_REG_RESET);
    DRIVER_DELAY(1000);
    /* Clear DM9051 Set and Disable Wakeup function */
    Dm9051_SpiWrite(base,pcs,DM9051_NCR, DM9051_NCR_DEFAULT);

    /* Set DM9051 SPI Type, This Part no standard, According to MCU SPI, Can reference DM9051 DataSheet and MCU SPI Set*/
    Dm9051_SpiWrite(base,pcs,DM9051_GPCR, DM9051_GPCR_GEP_CNTL);
    Dm9051_SpiWrite(base,pcs,DM9051_GPR, 0x00);		//Power on PHY
    DRIVER_DELAY(100);

    /* Set PHY mode */
    Dm9051_SetPhyMode(base,pcs,mode);

    /* Set mac address */
    for (i = 0, oft = DM9051_PAR; i < 6; i++, oft++) {
        Dm9051_SpiWrite(base,pcs,oft, mac[i]);
    }

    /* set multicast address */
    for(i = 0; i < 8; i++) { /* Clear Multicast set */
        /* Set Broadcast */
        Dm9051_SpiWrite(base,pcs,DM9051_MAR + i,  0x00);
    }


//		PRINTF("DM9051 MAC: ");
//	for (i = 0, oft = DM9051_PAR; i < 6; i++, oft++){
//		PRINTF("%02X:", Dm9051_SpiRead(base,pcs,oft));
//	}
//	PRINTF("\r\n");

    /************************************************
    *** Activate DM9051 and Setup DM9051 Registers **
    *************************************************/
    /* Clear TCR Register set */
    Dm9051_SpiWrite(base,pcs,DM9051_TCR, DM9051_TCR_DEFAULT);
    //LPSPI_MasterWrite(EXAMPLE_LPSPI_MASTER_BASEADDR,DM9051_TCR, 0x20); //Disable underrun retry.

    /* Discard long Packet and CRC error Packet */
    Dm9051_SpiWrite(base,pcs,DM9051_RCR, DM9051_RCR_DEFAULT);
    /*  Set 1.15 ms Jam Pattern Timer */
    Dm9051_SpiWrite(base,pcs,DM9051_BPTR, DM9051_BPTR_DEFAULT);

	/* transmit check sum */
	Dm9051_SpiWrite(base,pcs,DM9051_TCSCR, DM9051_TCSCR_DEF);

    /* Open / Close Flow Control */
    Dm9051_SpiWrite(base,pcs,DM9051_FCTR, DM9051_FCTR_DEFAULT);
    Dm9051_SpiWrite(base,pcs,DM9051_FCR,  DM9051_FCR_DEFAULT);

    /* Set Memory Conttrol Register£¬TX = 3K£¬RX = 13K */
    Dm9051_SpiWrite(base,pcs,DM9051_SMCR, DM9051_SMCR_DEFAULT);
    /* Set Send one or two command Packet*/
    Dm9051_SpiWrite(base,pcs,DM9051_TCR2, DM9051_TCR2_SET);

    //LPSPI_MasterWrite(EXAMPLE_LPSPI_MASTER_BASEADDR,DM9051_TCR2, 0x80);
    Dm9051_SpiWrite(base,pcs,DM9051_INTR, 0x0);

    /* Clear status */
    Dm9051_SpiWrite(base,pcs,DM9051_NSR, DM9051_NSR_CLR_STATUS);
    Dm9051_SpiWrite(base,pcs,DM9051_ISR, DM9051_ISR_CLR_STATUS);



    DRIVER_DELAY(500);

    Dm9051_SpiWrite(base,pcs,DM9051_IMR, DM9051_IMR_DEFAULT);    /* Enable TX/RX interrupt mask */
    Dm9051_SpiWrite(base,pcs,DM9051_RCR, (DM9051_RCR_DEFAULT | DM9051_RCR_RXEN));  /* Enable RX */

    return kStatus_Success;
}

status_t Dm9051_Send(LPSPI_Type *base,uint32_t pcs,uint8_t *data, uint16_t len)
{
	uint16_t cnt = 1000;
    ERR_CHECK((base != null) && (data != null) && (len != 0), return kStatus_Fail);

    /* Disable DM9051a interrupt */
    Dm9051_SpiWrite(base,pcs,DM9051_IMR, DM9051_IMR_OFF);

    while((Dm9051_SpiRead(base,pcs,DM9051_TCR) & DM9051_TCR_TXREQ) && cnt--) {
        DRIVER_DELAY(5);
    }

    //Write data to FIFO
    Dm9051_SpiWrite(base,pcs,DM9051_TXPLL, len & 0xff);
    Dm9051_SpiWrite(base,pcs,DM9051_TXPLH, (len >> 8) & 0xff);
    Dm9051_SpiWriteBuf(base,pcs,DM9051_MEM_WR_CONTINUOUS,data, len);
    Dm9051_SpiWrite(base,pcs,DM9051_TCR, DM9051_TCR_TXREQ); /* Cleared after TX complete */

    /* Enable DM9051a interrupt    */
    Dm9051_SpiWrite(base,pcs,DM9051_IMR, DM9051_IMR_DEFAULT);

    return kStatus_Success;
}




uint16_t Dm9051_Receive(LPSPI_Type *base,uint32_t pcs,uint8_t *data)
{
    uint8_t rxbyte;
    uint16_t rx_len = 0;

    uint16_t rx_status;
    uint8_t ReceiveData[4];

    ERR_CHECK((base != null) && (data != null), return 0);

    /* Disable DM9051a interrupt */
    Dm9051_SpiWrite(base,pcs,DM9051_IMR, DM9051_IMR_OFF);

    /* Check packet ready or not */
    rxbyte = Dm9051_SpiRead(base,pcs,DM9051_MRCMDX);
    rxbyte = Dm9051_SpiRead(base,pcs,DM9051_MRCMDX);

    if((rxbyte != 1) && (rxbyte != 0)) {
        /* Reset RX FIFO pointer */
        Dm9051_SpiWrite(base,pcs,DM9051_RCR, DM9051_RCR_DEFAULT);	//RX disable
        Dm9051_SpiWrite(base,pcs,DM9051_MPCR, 0x01);		//Reset RX FIFO pointer
        DRIVER_DELAY(100);
        Dm9051_SpiWrite(base,pcs,DM9051_RCR, (DM9051_RCR_DEFAULT | DM9051_RCR_RXEN));		//RX Enable

        /* restore receive interrupt                                                                            */
        Dm9051_SpiWrite(base,pcs,DM9051_IMR, DM9051_IMR_DEFAULT);

        return 0;
    }

    if (rxbyte) {
		
		Dm9051_SpiRead(base,pcs,DM9051_MRCMDX);		//dummy read
		
        Dm9051_SpiReadBuf(base,pcs,DM9051_MEM_RD_CONTINUOUS,ReceiveData, 4);
		
        rx_status = ReceiveData[0] + (ReceiveData[1] << 8);
        rx_len = ReceiveData[2] + (ReceiveData[3] << 8);

        if(rx_len <= DM9051_PKT_MAX) {
            Dm9051_SpiReadBuf(base,pcs,DM9051_MEM_RD_CONTINUOUS,data, rx_len);
        }

        if ((rx_status & 0xbf00) || (rx_len < 0x40) || (rx_len > DM9051_PKT_MAX) ) {

            if (rx_status & 0x8000) {
                printf("\r\nrx length error \r\n");
            }
            if (rx_len > DM9051_PKT_MAX) {
                printf("\r\nrx length too big \r\n");
            }
        }
    }
    /* restore receive interrupt */
    Dm9051_SpiWrite(base,pcs,DM9051_IMR, DM9051_IMR_DEFAULT);

    return rx_len;
}


uint8_t Dm9051_IsrGetClear(LPSPI_Type *base,uint32_t pcs)
{
    uint8_t status;

    ERR_CHECK((base != null), return 0);

    status = Dm9051_SpiRead(base,pcs,DM9051_ISR);
    Dm9051_SpiWrite(base,pcs,DM9051_ISR, status);    /* Clear ISR status  */
    return status;
}


