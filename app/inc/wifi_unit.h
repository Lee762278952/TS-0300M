#ifndef __WIFI_UNIT_H__
#define __WIFI_UNIT_H__

#include "stdint.h"
#include "conference.h"
#include "app.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* WIFI单元ID转换   ，WIFI单元ID从0x3000开始 */
#define WIFI_ID(_id)									(_id + WIFI_UNIT_START_ID)


/* 无线单元发送模式 */
typedef enum {
    kMode_Wifi_Unitcast,
    kMode_Wifi_Multicast,
    kMode_Wifi_UrgMulticast,
} WifiSendMode_EN;


/* 无线单元设备API数据结构 */
typedef struct {
//	void (*launch)(void);
	AppLauncher_S *launcher;

	void (*transWithExData)(WifiSendMode_EN mode,WifiUnitProtocol_S *prot, uint16_t exLen, uint8_t *exData);
	void (*transmit)(WifiSendMode_EN mode,WifiUnitProtocol_S *prot);
}WifiUnit_S;



/*******************************************************************************
 * API
 ******************************************************************************/
extern WifiUnit_S WifiUnit;
#endif
