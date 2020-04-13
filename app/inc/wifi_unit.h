#ifndef __WIFI_UNIT_H__
#define __WIFI_UNIT_H__

#include "stdint.h"
#include "conference.h"
#include "app.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* ���ߵ�Ԫ����ģʽ */
typedef enum {
    kMode_Wifi_Unitcast,
    kMode_Wifi_Multicast,
} WifiSendMode_EN;


/* ���ߵ�Ԫ�豸API���ݽṹ */
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
