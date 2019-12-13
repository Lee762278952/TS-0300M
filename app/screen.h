#ifndef __SCREEN_H__
#define __SCREEN_H__

#include "protocol.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
 /* ���ߵ�Ԫ�豸API���ݽṹ */
typedef struct {
	void (*launch)(void);
	void (*transWithExData)(ScreenProtocol_S *prot, uint16_t exLen, uint8_t *exData);
	void (*transmit)(ScreenProtocol_S *prot);
}Screen_S;


/*******************************************************************************
 * API
 ******************************************************************************/
extern Screen_S Screen;

#endif
