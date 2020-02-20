#ifndef __SCREEN_H__
#define __SCREEN_H__

#include "protocol.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
 /* 屏幕API数据结构 */
typedef struct {
	void (*init)(void);
	void (*launch)(void);
	void (*transWithExData)(ScreenProtocol_S *prot, uint16_t exLen, uint8_t *exData);
	void (*transmit)(ScreenProtocol_S *prot);
	void (*backlight)(uint8_t brightness);
}Screen_S;


/*******************************************************************************
 * API
 ******************************************************************************/
extern Screen_S Screen;

#endif

