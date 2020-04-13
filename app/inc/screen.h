#ifndef __SCREEN_H__
#define __SCREEN_H__

#include "protocol.h"
#include "app.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
 /* 屏幕API数据结构 */
typedef struct {
//	void (*init)(void);
//	void (*launch)(void);
	AppLauncher_S *launcher;

	void (*togglePage)(uint8_t page);
	void (*lock)(uint8_t page);
	void (*unlock)(void);
	bool (*isLock)(void);
	uint8_t (*currentPage)(void);
	void (*setVariable)(uint16_t reg,uint8_t len,uint8_t *var);
	void (*setLanguage)(uint8_t language);
	void (*transWithExData)(ScreenProtocol_S *prot, uint16_t exLen, uint8_t *exData);
	void (*transmit)(ScreenProtocol_S *prot);
	void (*backlight)(uint8_t brightness);
}Screen_S;


/*******************************************************************************
 * API
 ******************************************************************************/
extern Screen_S Screen;

#endif

