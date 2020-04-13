#ifndef __EXTERNAL_CTRL_H__
#define __EXTERNAL_CTRL_H__

#include "stdint.h"
#include "conference.h"
#include "app.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* 外部控制API数据结构 */
typedef struct {
//	void (*launch)(void);
	AppLauncher_S *launcher;

	void (*transmit)(EXE_DEST , ConfProtocol_S *prot);
	void (*transWithExData)(EXE_DEST dest, ConfProtocol_S *prot, uint16_t exLen, uint8_t *exData);
	void (*transByByte)(EXE_DEST dest, uint8_t *data, uint16_t len);
	bool (*connectSta)(EXE_DEST dest);
} ExternalCtrl_S;


/*******************************************************************************
 * API
 ******************************************************************************/
extern ExternalCtrl_S ExternalCtrl;
#endif
