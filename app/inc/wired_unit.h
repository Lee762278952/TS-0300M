#ifndef __WIRED_UNIT_H__
#define __WIRED_UNIT_H__

#include "stdint.h"
#include "conference.h"
#include "app.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* 有线单元设备API数据结构 */
typedef struct {
//	void (*launch)(void);
	AppLauncher_S *launcher;

	void (*transWithExData)(ConfProtocol_S *, uint16_t , uint8_t *);
	void (*transmit)(ConfProtocol_S *);
} WiredUnit_S;



/*******************************************************************************
 * API
 ******************************************************************************/
extern WiredUnit_S WiredUnit;
#endif
