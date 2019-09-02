#ifndef __EXTERNAL_CTRL_H__
#define __EXTERNAL_CTRL_H__

#include "stdint.h"
#include "conference.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* �ⲿ����API���ݽṹ */
typedef struct {
	void (*launch)(void);
	void (*transmit)(EXE_DEST , ConfProtocol_S *);
	void (*transWithExData)(EXE_DEST , ConfProtocol_S *, uint16_t , uint8_t *);
} ExternalCtrl_S;


/*******************************************************************************
 * API
 ******************************************************************************/
extern ExternalCtrl_S ExternalCtrl;
#endif
