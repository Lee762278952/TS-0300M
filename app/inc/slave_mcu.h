#ifndef _SLAVE_MCU_H__
#define _SLAVE_MCU_H__

#include "protocol.h"
#include "app.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
 /* 从单片机API数据结构 */
typedef struct {
//	void (*init)(void);
//	void (*launch)(void);
	AppLauncher_S *launcher;

	void (*transWithExData)(ConfProtocol_S *prot, uint16_t exLen, uint8_t *exData);
	void (*transmit)(ConfProtocol_S *prot);
}SlaveMcu_S;


/*******************************************************************************
 * API
 ******************************************************************************/
extern SlaveMcu_S SlaveMcu;

#endif


