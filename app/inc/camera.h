#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "stdint.h"
#include "app.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef struct {
	AppLauncher_S *launcher;

//	void (*init)(void);
	void (*cmdSend)(uint8_t *data,uint8_t len);
	void (*call)(uint16_t id, UnitType_EN type);
	void (*release)(uint16_t id, UnitType_EN type);
}Camera_S;

/*******************************************************************************
 * API
 ******************************************************************************/
extern Camera_S Camera;

#endif


