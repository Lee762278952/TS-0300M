#ifndef _DEBUG_H_
#define _DEBUG_H_
#include <stdio.h>
#include "fsl_debug_console.h"
#include "app.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef struct {
	AppLauncher_S *launcher;

	void (*d)(const char *fmt_s, ...);
	void (*i)(const char *fmt_s, ...);
	void (*e)(const char *fmt_s, ...);
}Log_S;

/*******************************************************************************
 * API
 ******************************************************************************/
extern Log_S Log;


#endif
