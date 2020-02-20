#ifndef _DEBUG_H_
#define _DEBUG_H_
#include <stdio.h>
#include "fsl_debug_console.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
//#define DEBUG					1
//#define DEBUG_FILE				0
//#define DEBUG_FUNCTION			0


//#if  defined(DEBUG) && (DEBUG)

//#if  defined(DEBUG_FILE) && (DEBUG_FILE)
//#define _DBG_FILE 			PRINTF("%s\\",__FILE__)
//#else
//#define _DBG_FILE 			
//#endif

//#if  defined(DEBUG_FUNCTION) && (DEBUG_FUNCTION)
//#define _DBG_FUNCTION 		PRINTF("%s: ",__FUNCTION__)
//#else
//#define _DBG_FUNCTION 	
//#endif

//extern void debug(const char *fmt_s, ...);

typedef struct {
	void (*init)(void);
	void (*d)(const char *fmt_s, ...);
	void (*i)(const char *fmt_s, ...);
	void (*e)(const char *fmt_s, ...);
}Log_S;

/*******************************************************************************
 * API
 ******************************************************************************/
extern Log_S Log;


#endif
