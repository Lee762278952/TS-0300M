#ifndef _RAMMANAGE_H_
#define _RAMMANAGE_H_

#include "stdint.h"
#include "ff.h"
#include "global_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MALLOC(size)									Ram.malloc(size,MAX_NUM,__FUNCTION__)
#define FREE(p)											Ram.free((void **)(&p))
 
/* API struct */
typedef struct {
	void (*init)(void);
	void *(*malloc)(size_t ,uint32_t ,const char *);
	void (*free)(void **);
}Ram_S;
/*******************************************************************************
 * API
 ******************************************************************************/
extern Ram_S Ram;

#endif

