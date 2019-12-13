#ifndef _HAL_H_
#define _HAL_H_

#include "global_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* �����ֵ */
#ifndef	MAX_NUM
#define MAX_NUM  										(0xffffffffUL)	
#endif

/* nullֵ */
#ifndef	null
#define null 											(0U)
#endif

/* ���������ж� */
#ifndef	ERR_CHECK
#define ERR_CHECK(condition, implement) 				do { if (!(condition)) {implement;}} while(0)
#endif

/* ��debug���������ж� */
#ifndef	ERR_CHECK_DBG
#define ERR_CHECK_DBG(condition, dbg, implement) 		do { if (!(condition)) {debug(dbg); implement;}} while(0)
#endif


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void HAL_Init(void);



#endif

