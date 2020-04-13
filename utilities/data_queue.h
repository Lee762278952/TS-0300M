#ifndef _DATA_QUEUE_H_
#define _DATA_QUEUE_H_

#include "global_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
//#define DATA_QUEUE_ERGODIC_DEBUG	1

typedef void * DataQueueHandler_S;

typedef struct {
    DataQueueHandler_S *(*creat)(uint32_t queueLen,uint32_t itemSize);
	bool (*enter)(DataQueueHandler_S *handler, void *item);
	bool (*exit)(DataQueueHandler_S *handler, void *item);
	uint32_t (*getSize)(DataQueueHandler_S *handler);
	uint32_t (*search)(DataQueueHandler_S *handler,void *item);
	void (*toArray)(DataQueueHandler_S *handler,void *array);
	bool (*deleted)(DataQueueHandler_S *handler,uint32_t index);
	void (*empty)(DataQueueHandler_S *handler);
	bool (*front)(DataQueueHandler_S *handler,void *item);
	bool (*rear)(DataQueueHandler_S *handler,void *item);
} DataQueue_S;



/*******************************************************************************
 * API
 ******************************************************************************/
extern DataQueue_S DataQueue;

#ifdef DATA_QUEUE_ERGODIC_DEBUG
extern void DataQueue_ErgodicDebug(DataQueueHandler_S *handler);
#endif

#endif

