/**
 *
 *	@name				data_queue.c
 *	@author 			KT
 *	@date 				2019/07/30
 *	@brief				
 *  @include			data_queue.h
 *
 *  @API					
 *
 *  @description   
 *
 **/
/*******************************************************************************
 * includes
 ******************************************************************************/
#include "data_queue.h"
#include "ram.h"
#include "log.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef struct {

	/* 队列项索引(index >= 1) */
	uint32_t index;
	void *next;
	void *previous;
	void *item;
}QueueItem_S;
 
typedef struct {
	
	/* 队列项大小 */
	uint32_t itemSize;
	
	/* 队列总长度 */
	uint32_t totalLen;
	
	/* 队列当前长度 */
	uint32_t qLength;
	
	/* 队列头部指针 */
	QueueItem_S *front;
	
	/* 队列尾部指针 */
	QueueItem_S *rear;
}Queue_S;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static DataQueueHandler_S *DataQueue_Creat(uint32_t queueLen,uint32_t itemSize);
static bool DataQueue_Enter(DataQueueHandler_S *handler, void *item);
static bool DataQueue_Exit(DataQueueHandler_S *handler, void *item);
static uint32_t DataQueue_GetSize(DataQueueHandler_S *handler);
static uint32_t DataQueue_Search(DataQueueHandler_S *handler,void *item);
static void DataQueue_ItemArray(DataQueueHandler_S *handler,void *array);
static bool DataQueue_Delete(DataQueueHandler_S *handler,uint32_t index);
static void DataQueue_Empty(DataQueueHandler_S *handler);
static bool DataQueue_Front(DataQueueHandler_S *handler,void *item);
static bool DataQueue_Rear(DataQueueHandler_S *handler,void *item);

/*******************************************************************************
 * Variables
 ******************************************************************************/
DataQueue_S DataQueue = {
	.creat = DataQueue_Creat,
	.enter = DataQueue_Enter,
	.exit = DataQueue_Exit,
	.getSize = DataQueue_GetSize,
	.search = DataQueue_Search,
	.toArray = DataQueue_ItemArray,
	.deleted = DataQueue_Delete,
	.empty = DataQueue_Empty,
	.front = DataQueue_Front,
	.rear = DataQueue_Rear,
};
 
 
/*******************************************************************************
 * Code
 ******************************************************************************/
 /**
* @Name  		DataQueue_Creat
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static DataQueueHandler_S *DataQueue_Creat(uint32_t queueLen,uint32_t itemSize){
	Queue_S *queue;
	
	if(itemSize == 0 || queueLen == 0)
		return null;
		
	queue = MALLOC(sizeof(Queue_S));
	
	queue->itemSize = itemSize;
	queue->totalLen = queueLen;
	queue->front = null;
	queue->rear = null;
	queue->qLength = 0;
	
	return (DataQueueHandler_S *)queue;
}

/**
* @Name  		DataQueue_Enter
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static bool DataQueue_Enter(DataQueueHandler_S *handler, void *item){
	Queue_S *queue = (Queue_S *)handler;
	QueueItem_S *qItem;
	if(queue == null || item == null || queue->qLength >= queue->totalLen)
		return false;
	
	qItem = MALLOC(sizeof(QueueItem_S));
	qItem->item = MALLOC(queue->itemSize);
	qItem->next = null;
	qItem->previous = null;
	
	memcpy(qItem->item,item,queue->itemSize);
	
	if(queue->front == null || queue->rear == null){
		qItem->index = 1;
		queue->front = qItem;
		queue->rear = qItem;
	}
	else{
		qItem->index = queue->rear->index + 1;
		qItem->previous = queue->rear;
		queue->rear->next = qItem;
		queue->rear = qItem;
	}
	
	queue->qLength += 1;
	
	return true;
}


/**
* @Name  		DataQueue_Exit
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static bool DataQueue_Exit(DataQueueHandler_S *handler, void *item){
	Queue_S *queue = (Queue_S *)handler;
	QueueItem_S *qItem;
	
	if(queue == null || item == null || queue->qLength == 0)
		return false;
		
	qItem = queue->front;
	memcpy(item,qItem->item,queue->itemSize);
	queue->qLength -= 1;
	if(queue->qLength == 0){
		queue->front = null;
		queue->rear = null;
	}
	else{
		queue->front = qItem->next;
		queue->front->previous = null;
	}
		
	FREE(qItem->item);
	FREE(qItem);
	
	return true;
}


/**
* @Name  		DataQueue_GetSize
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static uint32_t DataQueue_GetSize(DataQueueHandler_S *handler){
	Queue_S *queue = (Queue_S *)handler;
	
	if(queue == null)
		return null;
	
	return queue->qLength;
}

/**
* @Name  		DataQueue_Search
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static uint32_t DataQueue_Search(DataQueueHandler_S *handler,void *item){
	Queue_S *queue = (Queue_S *)handler;
	QueueItem_S *qItem;
	
	if(queue == null || item == null || queue->qLength == 0)
		return null;
	
	qItem = queue->front;
	do{
		if(memcmp(qItem->item,item,queue->itemSize) == 0){
			return qItem->index;
		}
		
		qItem = qItem->next;
		
		if(qItem == null)
			return 0;
	}while(true);
}

/**
* @Name  		DataQueue_ItemArray
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static void DataQueue_ItemArray(DataQueueHandler_S *handler,void *array){
	Queue_S *queue = (Queue_S *)handler;
	QueueItem_S *qItem;
	uint32_t i;
	uint8_t *temp;
	
	if(queue == null || array == null || queue->qLength == 0)
		return;
		
	temp = array;
	qItem = queue->front;
	for(i = 0;i < queue->qLength; i++){
		memcpy(temp + i * queue->itemSize, qItem->item, queue->itemSize);
		qItem = qItem->next;
	}
}


 /**
* @Name  		DataQueue_Delete
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static bool DataQueue_Delete(DataQueueHandler_S *handler,uint32_t index){
	Queue_S *queue = (Queue_S *)handler;
	QueueItem_S *qItem, *prev ,*next;
	
	if(queue == null || index == 0 || queue->qLength == 0)
		return false;
	
	qItem = queue->front;
	do{
		if(qItem->index == index){
			/* 删除目标队列项在队列中间 */
			if(qItem->next != null && qItem->previous != null){
				prev = qItem->previous;
				next = qItem->next;
				
				prev->next = next;
				next->previous = prev;
			}
			/* 删除目标队列项在队列尾部 */
			else if(qItem->next == null && qItem->previous != null){
				prev = qItem->previous;
				prev->next = null;
				queue->rear = prev;
			}
			/* 删除目标队列项在队列头部 */
			else if(qItem->next != null && qItem->previous == null){
				next = qItem->next;
				next->previous = null;
				queue->front = next;
			}
			
			FREE(qItem->item);
			FREE(qItem);
			queue->qLength -= 1;
			if(queue->qLength == 0){
				queue->front = null;
				queue->rear = null;
			}
			return true;
		}
		
		qItem = qItem->next;
		if(qItem == null)
			return false;
	}while(true);
}

 /**
* @Name  		DataQueue_Empty
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static void DataQueue_Empty(DataQueueHandler_S *handler){
	Queue_S *queue = (Queue_S *)handler;
	QueueItem_S *qItem,*next;
	
	if(queue == null || queue->qLength == 0)
		return;
	
	qItem = queue->front;
	do{
		next = qItem->next;
		FREE(qItem->item);
		FREE(qItem);
		if(next == null)
			break;
		qItem = next;
	}while(true);
	
	queue->qLength = 0;
	queue->front = null;
	queue->rear = null;
}


 /**
* @Name  		DataQueue_Front
* @Author  		KT
* @Description 	只获取队列头部项目，不进行出队列操作
* @para    		
*				
*
* @return		
*/
static bool DataQueue_Front(DataQueueHandler_S *handler,void *item){
	Queue_S *queue = (Queue_S *)handler;
	QueueItem_S *qItem;
	
	if(queue == null || item == null || queue->qLength == 0)
		return false;
		
	qItem = queue->front;
	memcpy(item,qItem->item,queue->itemSize);

	return true;
}

 /**
* @Name  		DataQueue_Front
* @Author  		KT
* @Description 	只获取队列尾部项目，不进行出队列操作
* @para    		
*				
*
* @return		
*/
static bool DataQueue_Rear(DataQueueHandler_S *handler,void *item){
	Queue_S *queue = (Queue_S *)handler;
	QueueItem_S *qItem;
	
	if(queue == null || item == null || queue->qLength == 0)
		return false;
		
	qItem = queue->rear;
	memcpy(item,qItem->item,queue->itemSize);

	return true;
}





#ifdef DATA_QUEUE_ERGODIC_DEBUG
void DataQueue_ErgodicDebug(DataQueueHandler_S *handler){
	Queue_S *queue = (Queue_S *)handler;
	QueueItem_S *qItem;
	uint32_t i,j;
	
	if(queue == null)
		return;
	
    Log.d("Queue itemSize = %d , totalLen = %d\r\n",queue->itemSize,queue->totalLen);
		
	if(!queue->qLength){
		Log.e("Queue is empty!!\r\n");
		return;
	}
	
	qItem = queue->front;
	
	for(i=0;i<queue->qLength;i++){
		uint8_t *temp;
		Log.d("item index = %d , item content : ",qItem->index);
		temp = qItem->item;
		for(j = 0;j<queue->itemSize;j++)
			printf("0x%X ",temp[j]);
		printf("\r\n");
		qItem = qItem->next;
	}
}
#endif


#if 0
/**
* @Name  		DataQueue_Creat
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static DataQueueHandler_S *DataQueue_Creat(uint32_t queueLen,uint32_t itemSize){
	QueueHandler_S *queue;
	
	if(itemSize == 0 || queueLen == 0)
		return null;
	
	queue = MALLOC(sizeof(QueueHandler_S));
	
	queue->itemSize = itemSize;
	queue->queueLen = queueLen;
	
	queue->dataStream = DataStream.creat(itemSize * queueLen);
	
	return (DataQueueHandler_S *)queue;
}

/**
* @Name  		DataQueue_Enter
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static bool DataQueue_Enter(DataQueueHandler_S *handler, void *item){
	QueueHandler_S *queue = (QueueHandler_S *)handler;;
	
	if(queue == null || item == null || queue->dataStream->emptySize == 0)
		return false;
	
	if(DataStream.write(queue->dataStream,item,queue->itemSize) == queue->itemSize)
		return true;
	else
		return false;
}


/**
* @Name  		DataQueue_Exit
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static bool DataQueue_Exit(DataQueueHandler_S *handler, void *item){
	QueueHandler_S *queue = (QueueHandler_S *)handler;;
	
	if(queue == null || item == null || queue->dataStream->filledSize == 0)
		return false;
	
	if(DataStream.read(queue->dataStream,item,queue->itemSize) == queue->itemSize)
		return true;
	else
		return false;
}


/**
* @Name  		DataQueue_GetSize
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static uint32_t DataQueue_GetSize(DataQueueHandler_S *handler){
	QueueHandler_S *queue = (QueueHandler_S *)handler;;
	
	if(queue == null)
		return null;
	
	return (DataStream.fillSize(queue->dataStream)/queue->itemSize);
}

/**
* @Name  		DataQueue_Find
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static void DataQueue_Search(DataQueueHandler_S *handler,void *item){
	QueueHandler_S *queue = (QueueHandler_S *)handler;
	void *array;
	
	if(queue == null || item == null)
		return;
		
	array = MALLOC(queue->dataStream->filledSize);
	
	DataStream.getData(queue->dataStream,array);
	
	
}

/**
* @Name  		DataQueue_ItemArray
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static void DataQueue_ItemArray(DataQueueHandler_S *handler,void *array){
	QueueHandler_S *queue = (QueueHandler_S *)handler;;
	
	if(queue == null || array == null)
		return;
		
	DataStream.getData(queue->dataStream,array);
}
#endif
