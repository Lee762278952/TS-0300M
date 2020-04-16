/**
 *
 *	@name					dataStream.c
 *	@author 			KT
 *	@date 				2019/05/07
 *	@brief				
 *  @include			data_stream.h
 *
 *  @API					
 *
 *  @description   
 *
 **/
/*******************************************************************************
 * includes
 ******************************************************************************/
#include "data_stream.h"
#include "ram.h"
#include "string.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static DataStreamHandler_S *DataStream_Creat(uint32_t Size);
static void DataStream_Dismiss(DataStreamHandler_S *streamHandler);
static uint32_t DataStream_Write(DataStreamHandler_S *streamHandler,const uint8_t *data,uint32_t size);
static uint32_t DataStream_Read(DataStreamHandler_S *streamHandler,uint8_t *data,uint32_t size);
static uint32_t DataStream_GetFillSize(DataStreamHandler_S *streamHandler);
static uint32_t DataStream_GetData(DataStreamHandler_S *streamHandler,uint8_t *data);
/*******************************************************************************
 * Variables
 ******************************************************************************/
 
/* API */
DataStream_S DataStream = {
	.creat = DataStream_Creat,
	.dismiss = DataStream_Dismiss,
	.write = DataStream_Write,
	.read = DataStream_Read,
	.fillSize = DataStream_GetFillSize,
	.getData = DataStream_GetData,
};
/*******************************************************************************
 * Code
 ******************************************************************************/
/**
* @Name  		DataStream_Creat
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static DataStreamHandler_S *DataStream_Creat(uint32_t Size){
	DataStreamHandler_S *streamHandler;
	
	/* 分配数据流对象内存 */
	streamHandler = MALLOC(sizeof(DataStreamHandler_S));
	if(streamHandler == NULL)	return NULL;

	/* 分配数据流的数据内存 */
	streamHandler->data = MALLOC(Size);
	if(streamHandler->data == NULL){
		FREE(streamHandler);
		return NULL;
	}
	
	/* 初始化数据流对象变量 */
	streamHandler->rProtect = false;
	streamHandler->wProtect = false;
	streamHandler->head = streamHandler->data;
	streamHandler->end = streamHandler->data;
	streamHandler->totalSize = Size;
	streamHandler->filledSize = 0;
	streamHandler->emptySize = Size;
	
	return streamHandler;
}

/**
* @Name  		DataStream_Dismiss
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static void DataStream_Dismiss(DataStreamHandler_S *streamHandler){
	if(streamHandler == NULL)
		return;
		
		if(streamHandler->data != NULL)
			FREE(streamHandler->data);
			
		FREE(streamHandler);
}

/**
* @Name  		DataStream_Write
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static uint32_t cntAbaondon;
static uint32_t DataStream_Write(DataStreamHandler_S *streamHandler,const uint8_t *data,uint32_t size){
	
	/* 实际写入大小*/
	uint32_t realSize = 0;
	/* 分段写入长度 */
	uint32_t secLen[2] = {0};

	if(streamHandler == NULL || data == NULL || streamHandler->wProtect == true || size == 0)
		return realSize;
		
	/* 写保护 */
	streamHandler->wProtect = true;
	
	
	/* 判断写入长度是否大于未写入数据空间的大小 */
	realSize = (size > streamHandler->emptySize) ? streamHandler->emptySize : size;

	if(realSize == 0){
		cntAbaondon++;
  	}else{
  		if(cntAbaondon != 0){
			printf("!!!Stream is full and abandon data %d times !!!\r\n",cntAbaondon);
			cntAbaondon = 0;
		}
	}
	
	/* 判断写入数据长度是否超过或等于数据空间尾部，超过从头写入 */
	if( (streamHandler->end + realSize) >= (streamHandler->data + streamHandler->totalSize)){
		secLen[0] = streamHandler->data + streamHandler->totalSize - streamHandler->end;
		secLen[1] = realSize - secLen[0];
		
		memcpy(streamHandler->end,data,secLen[0]);
		
		if(secLen[1] > 0)
			memcpy(streamHandler->data,data+secLen[0],secLen[1]);
		
		streamHandler->end = streamHandler->data + secLen[1];
	}
	
	/* 否则直接写入 */
	else{
		memcpy(streamHandler->end,data,realSize);
		streamHandler->end += realSize;
	}
		
	streamHandler->emptySize -= realSize;
	streamHandler->filledSize += realSize;
	
	
	streamHandler->wProtect = false;
	
	return realSize;
}

/**
* @Name  		DataStream_Read
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static uint32_t DataStream_Read(DataStreamHandler_S *streamHandler,uint8_t *data,uint32_t size){
	
	/* 实际读取大小*/
	uint32_t realSize = 0;
	
	/* 分段读取长度 */
	uint32_t secLen[2] = {0};

	if(streamHandler == NULL || data == NULL || streamHandler->rProtect == true || size == 0)
		return realSize;
	
	/* 读保护 */
	streamHandler->rProtect = true;
	
	/* 判断读取长度是否大于已写入数据的大小 */
	realSize = (size > streamHandler->filledSize) ? streamHandler->filledSize : size;
	
	/* 判断写入数据长度是否超过或等于数据空间尾部，超过从头写入 */
	if(streamHandler->head + realSize > streamHandler->data + streamHandler->totalSize){
		secLen[0] = streamHandler->data + streamHandler->totalSize - streamHandler->head;
		secLen[1] = realSize - secLen[0];
		
		memcpy(data,streamHandler->head,secLen[0]);
		
		if(secLen[1] > 0)
			memcpy(data+secLen[0],streamHandler->data,secLen[1]);
		
		streamHandler->head = streamHandler->data + secLen[1];
	}
	/* 否则直接读取 */
	else{
		memcpy(data,streamHandler->head,realSize);
		streamHandler->head += realSize;
	}
	
	streamHandler->emptySize += realSize;
	streamHandler->filledSize -= realSize;
	
	streamHandler->rProtect = false;
	
	return realSize;
}

/**
* @Name  		DataStream_GetSize
* @Author  		KT
* @Description 	
* @para    		
*				
*
* @return		
*/
static uint32_t DataStream_GetFillSize(DataStreamHandler_S *streamHandler){
	return streamHandler->filledSize;
}

/**
* @Name  		DataStream_GetData
* @Author  		KT
* @Description 	读取数据流内全部数据（但不改变指针位置和数据空间）
* @para    		
*				
*
* @return		
*/
static uint32_t DataStream_GetData(DataStreamHandler_S *streamHandler,uint8_t *data){
	/* 分段读取长度 */
	uint32_t secLen[2] = {0};

	if(streamHandler == null || data == null)
		return null;
	
	/* 如果已写入数据长度为‘0’，直接返回 */
	if(streamHandler->filledSize == 0)
		return streamHandler->filledSize;
	
	/* 判断写入数据长度是否超过或等于数据空间尾部，超过从头写入 */
	if(streamHandler->head + streamHandler->filledSize > streamHandler->data + streamHandler->totalSize){
		secLen[0] = streamHandler->data + streamHandler->totalSize - streamHandler->head;
		secLen[1] = streamHandler->filledSize - secLen[0];
		
		memcpy(data,streamHandler->head,secLen[0]);
		
		if(secLen[1] > 0)
			memcpy(data+secLen[0],streamHandler->data,secLen[1]);
	}
	/* 否则直接读取 */
	else{
		memcpy(data,streamHandler->head,streamHandler->filledSize);
	}
	
	return streamHandler->filledSize;
}
