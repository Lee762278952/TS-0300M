#ifndef _DATA_STREAM_H_
#define _DATA_STREAM_H_

#include "global_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* 数据流结构体 */
typedef struct {

//	uint8_t blockNum;
	/* 数据块读保护标志 */
	bool rProtect;

	/* 数据块写保护标志 */
	bool wProtect;
	
	uint8_t *head;
	
	uint8_t *end;
	
	uint8_t *data;
	
	uint32_t totalSize;
	
	uint32_t filledSize;
	
	uint32_t emptySize;
	
}DataStreamHandler_S;


typedef struct {
    DataStreamHandler_S *(*creat)(uint32_t );
	void (*dismiss)(DataStreamHandler_S *);
	uint32_t (*write)(DataStreamHandler_S *,const uint8_t *,uint32_t );
	uint32_t (*read)(DataStreamHandler_S *,uint8_t *,uint32_t );
	uint32_t (*fillSize)(DataStreamHandler_S *);
	uint32_t (*getData)(DataStreamHandler_S *,uint8_t *);
} DataStream_S;

/*******************************************************************************
 * API
 ******************************************************************************/
//extern DataStreamHandler_S *creatDataStream(uint32_t Size);
//extern uint32_t writeDataStream(DataStreamHandler_S *dataStreamHandler,uint8_t *data,uint32_t size);
extern DataStream_S DataStream;
#endif

