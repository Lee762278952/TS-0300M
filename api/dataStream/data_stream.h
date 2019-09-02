#ifndef _DATA_STREAM_H_
#define _DATA_STREAM_H_

#include "app_common.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* �������ṹ�� */
typedef struct {

//	uint8_t blockNum;
	/* ���ݿ��������־ */
	bool rProtect;

	/* ���ݿ�д������־ */
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
	uint32_t (*write)(DataStreamHandler_S *,uint8_t *,uint32_t );
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

