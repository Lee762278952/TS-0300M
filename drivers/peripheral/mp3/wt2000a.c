/**
 *
 *	@name				wt2000a.c
 *	@modify 			KT
 *	@date 				2019/11/21
 *	@brief				
 *
 *  @API				
 *
 *  @description   		
 *
 **/


#include "mp3/wt2000a.h"
#include "hal_uart.h"
#include "drivers_common.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define WT2000_UART_PORT								   		(LPUART6)
#define WT2000_UART_IRQ											(LPUART6_IRQn)
#define WT2000_UART_IRQ_HANDLER									LPUART6_IRQHandler

#define CMD_BUF_HEAD									   		(0x7E)
#define CMD_BUF_END										   		(0xEF)


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void Wt2000_UartSend(uint8_t cmd,const uint8_t *para,uint8_t paraLen);
/*******************************************************************************
 * Variables
 ******************************************************************************/
static Wt2000_AckCallback AckCallback = null;
static uint8_t CmdBuf[64] = {0};

/*******************************************************************************
 * Code
 ******************************************************************************/

void Wt2000_SetRecvEnable(bool enable){
	LPUART_EnableRx(WT2000_UART_PORT, enable);

	if(enable)
		EnableIRQ(WT2000_UART_IRQ);
	else
		DisableIRQ(WT2000_UART_IRQ);
}


status_t Wt2000_SetAckCallback(Wt2000_AckCallback callback){
	ERR_CHECK(callback != null, return kStatus_Fail);
	
	AckCallback = callback;

	return kStatus_Success;
}

status_t Wt2000_SendCommand(WtCmdType_EN cmd,const char *dir,const char *file,uint8_t *para,uint8_t pLen){
	uint8_t dLen,fLen,data[32] = {0};

	switch(cmd){
		/* 按文件名录音字段处理 */
		case kType_WtCmd_RecStart:{
			ERR_CHECK(dir != null && file != null, return kStatus_Fail);

			dLen = strlen(dir);
			fLen = strlen(file);

			ERR_CHECK(dLen != 0 && fLen != 0, return kStatus_Fail);

			/* 文件夹名固定 5 字符，文件名最多 22 个字符 */
			memcpy(&data[0],dir,dLen > 5 ? 5 : dLen);
			memcpy(&data[5],file,fLen > 22 ? 22 : fLen);

			Wt2000_UartSend(kType_WtCmd_RecStart,data,strlen((const char *)data));
		}break;
		
		/* 按文件名播音字段处理 */
		case kType_WtCmd_PlayFile:{
			ERR_CHECK(dir != null && file != null, return kStatus_Fail);

			dLen = strlen(dir);
			fLen = strlen(file);

			ERR_CHECK(dLen != 0 && fLen != 0, return kStatus_Fail);

			/* 文件夹名固定 5 字符，文件名大于8个字符时，取前6个字符加上"~1" */
			memcpy(&data[0],dir,dLen > 5 ? 5 : dLen);
			if(fLen > 8){
				memcpy(&data[5],file,6);
				sprintf((char *)&data[11],"~1");
			}
			else{
				memcpy(&data[5],file,fLen);
			}

			Wt2000_UartSend(kType_WtCmd_PlayFile,data,strlen((const char *)data));
		}break;

		/* 查询字段 */
		case kType_WtCmd_Volume:
		case kType_WtCmd_State:
		case kType_WtCmd_FileNum:
		case kType_WtCmd_DirFileNum:	
		case kType_WtCmd_CurrPlayFile:
		case kType_WtCmd_LinkSta:
		case kType_WtCmd_Space:
		/* 操作字段 */
		case kType_WtCmd_PlayOrPause:
		case kType_WtCmd_Stop:
		case kType_WtCmd_Next:
		case kType_WtCmd_Previous:
		case kType_WtCmd_RecStop:
			Wt2000_UartSend(cmd,null,null);
		break;
		case kType_WtCmd_PlayDirIndex:{
			ERR_CHECK(data != null,return kStatus_Fail);
			
			Wt2000_UartSend(cmd,para,pLen);		
		}
		break;
		
		default:
			break;
		
	}

	return kStatus_Success;
}

status_t Wt2000_SendConfig(WtCmdType_EN cmd,uint8_t cfg){

	switch(cmd){
		case kType_WtCmd_Disk:
		case kType_WtCmd_NeedReturn:
		case kType_WtCmd_RecordRate:
		case kType_WtCmd_SetVolume:
		case kType_WtCmd_SetPlayMode:
			Wt2000_UartSend(cmd,&cfg,1);
			return kStatus_Success;
		default:
			return kStatus_Fail;
	}
}

static void Wt2000_UartSend(uint8_t cmd,const uint8_t *para,uint8_t paraLen)
{
	uint8_t checksum = 0,len,i;
	
	ERR_CHECK(!(paraLen != 0 && para == null), return);
	
	/* 长度为"1byte len + 1byte cmd + nbyte paraLen + 1byte checksum" */
	len = 3 + paraLen;
	
	CmdBuf[0] = CMD_BUF_HEAD;
	
	CmdBuf[1] = len;
	CmdBuf[2] = cmd;
	
	if(para != null)
		memcpy(&CmdBuf[3],para,paraLen);

	for(i = 1;i < (3+paraLen);i++)
		checksum += CmdBuf[i];
	
	CmdBuf[3 + paraLen] = checksum;
	CmdBuf[4 + paraLen] = CMD_BUF_END;

	LPUART_WriteBlocking(WT2000_UART_PORT,CmdBuf,len + 2);

#if 0 
	printf("WT2k send : ");
	for(i = 0;i < len + 2;i++)
		printf("%X ",CmdBuf[i]);
	printf("\r\n");
#endif
}



#ifdef UART6_USE_IN_DRIVER
void WT2000_UART_IRQ_HANDLER(void)
{
	LPUART_Type *base = LPUART6;
	uint8_t data;
	uint32_t status = LPUART_GetStatusFlags(base);

    /* If new data arrived. */
    if (kLPUART_RxDataRegFullFlag & status)
    {
    	data = LPUART_ReadByte(base);
		
		if(AckCallback != null)
			AckCallback(data);
    }

	    /* If RX overrun. */
    if (kLPUART_RxOverrunFlag & status)
    {
        /* Clear overrun flag, otherwise the RX does not work. */
        base->STAT = ((base->STAT & 0x3FE00000U) | LPUART_STAT_OR_MASK);
    }
}
#endif



