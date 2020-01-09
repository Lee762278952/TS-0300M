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

#define SEND_CMD_ONE_PARA(_cmd,_para)							do{Para[0] = _para;Wt2000_SendCommand(_cmd,Para,1);}while(0)
#define SEND_CMD_NON_PARA(_cmd)									Wt2000_SendCommand(_cmd,null,null)


#define WT2000_PL_UPLAY_DIRNAME                           		(0xA9)                      //ָ�� U���� �ļ����е������ļ�������	  
#define WT2000_PL_FINDEX                                  		(0xA4)                      //ָ��   SD���� �ļ����е�������������	 
#define WT2000_PL_FNAME                                   		(0xA5)                      //ָ��   SD���� �ļ����е������ļ�������	  	
#define WT2000_PL_PAUSE                            		       	(0xAA)                      //��ͣ����������
#define WT2000_PL_STOP                                    		(0xAB)                      //ֹͣ��������
#define WT2000_PL_NEXT                                    		(0xAC)                      //��һ������
#define WT2000_PL_PREVIOUS                                     	(0xAD)                      //��һ������
#define WT2000_PL_VOL                                     		(0xAE)                      //������������
#define WT2000_PL_MODE                                    		(0xAF)                      //ָ������ģʽ

#define WT2000_PL_FF                                      		(0xD0)                      //�������
#define WT2000_PL_FP                                      		(0xD1)                      //��������
#define WT2000_SPACE                                      		(0xD2)                      //�ⲿ�洢������ѡ��������
#define WT2000_CHANNEL                                    		(0xD3)                      //ָ����Ƶ����ͨ��������				��
#define WT2000_REC_SPEED                                  		(0xD4)                      //����¼��Ʒ������
#define	WT2000_REC_INDEX		                          		(0xD7)                      //ָ���ļ������ļ�����¼��
#define WT2000_REC_FNAME                                  		(0xD8)                      //ָ���ļ������ļ���¼������
#define WT2000_REC_STOP                                   		(0xD9)                      //ֹͣ¼������

#define	WT2000_RD_PRONUM		 								(0xC0)						//�����̱��
#define WT2000_RD_VOLUME     									(0xC1)						//��ȡ����
#define WT2000_RD_MODE      									(0xC2)						//��ȡ��ǰ����״̬
#define WT2000_RD_FQTY       									(0xC6)						//��ȡָ���ļ��е������ļ����� SD��
#define WT2000_RD_FQTY_U		 								(0xC8)						//��ȡָ���ļ��е������ļ����� U��
#define WT2000_RD_FLASH_ST   									(0xCA)						//����ǰ�洢������״̬
#define WT2000_RD_FREE_SPACE 									(0xCE)						//��ȡ�洢���ʣ��ռ�     	(Ҫ�Ⱥܾò��лظ�)



typedef enum{
	r128KBPS=0,
	r96KBPS,
	r64KBPS,
	r32KBPS,
}MP3BitRate_EN;


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void Wt2000_SendCommand(uint8_t cmd,const uint8_t *para,uint8_t paraLen);
/*******************************************************************************
 * Variables
 ******************************************************************************/
static Wt2000_AckCallback AckCallback = null;
static uint8_t CmdBuf[64] = {0};
static uint8_t Para[30];

/*******************************************************************************
 * Code
 ******************************************************************************/

void Wt2000_CfgReturnNecessary(void){
	SEND_CMD_ONE_PARA(0xBA,0x01);
}

void Wt2000_CfgUsbDisk(void){
	SEND_CMD_ONE_PARA(0xD2,0x01);
}

void Wt2000_CfgMp3Rate(void){
	SEND_CMD_ONE_PARA(WT2000_REC_SPEED,r128KBPS);
}

void Wt2000_CfgDefVolume(void){
	SEND_CMD_ONE_PARA(WT2000_PL_VOL,0x1F);
}

//void Wt2000_CfgPlayMode(WtPlaySta_EN mode){
//	if(mode >= 0 && mode <= 4)
//		SEND_CMD_ONE_PARA(WT2000_PL_MODE,mode);
//}


void Wt2000_QueryState(WtCmdType_EN type){
	switch(type){
	case kType_WtCmd_Volume:
	case kType_WtCmd_State:
	case kType_WtCmd_FileNum:
	case kType_WtCmd_DirFileNum:	
	case kType_WtCmd_CurrPlayFile:
	case kType_WtCmd_LinkSta:
	case kType_WtCmd_Space:
		SEND_CMD_NON_PARA(type);
	break;
	default:break;
	}
}



void Wt2000_AudioRecord(const char *dir,  const char *file){
	uint8_t dLen,fLen,data[30] = {0};

	ERR_CHECK(dir != null && file != null, return);

	dLen = strlen(dir);
	fLen = strlen(file);

	ERR_CHECK(dLen != 0 && fLen != 0, return);

	/* �ļ������̶� 5 �ַ����ļ������ 22 ���ַ� */
	memcpy(&data[0],dir,dLen > 5 ? 5 : dLen);
	memcpy(&data[5],file,fLen > 22 ? 22 : fLen);

	Wt2000_SendCommand(WT2000_REC_FNAME,data,strlen((const char *)data));
}

void Wt2000_RecordStop(void){
	SEND_CMD_NON_PARA(WT2000_REC_STOP);
}

void Wt2000_AudioPlay(const char *dir, const char *file){
	uint8_t dLen,fLen,data[30] = {0};

	ERR_CHECK(dir != null && file != null, return);

	dLen = strlen(dir);
	fLen = strlen(file);

	ERR_CHECK(dLen != 0 && fLen != 0, return);

	/* �ļ������̶� 5 �ַ����ļ�������8���ַ�ʱ��ȡǰ6���ַ�����"~1" */
	memcpy(&data[0],dir,dLen > 5 ? 5 : dLen);
	if(fLen > 8){
		memcpy(&data[5],file,6);
		sprintf((char *)&data[11],"~1");
	}
	else{
		memcpy(&data[5],file,fLen);
	}

	Wt2000_SendCommand(WT2000_PL_UPLAY_DIRNAME,data,strlen((const char *)data));
	
}

void Wt2000_AudioPlayPause(void){
	SEND_CMD_NON_PARA(WT2000_PL_PAUSE);
}


void Wt2000_PlayNext(void){
	SEND_CMD_NON_PARA(WT2000_PL_NEXT);
}

void Wt2000_PlayPrevious(void){
	SEND_CMD_NON_PARA(WT2000_PL_PREVIOUS);
}


void Wt2000_PlayStop(void){
	SEND_CMD_NON_PARA(WT2000_PL_STOP);
}

void Wt2000_AckRecvEnable(bool enable){
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

static void Wt2000_SendCommand(uint8_t cmd,const uint8_t *para,uint8_t paraLen)
{
	uint8_t checksum = 0,len,i;
	
	ERR_CHECK(!(paraLen != 0 && para == null), return);
	
	/* ����Ϊ"1byte len + 1byte cmd + nbyte paraLen + 1byte checksum" */
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



