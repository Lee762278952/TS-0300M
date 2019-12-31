/**
 *
 *	@name				camera.c
 *	@author 			KT
 *	@date 				2019/12/19
 *	@brief
 *  @include			camera.h
 *
 *  @API
 *
 *  @description
 *
 **/

/*******************************************************************************
 * includes
 ******************************************************************************/
/* HAL */
#include "hal_gpio.h"

/* OS */
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"
#include "queue.h"

/* API */
#include "data_queue.h"

/* APP */
#include "external_ctrl.h"
#include "database.h"
#include "camera.h"
#include "conference.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Э��ʹ�� */
#define PROTOCOL_VISCA_ENABLE					(ENABLE)
#define PROTOCOL_Pelcd_D_ENABLE					(DISABLE)
 
 
#define CAM_485_TRANSMIT						(0x01)
#define CAM_485_RECEIVE							(0x00)

#define CAM_485_DEFAULT							CAM_485_RECEIVE


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* API */
static void Camera_Init(void);
static void Camera_CmdSend(uint8_t *cmd,uint8_t len);
static void Camera_Call(uint16_t id, UnitType_EN type);
static void Camera_Release(uint16_t id, UnitType_EN type);


/*******************************************************************************
 * Variables
 ******************************************************************************/
/* 485�շ����� */
static HAL_GpioIndex TransceiverCtrl;

/* ������ٶ��� */
static DataQueueHandler_S CameraQueue;

/* ��ǰ����ͨ�� */
static uint8_t CurrentCamChannel = 0;

/* API */
Camera_S Camera = {
	.init = Camera_Init,
	.cmdSend = Camera_CmdSend,
	.call = Camera_Call,
	.release = Camera_Release,
};


/*******************************************************************************
 * Code
 ******************************************************************************/
	/**
	* @Name 		Camera_Configure
	* @Author		KT
	* @Description
	* @para
	*
	*
	* @return
	*/
static void Camera_Init(void){
	TransceiverCtrl = HAL_GpioInit(GPIO1, 21, kGPIO_DigitalOutput, null, (gpio_interrupt_mode_t)null);
	HAL_SetGpioLevel(TransceiverCtrl, CAM_485_DEFAULT);

	CameraQueue = DataQueue.creat(20,sizeof(uint16_t));
}

/**
* @Name 		Camera_Configure
* @Author		KT
* @Description
* @para
*
*
* @return
*/
static void Camera_CmdSend(uint8_t *cmd,uint8_t len){
	ERR_CHECK(cmd != null, return);

	HAL_SetGpioLevel(TransceiverCtrl, CAM_485_TRANSMIT);
	DELAY(25);
	/* ��Ϊ������ٺ��ⲿ����ͨ��һ�����ڣ����������ٵ����ⲿ���Ʒ��ʹ��� */
	ExternalCtrl.transByByte(kType_NotiSrc_UartCtrl,cmd,len);
	
	/* ���ͱ������ʱ����ʱ������һ�ȥ485���գ���Ȼ������ȥ */
	DELAY(25);
	HAL_SetGpioLevel(TransceiverCtrl, CAM_485_DEFAULT);
}

/**
* @Name 		Camera_SetPosition
* @Author		KT
* @Description
* @para
*
*
* @return
*/
static void Camera_SetPosition(uint8_t ch, uint8_t pos){
	const uint8_t Visca[7] = {0x00,0x01,0x04,0x3F,0x02,0x00,0xFF};
	const uint8_t Pelcd_D[7] = {0xFF,0x01,0x00,0x07,0x00,0x00,0x08};
	const uint8_t ChannelSwitch[] = {0x00,0x01,0xFF,0x10,0x00,0xFB,0x00,0x01,0x0C};
	
	uint8_t *cmd;
	
	cmd = MALLOC(20);

	/* ������Ƶͨ���л�Э�� */
	if(CurrentCamChannel != ch){
		CurrentCamChannel = ch;
		
		memcpy(cmd,ChannelSwitch,sizeof(ChannelSwitch));
		cmd[7] = ch;
		cmd[8] = 0x0B + ch;
		Camera_CmdSend(cmd,sizeof(ChannelSwitch));
	}
	

#if PROTOCOL_VISCA_ENABLE
	/* ����ViscaЭ�� */
	memcpy(cmd,Visca,sizeof(Visca));
	cmd[0] = 0x80 + ch;
	cmd[5] = pos;
	Camera_CmdSend(cmd,sizeof(Visca));
#endif
	
#if PROTOCOL_Pelcd_D_ENABLE
	/* ����Pelcd_DЭ�� */
	memcpy(cmd,Pelcd_D,sizeof(Pelcd_D));
	cmd[1] = ch;
	cmd[5] = pos;
	Camera_CmdSend(cmd,sizeof(Pelcd_D));
#endif

	FREE(cmd);
}


/**
* @Name 		Camera_Configure
* @Author		KT
* @Description
* @para
*
*
* @return
*/
static void Camera_Call(uint16_t id, UnitType_EN type){

	
	UnitInfo_S *unitInfo;
	uint32_t index;
	uint16_t queueId;
	

	unitInfo = (type == tWired) ? Conference.wiredUnitInfo() : Conference.wifiUnitInfo();
	queueId = (type == tWired) ? id : id + WIFI_UNIT_START_ID;


	if(!(unitInfo[id].config->camCh >= 1 && unitInfo[id].config->camCh <= 5))
		return;

	index = DataQueue.search(CameraQueue,&queueId);

	if(index != 0){
		DataQueue.deleted(CameraQueue,index);
	}

	DataQueue.enter(CameraQueue,&queueId);

	Camera_SetPosition(unitInfo[id].config->camCh,unitInfo[id].config->camPos);


}

/**
* @Name 		Camera_Configure
* @Author		KT
* @Description
* @para
*
*
* @return
*/
static void Camera_Release(uint16_t id, UnitType_EN type){
	UnitInfo_S *unitInfo;
	uint32_t index;
	uint16_t rearId,queueId;
	

	queueId = (type == tWired) ? id : id + WIFI_UNIT_START_ID;


	index = DataQueue.search(CameraQueue,&queueId);
	DataQueue.rear(CameraQueue,&rearId);
	
	if(index == 0)
		return;


	/* ����ͷŵ��������IDΪ������ٶ���β��
	  (Ҳ���ǵ�ǰ����ͷ��׼��ID)����ô����Ҫ
	  ɾ����ID֮�󣬽���һ������β����ID��ȡ��
	  Ȼ����ø�ID��Ԥ��λ */
	
	if(queueId == rearId){
		DataQueue.deleted(CameraQueue,index);
		
		/* ɾ����Ҫ�ͷŵ�ID�������Ƿ�Ϊ�� */
		if(DataQueue.getSize(CameraQueue) > 0){
			
			/* ɾ����Ҫ�ͷŵ�ID���ٴλ�ȡ����β��ID */
			DataQueue.rear(CameraQueue,&rearId);

			if(rearId > 1 && rearId <= WIRED_UNIT_MAX_ONLINE_NUM){
				unitInfo = Conference.wiredUnitInfo();
			}
			else if(rearId> WIFI_UNIT_START_ID && rearId <= (WIFI_UNIT_START_ID + WIFI_UNIT_MAX_ONLINE_NUM)){
				unitInfo = Conference.wifiUnitInfo();
				rearId -= 0x3000;
			}
			else 
				return;
			
			Camera_SetPosition(unitInfo[rearId].config->camCh,unitInfo[rearId].config->camPos);
		}
		else{
			ConfSysInfo_S *sysInfo;

			sysInfo = Conference.getConfSysInfo();
			Camera_SetPosition(sysInfo->config->panorCh,sysInfo->config->panorPos);
		}
	}
	else{
		DataQueue.deleted(CameraQueue,index);
	}
}




