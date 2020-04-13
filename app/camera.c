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
#include "ram.h"

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
#define PROTOCOL_PELCD_D_ENABLE					(DISABLE)

/* ������ٿ��ƴ��ڷ����̶߳�ջ��С�����ȼ� */
#define CAMERA_SEND_TASK_STACK					(256)
#define CAMERA_SEND_TASK_PRIORITY				(5)

 
#define CAM_485_TRANSMIT						(0x01)
#define CAM_485_RECEIVE							(0x00)

#define CAM_485_DEFAULT							CAM_485_RECEIVE

typedef struct{
	uint8_t len;
	uint8_t *data;
}Cmd_S;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* LAUNCHER */
static void Camera_UartTransmitTask(void *pvParameters);
static void Camera_Init(void);


/* API */
static void Camera_Init(void);
static void Camera_CmdSend(uint8_t *data,uint8_t len);
static void Camera_Call(uint16_t id, UnitType_EN type);
static void Camera_Release(uint16_t id, UnitType_EN type);


/*******************************************************************************
 * Variables
 ******************************************************************************/
/* 485�շ����� */
static HAL_GpioHandler TransceiverCtrl;

/* ������ٶ��� */
static DataQueueHandler_S CameraQueue;

/* ���ڷ��Ͷ��� */
static QueueHandle_t UartQueue;

/* ��ǰ����ͨ�� */
static uint8_t CurrentCamChannel = 0;



/*******************************************************************************
 * Task & API
 ******************************************************************************/

static AppTask_S UartSend = {
	.task = Camera_UartTransmitTask,
	.name = "Camera.UartSend",	
	.stack = CAMERA_SEND_TASK_STACK,
	.para = null,
	.prio = CAMERA_SEND_TASK_PRIORITY,
	.handle = null
};


static AppTask_S *FuncTask[] = {&UartSend};


static AppLauncher_S Launcher = {
	.init = Camera_Init,
	.configTask = null,
	.funcNum  = 1,
	.funcTask = FuncTask,
};


/* API */
Camera_S Camera = {
	.launcher = &Launcher,

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
	TransceiverCtrl = HAL_GpioInit(GPIO1, 21, kGPIO_DigitalOutput, CAM_485_DEFAULT, (gpio_interrupt_mode_t)null);
	CameraQueue = DataQueue.creat(20,sizeof(uint16_t));
	/* ���ڷ��Ͷ��г�ʼ�� */
    UartQueue = xQueueCreate(20,sizeof(void *));
}

/**
* @Name 		Camera_UartSendTask
* @Author		KT
* @Description
* @para
*
*
* @return
*/
static void Camera_UartTransmitTask(void *pvParameters){
	Cmd_S *cmd;

	Log.i("Camera uart transmit task start!!\r\n");

	while(1){
		xQueueReceive(UartQueue, &cmd, MAX_NUM);

		HAL_SetGpioLevel(TransceiverCtrl, CAM_485_TRANSMIT);
		DELAY(25);
		/* ��Ϊ������ٺ��ⲿ����ͨ��һ�����ڣ����������ٵ����ⲿ���Ʒ��ʹ��� */
		ExternalCtrl.transByByte(kType_NotiSrc_UartCtrl,cmd->data,cmd->len);
#if 1
		Log.d("Camera transmit: \r\n");
			for(uint8_t i = 0;i < cmd->len;i++){
				printf("%02X ",cmd->data[i]);
			}
			printf("\r\n");
#endif
		
		/* ���ͱ������ʱ����ʱ������һ�ȥ485���գ���Ȼ������ȥ */
		DELAY(25);
		HAL_SetGpioLevel(TransceiverCtrl, CAM_485_DEFAULT);

		FREE(cmd->data);
		FREE(cmd);
	}
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
static void Camera_CmdSend(uint8_t *data,uint8_t len){
	ERR_CHECK(data != null, return);

	Cmd_S *cmd;

	cmd = MALLOC(sizeof(Cmd_S));
	cmd->data = MALLOC(len);
	cmd->len = len;
	memcpy(cmd->data,data,len);
	xQueueSend(UartQueue, &cmd, MAX_NUM);
	
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
	
#if PROTOCOL_VISCA_ENABLE
	const uint8_t Visca[7] = {0x00,0x01,0x04,0x3F,0x02,0x00,0xFF};
#endif
	
#if PROTOCOL_PELCD_D_ENABLE
	const uint8_t Pelcd_D[7] = {0xFF,0x01,0x00,0x07,0x00,0x00,0x08};
#endif
	const uint8_t ChannelSwitch[] = {0x00,0x01,0xFF,0x10,0x00,0xFB,0x00,0x01,0x0C};
	
	uint8_t data[20],len;
	

	/* ������Ƶͨ���л�Э�� */
	if(CurrentCamChannel != ch){
		CurrentCamChannel = ch;
		len = sizeof(ChannelSwitch);
		
		memcpy(data,ChannelSwitch,len);
		data[7] = ch;
		data[8] = 0x0B + ch;
		Camera_CmdSend(data,len);
	}
	

#if PROTOCOL_VISCA_ENABLE
	/* ����ViscaЭ�� */
	len = sizeof(Visca);
	memcpy(data,Visca,len);
	data[0] = 0x80 + ch;
	data[5] = pos;
	Camera_CmdSend(data,len);
#endif
	
#if PROTOCOL_PELCD_D_ENABLE
	/* ����Pelcd_DЭ�� */
	len = sizeof(Pelcd_D);
	memcpy(data,Pelcd_D,len);
	data[1] = ch;
	data[5] = pos;
	Camera_CmdSend(data,len);
#endif

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
	

	unitInfo = (type == kType_Unit_Wired) ? Conference.wiredUnitInfo() : Conference.wifiUnitInfo();
	queueId = (type == kType_Unit_Wired) ? id : id + WIFI_UNIT_START_ID;


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
	

	queueId = (type == kType_Unit_Wired) ? id : id + WIFI_UNIT_START_ID;


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

			if(rearId > 0 && rearId <= WIRED_UNIT_MAX_ONLINE_NUM){
				unitInfo = Conference.wiredUnitInfo();
			}
			else if(rearId > WIFI_UNIT_START_ID && rearId <= (WIFI_UNIT_START_ID + WIFI_UNIT_MAX_ONLINE_NUM)){
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





