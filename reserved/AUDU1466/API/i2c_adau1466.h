/*
*  File Name:      i2c_adau1466.h
*  Created on:     2019��12��03��
*  Author:         ���� 
*  description :   ��ؽṹ�塢ö�١��궨�塢��������	
*/
#include "fsl_common.h"


/* Macros for the touch touch controller. */
#define I2C_ADAU1466_BASE  LPI2C3

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_DIVIDER (5U)


#define I2C_ADAU1466_CLOCK_FREQ ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (LPI2C_CLOCK_SOURCE_DIVIDER + 1U))
#define I2C_ADAU1466_BAUDRATE 400000U //400K


#define UNIT_OPEN		0x01
#define UNIT_CLOSE	0x00

// ----------------- DSP����ö�� -----------------------------------------
typedef enum
{
	DSP_VOLUME_0_DB = 0x00,	  		//�����
	DSP_VOLUME_N1_DB,	  					//-1db
	DSP_VOLUME_N2_DB,	  					//-2db
	DSP_VOLUME_N3_DB,	  					//-3db
	DSP_VOLUME_N4_DB,	  					//-4db
	DSP_VOLUME_N5_DB,	  					//-5db
	DSP_VOLUME_N6_DB,	  					//-6db
	DSP_VOLUME_N7_DB,	  					//-7db
	DSP_VOLUME_N8_DB,	  					//-8db
	DSP_VOLUME_N9_DB,	  					//-9db
	DSP_VOLUME_N10_DB,	  				//-10db
	DSP_VOLUME_N11_DB,	  				//-11db
	DSP_VOLUME_N12_DB,	  				//-12db
	DSP_VOLUME_N13_DB,	  				//-13db
	DSP_VOLUME_N14_DB,	  				//-14db
	DSP_VOLUME_N15_DB,	  				//-15db
	DSP_VOLUME_N16_DB,	  				//-16db
	DSP_VOLUME_N17_DB,	  				//-17db
	DSP_VOLUME_N18_DB,	  				//-18db
	DSP_VOLUME_N19_DB,	  				//-19db
	DSP_VOLUME_N20_DB,	  				//-20db
	DSP_VOLUME_N21_DB,	  				//-21db
	DSP_VOLUME_N22_DB,	  				//-22db
	DSP_VOLUME_N23_DB,	  				//-23db
	DSP_VOLUME_N24_DB,	  				//-24db
	DSP_VOLUME_N25_DB,	  				//-25db
	DSP_VOLUME_N26_DB,	  				//-26db
	DSP_VOLUME_N27_DB,	  				//-27db
	DSP_VOLUME_N28_DB,	  				//-28db
	DSP_VOLUME_N29_DB,	  				//-29db
	DSP_VOLUME_N30_DB,	  				//-30db
	DSP_VOLUME_N144_DB,						//-144DB,mute

}DspVolume_E;

// ----------------- DSP mute����ö�� -----------------------------------------
typedef enum
{
	DSP_MUTE_OFF = 0x00,	  	//not mute
	DSP_MUTE_ON,	  					//mute
}DspMute_E;

// ----------------- DSP оƬѡ��ö�� -----------------------------------------
typedef enum
{
	DSP_DEVICE_1 = 0x00,	  		//dsp1
	DSP_DEVICE_2,	  					  //dsp2
}DspDevice_E;

// ----------------- DSPͬ������ö�� -----------------------------------------
typedef enum
{
	DSP_MONO_OFF = 0x00,	  	//����ģʽ
	DSP_MONO_ON,	  				//ͬ��ģʽ
}DspMono_E;

// ----------------- DSP page�л�ö�� -----------------------------------------
typedef enum
{
	DSP_PAGE_0 = 0x00,	  	//page0
	DSP_PAGE_1,	  				  //page1
}DspPage_E;

// ----------------- DSP EQȡֵö�� -----------------------------------------
typedef enum
{
	DSP_EQ_0_DB = 0x00,	  		//0db
	DSP_EQ_1_DB,	  				  //1db
	DSP_EQ_2_DB,	  				  //2db
	DSP_EQ_3_DB,	  				  //3db
	DSP_EQ_4_DB,	  				  //4db
	DSP_EQ_5_DB,	  				  //5db
	DSP_EQ_6_DB,	  				  //6db
	DSP_EQ_7_DB,	  				  //7db
	DSP_EQ_8_DB,	  				  //8db
	DSP_EQ_9_DB,	  				  //9db
	DSP_EQ_10_DB,	  				  //10db
	DSP_EQ_N1_DB,	  				  //-1db
	DSP_EQ_N2_DB,	  				  //-2db
	DSP_EQ_N3_DB,	  				  //-3db
	DSP_EQ_N4_DB,	  				  //-4db
	DSP_EQ_N5_DB,	  				  //-5db
	DSP_EQ_N6_DB,	  				  //-6db
	DSP_EQ_N7_DB,	  				  //-7db
	DSP_EQ_N8_DB,	  				  //-8db
	DSP_EQ_N9_DB,	  				  //-9db
	DSP_EQ_N10_DB,	  				//-10db
}DspEqValue_E;

// ----------------- DSP EQƵ��ö�� -----------------------------------------
typedef enum
{
	DSP_EQ_100 = 0x00,	  		//��1��EQ
	DSP_EQ_200,	  				  	//��2��EQ
	DSP_EQ_400,	  				  	//��3��EQ
	DSP_EQ_600,	  				 		//��4��EQ
	DSP_EQ_1000,	  				  //��5��EQ
	DSP_EQ_3000,	  				  //��6��EQ
	DSP_EQ_6000,	  				  //��7��EQ
	DSP_EQ_12000,	  				  //��8��EQ
	DSP_EQ_14000,	  				  //��9��EQ
	DSP_EQ_16000,	  				  //��10��EQ

}DspEqPart_E;

// ----------------- DSP ģʽö�� -----------------------------------------
typedef enum
{
	DSP_MODE_WIRE = 0x00,	  		//����ģʽ
	DSP_MODE_PARTITION,					//����ģʽ
	DSP_MODE_SI,								//ͬ��ģʽ
	DSP_MODE_WIFI,							//����ģʽ
}DspSysMode_E;

// ----------------- DSP ���ͨ��ö�� -----------------------------------------
typedef enum
{
	DSP_OUTPUT_CH1 = 0x00,	  		//CH1
	DSP_OUTPUT_CH2,								//CH2
	DSP_OUTPUT_CH3,								//CH3
	DSP_OUTPUT_CH4,								//CH4
	DSP_OUTPUT_CH5,								//CH5
	DSP_OUTPUT_CH6,								//CH6
	DSP_OUTPUT_CH7,								//CH7
	DSP_OUTPUT_CH8,								//CH8
	DSP_OUTPUT_CH9,								//CH9
	DSP_OUTPUT_CH10,							//CH10
	DSP_OUTPUT_CH11,							//CH11
	DSP_OUTPUT_CH12,							//CH12
	DSP_OUTPUT_CH13,							//CH13
	DSP_OUTPUT_CH14,							//CH14
	DSP_OUTPUT_CH15,							//CH15
	DSP_OUTPUT_CH16,							//CH16
	DSP_OUTPUT_LINE_OUT1,					//line out1���
	DSP_OUTPUT_LINE_OUT2,					//line out2���
	DSP_OUTPUT_USB_OUT,						//USB���
	DSP_OUTPUT_DANTE_OUT,					//dante ���
	DSP_OUTPUT_DOWN_WIRE,					//�´������߻�Ͳ
	DSP_OUTPUT_DOWN_WIFI,					//�´������߻�Ͳ

}DspOutput_E;

// ----------------- DSP �������������Դö�� -----------------------------------------
typedef enum
{
	DSP_OUTPUT_IN1 = 0x00,	  		//in1
	DSP_OUTPUT_IN2,								//in2
	DSP_OUTPUT_LINE_IN1,					//line in 1
	DSP_OUTPUT_LINE_IN2,					//line in 2
	DSP_OUTPUT_USB_IN,						//usb����
	DSP_OUTPUT_DANTE_IN,					//dante ����
	DSP_ALL_MIC_MIX,							//���л�Ͳ����

}DspInputSrc_E;

// ----------------- DSP ��ͲԴö�� -----------------------------------------
typedef enum
{
	DSP_UINT_WIRE_1 = 00,	  		//0x81
	DSP_UINT_WIRE_2,	  				//0x82
	DSP_UINT_WIRE_3,	  				//0x83
	DSP_UINT_WIRE_4,	  				//0x84
	DSP_UINT_WIRE_5,	  				//0x85
	DSP_UINT_WIRE_6,	  				//0x86
	DSP_UINT_WIRE_7,	  				//0x87
	DSP_UINT_WIRE_8,	  				//0x88
	
	DSP_UINT_WIFI_1,	  				//0x81
	DSP_UINT_WIFI_2,	  				//0x82
	DSP_UINT_WIFI_3,	  				//0x83
	DSP_UINT_WIFI_4,	  				//0x84
	DSP_UINT_WIFI_5,	  				//0x85
	DSP_UINT_WIFI_6,	  				//0x86

}DspUintSrc_E;

// ----------------- DSP ���������ö�� -----------------------------------------
typedef enum
{
	DSP_OUTPUT_VOLUME = 0x00,	  		//����
	DSP_OUTPUT_EQ,									//EQ
	DSP_OUTPUT_DELAY,								//DELAY

}DspOutputType_E;

// ----------------- API��Ϣ------------------------------------------------------
typedef	struct	
{
	DspSysMode_E mode;										//��ǰģʽ
	uint8_t	uchChannelSrc[16][14];				//CH1 - CH16����Դ

}DspInfoStruct_S;

extern	DspInfoStruct_S	g_unDspInfo;

void ADAU1466_Test(void);
/*
*@Function NAME��   void I2C_ADAU1466_Init()
*@description:     	Ӳ��I2C��ʼ����ADAU1466���ӣ�1061����
*@Author:           ����
*@param[out]:       ��
*@param[in]:        ��
*@return:           ��
*/
extern void I2C_ADAU1466_Init(void);

/*
*@Function NAME��   status_t ADAU1466_InputSource(DspOutput_E out,DspInputSrc_E src,DspVolume_E vol)
*@description:     	output������Դ����
*@Author:           ����
*@param[out]:       ��
*@param[in]:        DspOutput_E out������ӿ�
*@param[in]:        DspInputSrc_E src������ӿڶ�Ӧ������Դ
*@param[in]:        DspVolume_E vol������ӿڶ�Ӧ������Դ��������-30db~0db��-31:mute
*@return:           status
*/
status_t ADAU1466_InputSource(DspOutput_E out,DspInputSrc_E src,DspVolume_E vol);

/*
*@Function NAME��   status_t ADAU1466_InputSourceReset(void)
*@description:     	output������Դ���ûָ�Ĭ��
*@Author:           ����
*@param[out]:       ��
*@param[in]:        ��
*@return:           status
*/
extern status_t ADAU1466_InputSourceReset(void);

/*
*@Function NAME��   status_t ADAU1466_NormalOutputHandle(DspOutput_E out,DspOutputType_E src,DspEqPart_E part,uint8_t uchVol)
*@description:     	����output�����������
*@Author:           ����
*@param[out]:       ��
*@param[in]:        DspOutput_E out������ӿ�
*@param[in]:        DspOutputType_E src��Ҫ���õ��������
*@param[in]:        DspEqPart_E part��EQ���������ǵ�EQ��ʱ�򣬴˲�����Ч����0����
*@param[in]:        uint8_t uchVol��������͵����ֵ
*@return:           status
*/
extern status_t ADAU1466_NormalOutputHandle(DspOutput_E out,DspOutputType_E src,DspEqPart_E part,uint8_t uchVol);

/*
*@Function NAME��   status_t ADAU1466_NormalOutputReset(void)
*@description:     	����output��������ָ�Ĭ��
*@Author:           ����
*@param[out]:       ��
*@param[in]:        ��
*@return:           status
*/
extern status_t ADAU1466_NormalOutputReset(void);

/*
*@Function NAME��   status_t ADAU1466_ChannelOutputHandle(DspOutput_E out,DspOutputType_E src,DspEqPart_E part,uint8_t uchVol)
*@description:     	CH1-CH16 output�����������
*@Author:           ����
*@param[out]:       ��
*@param[in]:        DspOutput_E out������ӿ�
*@param[in]:        DspOutputType_E src��Ҫ���õ��������
*@param[in]:        DspEqPart_E part��EQ���������ǵ�EQ��ʱ�򣬴˲�����Ч����0����
*@param[in]:        uint8_t uchVol��������͵����ֵ
*@return:           status
*/
extern status_t ADAU1466_ChannelOutputHandle(DspOutput_E out,DspOutputType_E src,DspEqPart_E part,uint8_t uchVol);

/*
*@Function NAME��   status_t ADAU1466_ChannelOutputReset(void)
*@description:     	CH1-CH16 output����������ûָ�Ĭ��
*@Author:           ����
*@param[out]:       ��
*@param[in]:        ��
*@return:           status
*/
extern status_t ADAU1466_ChannelOutputReset(void);

/*
*@Function NAME��   status_t ADAU1466_ChannelInputSource(DspOutput_E out,DspInputSrc_E src,DspVolume_E vol)
*@description:     	CH1-CH16 output������Դ����
*@Author:           ����
*@param[out]:       ��
*@param[in]:        DspOutput_E out������ӿ�
*@param[in]:        DspInputSrc_E src������ӿڶ�Ӧ������Դ����Ҫ����ģʽ
*@param[in]:        DspVolume_E vol������ӿڶ�Ӧ������Դ��������-30db~0db��mute
*@return:           status
*/
extern status_t ADAU1466_ChannelInputSource(DspOutput_E out,DspInputSrc_E src,DspVolume_E vol);

/*
*@Function NAME��   status_t ADAU1466_ChannelInputSourceReset(void)
*@description:     	CH1-CH16 output������Դ���ûָ�Ĭ��
*@Author:           ����
*@param[out]:       ��
*@param[in]:        ��
*@return:           status
*/
extern status_t ADAU1466_ChannelInputSourceReset(void);

/*
*@Function NAME��   status_t ADAU1466_SetMode(DspSysMode_E mod)
*@description:     	ģʽ����
*@Author:           ����
*@param[out]:       ��
*@param[in]:        DspSysMode_E mod
*@return:           status
*/
extern status_t ADAU1466_SetMode(DspSysMode_E mod);

/*
*@Function NAME��   status_t ADAU1466_UnitCtrl(DspUintSrc_E src,DspOutput_E out,)
*@description:     	ģʽ����
*@Author:           ����
*@param[out]:       ��
*@param[in]:        DspUintSrc_E src�����ߺ����ߵ�Ԫ��ʵ����Ƶͨ����0x81~0x88
*@param[in]:        DspOutput_E out��out[0]=Ҫ������ͨ��������out[1]����һ��Ҫ������ͨ���ţ�out[2]:�ڶ���Ҫ������ͨ���ţ���������
*@param[in]:        DspVolume_E ovol��ovol[0]=Ҫ��������ܸ�����ovol[1]:��һ��ͨ����Ӧ��������ovol[2]:�ڶ���ͨ����Ӧ����������������
*@return:           status
*/
extern status_t ADAU1466_UnitCtrl(DspUintSrc_E src,uint8_t *out,uint8_t *ovol);





