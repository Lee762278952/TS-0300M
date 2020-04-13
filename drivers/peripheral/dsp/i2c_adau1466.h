/*
*  File Name:      i2c_adau1466.h
*  Created on:     2019年12月03日
*  Author:         陈泽芳 
*  description :   相关结构体、枚举、宏定义、变量定义	
*/
#include "fsl_common.h"


/* Macros for the touch touch controller. */
#define I2C_ADAU1466_BASE  LPI2C3

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_DIVIDER (5U)


#define I2C_ADAU1466_CLOCK_FREQ ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (LPI2C_CLOCK_SOURCE_DIVIDER + 1U))
//#define I2C_ADAU1466_BAUDRATE 400000U //400K
#define I2C_ADAU1466_BAUDRATE 600000U //600K


#define UNIT_OPEN		0x01
#define UNIT_CLOSE	0x00

// ----------------- DSP音量枚举 -----------------------------------------
typedef enum
{
	DSP_VOLUME_0_DB = 0x00,	  		//最大声
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

// ----------------- DSP mute开关枚举 -----------------------------------------
typedef enum
{
	DSP_MUTE_OFF = 0x00,	  	//not mute
	DSP_MUTE_ON,	  					//mute
}DspMute_E;

// ----------------- DSP 芯片选择枚举 -----------------------------------------
typedef enum
{
	DSP_DEVICE_1 = 0x00,	  		//dsp1
	DSP_DEVICE_2,	  					  //dsp2
}DspDevice_E;

// ----------------- DSP同传开关枚举 -----------------------------------------
typedef enum
{
	DSP_MONO_OFF = 0x00,	  	//其他模式
	DSP_MONO_ON,	  				//同传模式
}DspMono_E;

// ----------------- DSP page切换枚举 -----------------------------------------
typedef enum
{
	DSP_PAGE_0 = 0x00,	  	//page0
	DSP_PAGE_1,	  				  //page1
}DspPage_E;

// ----------------- DSP EQ取值枚举 -----------------------------------------
typedef enum
{
	DSP_EQ_N10_DB=0x00,	  		//-10db
	DSP_EQ_N9_DB,	  				  //-9db
	DSP_EQ_N8_DB,	  				  //-8db
	DSP_EQ_N7_DB,	  				  //-7db
	DSP_EQ_N6_DB,	  				  //-6db
	DSP_EQ_N5_DB,	  				  //-5db
	DSP_EQ_N4_DB,	  				  //-4db
	DSP_EQ_N3_DB,	  				  //-3db
	DSP_EQ_N2_DB,	  				  //-2db
	DSP_EQ_N1_DB,	  				  //-1db
	DSP_EQ_0_DB,				  		//0db
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
	
}DspEqValue_E;

// ----------------- DSP EQ频率枚举 -----------------------------------------
typedef enum
{
	DSP_EQ_100 = 0x00,	  		//第1段EQ
	DSP_EQ_200,	  				  	//第2段EQ
	DSP_EQ_400,	  				  	//第3段EQ
	DSP_EQ_600,	  				 		//第4段EQ
	DSP_EQ_1000,	  				  //第5段EQ
	DSP_EQ_3000,	  				  //第6段EQ
	DSP_EQ_6000,	  				  //第7段EQ
	DSP_EQ_12000,	  				  //第8段EQ
	DSP_EQ_14000,	  				  //第9段EQ
	DSP_EQ_16000,	  				  //第10段EQ

}DspEqPart_E;

// ----------------- DSP 模式枚举 -----------------------------------------
typedef enum
{
	DSP_MODE_WIRE = 0x00,	  		//有线模式
	DSP_MODE_PARTITION,					//分区模式
	DSP_MODE_SI,								//同传模式
	DSP_MODE_WIFI,							//无线模式
}DspSysMode_E;

// ----------------- DSP 输出通道枚举 -----------------------------------------
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
	DSP_OUTPUT_LINE_OUT1,					//line out1输出
	DSP_OUTPUT_LINE_OUT2,					//line out2输出
	DSP_OUTPUT_USB_OUT,						//USB输出
	DSP_OUTPUT_DANTE_OUT,					//dante 输出
	DSP_OUTPUT_DOWN_WIRE,					//下传给有线话筒
	DSP_OUTPUT_DOWN_WIFI,					//下传给无线话筒

}DspOutput_E;

// ----------------- DSP 常规输出的输入源枚举 -----------------------------------------
typedef enum
{
	DSP_OUTPUT_IN1 = 0x00,	  		//in1
	DSP_OUTPUT_IN2,								//in2
	DSP_OUTPUT_LINE_IN1,					//line in 1
	DSP_OUTPUT_LINE_IN2,					//line in 2
	DSP_OUTPUT_USB_IN,						//usb输入
	DSP_OUTPUT_DANTE_IN,					//dante 输入
	DSP_ALL_MIC_MIX,							//所有话筒混音

}DspInputSrc_E;

// ----------------- DSP 话筒源枚举 -----------------------------------------
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

// ----------------- DSP 总输出类型枚举 -----------------------------------------
typedef enum
{
	DSP_OUTPUT_VOLUME = 0x00,	  		//音量
	DSP_OUTPUT_EQ,									//EQ
	DSP_OUTPUT_DELAY,								//DELAY

}DspOutputType_E;

// ----------------- API信息------------------------------------------------------
typedef	struct	
{
	DspSysMode_E mode;										//当前模式
	uint8_t	uchChannelSrc[16][14];				//CH1 - CH16输入源

}DspInfoStruct_S;

extern	DspInfoStruct_S	g_unDspInfo;

//void ADAU1466_Test(void);
/*
*@Function NAME：   void I2C_ADAU1466_Init()
*@description:     	硬件I2C初始化，ADAU1466做从，1061做主
*@Author:           陈泽芳
*@param[out]:       无
*@param[in]:        无
*@return:           无
*/
//extern void I2C_ADAU1466_Init(void);
status_t ADAU1466_1_Init(uint8_t section);
status_t ADAU1466_2_Init(uint8_t section);
/*
*@Function NAME：   status_t ADAU1466_2_AliveDetect(void)
*@description:     	DSP1,2存活检测
*@Author:           陈泽芳
*@param[out]:       无
*@param[in]:        无
*@return:           无
*/
status_t ADAU1466_1_AliveDetect(void);
status_t ADAU1466_2_AliveDetect(void);



/*
*@Function NAME：   status_t ADAU1466_InputSource(DspOutput_E out,DspInputSrc_E src,DspVolume_E vol)
*@description:     	output的输入源配置
*@Author:           陈泽芳
*@param[out]:       无
*@param[in]:        DspOutput_E out，输出接口
*@param[in]:        DspInputSrc_E src，输出接口对应的输入源
*@param[in]:        DspVolume_E vol，输出接口对应的输入源的音量，-30db~0db，-31:mute
*@return:           status
*/
extern status_t ADAU1466_InputSource(DspOutput_E out,DspInputSrc_E src,DspVolume_E vol);

/*
*@Function NAME：   status_t ADAU1466_InputSourceReset(void)
*@description:     	output的输入源配置恢复默认
*@Author:           陈泽芳
*@param[out]:       无
*@param[in]:        无
*@return:           status
*/
extern status_t ADAU1466_InputSourceReset(void);

/*
*@Function NAME：   status_t ADAU1466_NormalOutputHandle(DspOutput_E out,DspOutputType_E src,DspEqPart_E part,uint8_t uchVol)
*@description:     	常规output的总输出配置
*@Author:           陈泽芳
*@param[out]:       无
*@param[in]:        DspOutput_E out，输出接口
*@param[in]:        DspOutputType_E src，要配置的输出类型
*@param[in]:        DspEqPart_E part，EQ段数，不是调EQ的时候，此参数无效，填0即可
*@param[in]:        uint8_t uchVol，输出类型的输出值
*@return:           status
*/
extern status_t ADAU1466_NormalOutputHandle(DspOutput_E out,DspOutputType_E src,DspEqPart_E part,uint8_t uchVol);

/*
*@Function NAME：   status_t ADAU1466_NormalOutputReset(void)
*@description:     	常规output的总输出恢复默认
*@Author:           陈泽芳
*@param[out]:       无
*@param[in]:        无
*@return:           status
*/
extern status_t ADAU1466_NormalOutputReset(void);

/*
*@Function NAME：   status_t ADAU1466_ChannelOutputHandle(DspOutput_E out,DspOutputType_E src,DspEqPart_E part,uint8_t uchVol)
*@description:     	CH1-CH16 output的总输出配置
*@Author:           陈泽芳
*@param[out]:       无
*@param[in]:        DspOutput_E out，输出接口
*@param[in]:        DspOutputType_E src，要配置的输出类型
*@param[in]:        DspEqPart_E part，EQ段数，不是调EQ的时候，此参数无效，填0即可
*@param[in]:        uint8_t uchVol，输出类型的输出值
*@return:           status
*/
extern status_t ADAU1466_ChannelOutputHandle(DspOutput_E out,DspOutputType_E src,DspEqPart_E part,uint8_t uchVol);

/*
*@Function NAME：   status_t ADAU1466_ChannelOutputReset(void)
*@description:     	CH1-CH16 output的总输出配置恢复默认
*@Author:           陈泽芳
*@param[out]:       无
*@param[in]:        无
*@return:           status
*/
extern status_t ADAU1466_ChannelOutputReset(void);

/*
*@Function NAME：   status_t ADAU1466_ChannelInputSource(DspOutput_E out,DspInputSrc_E src,DspVolume_E vol)
*@description:     	CH1-CH16 output的输入源配置
*@Author:           陈泽芳
*@param[out]:       无
*@param[in]:        DspOutput_E out，输出接口
*@param[in]:        DspInputSrc_E src，输出接口对应的输入源，需要区分模式
*@param[in]:        DspVolume_E vol，输出接口对应的输入源的音量，-30db~0db，mute
*@return:           status
*/
extern status_t ADAU1466_ChannelInputSource(DspOutput_E out,DspInputSrc_E src,DspVolume_E vol);

/*
*@Function NAME：   status_t ADAU1466_ChannelInputSourceReset(void)
*@description:     	CH1-CH16 output的输入源配置恢复默认
*@Author:           陈泽芳
*@param[out]:       无
*@param[in]:        无
*@return:           status
*/
extern status_t ADAU1466_ChannelInputSourceReset(void);

/*
*@Function NAME：   status_t ADAU1466_SetMode(DspSysMode_E mod)
*@description:     	模式设置
*@Author:           陈泽芳
*@param[out]:       无
*@param[in]:        DspSysMode_E mod
*@return:           status
*/
extern status_t ADAU1466_SetMode(DspSysMode_E mod);

/*
*@Function NAME：   status_t ADAU1466_UnitCtrl(DspUintSrc_E src,uint8_t *out,uint8_t *ovol)
*@description:     	开关话筒配置
*@Author:           陈泽芳
*@param[out]:       无
*@param[in]:        DspUintSrc_E src，有线和无线单元的实际音频通道号0x81~0x88
*@param[in]:        uint8_t *out，out[0]=要配的输出通道总数，out[1]：第一个要配的输出通道号，out[2]:第二个要配的输出通道号，依次类推
*@param[in]:        uint8_t *ovol，ovol[0]=要配的音量总个数，ovol[1]:第一个通道对应的音量，ovol[2]:第二个通道对应的音量，依次类推
*@return:           status
*/
extern status_t ADAU1466_UnitCtrl(DspUintSrc_E src,uint8_t *out,uint8_t *ovol);





