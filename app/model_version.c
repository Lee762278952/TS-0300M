/**
 *
 *	@name				model_version.c
 *	@author 			KT
 *	@date 				2019/08/30
 *	@brief
 *
 *  @API
 *
 *  @description
 *
 **/
/*******************************************************************************
 * includes
 ******************************************************************************/
#include <stdio.h>  
#include <stdarg.h>  
#include <string.h>
#include <stdint.h>
#include "app_common.h"

/* 型号名称 */
#define DEVICE_MODEL_NAME								(deviceModel[DEVICE_MODEL])


const char deviceModel[2][10] = {"TS-0300M",""};
const int  MONTH_PER_YEAR=12;
const char szEnglishMonth[MONTH_PER_YEAR][4]={ "Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
char buildTime[40];

char *APP_BuildTime(void)
{        
	char szTmpDate[40]={0};
	char szTmpTime[20]={0};
	char szMonth[4]={0};
	int iYear,iMonth,iDay,iHour,iMin,iSec;//,,

	//获取编译日期、时间
	sprintf(szTmpDate,"%s",__DATE__); //"Sep 18 2010"
	sprintf(szTmpTime,"%s",__TIME__);    //"10:59:19"
	
	sscanf(szTmpDate,"%s %d %d",szMonth,&iDay,&iYear);
	sscanf(szTmpTime,"%d:%d:%d",&iHour,&iMin,&iSec);
			
	for(int i=0;MONTH_PER_YEAR;i++)
	{
		if(strncmp(szMonth,szEnglishMonth[i],3)==0)
		{
				iMonth=i+1;
				break;
		}
	}
	sprintf(buildTime,"%04d-%02d-%02d %02d:%02d:%02d",iYear,iMonth,iDay,iHour,iMin,iSec);
	return buildTime;
}

void APP_PrintDeviceMsg(void){
	debug("\r\n********************************************\r\n");
	debug("**      Model:   %s\r\n",DEVICE_MODEL_NAME);
	debug("**    Version:   %s\r\n",APP_VERSION);
	debug("** Build time:   %s\r\n",APP_BUILD_TIME);
	debug("********************************************\r\n\r\n");
}


