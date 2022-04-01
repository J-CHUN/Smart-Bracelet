#include "stm32f4xx.h"
#include "delay.h"
#include "oled.h"
#include "spi.h"
#include "iic.h"
#include "rtc.h"
#include "mpu6050.h"
#include "stepAlgorithm.h"
#include "watchInfo.h"
int main()//裸机程序结构  
{
	u8 err;
	u8 uiBuf[40];
	accValue_t accValue;
	static sportsInfo_t userSportsInfo;
	static u8 time_cnt;
	static timeStamp_t timeStamp;
	short aacx,aacy,aacz;									//加速度传感器原始数据
	static u8 tempSecond;								  //保存秒钟暂态量
	dateAndTime_t *rtcTime; 							  //获取年月日时分秒
	delay_init(100);
	Spi1_init();
	Oled_init();
	RTC_init();
	IIC_init();
	MPU_Init();
	WatchInfo_init();
	while (1)
	{		
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);			  //得到加速度传感器数据
		rtcTime = RTC_getDateAndTime(); 					  //获取当前RTC的值
		if(tempSecond != timeStamp.second)					 //秒更新
		{
			tempSecond = timeStamp.second;
			timeStamp.twentyMsCount = 0 ;//20ms计数变量清零
		}
		else												  //秒不更新，1秒等于50*20ms
		{
			timeStamp.twentyMsCount ++;//20ms计数变量++
		}
		timeStamp.hour	 = rtcTime->hour;
		timeStamp.minute = rtcTime->minute;
		timeStamp.second = rtcTime->second;
		//将三轴数据转换为以g为单位的数据
		accValue.accX = ((float)(int)aacx/16384) *10;
		accValue.accY = ((float)(int)aacy/16384) *10;
		accValue.accZ = ((float)(int)aacz/16384) *10; 
		userSportsInfo = *onSensorChanged(&accValue,&timeStamp,WatchInfo_getUserInfo(&err)); //调用计数算法
		sprintf((char*)uiBuf,"Step:%05d ",userSportsInfo.stepCount); // 显示步数
		OLED_showString(10,0,uiBuf,16); 
		sprintf((char*)uiBuf,"kal:%.1f KAL",userSportsInfo.calories); // 显示卡路里
		OLED_showString(10,2,uiBuf,16); 	
		sprintf((char*)uiBuf,"dis:%.1f M",userSportsInfo.distance); // 显示里程
		OLED_showString(10,4,uiBuf,16); 			
	
		if(accValue.accZ>=0)
		{
			sprintf((char*)uiBuf,"Z:%02d m/s^2",(char)accValue.accZ); // 以整数显示
			OLED_showString(10,6,uiBuf,16);				
		}
	  else
		{
			sprintf((char*)uiBuf,"Z:-%02d",(char)(-accValue.accZ)); // 以整数显示
			OLED_showString(10,6,uiBuf,16);				
		}		

		
		delay_ms(20);  //以50Hz的频率去读取三轴加速度的XYZ轴加速度值
	}
}






