#include "stm32f4xx.h"
#include "delay.h"
#include "oled.h"
#include "spi.h"
#include "iic.h"
#include "rtc.h"
#include "mpu6050.h"
#include "stepAlgorithm.h"
#include "watchInfo.h"
int main()//�������ṹ  
{
	u8 err;
	u8 uiBuf[40];
	accValue_t accValue;
	static sportsInfo_t userSportsInfo;
	static u8 time_cnt;
	static timeStamp_t timeStamp;
	short aacx,aacy,aacz;									//���ٶȴ�����ԭʼ����
	static u8 tempSecond;								  //����������̬��
	dateAndTime_t *rtcTime; 							  //��ȡ������ʱ����
	delay_init(100);
	Spi1_init();
	Oled_init();
	RTC_init();
	IIC_init();
	MPU_Init();
	WatchInfo_init();
	while (1)
	{		
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);			  //�õ����ٶȴ���������
		rtcTime = RTC_getDateAndTime(); 					  //��ȡ��ǰRTC��ֵ
		if(tempSecond != timeStamp.second)					 //�����
		{
			tempSecond = timeStamp.second;
			timeStamp.twentyMsCount = 0 ;//20ms������������
		}
		else												  //�벻���£�1�����50*20ms
		{
			timeStamp.twentyMsCount ++;//20ms��������++
		}
		timeStamp.hour	 = rtcTime->hour;
		timeStamp.minute = rtcTime->minute;
		timeStamp.second = rtcTime->second;
		//����������ת��Ϊ��gΪ��λ������
		accValue.accX = ((float)(int)aacx/16384) *10;
		accValue.accY = ((float)(int)aacy/16384) *10;
		accValue.accZ = ((float)(int)aacz/16384) *10; 
		userSportsInfo = *onSensorChanged(&accValue,&timeStamp,WatchInfo_getUserInfo(&err)); //���ü����㷨
		sprintf((char*)uiBuf,"Step:%05d ",userSportsInfo.stepCount); // ��ʾ����
		OLED_showString(10,0,uiBuf,16); 
		sprintf((char*)uiBuf,"kal:%.1f KAL",userSportsInfo.calories); // ��ʾ��·��
		OLED_showString(10,2,uiBuf,16); 	
		sprintf((char*)uiBuf,"dis:%.1f M",userSportsInfo.distance); // ��ʾ���
		OLED_showString(10,4,uiBuf,16); 			
	
		if(accValue.accZ>=0)
		{
			sprintf((char*)uiBuf,"Z:%02d m/s^2",(char)accValue.accZ); // ��������ʾ
			OLED_showString(10,6,uiBuf,16);				
		}
	  else
		{
			sprintf((char*)uiBuf,"Z:-%02d",(char)(-accValue.accZ)); // ��������ʾ
			OLED_showString(10,6,uiBuf,16);				
		}		

		
		delay_ms(20);  //��50Hz��Ƶ��ȥ��ȡ������ٶȵ�XYZ����ٶ�ֵ
	}
}






