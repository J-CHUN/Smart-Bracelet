/*******************************************************************************
* INCLUDES
*/
#include "stepAlgorithm.h"
#include "rtc.h"
#include "math.h"
#include "mpu6050.h"
/*******************************************************************************
* CONSTANTS
*/
#define TRUE 1     
#define FALSE 0
#define VALUE_NUM 4

/*******************************************************************************
* TYPEDEFS
*/


/*******************************************************************************
* LOCAL VARIABLES
*/
//�����������  
float oriValues[3] = {0};    
//���ڴ�ż�����ֵ�Ĳ��岨�Ȳ�ֵ  
float tempValue[VALUE_NUM] ={0};  
int tempCount = 0;  
//�Ƿ������ı�־λ  
u8 isDirectionUp = FALSE;  
//������������  
int continueUpCount = 0;  
//��һ��ĳ��������Ĵ�����Ϊ�˼�¼�������������  
int continueUpFormerCount = 0;  
//��һ���״̬�����������½�  
u8 lastStatus = FALSE;  
//����ֵ  
float peakOfWave = 0;  
//����ֵ  
float valleyOfWave = 0;  
//�˴β����ʱ��  
long timeOfThisPeak = 0;  
//�ϴβ����ʱ��  
long timeOfLastPeak = 0;  
//��ǰ��ʱ��  
long timeOfNow = 0;  
//��ǰ��������ֵ  
float gravityNew = 0;  
//�ϴδ�������ֵ  
float gravityOld = 0;  
//��̬��ֵ��Ҫ��̬�����ݣ����ֵ������Щ��̬���ݵ���ֵ  
float initialValue = (float) 1.3;  
//��ʼ��ֵ  
float ThreadValue = (float) 2.0;
//������ֵ
accValue_t accValue;
//������Ϣ:��·���̡�����
static sportsInfo_t sportsInfo;
//�Ʋ�����
static u8 stepTempCount =0;

/*******************************************************************************
* LOCAL FUNCTIONS
*/


/*******************************************************************************
* PROFILE CALLBACKS
*/



/*******************************************************************************
* PUBLIC FUNCTIONS
*/
/*******************************************************************************
* ��������onSensorChanged
* ���������� G-Sensor�������һֱ��������������������ݽ���ƽ���Ϳ����ŵĴ���  
*					����DetectorNewStep��ⲽ�� 
*					
* ���ߣ�TAO  
* ����˵����  
*                 ���룺
*					    pAccValue��G-sensor��ԭʼ����
*						timeStamp_p����̬ʱ���
* ����ֵ˵����
* �޸ļ�¼��
*******************************************************************************/
sportsInfo_t *onSensorChanged(accValue_t *pAccValue,timeStamp_t *timeStamp_p,personInfo_t * personInfo) {  
  accValue_t *p = pAccValue;
  personInfo_t *userInfo = personInfo;
  timeStamp_t *time_p = timeStamp_p;
  oriValues[0] = p->accX;
  oriValues[1] = p->accY;
  oriValues[2] = p->accZ;
  //���������ݽ���ƽ���Ϳ����ŵĴ��� 
  gravityNew = (float) sqrt(oriValues[0] * oriValues[0]  
                            + oriValues[1] * oriValues[1] + oriValues[2] * oriValues[2]);  


  //��ⲽ��
  return DetectorNewStep(gravityNew,time_p,userInfo); 
}  
/*******************************************************************************
* ��������DetectorNewStep
* ���������� 
*		  �������£������⵽�˲��壬���ҷ���ʱ����Լ���ֵ�����������ж�Ϊ1�� 		
*		  ��ֵ���£�����ʱ������������岨�Ȳ�ֵ����initialValue���򽫸ò�ֵ������ֵ�ļ����� 		
* ���ߣ�    TAO  
* ����˵����  
		���룺
		values�����������G-sensor����
		timeStamp_p��ʱ���
* ����ֵ˵����
* �޸ļ�¼��
*******************************************************************************/
sportsInfo_t *DetectorNewStep(float values,timeStamp_t *timeStamp_p,personInfo_t * personInfo) 
{  
  static u32 time_old;
  personInfo_t *userInfo = personInfo;
  static u32 step_per_2_second;  //ÿ�������ߵĲ���
  float step_lenth,walk_speed,walk_distance,Calories;//����
  u32 time_now;
  timeStamp_t *time_p = timeStamp_p;
  if (gravityOld == 0) 
  {  
    gravityOld = values;  
  } 
  else 
  {  
    if (DetectorPeak(values, gravityOld))//��⵽����
    {  
      timeOfLastPeak = timeOfThisPeak;//�����ϴβ����ʱ��  
      //��ʱ���ת��Ϊ�Ժ���msΪ��λ
      time_now = timeOfNow = ((time_p->hour*60+time_p->minute)*60+time_p->second)*1000+time_p->twentyMsCount*20; //��ȡʱ�� ,��ת��Ϊ����
      //�����⵽�˲��壬���ҷ���ʱ����Լ���ֵ�����������ж�Ϊ1�� 
      if (  (timeOfNow - timeOfLastPeak >= 250 )//TAO �޸�Ϊ300����ֹ��΢����Ҳ���ⲽ��
          //&& (timeOfNow - timeOfLastPeak <= 2000)
          &&(peakOfWave - valleyOfWave >= ThreadValue)
            )
      {  
        timeOfThisPeak = timeOfNow; //���´˴β���ʱ�� 
        
        
       	stepTempCount++;// :��1Ϊ1��
        step_per_2_second ++;
				
				
				
				
        // :�������㿨·������˳���Ϊ������������µĽ����:��̺Ϳ�·��ƫ��
        if((time_now - time_old) >= 2000 )    //���ʱ�����2��
        {

          if( 1 == step_per_2_second )				   
          {
            step_lenth = userInfo->height/5;
          }
          else if( 2 == step_per_2_second )
          {
            step_lenth = userInfo->height/4;
          }
          else if( 3 == step_per_2_second )
          {
            step_lenth = userInfo->height/3;
          }
          else if( 4 == step_per_2_second )
          {
            step_lenth = userInfo->height/2;
          }
          else if(5 == step_per_2_second)			  // ��Ϊ��ʹ�Ʋ�׼ȷ����������ֵΪ5����������·��׼ȷ��
          {
            step_lenth = userInfo->height/1.2f;
          }
          else if( 7 == step_per_2_second )
          {
            step_lenth = userInfo->height;
          }
          else if(step_per_2_second >= 8)				//		step_diff>8
          {
            step_lenth = userInfo->height*1.2f;
          }
          else 
          {
            step_lenth = 0;
          }
					
					
          walk_speed = step_per_2_second*step_lenth/2;   //�ٶ� ����λ����/��
          walk_distance  = step_per_2_second*step_lenth; //���߾��룬��λ����
          Calories = 4.5f*walk_speed*(userInfo->weight/2)/1800;  // :weight����kgΪ��λ ��·�﹫ʽ����
          sportsInfo.calories  += Calories;
          sportsInfo.distance  += walk_distance;		
          time_old = time_now;         //����ʱ��
          step_per_2_second = 0;
          
        }   
        else 
        {
          //do nothing
        }		
        /* 
        * ������Ч�˶��� 
        * 1.������¼5�ſ�ʼ�Ʋ� 
        * 2.�����¼�Ĳ��û�ͣס����3�룬��ǰ��ļ�¼ʧЧ���´δ�ͷ��ʼ 
        * 3.����4��¼�˲��û������˶���֮ǰ�����ݲ���Ч 
        * */  				
        if ((stepTempCount< 5 )&&(timeOfNow - timeOfLastPeak >= 3000))          
        {
          stepTempCount = 0;
        }
        else if((stepTempCount>= 5)&&(timeOfNow - timeOfLastPeak <= 3000))
        {
          sportsInfo.stepCount += stepTempCount;          
          stepTempCount         = 0; 				
        }
        else
        {
          //do nothing
        }
        
        
      }  
      // :���·�ֵ,����:��ֵ����һֱ��󣬲��ܱ�С?
      if (timeOfNow - timeOfLastPeak >= 250  
          && (peakOfWave - valleyOfWave >= initialValue)) 
      {  
        timeOfThisPeak = timeOfNow;  
        ThreadValue = Peak_Valley_Thread(peakOfWave - valleyOfWave);//���·�ֵ  
      }  
    }  
  }  
  gravityOld = values;  
  return &sportsInfo;
}

/*******************************************************************************
* ��������DetectorPeak
* ���������� 
*��Ⲩ�� �������ĸ������ж�Ϊ���壺 
*��1��Ŀǰ��Ϊ�½������ƣ�isDirectionUpΪFALSE 
*��2��֮ǰ�ĵ�Ϊ���������ƣ�lastStatusΪTRUE 
*��3��������Ϊֹ�������������ڵ���2��
*��4������ֵ����20 // :�����ֵ�޸�Ϊ15
*��¼����ֵ ��
*��1���۲첨��ͼ�����Է����ڳ��ֲ��ӵĵط������ȵ���һ�����ǲ��壬�бȽ����Ե������Լ���ֵ 
*��2������Ҫ��¼ÿ�εĲ���ֵ��Ϊ�˺��´εĲ������Ա� 	
* ���ߣ�TAO  
* ����˵����  
*		���룺
*			newValue�����µľ��������G-sensor����
*			oldValue��ǰһ�������G-sensor����
* ����ֵ˵����
* �޸ļ�¼��
*******************************************************************************/
u8 DetectorPeak(float newValue, float oldValue) 
{  
  lastStatus = isDirectionUp;  
  if (newValue >= oldValue) //�������ݳ���������
  {  
    isDirectionUp = TRUE;  
    continueUpCount++;  
  }
  else						//���ݳ��½�����
  {  
    continueUpFormerCount = continueUpCount;  
    continueUpCount = 0;  
    isDirectionUp = FALSE;  
  }  
  
  if ((!isDirectionUp) && lastStatus  
      && (continueUpFormerCount >= 2 && oldValue >= 20))
    
  {  
    peakOfWave = oldValue;  
    return TRUE;  
  } 
  else if ((!lastStatus) && isDirectionUp) 
  {  
    valleyOfWave = oldValue;  
    return FALSE;  
  }
  else
  {  
    return FALSE;  
  }  
}
/*******************************************************************************
* ��������Peak_Valley_Thread
* ����������  
*					��ֵ�ļ��� 
*						1.ͨ�����岨�ȵĲ�ֵ������ֵ 
*						2.��¼4��ֵ������tempValue[]������ 
*						3.�ڽ����鴫�뺯��averageValue�м�����ֵ 
*					
* ���ߣ�TAO  
* ����˵���� 
* ����ֵ˵����
* �޸ļ�¼��
*******************************************************************************/
float Peak_Valley_Thread(float value) 
{  
  float tempThread = ThreadValue; 
  u8 i = 0;
  if (tempCount < VALUE_NUM)
  {  
    tempValue[tempCount] = value;  
    tempCount++;  
  } 
  else 
  {  
    tempThread = averageValue(tempValue, VALUE_NUM);//���㷧ֵ  
    for ( i = 1;i < VALUE_NUM;i++)//������λ����
    {  
      tempValue[i - 1] = tempValue[i];  
    }  
    tempValue[VALUE_NUM - 1] = value;  
  }  
  return tempThread;  
  
}

/*******************************************************************************
* ��������averageValue
* ����������  
*		 �ݶȻ���ֵ 
*			1.��������ľ�ֵ 
*			2.ͨ����ֵ����ֵ�ݶȻ���һ����Χ�� 
*					
* ���ߣ�TAO  
* ����˵���� 
* ����ֵ˵����
* �޸ļ�¼��
*******************************************************************************/
float averageValue(float value[], int n)
{  
  float ave = 0;  
  u8 i =0;
  for ( i = 0; i < n; i++)
  {  
    ave += value[i];//���  
  }  
  ave = ave / VALUE_NUM;//��ƽ��ֵ  
  if (ave >= 8)  
    ave = (float) 4.3;  //????
  else if (ave >= 7 && ave < 8)  
    ave = (float) 3.3;  
  else if (ave >= 4 && ave < 7)  
    ave = (float) 2.3;  
  else if (ave >= 3 && ave < 4)  
    ave = (float) 2.0;  
  else
  {  
    ave = (float) 1.3;  
  }  
  return ave;  
}


/*******************************************************************************
*******************************************************************************/
