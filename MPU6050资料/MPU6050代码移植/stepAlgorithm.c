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
//存放三轴数据  
float oriValues[3] = {0};    
//用于存放计算阈值的波峰波谷差值  
float tempValue[VALUE_NUM] ={0};  
int tempCount = 0;  
//是否上升的标志位  
u8 isDirectionUp = FALSE;  
//持续上升次数  
int continueUpCount = 0;  
//上一点的持续上升的次数，为了记录波峰的上升次数  
int continueUpFormerCount = 0;  
//上一点的状态，上升还是下降  
u8 lastStatus = FALSE;  
//波峰值  
float peakOfWave = 0;  
//波谷值  
float valleyOfWave = 0;  
//此次波峰的时间  
long timeOfThisPeak = 0;  
//上次波峰的时间  
long timeOfLastPeak = 0;  
//当前的时间  
long timeOfNow = 0;  
//当前传感器的值  
float gravityNew = 0;  
//上次传感器的值  
float gravityOld = 0;  
//动态阈值需要动态的数据，这个值用于这些动态数据的阈值  
float initialValue = (float) 1.3;  
//初始阈值  
float ThreadValue = (float) 2.0;
//三轴轴值
accValue_t accValue;
//行走信息:卡路里、里程、步数
static sportsInfo_t sportsInfo;
//计步缓存
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
* 函数名：onSensorChanged
* 功能描述： G-Sensor工作后会一直调用这个函数对三轴数据进行平方和开根号的处理  
*					调用DetectorNewStep检测步子 
*					
* 作者：TAO  
* 参数说明：  
*                 输入：
*					    pAccValue：G-sensor的原始数据
*						timeStamp_p：动态时间戳
* 返回值说明：
* 修改记录：
*******************************************************************************/
sportsInfo_t *onSensorChanged(accValue_t *pAccValue,timeStamp_t *timeStamp_p,personInfo_t * personInfo) {  
  accValue_t *p = pAccValue;
  personInfo_t *userInfo = personInfo;
  timeStamp_t *time_p = timeStamp_p;
  oriValues[0] = p->accX;
  oriValues[1] = p->accY;
  oriValues[2] = p->accZ;
  //对三轴数据进行平方和开根号的处理 
  gravityNew = (float) sqrt(oriValues[0] * oriValues[0]  
                            + oriValues[1] * oriValues[1] + oriValues[2] * oriValues[2]);  


  //检测步子
  return DetectorNewStep(gravityNew,time_p,userInfo); 
}  
/*******************************************************************************
* 函数名：DetectorNewStep
* 功能描述： 
*		  步伐更新：如果检测到了波峰，并且符合时间差以及阈值的条件，则判定为1步 		
*		  阀值更新：符合时间差条件，波峰波谷差值大于initialValue，则将该差值纳入阈值的计算中 		
* 作者：    TAO  
* 参数说明：  
		输入：
		values：经过处理的G-sensor数据
		timeStamp_p：时间戳
* 返回值说明：
* 修改记录：
*******************************************************************************/
sportsInfo_t *DetectorNewStep(float values,timeStamp_t *timeStamp_p,personInfo_t * personInfo) 
{  
  static u32 time_old;
  personInfo_t *userInfo = personInfo;
  static u32 step_per_2_second;  //每两秒所走的步数
  float step_lenth,walk_speed,walk_distance,Calories;//步长
  u32 time_now;
  timeStamp_t *time_p = timeStamp_p;
  if (gravityOld == 0) 
  {  
    gravityOld = values;  
  } 
  else 
  {  
    if (DetectorPeak(values, gravityOld))//检测到波峰
    {  
      timeOfLastPeak = timeOfThisPeak;//更新上次波峰的时间  
      //将时间戳转换为以毫秒ms为单位
      time_now = timeOfNow = ((time_p->hour*60+time_p->minute)*60+time_p->second)*1000+time_p->twentyMsCount*20; //获取时间 ,并转化为毫秒
      //如果检测到了波峰，并且符合时间差以及阈值的条件，则判定为1步 
      if (  (timeOfNow - timeOfLastPeak >= 250 )//TAO 修改为300，防止轻微动都也会检测步子
          //&& (timeOfNow - timeOfLastPeak <= 2000)
          &&(peakOfWave - valleyOfWave >= ThreadValue)
            )
      {  
        timeOfThisPeak = timeOfNow; //更新此次波峰时间 
        
        
       	stepTempCount++;// :加1为1步
        step_per_2_second ++;
				
				
				
				
        // :这样计算卡路里，不能滤除人为的误操作，导致的结果是:里程和卡路里偏大
        if((time_now - time_old) >= 2000 )    //如果时间过了2秒
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
          else if(5 == step_per_2_second)			  // ：为了使计步准确，设置上限值为5步，牺牲卡路里准确性
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
					
					
          walk_speed = step_per_2_second*step_lenth/2;   //速度 ，单位：米/秒
          walk_distance  = step_per_2_second*step_lenth; //行走距离，单位：米
          Calories = 4.5f*walk_speed*(userInfo->weight/2)/1800;  // :weight是以kg为单位 卡路里公式计算
          sportsInfo.calories  += Calories;
          sportsInfo.distance  += walk_distance;		
          time_old = time_now;         //更新时间
          step_per_2_second = 0;
          
        }   
        else 
        {
          //do nothing
        }		
        /* 
        * 处理无效运动： 
        * 1.连续记录5才开始计步 
        * 2.例如记录的步用户停住超过3秒，则前面的记录失效，下次从头开始 
        * 3.连续4记录了步用户还在运动，之前的数据才有效 
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
      // :更新阀值,问题:阀值不会一直变大，不能变小?
      if (timeOfNow - timeOfLastPeak >= 250  
          && (peakOfWave - valleyOfWave >= initialValue)) 
      {  
        timeOfThisPeak = timeOfNow;  
        ThreadValue = Peak_Valley_Thread(peakOfWave - valleyOfWave);//更新阀值  
      }  
    }  
  }  
  gravityOld = values;  
  return &sportsInfo;
}

/*******************************************************************************
* 函数名：DetectorPeak
* 功能描述： 
*检测波峰 。以下四个条件判断为波峰： 
*（1）目前点为下降的趋势：isDirectionUp为FALSE 
*（2）之前的点为上升的趋势：lastStatus为TRUE 
*（3）到波峰为止，持续上升大于等于2次
*（4）波峰值大于20 // :把这个值修改为15
*记录波谷值 ：
*（1）观察波形图，可以发现在出现步子的地方，波谷的下一个就是波峰，有比较明显的特征以及差值 
*（2）所以要记录每次的波谷值，为了和下次的波峰做对比 	
* 作者：TAO  
* 参数说明：  
*		输入：
*			newValue：最新的经过处理的G-sensor数据
*			oldValue：前一个处理的G-sensor数据
* 返回值说明：
* 修改记录：
*******************************************************************************/
u8 DetectorPeak(float newValue, float oldValue) 
{  
  lastStatus = isDirectionUp;  
  if (newValue >= oldValue) //采样数据呈上升趋势
  {  
    isDirectionUp = TRUE;  
    continueUpCount++;  
  }
  else						//数据呈下降趋势
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
* 函数名：Peak_Valley_Thread
* 功能描述：  
*					阈值的计算 
*						1.通过波峰波谷的差值计算阈值 
*						2.记录4个值，存入tempValue[]数组中 
*						3.在将数组传入函数averageValue中计算阈值 
*					
* 作者：TAO  
* 参数说明： 
* 返回值说明：
* 修改记录：
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
    tempThread = averageValue(tempValue, VALUE_NUM);//计算阀值  
    for ( i = 1;i < VALUE_NUM;i++)//线性移位更新
    {  
      tempValue[i - 1] = tempValue[i];  
    }  
    tempValue[VALUE_NUM - 1] = value;  
  }  
  return tempThread;  
  
}

/*******************************************************************************
* 函数名：averageValue
* 功能描述：  
*		 梯度化阈值 
*			1.计算数组的均值 
*			2.通过均值将阈值梯度化在一个范围里 
*					
* 作者：TAO  
* 参数说明： 
* 返回值说明：
* 修改记录：
*******************************************************************************/
float averageValue(float value[], int n)
{  
  float ave = 0;  
  u8 i =0;
  for ( i = 0; i < n; i++)
  {  
    ave += value[i];//求和  
  }  
  ave = ave / VALUE_NUM;//求平均值  
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
