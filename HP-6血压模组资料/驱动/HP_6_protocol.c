/****************************************************************
Copyright: 2012-2016, Veepoo Tech. Co., Ltd.
File name: 		HP_6_protocol.c
Description: 
Author: 			WJ
Version: 
Date: 			2016-09-02
History: 
*****************************************************************/
// IIC 速率使用100K；7bite地址：0x66

#include <stdbool.h>
#include <stdint.h>
#include "HP_6_crc16.h"
#include "HP_6_protocol.h"
/****************************************************************
*nrf51822平台的头文件
*/
#include "nrf_delay.h"
#include "Hp_6_twi_master.h"	//nrf51822平台
#include "Hp_6_I2C.h"

/****************************************************************
* 宏定义
*/
#define PROTOCLO_HEADER			0xc8d7b6a5
#define PROTOCLO_CMD_INDEX		4
#define PROTOCLO_DATA_INDEX		5
#define PROTOCLO_BUF_MAX_LEN	24

#define HP_6_IIC_ADDRESS 		0x66

/****************************************************************
* 全局变量
*/

/****************************************************************
* 本地静态变量
*/
static uint8_t tx_buf[PROTOCLO_BUF_MAX_LEN] = {0};
static uint8_t rx_buf[PROTOCLO_BUF_MAX_LEN] = {0};	

//开启血压测量
const uint8_t cmd_bp_open[]=
	{0xc8,0xd7,0xb6,0xa5,0x90,0x01,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//关闭血压测量
const uint8_t cmd_bp_close[]=
	{0xc8,0xd7,0xb6,0xa5,0x90,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//获取血压测量结果
const uint8_t cmd_bp_result[]=
	{0xc8,0xd7,0xb6,0xa5,0x90,0x02,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

//开启心率测量
const uint8_t cmd_rate_open[]=
	{0xc8,0xd7,0xb6,0xa5,0xD0,0x01,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//关闭心率测量
const uint8_t cmd_rate_close[]=
	{0xc8,0xd7,0xb6,0xa5,0xD0,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//获取心率测量结果
const uint8_t cmd_rate_result[]=
	{0xc8,0xd7,0xb6,0xa5,0xD0,0x02,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

//获取ADC 数据
const uint8_t cmd_get_adc[]=
	{0xc8,0xd7,0xb6,0xa5,0x91,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

//设置低功耗
const uint8_t cmd_set_powersaving[]=
	{0xc8,0xd7,0xb6,0xa5,0x70,0x01,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

//获取版本信息
const uint8_t cmd_get_version[]=
	{0xc8,0xd7,0xb6,0xa5,0xa2,0x02,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
/****************************************************************
* 本地函数声明
*/

/****************************************************************
** Function name:			HP_6_Delay_ms
** Descriptions:            延时函数，单位毫秒
** input parameters:
** output parameters:
** Returned value:
** Created by:				WJ              
** Created Date:           	2016-09-02
**-----------------------------------------------------------------------
** Modified by:
** Modified date:
**-----------------------------------------------------------------------
*****************************************************************/ 
void HP_6_Delay_ms(uint32_t ms)
{
	//delay_ms(5);	//加入平台相关的延时函数
	nrf_delay_ms(5); //nrf51822 延时函数，单位毫秒
}

/****************************************************************
** Function name:			HP_6_GetResultData
** Descriptions:            	获取结果数据
** input parameters:
** output parameters:
** Returned value:
** Created by:				WJ              
** Created Date:           	2016-09-02
**-----------------------------------------------------------------------
** Modified by:
** Modified date:
**-----------------------------------------------------------------------
*****************************************************************/ 
uint16_t HP_6_GetResultData(uint8_t *data)
{
	uint8_t i = 0;
	
	if(!data)
	{
		return 0;
	}
	
	for(i=0; i < PROTOCLO_BUF_MAX_LEN; i++)
	{
		data[i] = rx_buf[i];
	}
	
	return PROTOCLO_BUF_MAX_LEN;
}
/****************************************************************
** Function name:			HP_6_SendCmd
** Descriptions:            	发送命令到从机并获取结果
** input parameters:
** output parameters:
** Returned value:
** Created by:				WJ              
** Created Date:           	2016-09-02
**-----------------------------------------------------------------------
** Modified by:
** Modified date:
**-----------------------------------------------------------------------
*****************************************************************/ 
uint8_t HP_6_SendCmd(uint8_t *tx_buf, uint8_t *rx_buf)
{
	uint8_t state = HP_6_SUCCESS;
	uint16_t crc;	

	crc = Crc16(&tx_buf[4], 18);	//小端序
	*(uint16_t*)(&tx_buf[22]) = crc;

	Hp_6_I2CWrite(HP_6_IIC_ADDRESS, tx_buf, 24);
    HP_6_Delay_ms(5); 
    Hp_6_I2CRead(HP_6_IIC_ADDRESS, rx_buf, 24);
 	
	crc = *(uint16_t*)(&rx_buf[22]);	//小端序
	if(crc != Crc16(&rx_buf[4], 18))	//校验失败
	{
		state = HP_6_FAILURE;
	}
	
	return state;
}

/****************************************************************
** Function name:			HP_6_GetRateResult
** Descriptions:            	获取心率测量结果
** input parameters:
** output parameters:
** Returned value:
** Created by:				WJ              
** Created Date:           	2016-09-02
**-----------------------------------------------------------------------
** Modified by:
** Modified date:
**-----------------------------------------------------------------------
*****************************************************************/ 
uint8_t HP_6_GetRateResult(void)
{
	uint32_t i;	
	for(i=0; i < PROTOCLO_BUF_MAX_LEN; i++)
	{
		tx_buf[i] = cmd_rate_result[i];		
	}
	
	return HP_6_SendCmd( tx_buf, rx_buf);   
}

/****************************************************************
** Function name:			HP_6_OpenRate
** Descriptions:            	开启心率测量
** input parameters:
** output parameters:
** Returned value:
** Created by:				WJ              
** Created Date:           	2016-09-02
**-----------------------------------------------------------------------
** Modified by:
** Modified date:
**-----------------------------------------------------------------------
*****************************************************************/ 
uint8_t HP_6_OpenRate(void)
{
	uint32_t i;
		
	for( i = 0; i < PROTOCLO_BUF_MAX_LEN; i++)
	{
		tx_buf[i] = cmd_rate_open[i];
	}
	
	return HP_6_SendCmd( tx_buf, rx_buf);       
}

/****************************************************************
** Function name:			HP_6_OpenRate
** Descriptions:            	关闭心率测量
** input parameters:
** output parameters:
** Returned value:
** Created by:				WJ              
** Created Date:           	2016-09-02
**-----------------------------------------------------------------------
** Modified by:
** Modified date:
**-----------------------------------------------------------------------
*****************************************************************/ 
uint8_t HP_6_CloseRate(void)
{
	uint32_t i;
		
	for( i = 0; i < PROTOCLO_BUF_MAX_LEN; i++)
	{
		tx_buf[i] = cmd_rate_close[i];
	}
	
	return HP_6_SendCmd( tx_buf, rx_buf);       
}

/****************************************************************
** Function name:			HP_6_GetBpResult
** Descriptions:            	获取血压测量结果
** input parameters:
** output parameters:
** Returned value:
** Created by:				WJ              
** Created Date:           	2016-09-02
**-----------------------------------------------------------------------
** Modified by:
** Modified date:
**-----------------------------------------------------------------------
*****************************************************************/ 
uint8_t HP_6_GetBpResult(void)
{
	uint32_t i;	
	
	for(i=0; i < PROTOCLO_BUF_MAX_LEN; i++)
	{
		tx_buf[i] = cmd_bp_result[i];	
	}

	return HP_6_SendCmd( tx_buf, rx_buf);  
}

/****************************************************************
** Function name:			HP_6_OpenBp
** Descriptions:            	开启心率测量
** input parameters:
** output parameters:
** Returned value:
** Created by:				WJ              
** Created Date:           	2016-09-02
**-----------------------------------------------------------------------
** Modified by:
** Modified date:
**-----------------------------------------------------------------------
*****************************************************************/ 
uint8_t HP_6_OpenBp(void)
{
	uint32_t i;
		
	for( i = 0; i < PROTOCLO_BUF_MAX_LEN; i++)
	{
		tx_buf[i] = cmd_bp_open[i];
	}
	
	return HP_6_SendCmd( tx_buf, rx_buf);       
}

/****************************************************************
** Function name:			HP_6_CloseBp
** Descriptions:            	关闭血压测量
** input parameters:
** output parameters:
** Returned value:
** Created by:				WJ              
** Created Date:           	2016-09-02
**-----------------------------------------------------------------------
** Modified by:
** Modified date:
**-----------------------------------------------------------------------
*****************************************************************/ 
uint8_t HP_6_CloseBp(void)
{
	uint32_t i;
		
	for( i = 0; i < PROTOCLO_BUF_MAX_LEN; i++)
	{
		tx_buf[i] = cmd_bp_close[i];
	}
	
	return HP_6_SendCmd( tx_buf, rx_buf);       
}

/****************************************************************
** Function name:			HP_6_GetADC
** Descriptions:            	关闭血压测量
** input parameters:
** output parameters:
** Returned value:
** Created by:				WJ              
** Created Date:           	2016-09-02
**-----------------------------------------------------------------------
** Modified by:
** Modified date:
**-----------------------------------------------------------------------
*****************************************************************/ 
uint8_t HP_6_GetADC(uint8_t num, uint8_t id)
{
	uint32_t i;
	uint16_t crc;
		
	for( i = 0; i < PROTOCLO_BUF_MAX_LEN; i++)
	{
		tx_buf[i] = cmd_get_adc[i];
	}
	tx_buf[PROTOCLO_DATA_INDEX] =  num;
	tx_buf[PROTOCLO_DATA_INDEX+1] =  id;
	
	crc = Crc16(&tx_buf[4],18);	
	*(uint16_t*)(&tx_buf[22]) = crc;
	
	return HP_6_SendCmd( tx_buf, rx_buf);      
}

/****************************************************************
** Function name:			HP_6_PowerSaving
** Descriptions:            设置低功耗
** input parameters:
** output parameters:
** Returned value:
** Created by:				WJ              
** Created Date:           	2016-09-02
**-----------------------------------------------------------------------
** Modified by:
** Modified date:
**-----------------------------------------------------------------------
*****************************************************************/ 
uint8_t HP_6_PowerSaving(void)
{
	uint32_t i;
		
	for(i=0; i<PROTOCLO_BUF_MAX_LEN; i++)
	{
		tx_buf[i] = cmd_set_powersaving[i];
	}

	return HP_6_SendCmd( tx_buf, rx_buf);
}

/****************************************************************
** Function name:			HP_6_VersionInfo
** Descriptions:            获取版本信息
** input parameters:
** output parameters:
** Returned value:
** Created by:				WJ              
** Created Date:           	2016-09-02
**-----------------------------------------------------------------------
** Modified by:
** Modified date:
**-----------------------------------------------------------------------
*****************************************************************/  
uint8_t HP_6_VersionInfo(void)
{
	uint32_t i;
	
	for(i=0; i<PROTOCLO_BUF_MAX_LEN; i++)
	{
		tx_buf[i] = cmd_get_version[i];
	}

	return HP_6_SendCmd( tx_buf, rx_buf);	    
}

