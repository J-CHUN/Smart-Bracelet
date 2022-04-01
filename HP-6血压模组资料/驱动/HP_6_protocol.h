/*****************************************************************
Copyright: 2012-2016, Veepoo Tech. Co., Ltd.
File name: 		HP_6_protocol.h
Description: 
Author: 			WJ
Version: 
Date: 			2016-09-02
History: 
*****************************************************************/
// IIC 速率使用100K；7bite地址：0x66

#ifndef HP_6_PROTOCOL_H
#define HP_6_PROTOCOL_H

#define HP_6_FAILURE			0x00
#define HP_6_SUCCESS			0x01

uint16_t HP_6_GetResultData(uint8_t *data);

uint8_t HP_6_GetRateResult(void);
uint8_t HP_6_OpenRate(void);
uint8_t HP_6_CloseRate(void);

uint8_t HP_6_GetBpResult(void);
uint8_t HP_6_OpenBp(void);
uint8_t HP_6_CloseBp(void);

uint8_t HP_6_GetADC(uint8_t num, uint8_t id);

uint8_t HP_6_PowerSaving(void);

uint8_t HP_6_VersionInfo(void);

#endif

