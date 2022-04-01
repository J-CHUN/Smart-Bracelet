
#include "Hp_6_twi_master.h"
#include <string.h>

bool Hp_6_I2CWrite(uint8_t Hp_6_address, uint8_t *pData, uint8_t bytes)
{
	return hp_6_twi_master_transfer(Hp_6_address<<0x01, pData, bytes, TWI_ISSUE_STOP);	
}

bool Hp_6_I2CRead(uint8_t Hp_6_address, uint8_t *pData, uint8_t bytes)
{
	bool transfer_succeeded;
	transfer_succeeded &= hp_6_twi_master_transfer((Hp_6_address<<0x01)|TWI_READ_BIT, pData, bytes, TWI_ISSUE_STOP);
	return(transfer_succeeded);
}

