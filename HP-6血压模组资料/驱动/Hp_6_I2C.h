
#ifndef HP_6_I2C_H
#define HP_6_I2C_H
extern uint8_t Hp_6_address;
bool Hp_6_I2CWrite(uint8_t Hp_6_address, uint8_t *pData, uint8_t bytes)
bool Hp_6_I2CRead(uint8_t Hp_6_address, uint8_t *pData, uint8_t bytes);
#endif
