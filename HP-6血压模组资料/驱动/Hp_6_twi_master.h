 /* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#ifndef TWI_MASTER_H
#define TWI_MASTER_H

/*lint ++flb "Enter library region" */

#include <stdbool.h>
#include <stdint.h>

#include "hal_i2c.h"
#include "nrf_delay.h"
#include "board.h"
#include "nrf_gpio.h"

/** @file
* @brief Software controlled TWI Master driver.
*
*
* @defgroup lib_driver_twi_master Software controlled TWI Master driver
* @{
* @ingroup nrf_drivers
* @brief Software controlled TWI Master driver.
*
* Supported features:
* - Repeated start
* - No multi-master
* - Only 7-bit addressing
* - Supports clock stretching (with optional SMBus style slave timeout)
* - Tries to handle slaves stuck in the middle of transfer
*/

#define TWI_READ_BIT                 (0x01)        //!< If this bit is set in the address field, transfer direction is from slave to master.

#define TWI_ISSUE_STOP               ((bool)true)  //!< Parameter for @ref twi_master_transfer
#define TWI_DONT_ISSUE_STOP          ((bool)false) //!< Parameter for @ref twi_master_transfer

#define TWI_SCL_HIGH()   do{nrf_gpio_cfg_output(HP_6_SCL);nrf_gpio_pin_set(HP_6_SCL);}while(0)
#define TWI_SCL_LOW()    do{nrf_gpio_cfg_output(HP_6_SCL);nrf_gpio_pin_clear(HP_6_SCL);}while(0)
#define TWI_SDA_HIGH()   do{nrf_gpio_cfg_output(HP_6_SDA);nrf_gpio_pin_set(HP_6_SDA);}while(0)
#define TWI_SDA_LOW()    do{nrf_gpio_cfg_output(HP_6_SDA);nrf_gpio_pin_clear(HP_6_SDA);}while(0)
#define TWI_SDA_INPUT()  do{nrf_gpio_cfg_input(HP_6_SDA, BUTTON_PULL);}while(0)
#define TWI_SDA_OUTPUT() do{nrf_gpio_cfg_output(HP_6_SDA);}while(0)
#define TWI_SCL_OUTPUT() do{nrf_gpio_cfg_output(HP_6_SCL);}while(0)

#define	TWI_DIS_CLK()		do { nrf_gpio_range_cfg_input(HP_6_SCL, HP_6_SCL, NRF_GPIO_PIN_NOPULL);}	while(0)
#define	TWI_DIS_SDA()		do { nrf_gpio_range_cfg_input(HP_6_SDA, HP_6_SDA, NRF_GPIO_PIN_NOPULL);}	while(0)
#define	TWI_DISABLE()		do { TWI_DIS_CLK(); TWI_DIS_SDA();}while(0)

#define TWI_SDA_READ() ((NRF_GPIO->IN >> HP_6_SDA) & 0x1UL)                     /*!< Reads current state of SDA */
#define TWI_SCL_READ() ((NRF_GPIO->IN >> HP_6_SCL) & 0x1UL)                    /*!< Reads current state of SCL */

#define TWI_DELAY() nrf_delay_us(4) /*!< Time to wait when pin states are changed. For fast-mode the delay can be zero and for standard-mode 4 us delay is sufficient. */

/**
 * @brief Function for initializing TWI bus IO pins and checks if the bus is operational.
 *
 * Both pins are configured as Standard-0, No-drive-1 (open drain).
 *
 * @return
 * @retval true TWI bus is clear for transfers.
 * @retval false TWI bus is stuck.
 */
bool hp_6_twi_master_init(void);

/**
 * @brief Function for transferring data over TWI bus.
 *
 * If TWI master detects even one NACK from the slave or timeout occurs, STOP condition is issued
 * and the function returns false.
 * Bit 0 (@ref TWI_READ_BIT) in the address parameter controls transfer direction;
 * - If 1, master reads data_length number of bytes from the slave
 * - If 0, master writes data_length number of bytes to the slave.
 *
 * @note Make sure at least data_length number of bytes is allocated in data if TWI_READ_BIT is set.
 * @note @ref TWI_ISSUE_STOP
 *
 * @param address Data transfer direction (LSB) / Slave address (7 MSBs).
 * @param data Pointer to data.
 * @param data_length Number of bytes to transfer.
 * @param issue_stop_condition If @ref TWI_ISSUE_STOP, STOP condition is issued before exiting function. If @ref TWI_DONT_ISSUE_STOP, STOP condition is not issued before exiting function. If transfer failed for any reason, STOP condition will be issued in any case.
 * @return
 * @retval true Data transfer succeeded without errors.
 * @retval false Data transfer failed.
 */
bool hp_6_twi_master_transfer(uint8_t address, uint8_t *data, uint8_t data_length, bool issue_stop_condition);

/**
 *@}
 **/

/*lint --flb "Leave library region" */
#endif //TWI_MASTER_H
