/*
 * i2cs.h
 */

#ifndef I2CS_H_
#define I2CS_H_

#ifdef __cplusplus
extern "C" {
#endif

//including core_cm3 directly gives errors
#include "stm32f0xx.h"

// TIMINGR register values
// These values are for Sm mode(100khz) I2C from Reference Manual when I2C clock frequency is 48MHz
// For different configuration refer to Reference Manual for STM32F0
#define PRESC 0x0
#define SCLL 0x13
#define SCLH 0xF
#define SDADEL 0x2
#define SCLDEL 0x4

//max amount of while loop cycles it takes for the loop to finish
//if an amount of loops in any of the while loop exceeds LOOP_TIMEOUT number,
//I2C peripheral is disabled and function returns I2C_ERROR_TIMEOUT
#define LOOP_TIMEOUT 180000U

#define I2C_SUCCESS 0
#define I2C_ERROR_NACK -1 //no acknowledgement from the slave device
#define I2C_ERROR_TIMEOUT -2
#define I2C_ERROR_BUSY -3 //i2c line is busy
#define I2C_ERROR_BUSY_MASTER -4 //i2c line is busy and microprocessor is a master
#define I2C_ERROR_HW -5 //hardware error

void i2cs_init(void);
int i2cs_ping_device(uint8_t address);
int i2cs_receive_data(uint8_t address, uint8_t *return_data, uint16_t data_bytes_count);
int i2cs_receive_data_from_address(uint8_t i2c_address, uint8_t data_address, uint8_t *return_data, uint16_t data_bytes_count);
int i2cs_start_transmission(uint8_t address, uint8_t rw_mode);
int i2cs_end_transmission();
int i2cs_send_byte(uint8_t data_byte);
int i2cs_send_byte_array(const uint8_t* data_ptr, uint8_t listSize);


#ifdef __cplusplus
}
#endif

#endif /* I2CS_H_ */
