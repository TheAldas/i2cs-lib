/*
 * i2cs.c
 *
 */

#include "i2cs.h"


void i2cs_init(void)
{
  /* configure ports, clocks and pins for i2c1 communication */

  //DIsable peripheral
  I2C1 -> CR1 &= ~(I2C_CR1_PE_Msk);

  //enable i2c1 PB10 and PB11 pins port, alternative function clock and i2c1
  RCC -> AHBENR |= (RCC_AHBENR_GPIOBEN_Msk);

  //configure i2c1(PB9 and PB8) SDA and SCL pins
  GPIOB -> MODER &= ~(GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER9_Msk);
  GPIOB -> MODER |= ((0b10 << GPIO_MODER_MODER8_Pos) | (0b10 << GPIO_MODER_MODER9_Pos));

  GPIOB -> AFR[1] &= ~(0b1111 << GPIO_AFRH_AFSEL8_Pos | 0b1111 << GPIO_AFRH_AFSEL9_Pos);
  GPIOB -> AFR[1] |= (0b1 << GPIO_AFRH_AFSEL8_Pos | 0b1 << GPIO_AFRH_AFSEL9_Pos);

  RCC -> APB1ENR |= RCC_APB1ENR_I2C1EN_Msk;

  /* configure i2c1 */
  I2C1 -> CR1 |= I2C_CR1_SWRST_Msk;
  I2C1 -> CR1 &= ~(I2C_CR1_SWRST_Msk);

  I2C1 -> TIMINGR = (PRESC << I2C_TIMINGR_PRESC_Pos | SCLDEL << I2C_TIMINGR_SCLDEL_Pos |
      SDADEL << I2C_TIMINGR_SDADEL_Pos | SCLH << I2C_TIMINGR_SDADEL_Pos | SCLL << I2C_TIMINGR_SDADEL_Pos);
  I2C1 -> CR1 |= I2C_CR1_PE_Msk;
}

int i2cs_ping_device(uint8_t address)
{
  int8_t temp = i2cs_start_transmission(address, 0);
  i2cs_end_transmission();
  return temp;
}

//rw_mode: 1 - read; 0 - write;
int i2cs_start_transmission(uint8_t address, uint8_t rw_mode)
{
  I2C1 -> CR1 |= I2C_CR1_PE_Msk;
  //  if((I2C1 -> SR2) & I2C_SR2_BUSY_Msk)
  //    {
  //      if((I2C1 -> SR2) & I2C_SR2_MSL_Msk) continue; //return I2C_ERROR_BUSY_MASTER; //if i2c is busy and in master mode
  //      else return I2C_ERROR_BUSY; //if i2c is busy and microprocessor is not in master mode
  //    }

  I2C1 -> CR2 = 0;
  I2C1 -> CR2 |= (((address << I2C_CR2_SADD_Pos) << 1) | (rw_mode << I2C_CR2_RD_WRN_Pos) | (128 << I2C_CR2_NBYTES_Pos) | I2C_CR2_RELOAD_Msk);

  I2C1 -> CR2 |= I2C_CR2_START_Msk;

  uint32_t timeout_cycles = 0;
  volatile uint32_t status_register = (I2C1 -> ISR);
  while(!(status_register & (I2C_ISR_TXIS_Msk | I2C_ISR_RXNE_Msk)))
    {
      if(++timeout_cycles > LOOP_TIMEOUT || (status_register & I2C_ISR_TIMEOUT_Msk))
	{
	  i2cs_end_transmission();
	  return I2C_ERROR_TIMEOUT;
	}
      else if(status_register & I2C_ISR_NACKF_Msk)
	{
	  return I2C_ERROR_NACK;
	}
      status_register = (I2C1 -> ISR);
    }
  return I2C_SUCCESS;
}

int i2cs_end_transmission()
{
  if(!((I2C1 -> ISR) & I2C_ISR_BUSY_Msk)) return I2C_SUCCESS;

  uint32_t timeout_cycles = 0;
  while(!((I2C1 -> ISR) & (I2C_ISR_NACKF_Msk | I2C_ISR_TXE_Msk)))
    {
      if((++timeout_cycles) > LOOP_TIMEOUT) break;
    }
  I2C1 -> CR2 |= I2C_CR2_STOP_Msk;

  timeout_cycles = 0;
  while((I2C1 -> ISR) & I2C_ISR_BUSY_Msk)
    {
      if(++timeout_cycles > LOOP_TIMEOUT) return I2C_ERROR_TIMEOUT;
    }

  return I2C_SUCCESS;
}

int i2cs_send_byte(uint8_t data_byte)
{
  if(!((I2C1 -> ISR) & I2C_ISR_BUSY_Msk)) return I2C_ERROR_HW;

  uint32_t timeout_cycles = 0;
  volatile uint32_t status_register = (I2C1 -> ISR);
  while(!(status_register & (I2C_ISR_TXIS_Msk)))
    {
      if(++timeout_cycles > LOOP_TIMEOUT || (status_register & I2C_ISR_TIMEOUT_Msk))
	{
	  i2cs_end_transmission();
	  return I2C_ERROR_TIMEOUT;
	}
      else if(status_register & I2C_ISR_NACKF_Msk)
	{
	  i2cs_end_transmission();
	  return I2C_ERROR_NACK;
	}
      status_register = (I2C1 -> ISR);
    }

  I2C1 -> TXDR = data_byte;
  timeout_cycles = 0;
  status_register = (I2C1 -> ISR);
  while(!(status_register & (I2C_ISR_TXIS_Msk | I2C_ISR_TCR)))
    {
      if(++timeout_cycles > LOOP_TIMEOUT || (status_register & I2C_ISR_TIMEOUT_Msk))
	{
	  i2cs_end_transmission();
	  return I2C_ERROR_TIMEOUT;
	}
      else if(status_register & I2C_ISR_NACKF_Msk)
	{
	  i2cs_end_transmission();
	  return I2C_ERROR_NACK;
	}
      status_register = (I2C1 -> ISR);
    }
  if(status_register & I2C_ISR_TCR_Msk) I2C1 ->CR2 |= (255 << I2C_CR2_NBYTES_Pos);
  return I2C_SUCCESS;
}

int i2cs_send_byte_array(const uint8_t* data_ptr, uint8_t listSize)
{
  int16_t status = I2C_SUCCESS;
  while (listSize-- && status == I2C_SUCCESS)
    {
      status = i2cs_send_byte(*data_ptr++);
    }
  if(status != I2C_SUCCESS) i2cs_end_transmission();
  return status;
}

int i2cs_receive_data(uint8_t i2c_address, uint8_t *return_data, uint16_t data_bytes_count)
{
  int8_t status = I2C_SUCCESS;
  status = i2cs_start_transmission(i2c_address, 1);
  if(status != I2C_SUCCESS) return status;
  uint32_t timeout_cycles;
  while(data_bytes_count--)
    {
      timeout_cycles = 0;
      while(!((I2C1 -> ISR) & I2C_ISR_RXNE_Msk))
	{
	  if(timeout_cycles++ > LOOP_TIMEOUT) return I2C_ERROR_TIMEOUT;
	}
      *return_data++ = I2C1->RXDR;
    }
  I2C1->CR2 |= I2C_CR2_STOP_Msk;
  return status;
}

int i2cs_receive_data_from_address(uint8_t i2c_address, uint8_t data_address, uint8_t *return_data, uint16_t data_bytes_count)
{
  int8_t status = I2C_SUCCESS;
  status = i2cs_start_transmission(i2c_address, 0);
  status = i2cs_send_byte(data_address);
  status = i2cs_start_transmission(i2c_address, 1);
  if(status != I2C_SUCCESS) return status;
  uint32_t timeout_cycles;
  while(data_bytes_count--)
    {
      timeout_cycles = 0;
      while(!((I2C1 -> ISR) & I2C_ISR_RXNE_Msk))
	{
	  if(timeout_cycles++ > LOOP_TIMEOUT) return I2C_ERROR_TIMEOUT;
	}
      *return_data++ = I2C1->RXDR;
    }
  I2C1->CR2 |= I2C_CR2_STOP_Msk;
  return status;
}

