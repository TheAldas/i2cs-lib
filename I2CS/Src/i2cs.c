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

  //enable i2c1 pins port, alternative function clock and i2c1
  RCC -> APB2ENR |= (RCC_APB2ENR_IOPBEN_Msk | RCC_APB2ENR_AFIOEN_Msk);
  RCC -> APB1ENR |= RCC_APB1ENR_I2C1EN_Msk;

  //configure i2c1 SDA and SCL pins
  GPIOB -> CRL |= GPIO_CRL_CNF6_Msk | GPIO_CRL_CNF7_Msk | GPIO_CRL_MODE6_Msk | GPIO_CRL_MODE7_Msk;

  /* configure i2c1 */
  I2C1 -> CR1 |= I2C_CR1_SWRST_Msk;
  I2C1 -> CR1 &= ~(I2C_CR1_SWRST_Msk);

  //Set PCLK1 frequency to APB1 freq
  I2C1 -> CR2 &= ~I2C_CR2_FREQ_Msk; //Convert from Hz to MHz and
  I2C1 -> CR2 |= APB1_FREQ/1000000U; //Convert from Hz to MHz and set I2C1_CR2 register's FREQ bits to MHz value
  //CCR - T high = CCR * T PCLK1
  //		T low = CCR * T PCLK1
  // T low should be more than 4.7us for Standard mode i2c
  I2C1 -> CCR = I2C_SPEED_MODE;
  //T Rise = (T RiseMax x F PCLK1) + 1 | T RiseMax = 1us in Sm mode
  //Here maximum rise time is configured for 1us(As in specifications of Sm mode)
  I2C1 -> TRISE = (APB1_FREQ / 1000000U) + 1;
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

  I2C1 -> CR1 |= I2C_CR1_START_Msk;
  uint32_t timeout_cycles = 0;
  while(!((I2C1 -> SR1) & I2C_SR1_SB_Msk))
    {
      if(++timeout_cycles > LOOP_TIMEOUT)
	{
	  i2cs_end_transmission();
	  return I2C_ERROR_TIMEOUT;
	}
    }
  I2C1 -> DR = (address << 1) + rw_mode;

  /* @note: On STM32CubeIDE, when in debugger mode I2C1 or (SR1 and SR2) are in live expressions,
   * 	this while loop never finishes even if the address is sent and acknowledgement is received from a slave
   * 	cause ADDR bit is cleared when SR1 and then SR2 register is read */
  timeout_cycles = 0;
  while(!((I2C1 -> SR1) & (I2C_SR1_ADDR_Msk | I2C_SR1_AF_Msk)))
    {
      if((++timeout_cycles) > LOOP_TIMEOUT) {
	  i2cs_end_transmission();
	  return I2C_ERROR_TIMEOUT;
      }
    }
  if((I2C1 -> SR1) & I2C_SR1_AF_Msk)
    {
      i2cs_end_transmission();
      return I2C_ERROR_NACK;
    }

  //clear ADDR bit in SR1
  volatile uint32_t temp = (I2C1 -> SR1) & (I2C1 -> SR2);

  timeout_cycles = 0;
  while(((I2C1 -> SR1) & I2C_SR1_ADDR_Msk))
    {
      temp = I2C1 -> SR1;
      temp = I2C1 -> SR2;
      if((!((I2C1 -> SR1) & I2C_SR1_ADDR_Msk))) break;
      else if((++timeout_cycles) > LOOP_TIMEOUT)
	{
	  i2cs_end_transmission();
	  return I2C_ERROR_TIMEOUT;
	}
    }
  return I2C_SUCCESS;
}

int i2cs_end_transmission()
{
  if(!((I2C1 -> SR2) & I2C_SR2_BUSY_Msk)) return I2C_SUCCESS;

  uint32_t timeout_cycles = 0;
  while(!((I2C1 -> SR1) & (I2C_SR1_AF_Msk | I2C_SR1_TXE_Msk | I2C_SR1_BTF_Msk)))
    {
      if((++timeout_cycles) > LOOP_TIMEOUT) break;
    }
  I2C1 -> CR1 |= I2C_CR1_STOP_Msk;
  I2C1 -> CR1 &= ~(I2C_CR1_PE_Msk);

  timeout_cycles = 0;
  while((I2C1 -> SR2) & I2C_SR2_BUSY_Msk)
    {
      if(++timeout_cycles > LOOP_TIMEOUT) return I2C_ERROR_TIMEOUT;
    }

  return I2C_SUCCESS;
}

int i2cs_send_byte(uint8_t data_byte)
{
  //return I2C_SUCCESS;
  if(!((I2C1 -> SR2) & I2C_SR2_BUSY_Msk)) return I2C_ERROR_HW;

  uint32_t timeout_cycles = 0;
  while(!((I2C1 -> SR1) & I2C_SR1_TXE_Msk))
    {
      if(++timeout_cycles > LOOP_TIMEOUT)
	{
	  i2cs_end_transmission();
	  return I2C_ERROR_TIMEOUT;
	}
    }

  I2C1 -> DR = data_byte;
  timeout_cycles = 0;
  while(!((I2C1 -> SR1) & I2C_SR1_BTF_Msk))
    {
      if((I2C1 -> SR1) & (I2C_SR1_AF_Msk))
	{
	  i2cs_end_transmission();
	  return I2C_ERROR_NACK;
	}
      else if(++timeout_cycles > LOOP_TIMEOUT)
	{
	  i2cs_end_transmission();
	  return I2C_ERROR_TIMEOUT;
	}

    }
  return I2C_SUCCESS;
}

int i2cs_send_byte_array(const uint8_t* data_ptr, uint8_t listSize)
{
  if(((I2C1 -> SR2) & I2C_SR2_BUSY_Msk) && !((I2C1 -> SR2) & I2C_SR2_MSL_Msk)) return I2C_ERROR_TIMEOUT;

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
  if(data_bytes_count > 1)
    {
      I2C1->CR1 |= (I2C_CR1_ACK_Msk);
    }
  status = i2cs_start_transmission(i2c_address, 1);
  if(status != I2C_SUCCESS) return status;
  uint32_t timeout_cycles;
  while(data_bytes_count > 1)
    {
      timeout_cycles = 0;
      while(!((I2C1 -> SR1) & I2C_SR1_RXNE_Msk))
	{
	  if(timeout_cycles++ > LOOP_TIMEOUT) return I2C_ERROR_TIMEOUT;
	}
      *return_data++ = I2C1->DR;
      data_bytes_count--;
    }
  I2C1->CR1 &= ~(I2C_CR1_ACK_Msk);
  I2C1->CR1 |= I2C_CR1_STOP_Msk;
  timeout_cycles = 0;
  while(!((I2C1->SR1) & I2C_SR1_RXNE))
    {
      if(++timeout_cycles > LOOP_TIMEOUT)
	if(timeout_cycles++ > LOOP_TIMEOUT) return I2C_ERROR_TIMEOUT;
    }

  *return_data++ = I2C1->DR;
  I2C1 -> CR1 |= I2C_CR1_STOP_Msk;
  I2C1 -> CR1 &= ~(I2C_CR1_PE_Msk);
  return status;
}

int i2cs_receive_data_from_address(uint8_t i2c_address, uint8_t data_address, uint8_t *return_data, uint16_t data_bytes_count)
{
  int8_t status = I2C_SUCCESS;
  status = i2cs_start_transmission(i2c_address, 0);
  status = i2cs_send_byte(data_address);
  if(data_bytes_count > 1)
    {
      I2C1->CR1 |= (I2C_CR1_ACK_Msk);
    }
  status = i2cs_start_transmission(i2c_address, 1);
  if(status != I2C_SUCCESS) return status;
  uint32_t timeout_cycles;
  while(data_bytes_count > 1)
    {
      timeout_cycles = 0;
      while(!((I2C1 -> SR1) & I2C_SR1_RXNE_Msk))
	{
	  if(timeout_cycles++ > LOOP_TIMEOUT) return I2C_ERROR_TIMEOUT;
	}
      *return_data++ = I2C1->DR;
      data_bytes_count--;
    }
  I2C1->CR1 &= ~(I2C_CR1_ACK_Msk);
  I2C1->CR1 |= I2C_CR1_STOP_Msk;
  timeout_cycles = 0;
  while(!((I2C1->SR1) & I2C_SR1_RXNE))
    {
      if(timeout_cycles++ > LOOP_TIMEOUT) return I2C_ERROR_TIMEOUT;
    }

  *return_data++ = I2C1->DR;
  I2C1 -> CR1 |= I2C_CR1_STOP_Msk;
  I2C1 -> CR1 &= ~(I2C_CR1_PE_Msk);
  return status;
}

