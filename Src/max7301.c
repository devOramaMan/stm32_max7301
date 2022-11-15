/**
  ******************************************************************************
  * @file       max7301.c
  * @brief      C library for the MAX7301 gpio expander for STM32 HAL.
  *             
  * @see        https://datasheets.maximintegrated.com/en/ds/MAX7301.pdf
  ******************************************************************************
  * @author     Andreas d'Erasmo
  *
  * @date 14.11.2022 Created (Ported from ardrino SW packacge)
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "max7301.h"



#include "stdbool.h"
#include "stdint.h"


/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
#define MAX7301_ERR 1
#define MAX7301_OK 0
#define MAX7301_OK_TIMEOUT_VALUE 500
#define MAX7301_SIZE 2
    
    
#define MAX7301_MODE 0x04

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* public code ---------------------------------------------------------------*/
/**
 * @brief  MAX7301_Init - init GPIO Exstender
 *
 * @arg {MAX7301_Handle_t} pinCLK - Clock pin.
 * @arg {uint8_t} x - Select model AAX.
 */
void MAX7301_Init(MAX7301_Handle_t * pHandle, uint8_t x) 
{
  //Set gpio High
  pHandle->csPort->BSRR = (uint32_t) pHandle->csPin;

  if (x)
  {
    // Not model AAX, disable unavailable pins.
    MAX7301_write(pHandle, 0x09, 0x55);
    MAX7301_write(pHandle, 0x0A, 0x55);
  }
}

/**
  * @brief 		MAX7301_read - read GPIO Exstender
  * @param 		*pHandle, pointer to the MAX7301 typedef struct
  * @param 		reg, the reg to modify
  * @param 		*val, Ptr to value set in func
  * @return 	0 on success, 1 on spi transmission error
  */
uint8_t MAX7301_read(MAX7301_Handle_t * pHandle, uint8_t reg , uint8_t * val)
{
  uint8_t ret;
  uint8_t lreg[MAX7301_SIZE];
  uint8_t lval[MAX7301_SIZE] = {0,0};
  
  lreg[0] = (reg | 0x80);
  lreg[1] = MAX7301_NOP;
  
  
  /*    Write the register to read Read the register  */
  pHandle->csPort->BSRR = (pHandle->csPin << 16U);  
  
  if(!HAL_SPI_Transmit(pHandle->spiHandle, lreg, MAX7301_SIZE, MAX7301_OK_TIMEOUT_VALUE))
  {
    ret = MAX7301_OK;
  }
  else
  {
    ret = MAX7301_ERR;
  }
  
  pHandle->csPort->BSRR = (uint32_t) pHandle->csPin;
  
  if(ret)
    return ret;
  
  
  /*    Read the register  */
  pHandle->csPort->BSRR = (pHandle->csPin << 16U);
  
  
  lreg[0] = MAX7301_NOP;
  lreg[1] = MAX7301_NOP;
  
  if(!HAL_SPI_TransmitReceive(pHandle->spiHandle, lreg, lval, MAX7301_SIZE, MAX7301_OK_TIMEOUT_VALUE))
  {
    ret = MAX7301_OK;
    *val = lval[1];
  }
  else
  {
    ret = MAX7301_ERR;
  }
  pHandle->csPort->BSRR = (uint32_t) pHandle->csPin;
  
  
  return ret;
}


/**
  * @brief 		MAX7301_write - write GPIO Exstender
  * @param 		*pHandle, pointer to the MAX7301 typedef struct
  * @param 		reg, the reg to modify
  * @param 		val, Ptr to value to set
  * @return 	0 on success, 1 on spi transmission error
  */
uint8_t MAX7301_write(MAX7301_Handle_t * pHandle, uint8_t reg, uint8_t val)
{
  uint8_t ret;
  uint8_t lregval[MAX7301_SIZE];
  
  lregval[0] = (reg & ~0x80);
  lregval[1] = val;
  
  
  /*    Write the register to read Read the register  */
  
  pHandle->csPort->BSRR = (uint32_t) (pHandle->csPin << 16U);
  
  
  if(!HAL_SPI_Transmit(pHandle->spiHandle, lregval, MAX7301_SIZE, MAX7301_OK_TIMEOUT_VALUE))
  {
    ret = MAX7301_OK;
  }
  else
  {
    ret = MAX7301_ERR;
  }
  
  pHandle->csPort->BSRR = (uint32_t) pHandle->csPin;
  
  return ret;
}

/**
  * @brief 		MAX7301_enable - Set normal operation mode.
  * @param 		*pHandle, pointer to the MAX7301 typedef struct
  * @return 	0 on success, 1 on spi transmission error
  */
uint8_t MAX7301_enable(MAX7301_Handle_t * pHandle)
{
  uint8_t val;
  
  if(MAX7301_read(pHandle,MAX7301_MODE,&val))
  {
    return MAX7301_ERR;
  }
  
  return MAX7301_write(pHandle, MAX7301_MODE, (val | 0x01));
}


/**
  * @brief 		MAX7301_disable - Set shutdown mode.
  * @param 		*pHandle, pointer to the MAX7301 typedef struct
  * @return 	0 on success, 1 on spi transmission error
  */
uint8_t MAX7301_disable(MAX7301_Handle_t * pHandle)
{
  uint8_t val;
  
  if(MAX7301_read(pHandle,MAX7301_MODE,&val))
  {
    return MAX7301_ERR;
  }
  
  return MAX7301_write(pHandle, MAX7301_MODE, (val & ~0x01));
}

/**
  * @brief 		MAX7301_getPinMode - Get pin configuration.
  * @param 		*pHandle, pointer to the MAX7301 typedef struct
  * @param 		pin, GPIO pin number (first 0-3 is virtual)
  * @param 		*val, pointer to set the readback value
  * @return 	0 on success, 1 on spi transmission error
  */
uint8_t  MAX7301_getPinMode(MAX7301_Handle_t * pHandle, uint8_t pin, uint8_t * val)
{
  uint8_t ret = MAX7301_ERR;
  uint8_t lval;
  
  if(!MAX7301_read(pHandle, (pin / 4) + 0x08, &lval))
  {
    *val = (lval >> (2 * (pin % 4))) & 0x03;
    ret = MAX7301_OK;
  }
  return ret;
}

/**
 * @brief 		MAX7301_setPinMode - Configure transition detection mask.
 * @param 		*pHandle, pointer to the MAX7301 typedef struct
 * @param 		pin, GPIO pin number (first 0-3 is virtual)
 * @param 		mode, Pin Configuration
 * @return 	  0 on success, 1 on spi transmission error
 */
uint8_t MAX7301_setPinMode( MAX7301_Handle_t * pHandle, uint8_t pin, uint8_t mode )
{
  uint8_t reg = (pin / 4) + 0x08,
       offset = 2 * (pin % 4);
  uint8_t val;
  
  
  if(MAX7301_read(pHandle, reg, &val))
    return MAX7301_ERR;

  return MAX7301_write(pHandle, reg, val & ~(0x03 << offset) | mode << offset);
}

/**
  * @brief 		MAX7301_digitalRead - Read from a GPIO pin.
  * @param 		*pHandle, pointer to the MAX7301 typedef struct
  * @param 		pin, GPIO pin number (first 0-3 is virtual)
  * @param 		*val, pointer to set the readback value
  * @return 	0 on success, 1 on spi transmission error
  */
uint8_t MAX7301_digitalRead(MAX7301_Handle_t *pHandle, uint8_t pin, uint8_t * val)
{ 
  return MAX7301_read(pHandle, pin + 0x20, val);
}

/**
  * @brief 		MAX7301_digitalReadRange - Read up to 8 consecutive GPIO pins.
  * @param 		*pHandle, pointer to the MAX7301 typedef struct
  * @param 		pin, the first gpio pin number (first 0-3 is virtual)
  * @param 		*val, pointer to set the readback value
  * @return 	0 on success, 1 on spi transmission error
  */
uint8_t MAX7301_digitalReadRange(MAX7301_Handle_t *pHandle, uint8_t pin, uint8_t * val)
{
  return MAX7301_read(pHandle, pin + 0x40, val);
}

/**
  * @brief 		MAX7301_digitalWrite - Write to a GPIO pin.
  * @param 		*pHandle, pointer to the MAX7301 typedef struct
  * @param 		pin, GPIO pin number (first 0-3 is virtual)
  * @param 		data, write data
  * @return 	0 on success, 1 on spi transmission error
  */
uint8_t MAX7301_digitalWrite(MAX7301_Handle_t * pHandle, uint8_t pin, uint8_t data)
{
  return MAX7301_write( pHandle, pin + 0x40, data);
}

/**
  * @brief 		MAX7301_enableTransitionDetection - Enable transition detection.
  * @param 		*pHandle, pointer to the MAX7301 typedef struct
  * @return 	0 on success, 1 on spi transmission error
  */
uint8_t MAX7301_enableTransitionDetection(MAX7301_Handle_t * pHandle) 
{
  uint8_t val;
  if(MAX7301_read(pHandle, 0x04, &val))
    return MAX7301_ERR;
      
  return MAX7301_write(pHandle, 0x04,  val | 0x80);
}

/**
  * @brief 		MAX7301_disableTransitionDetection - Disable transition detection.
  * @param 		*pHandle, pointer to the MAX7301 typedef struct
  * @return 	0 on success, 1 on spi transmission error
  */
uint8_t MAX7301_disableTransitionDetection(MAX7301_Handle_t * pHandle)
{
  uint8_t val;
  if(MAX7301_read(pHandle, 0x04, &val))
    return MAX7301_ERR;
  return MAX7301_write(pHandle, 0x04, val & ~0x80);
}

/**
 * @brief 		MAX7301_configureTransitionDetection - Configure transition detection mask.
 * @param 		*pHandle, pointer to the MAX7301 typedef struct
 * @param 		pin, GPIO pin number (first 0-3 is virtual)
 * @param 		mode, Pin Configuration
 * @return 	  0 on success, 1 on spi transmission error
 */
uint8_t MAX7301_configureTransitionDetection( MAX7301_Handle_t * pHandle, uint8_t pin, uint8_t mode )
{
  uint8_t offset = pin - 24;
  uint8_t val;
  
  if(MAX7301_read(pHandle, 0x06, &val))
    return MAX7301_ERR;

  return MAX7301_write(pHandle, 0x06, val & ~(0x01 << offset) | mode << offset);
}





