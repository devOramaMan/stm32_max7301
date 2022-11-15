/**
  ******************************************************************************
  * @file       max7301.h
  * @brief      Header for max7301.c file.
  * @brief      C library for the MAX7301 gpio expander for STM32 HAL.
  * @see        https://datasheets.maximintegrated.com/en/ds/MAX7301.pdf
  ******************************************************************************
  * @author     Andreas d'Erasmo
  *
  * @date 14.11.2022 Created
  *
  ******************************************************************************
  */

#ifndef MAX7301_H
#define MAX7301_H

#ifdef __cplusplus
extern "C" {
#endif
  
#include "stdint.h"
#include "stm32f4xx_hal.h"
    
/* Exported define -----------------------------------------------------------*/
#define MAX7301_NOP 0x00
#define MAX7301_GPIO_OUTPUT 0x01
#define MAX7301_GPIO_INPUT 0x02
#define MAX7301_GPIO_INPUT_PULLUP 0x03
#define MAX7301_GPIO_LOW 0
#define MAX7301_GPIO_HIGH 1
    
  
/* Exported types ------------------------------------------------------------*/
  
typedef struct
{
  SPI_HandleTypeDef* spiHandle;
  uint32_t csPin;
  GPIO_TypeDef * csPort;
}MAX7301_Handle_t;
  
  
/* Exported functions prototypes ---------------------------------------------*/

void MAX7301_Init(MAX7301_Handle_t *, uint8_t);

uint8_t MAX7301_read(MAX7301_Handle_t *, uint8_t, uint8_t *);

uint8_t MAX7301_write(MAX7301_Handle_t *, uint8_t, uint8_t );

uint8_t MAX7301_enable(MAX7301_Handle_t *);

uint8_t MAX7301_disable(MAX7301_Handle_t *);

uint8_t MAX7301_getPinMode(MAX7301_Handle_t *, uint8_t, uint8_t *);

uint8_t MAX7301_setPinMode(MAX7301_Handle_t * pHandle, uint8_t, uint8_t);

uint8_t MAX7301_digitalRead(MAX7301_Handle_t *, uint8_t, uint8_t *);

uint8_t MAX7301_digitalReadRange(MAX7301_Handle_t *, uint8_t, uint8_t *);

uint8_t MAX7301_digitalWrite(MAX7301_Handle_t *, uint8_t, uint8_t);
         
uint8_t MAX7301_enableTransitionDetection( MAX7301_Handle_t * );

uint8_t MAX7301_disableTransitionDetection( MAX7301_Handle_t * );

uint8_t MAX7301_configureTransitionDetection( MAX7301_Handle_t *, uint8_t, uint8_t );





#ifdef __cplusplus
}
#endif

#endif /* MAX7301_H */
