#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "UserHeader.h"
#include "HardwareConfig.h"
#include "SX1280_LoRa_Lite.h"

/* Private function prototypes -----------------------------------------------*/
void Delay(uint16_t nCount);
void GPIO_Config(void);
void ADC_Config(void);
uint16_t ADC_Scan(void);
void DS1307_Reset(void);
void DS1307_SoutConfig(void);
void CLK_Config(void);
void TIM4_Config(void);
void TIM4_Delay(uint32_t ms);
void I2C_Config(void);
void I2C_Write(uint8_t slaveAddress, uint8_t *dataBuffer, uint8_t size);
void I2C_Read(uint8_t slaveAddress, uint8_t *dataBuffer, uint8_t size);
void txDoneIRQ(void);
void rxDoneIRQ(void);
void txTimeoutIRQ(void);
void rxTimeoutIRQ(void);
void SX1280_Config(void);
void SX1280_setListen(void);
void SX1280_setTransmit(uint8_t *payload, uint8_t size);

uint16_t generateEmergencySlot(void);
void SendPacket(void);

#endif /* __MAIN_H__ */
