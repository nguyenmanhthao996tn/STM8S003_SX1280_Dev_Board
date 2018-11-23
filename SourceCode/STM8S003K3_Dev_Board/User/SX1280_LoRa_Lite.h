#ifndef __SX1280_LORA_LITE_H__
#define __SX1280_LORA_LITE_H__

#include "stm8s.h"
#include "Radio_Header.h"
#include "HardwareConfig.h"

void SX1280_Init(RadioCallbacks_t *callbacks);
void SX1280_WriteCommand(uint8_t command, uint8_t *buffer, uint16_t size);
void SX1280_ReadCommand(uint8_t command, uint8_t *buffer, uint16_t size);
void SX1280_WriteRegister(uint16_t address, uint8_t *buffer, uint16_t size);
void SX1280_ReadRegister(uint16_t address, uint8_t *buffer, uint16_t size);
void SX1280_WriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t size);
void SX1280_ReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t size);
void SX1280_WaitOnBusy(void);
void SX1280_SetRegulatorMode(uint8_t mode);
void SX1280_SetStandby(uint8_t mode);
void SX1280_SetPacketType(uint8_t packetType);
void SX1280_SetModulationParams(ModulationParams_t *modParams);
void SX1280_SetPacketParams(PacketParams_t *packetParams);
void SX1280_SetRfFrequency(uint32_t rfFrequency);
void SX1280_SetBufferBaseAddresses(uint8_t txBaseAddress, uint8_t rxBaseAddress);
void SX1280_SetTxParams(int8_t power, uint8_t rampTime);
uint8_t SX1280_SetSyncWord(uint8_t syncWordIdx, uint8_t *syncWord);
void SX1280_SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);
void SX1280_SetRx(TickTime_t timeout);
void SX1280_ClearIrqStatus(uint16_t irqMask);
uint8_t SX1280_GetPacketType(void);
void SX1280_SendPayload(uint8_t *payload, uint8_t size, TickTime_t timeout, uint8_t offset);
void SX1280_SetPayload(uint8_t *buffer, uint8_t size, uint8_t offset);
void SX1280_SetTx(TickTime_t timeout);
uint8_t SX1280_GetPayload(uint8_t *payload, uint8_t *size, uint8_t maxSize);
void SX1280_GetRxBufferStatus(uint8_t *rxPayloadLength, uint8_t *rxStartBufferPointer);
uint16_t SX1280_GetIrqStatus(void);
void SX1280_ProcessIrqs(void);

#endif /* __SX1280_LORA_LITE_H__ */
