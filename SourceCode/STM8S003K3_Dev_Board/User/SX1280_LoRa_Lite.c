#include "SX1280_LoRa_Lite.h"
#include "main.h"

static RadioCallbacks_t *__SX1280_RadioCallbacks;
static uint8_t __OperatingMode = MODE_STDBY_RC;
static uint8_t __PacketType = PACKET_TYPE_NONE;
static bool __PollingMode = FALSE;
static bool __IrqState = FALSE;

uint8_t __ReadRegister_1(uint16_t address)
{
    uint8_t reg = 0;
    SX1280_ReadRegister(address, &reg, 1);
    return reg;
}

void SX1280_Init(RadioCallbacks_t *callbacks)
{
    __SX1280_RadioCallbacks = callbacks;

    /********************* GPIO Init *********************/
    // NSS
    GPIO_Init(SX1280_NSS_PORT, (GPIO_Pin_TypeDef)SX1280_NSS_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_WriteHigh(SX1280_NSS_PORT, (GPIO_Pin_TypeDef)SX1280_NSS_PIN);

    // NRESET
    GPIO_Init(SX1280_RESET_PORT, (GPIO_Pin_TypeDef)SX1280_RESET_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_WriteHigh(SX1280_RESET_PORT, (GPIO_Pin_TypeDef)SX1280_RESET_PIN);

    // BUSY
    GPIO_Init(SX1280_BUSY_PORT, (GPIO_Pin_TypeDef)SX1280_BUSY_PIN, GPIO_MODE_IN_PU_NO_IT);

    /********************* Reset *********************/
    GPIO_WriteLow(SX1280_RESET_PORT, (GPIO_Pin_TypeDef)SX1280_RESET_PIN);
    TIM4_Delay(10);
    GPIO_WriteHigh(SX1280_RESET_PORT, (GPIO_Pin_TypeDef)SX1280_RESET_PIN);
    TIM4_Delay(10);

    /********************* SPI Init *********************/
    /* SPI configuration */
    SPI_DeInit();

    /* Initialize SPI in Slave mode  */
    SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_2, SPI_MODE_MASTER, SPI_CLOCKPOLARITY_LOW,
             SPI_CLOCKPHASE_1EDGE, SPI_DATADIRECTION_2LINES_FULLDUPLEX, SPI_NSS_SOFT, (uint8_t)0x07);

    /* Enable the SPI*/
    SPI_Cmd(ENABLE);

    /********************* IoIrqInit *********************/
    disableInterrupts();
    GPIO_Init(SX1280_DIO1_PORT, (GPIO_Pin_TypeDef)SX1280_DIO1_PIN, GPIO_MODE_IN_FL_IT);

    EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_RISE_ONLY);
    EXTI_SetTLISensitivity(EXTI_TLISENSITIVITY_RISE_ONLY);
    enableInterrupts();

    /********************* Wakeup *********************/
    GPIO_WriteLow(SX1280_NSS_PORT, (GPIO_Pin_TypeDef)SX1280_NSS_PIN);
    SPI_SendData(0xC0);
    SPI_SendData(0);
    GPIO_WriteHigh(SX1280_NSS_PORT, (GPIO_Pin_TypeDef)SX1280_NSS_PIN);
    SX1280_WaitOnBusy();

    /********************* SetRegistersDefault *********************/
}

void SX1280_WriteCommand(uint8_t command, uint8_t *buffer, uint16_t size)
{
    SX1280_WaitOnBusy();

    GPIO_WriteLow(SX1280_NSS_PORT, (GPIO_Pin_TypeDef)SX1280_NSS_PIN);
    SPI_SendData((uint8_t)command); // RadioSpi->write((uint8_t)command);
    for (uint16_t i = 0; i < size; i++)
    {
        SPI_SendData(buffer[i]); // RadioSpi->write(buffer[i]);
    }
    GPIO_WriteHigh(SX1280_NSS_PORT, (GPIO_Pin_TypeDef)SX1280_NSS_PIN);

    if (command != RADIO_SET_SLEEP)
    {
        SX1280_WaitOnBusy();
    }
}

void SX1280_ReadCommand(uint8_t command, uint8_t *buffer, uint16_t size)
{
    SX1280_WaitOnBusy();

    GPIO_WriteLow(SX1280_NSS_PORT, (GPIO_Pin_TypeDef)SX1280_NSS_PIN);
    if (command == RADIO_GET_STATUS)
    {
        SPI_SendData((uint8_t)command);
        while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET)
        {
        }
        buffer[0] = SPI_ReceiveData();
        SPI_SendData(0);
        SPI_SendData(0);
    }
    else
    {
        SPI_SendData((uint8_t)command);
        SPI_SendData(0);
        for (uint16_t i = 0; i < size; i++)
        {
            SPI_SendData(0);
            while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET)
            {
            }
            buffer[i] = SPI_ReceiveData();
        }
    }
    GPIO_WriteHigh(SX1280_NSS_PORT, (GPIO_Pin_TypeDef)SX1280_NSS_PIN);

    SX1280_WaitOnBusy();
}

void SX1280_WriteRegister(uint16_t address, uint8_t *buffer, uint16_t size)
{
    SX1280_WaitOnBusy();

    GPIO_WriteLow(SX1280_NSS_PORT, (GPIO_Pin_TypeDef)SX1280_NSS_PIN);
    SPI_SendData(RADIO_WRITE_REGISTER);    // RadioSpi->write( RADIO_WRITE_REGISTER );
    SPI_SendData((address & 0xFF00) >> 8); // RadioSpi->write( ( address & 0xFF00 ) >> 8 );
    SPI_SendData(address & 0x00FF);        // RadioSpi->write( address & 0x00FF );
    for (uint16_t i = 0; i < size; i++)
    {
        SPI_SendData(buffer[i]); // RadioSpi->write( buffer[i] );
    }
    GPIO_WriteHigh(SX1280_NSS_PORT, (GPIO_Pin_TypeDef)SX1280_NSS_PIN);

    SX1280_WaitOnBusy();
}

void SX1280_ReadRegister(uint16_t address, uint8_t *buffer, uint16_t size)
{
    SX1280_WaitOnBusy();

    GPIO_WriteLow(SX1280_NSS_PORT, (GPIO_Pin_TypeDef)SX1280_NSS_PIN);
    SPI_SendData(0x19);
    SPI_SendData((address & 0xFF00) >> 8); // RadioSpi->write( ( address & 0xFF00 ) >> 8 );
    SPI_SendData(address & 0x00FF);        // RadioSpi->write( address & 0x00FF );
    SPI_SendData(0);                       // RadioSpi->write( 0 );
    for (uint16_t i = 0; i < size; i++)
    {
        SPI_SendData(0);
        while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET)
        {
        }
        buffer[i] = SPI_ReceiveData();
    }
    GPIO_WriteHigh(SX1280_NSS_PORT, (GPIO_Pin_TypeDef)SX1280_NSS_PIN);

    SX1280_WaitOnBusy();
}

void SX1280_WriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t size)
{
    SX1280_WaitOnBusy();

    GPIO_WriteLow(SX1280_NSS_PORT, (GPIO_Pin_TypeDef)SX1280_NSS_PIN);
    SPI_SendData(RADIO_WRITE_BUFFER); // RadioSpi->write( RADIO_WRITE_BUFFER );
    SPI_SendData(offset);             // RadioSpi->write( offset );
    for (uint16_t i = 0; i < size; i++)
    {
        SPI_SendData(buffer[i]); // RadioSpi->write( buffer[i] );
    }
    GPIO_WriteHigh(SX1280_NSS_PORT, (GPIO_Pin_TypeDef)SX1280_NSS_PIN);

    SX1280_WaitOnBusy();
}

void SX1280_ReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t size)
{
    SX1280_WaitOnBusy();

    GPIO_WriteLow(SX1280_NSS_PORT, (GPIO_Pin_TypeDef)SX1280_NSS_PIN);
    SPI_SendData(RADIO_READ_BUFFER); // RadioSpi->write( RADIO_READ_BUFFER );
    SPI_SendData(offset);            // RadioSpi->write( offset );
    SPI_SendData(0);                 // RadioSpi->write( 0 );
    for (uint16_t i = 0; i < size; i++)
    {
        SPI_SendData(0);
        while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET)
        {
        }
        buffer[i] = SPI_ReceiveData(); // buffer[i] = RadioSpi->write( 0 );
    }
    GPIO_WriteHigh(SX1280_NSS_PORT, (GPIO_Pin_TypeDef)SX1280_NSS_PIN);

    SX1280_WaitOnBusy();
}

void SX1280_WaitOnBusy(void)
{
    uint8_t returnValue = GPIO_ReadInputData(SX1280_BUSY_PORT);
    while ((returnValue & SX1280_BUSY_PIN) != 0)
    {
        returnValue = GPIO_ReadInputData(SX1280_BUSY_PORT);
    }
}

void SX1280_SetRegulatorMode(uint8_t mode)
{
    SX1280_WriteCommand(RADIO_SET_REGULATORMODE, (uint8_t *)&mode, 1);
}

void SX1280_SetStandby(uint8_t standbyConfig)
{
    SX1280_WriteCommand(RADIO_SET_STANDBY, (uint8_t *)&standbyConfig, 1);
    if (standbyConfig == STDBY_RC)
    {
        __OperatingMode = MODE_STDBY_RC;
    }
    else
    {
        __OperatingMode = MODE_STDBY_XOSC;
    }
}

void SX1280_SetPacketType(uint8_t packetType)
{
    // Save packet type internally to avoid questioning the radio
    __PacketType = packetType;

    SX1280_WriteCommand(RADIO_SET_PACKETTYPE, (uint8_t *)&packetType, 1);
}

void SX1280_SetModulationParams(ModulationParams_t *modParams)
{
    uint8_t buf[3];

    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    if (__PacketType != modParams->PacketType)
    {
        SX1280_SetPacketType(modParams->PacketType);
    }

    if (modParams->PacketType == PACKET_TYPE_LORA)
    {
        buf[0] = modParams->Params.LoRa.SpreadingFactor;
        buf[1] = modParams->Params.LoRa.Bandwidth;
        buf[2] = modParams->Params.LoRa.CodingRate;
    }
    else
    {
        buf[0] = 0;
        buf[1] = 0;
        buf[2] = 0;
    }
    SX1280_WriteCommand(RADIO_SET_MODULATIONPARAMS, buf, 3);
}

void SX1280_SetPacketParams(PacketParams_t *packetParams)
{
    uint8_t buf[7];
    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    if (__PacketType != packetParams->PacketType)
    {
        SX1280_SetPacketType(packetParams->PacketType);
    }

    if (packetParams->PacketType == PACKET_TYPE_LORA)
    {
        buf[0] = packetParams->Params.LoRa.PreambleLength;
        buf[1] = packetParams->Params.LoRa.HeaderType;
        buf[2] = packetParams->Params.LoRa.PayloadLength;
        buf[3] = packetParams->Params.LoRa.Crc;
        buf[4] = packetParams->Params.LoRa.InvertIQ;
        buf[5] = 0;
        buf[6] = 0;
    }
    else
    {
        buf[0] = 0;
        buf[1] = 0;
        buf[2] = 0;
        buf[3] = 0;
        buf[4] = 0;
        buf[5] = 0;
        buf[6] = 0;
    }

    SX1280_WriteCommand(RADIO_SET_PACKETPARAMS, buf, 7);
}

void SX1280_SetRfFrequency(uint32_t rfFrequency)
{
    uint8_t buf[3];
    uint32_t freq = 0;

    freq = (uint32_t)((double)rfFrequency / (double)FREQ_STEP);
    buf[0] = (uint8_t)((freq >> 16) & 0xFF);
    buf[1] = (uint8_t)((freq >> 8) & 0xFF);
    buf[2] = (uint8_t)(freq & 0xFF);
    SX1280_WriteCommand(RADIO_SET_RFFREQUENCY, buf, 3);
}

void SX1280_SetBufferBaseAddresses(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
    uint8_t buf[2];

    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;
    SX1280_WriteCommand(RADIO_SET_BUFFERBASEADDRESS, buf, 2);
}

void SX1280_SetTxParams(int8_t power, uint8_t rampTime)
{
    uint8_t buf[2];

    // The power value to send on SPI/UART is in the range [0..31] and the
    // physical output power is in the range [-18..13]dBm
    buf[0] = power + 18;
    buf[1] = (uint8_t)rampTime;
    SX1280_WriteCommand(RADIO_SET_TXPARAMS, buf, 2);
}

uint8_t SX1280_SetSyncWord(uint8_t syncWordIdx, uint8_t *syncWord)
{
    // Do NOTHING for LoRa Packet
    return 1;
}

void SX1280_SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
    uint8_t buf[8];

    buf[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)(irqMask & 0x00FF);
    buf[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
    buf[3] = (uint8_t)(dio1Mask & 0x00FF);
    buf[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
    buf[5] = (uint8_t)(dio2Mask & 0x00FF);
    buf[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
    buf[7] = (uint8_t)(dio3Mask & 0x00FF);
    SX1280_WriteCommand(RADIO_SET_DIOIRQPARAMS, buf, 8);
}

void SX1280_SetRx(TickTime_t timeout)
{
    uint8_t buf[3];
    buf[0] = timeout.PeriodBase;
    buf[1] = (uint8_t)((timeout.PeriodBaseCount >> 8) & 0x00FF);
    buf[2] = (uint8_t)(timeout.PeriodBaseCount & 0x00FF);

    SX1280_ClearIrqStatus(IRQ_RADIO_ALL);

    SX1280_WriteCommand(RADIO_SET_RX, buf, 3);
    __OperatingMode = MODE_RX;
}

void SX1280_ClearIrqStatus(uint16_t irqMask)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(((uint16_t)irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)((uint16_t)irqMask & 0x00FF);
    SX1280_WriteCommand(RADIO_CLR_IRQSTATUS, buf, 2);
}

uint8_t SX1280_GetPacketType(void)
{
    return __PacketType;
}

void SX1280_SendPayload(uint8_t *payload, uint8_t size, TickTime_t timeout, uint8_t offset)
{
    SX1280_SetPayload(payload, size, offset);
    SX1280_SetTx(timeout);
}

void SX1280_SetPayload(uint8_t *buffer, uint8_t size, uint8_t offset)
{
    SX1280_WriteBuffer(offset, buffer, size);
}

void SX1280_SetTx(TickTime_t timeout)
{
    uint8_t buf[3];
    buf[0] = timeout.PeriodBase;
    buf[1] = (uint8_t)((timeout.PeriodBaseCount >> 8) & 0x00FF);
    buf[2] = (uint8_t)(timeout.PeriodBaseCount & 0x00FF);

    SX1280_ClearIrqStatus(IRQ_RADIO_ALL);

    SX1280_WriteCommand(RADIO_SET_TX, buf, 3);
    __OperatingMode = MODE_TX;
}

uint8_t SX1280_GetPayload(uint8_t *buffer, uint8_t *size, uint8_t maxSize)
{
    uint8_t offset;

    SX1280_GetRxBufferStatus(size, &offset);
    if (*size > maxSize)
    {
        return 1;
    }
    SX1280_ReadBuffer(offset, buffer, *size);
    return 0;
}

void SX1280_GetRxBufferStatus(uint8_t *rxPayloadLength, uint8_t *rxStartBufferPointer)
{
    uint8_t status[2];

    SX1280_ReadCommand(RADIO_GET_RXBUFFERSTATUS, status, 2);

    // In case of LORA fixed header, the rxPayloadLength is obtained by reading
    // the register REG_LR_PAYLOADLENGTH
    if ((SX1280_GetPacketType() == PACKET_TYPE_LORA) && (__ReadRegister_1(REG_LR_PACKETPARAMS) >> 7 == 1))
    {
        *rxPayloadLength = __ReadRegister_1(REG_LR_PAYLOADLENGTH);
    }
    else
    {
        *rxPayloadLength = status[0];
    }

    *rxStartBufferPointer = status[1];
}

uint16_t SX1280_GetIrqStatus(void)
{
    uint8_t irqStatus[2];
    SX1280_ReadCommand(RADIO_GET_IRQSTATUS, irqStatus, 2);
    return (irqStatus[0] << 8) | irqStatus[1];
}

void SX1280_ProcessIrqs(void)
{
    uint8_t packetType = PACKET_TYPE_NONE;

    if (__PollingMode == TRUE)
    {
        if (__IrqState == TRUE)
        {
            // __disable_irq( );
            __IrqState = FALSE;
            // __enable_irq( );
        }
        else
        {
            return;
        }
    }

    if (__SX1280_RadioCallbacks == NULL)
    {
        return;
    }

    packetType = SX1280_GetPacketType();
    uint16_t irqRegs = SX1280_GetIrqStatus();
    SX1280_ClearIrqStatus(IRQ_RADIO_ALL);

    switch (packetType)
    {
    case PACKET_TYPE_LORA:
        switch (__OperatingMode)
        {
        case MODE_RX:
            if ((irqRegs & IRQ_RX_DONE) == IRQ_RX_DONE)
            {
                if ((irqRegs & IRQ_CRC_ERROR) == IRQ_CRC_ERROR)
                {
                    if (__SX1280_RadioCallbacks->rxError != NULL)
                    {
                        __SX1280_RadioCallbacks->rxError(IRQ_CRC_ERROR_CODE);
                    }
                }
                else
                {
                    if (__SX1280_RadioCallbacks->rxDone != NULL)
                    {
                        __SX1280_RadioCallbacks->rxDone();
                    }
                }
            }
            if ((irqRegs & IRQ_HEADER_VALID) == IRQ_HEADER_VALID)
            {
                if (__SX1280_RadioCallbacks->rxHeaderDone != NULL)
                {
                    __SX1280_RadioCallbacks->rxHeaderDone();
                }
            }
            if ((irqRegs & IRQ_HEADER_ERROR) == IRQ_HEADER_ERROR)
            {
                if (__SX1280_RadioCallbacks->rxError != NULL)
                {
                    __SX1280_RadioCallbacks->rxError(IRQ_HEADER_ERROR_CODE);
                }
            }
            if ((irqRegs & IRQ_RX_TX_TIMEOUT) == IRQ_RX_TX_TIMEOUT)
            {
                if (__SX1280_RadioCallbacks->rxTimeout != NULL)
                {
                    __SX1280_RadioCallbacks->rxTimeout();
                }
            }
            if ((irqRegs & IRQ_RANGING_SLAVE_REQUEST_DISCARDED) == IRQ_RANGING_SLAVE_REQUEST_DISCARDED)
            {
                if (__SX1280_RadioCallbacks->rxError != NULL)
                {
                    __SX1280_RadioCallbacks->rxError(IRQ_RANGING_ON_LORA_ERROR_CODE);
                }
            }
            break;
        case MODE_TX:
            if ((irqRegs & IRQ_TX_DONE) == IRQ_TX_DONE)
            {
                if (__SX1280_RadioCallbacks->txDone != NULL)
                {
                    __SX1280_RadioCallbacks->txDone();
                }
            }
            if ((irqRegs & IRQ_RX_TX_TIMEOUT) == IRQ_RX_TX_TIMEOUT)
            {
                if (__SX1280_RadioCallbacks->txTimeout != NULL)
                {
                    __SX1280_RadioCallbacks->txTimeout();
                }
            }
            break;
        case MODE_CAD:
            if ((irqRegs & IRQ_CAD_DONE) == IRQ_CAD_DONE)
            {
                if ((irqRegs & IRQ_CAD_DETECTED) == IRQ_CAD_DETECTED)
                {
                    if (__SX1280_RadioCallbacks->cadDone != NULL)
                    {
                        __SX1280_RadioCallbacks->cadDone(TRUE);
                    }
                }
                else
                {
                    if (__SX1280_RadioCallbacks->cadDone != NULL)
                    {
                        __SX1280_RadioCallbacks->cadDone(FALSE);
                    }
                }
            }
            else if ((irqRegs & IRQ_RX_TX_TIMEOUT) == IRQ_RX_TX_TIMEOUT)
            {
                if (__SX1280_RadioCallbacks->rxTimeout != NULL)
                {
                    __SX1280_RadioCallbacks->rxTimeout();
                }
            }
            break;
        default:
            // Unexpected IRQ: silently returns
            break;
        }
        break;
    default:
        // Unexpected IRQ: silently returns
        break;
    }
}
