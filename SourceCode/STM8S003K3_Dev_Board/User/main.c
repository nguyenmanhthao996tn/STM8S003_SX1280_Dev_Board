/**
  ******************************************************************************
  * @file    Project/main.c 
  * @author  MCD Application Team
  * @version V2.3.0
  * @date    16-June-2017
  * @brief   Main program body
   ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private variables ---------------------------------------------------------*/
__IO uint32_t time4Counter = 0;

I2CDataInfo_t i2cDataInfo;
uint8_t __IO i2cBuffer[I2C_BUFFER_SIZE];

RadioCallbacks_t callbacks = {
    txDoneIRQ,
    rxDoneIRQ,
    NULL,
    NULL,
    txTimeoutIRQ,
    rxTimeoutIRQ,
    NULL,
    NULL,
    NULL};
bool receiveTimeSyncPacketFlag = FALSE;
SX1280Flag_t SX1280Flag = NONE_FLAG;

static AppState_t AppState = NOT_SYNC_STATE;
static uint16_t slot = 0;
__IO uint16_t currentTimeSlot = 0;
__IO uint16_t lastTimeSync = 0;
static uint8_t payload[2] = {DEVICE_ID, 0};

/* Private functions ---------------------------------------------------------*/

void main(void)
{
  uint32_t adcRandomDelayMs;

  CLK_Config();
  GPIO_Config();
  ADC_Config();
  I2C_Config();
  TIM4_Config();

  TIM4_Delay(100);

  SX1280_Config();
  SX1280_setListen();

  DS1307_Reset();
  DS1307_SoutConfig();

  /* Infinite loop */
  while (1)
  {
    if (receiveTimeSyncPacketFlag == TRUE)
    {
      receiveTimeSyncPacketFlag = FALSE;
      lastTimeSync = 0;
      DS1307_Reset();
      if (AppState == NOT_SYNC_STATE)
      {
        AppState = IDLE_STATE;
      }
    }

    switch (AppState)
    {
    case NOT_SYNC_STATE:
      // Do nothing
      break;
    case IDLE_STATE:
      if (currentTimeSlot == 0)
      {
        if (lastTimeSync > (uint16_t)(1.1 * NUMBER_OF_TIMESLOT))
        {
          AppState = NOT_SYNC_STATE;
        }
      }
      break;
    case UPLINK_STATE:
      if (slot == currentTimeSlot)
      {
        // Wait a random time
        adcRandomDelayMs = ADC_Scan();
        adcRandomDelayMs = (adcRandomDelayMs * 100) / 1024;
        TIM4_Delay(adcRandomDelayMs);

        // Send Packet
        SX1280_setTransmit(payload, 2);

        // Move to UPLINK_ACK_WAIT_STATE
        SX1280Flag = NONE_FLAG;
        AppState = UPLINK_ACK_WAIT_STATE;
      }
      break;
    case UPLINK_ACK_WAIT_STATE:
      if (SX1280Flag == TX_DONE)
      {
        SX1280_setListen();
        AppState = DOWNLINK_STATE;
      }
      else if (SX1280Flag == TX_TIMEOUT)
      {
        AppState = ERROR_STATE;
        payload[1]++; // Increase counter
      }
      break;
    case DOWNLINK_STATE:
      if (slot == currentTimeSlot)
      {
        if (SX1280Flag == RX_DONE)
        {
          // Process DOWNLINK packet
          AppState = IDLE_STATE;
        }
        else if (SX1280Flag == RX_TIMEOUT)
        {
          AppState = ERROR_STATE;
          payload[1]++; // Increase counter
        }
      }
      else
      {
        AppState = ERROR_STATE;
        payload[1]++; // Increase counter
      }
      break;
    case ERROR_STATE:
      slot = generateEmergencySlot();
      AppState = UPLINK_STATE;
      break;
    default:
      AppState = NOT_SYNC_STATE;
      break;
    }
  }
}

void Delay(uint16_t nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
}

void GPIO_Config(void)
{
  // LED
  GPIO_Init(LED_USR_PORT, (GPIO_Pin_TypeDef)LED_USR_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
}

void ADC_Config(void)
{
  /*  Init GPIO for ADC1 */
  GPIO_Init(GPIOB, GPIO_PIN_0, GPIO_MODE_IN_FL_NO_IT);

  /* De-Init ADC peripheral*/
  ADC1_DeInit();

  /* Init ADC1 peripheral */
  ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, ADC1_CHANNEL_0, ADC1_PRESSEL_FCPU_D2,
            ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_CHANNEL0,
            DISABLE);
}

uint16_t ADC_Scan(void)
{
  uint16_t result = 0;
  ADC1_StartConversion();
  while (ADC1_GetFlagStatus(ADC1_FLAG_EOC) == 0x00)
  {
  }
  result = ADC1_GetConversionValue();
  ADC1_ClearFlag(ADC1_FLAG_EOC);

  return result;
}

void DS1307_Reset(void)
{
  uint8_t ds1307Registers[3] = {0x00, 0x00, 0x00};

  // Reset clock register and set SOUT configuration
  ds1307Registers[0] = 0x07;
  ds1307Registers[1] = 0x10;                         // OUT = 0, SQWE = 1, RS1 = 0, RS0 = 0: 1Hz SOUT
  I2C_Write(DS1307_I2C_ADDRESS, ds1307Registers, 2); // Address
  TIM4_Delay(5);

  // Reset Second register & minute register
  ds1307Registers[0] = 0x00;
  ds1307Registers[1] = 0x00;
  I2C_Write(DS1307_I2C_ADDRESS, ds1307Registers, 3);
  TIM4_Delay(5);

  // Start DS1307
  ds1307Registers[0] = 0x00;
  ds1307Registers[1] = 0x00;                         // Clear CH bit
  I2C_Write(DS1307_I2C_ADDRESS, ds1307Registers, 2); // Address
  TIM4_Delay(5);
}

void DS1307_SoutConfig(void)
{
  // DS1307 SOUT Pin
  disableInterrupts();

  GPIO_Init(DS1307_SOUT_PORT, (GPIO_Pin_TypeDef)DS1307_SOUT_PIN, GPIO_MODE_IN_FL_IT);

  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_ONLY);
  EXTI_SetTLISensitivity(EXTI_TLISENSITIVITY_RISE_ONLY);

  /* enable interrupts */
  enableInterrupts();
}

void CLK_Config(void)
{
  /* Initialization of the clock */
  /* Clock divider to HSI/1 */
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
}

void TIM4_Config(void)
{
  /* TIM4 configuration:
   - TIM4CLK is set to 16 MHz, the TIM4 Prescaler is equal to 128 so the TIM1 counter
   clock used is 16 MHz / 128 = 125 000 Hz
  - With 125 000 Hz we can generate time base:
      max time base is 2.048 ms if TIM4_PERIOD = 255 --> (255 + 1) / 125000 = 2.048 ms
      min time base is 0.016 ms if TIM4_PERIOD = 1   --> (  1 + 1) / 125000 = 0.016 ms
  - In this example we need to generate a time base equal to 1 ms
   so TIM4_PERIOD = (0.001 * 125000 - 1) = 124 */

  /* Time base configuration */
  TIM4_TimeBaseInit(TIM4_PRESCALER_128, 124);
  /* Clear TIM4 update flag */
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);
  /* Enable update interrupt */
  TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);

  /* enable interrupts */
  enableInterrupts();

  /* Enable TIM4 */
  TIM4_Cmd(ENABLE);
}

void TIM4_Delay(uint32_t ms)
{
  time4Counter = ms;
  while (time4Counter > 0)
    ;
}

void I2C_Config(void)
{
  /* I2C Initialize */
  I2C_Init(100000, 0x52, I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT, 16);

  /* Enable Buffer and Event Interrupt*/
  I2C_ITConfig((I2C_IT_TypeDef)(I2C_IT_EVT | I2C_IT_BUF), ENABLE);

  /* enable interrupts */
  enableInterrupts();
}

void I2C_Write(uint8_t slaveAddress, uint8_t *dataBuffer, uint8_t size)
{
  uint8_t i = 0;

  i2cDataInfo.Address = slaveAddress;
  i2cDataInfo.Direction = I2C_DIRECTION_TX;
  i2cDataInfo.numberOfByteToTransmit = size;

  for (; i < size; i++)
  {
    i2cBuffer[i] = dataBuffer[i];
  }

  /* Enable Buffer and Event Interrupt*/
  I2C_ITConfig((I2C_IT_TypeDef)(I2C_IT_EVT | I2C_IT_BUF), ENABLE);

  /*  Wait while the bus is busy */
  while (I2C_GetFlagStatus(I2C_FLAG_BUSBUSY))
    ;

  /* Send START condition */
  I2C_GenerateSTART(ENABLE);
}

void I2C_Read(uint8_t slaveAddress, uint8_t *dataBuffer, uint8_t size)
{
  uint8_t bufferIndex = 0, numberOfByteToRead = size;

  /*  Wait while the bus is busy */
  while (I2C_GetFlagStatus(I2C_FLAG_BUSBUSY))
    ;

  /* Send START condition */
  I2C_GenerateSTART(ENABLE);

  /* Test on EV5 and clear it */
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT))
    ;

  /* Send slave Address for write */
  I2C_Send7bitAddress(slaveAddress, I2C_DIRECTION_RX);

  /* Test on EV6 and clear it */
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    ;

  /* While there is data to be read */
  while (numberOfByteToRead)
  {
    if (numberOfByteToRead != 3) /* Receive bytes from first byte until byte N-3 */
    {
      while ((I2C_GetFlagStatus(I2C_FLAG_TRANSFERFINISHED) == RESET))
        ; /* Poll on BTF */

      /* Read a byte from the Slave */
      dataBuffer[bufferIndex] = I2C_ReceiveData();

      /* Point to the next location where the byte read will be saved */
      bufferIndex++;

      /* Decrement the read bytes counter */
      numberOfByteToRead--;
    }

    if (numberOfByteToRead == 3) /* it remains to read three data: data N-2, data N-1, Data N */
    {
      /* Data N-2 in DR and data N -1 in shift register */
      while ((I2C_GetFlagStatus(I2C_FLAG_TRANSFERFINISHED) == RESET))
        ; /* Poll on BTF */

      /* Clear ACK */
      I2C_AcknowledgeConfig(I2C_ACK_NONE);

      /* Disable general interrupts */
      disableInterrupts();

      /* Read Data N-2 */
      dataBuffer[bufferIndex] = I2C_ReceiveData();

      /* Point to the next location where the byte read will be saved */
      bufferIndex++;

      /* Program the STOP */
      I2C_GenerateSTOP(ENABLE);

      /* Read DataN-1 */
      dataBuffer[bufferIndex] = I2C_ReceiveData();

      /* Enable General interrupts */
      enableInterrupts();

      /* Point to the next location where the byte read will be saved */
      bufferIndex++;

      while ((I2C_GetFlagStatus(I2C_FLAG_RXNOTEMPTY) == RESET))
        ; /* Poll on RxNE */

      /* Read DataN */
      dataBuffer[bufferIndex] = I2C_ReceiveData();

      /* Reset the number of bytes to be read by master */
      numberOfByteToRead = 0;
    }
  }
}

void txDoneIRQ(void)
{
  SX1280Flag = TX_DONE;
}

void rxDoneIRQ(void)
{
  uint8_t size, offset;

  SX1280Flag = RX_DONE;

  SX1280_GetRxBufferStatus(&size, &offset);
  if (size == 1)
  {
    receiveTimeSyncPacketFlag = TRUE;
  }
}

void txTimeoutIRQ(void)
{
  SX1280Flag = TX_TIMEOUT;
}

void rxTimeoutIRQ(void)
{
  SX1280Flag = RX_TIMEOUT;
}

void SX1280_Config(void)
{
  PacketParams_t packetParams = {PACKET_TYPE_LORA, {{12, LORA_PACKET_EXPLICIT, 253, LORA_CRC_ON, LORA_IQ_NORMAL}}};
  ModulationParams_t modulationParams = {PACKET_TYPE_LORA, {{LORA_SF8, LORA_BW_0200, LORA_CR_LI_4_5}}};

  SX1280_Init(&callbacks);
  SX1280_SetRegulatorMode(USE_DCDC);
  SX1280_SetStandby(STDBY_RC);
  SX1280_SetPacketType(modulationParams.PacketType);
  SX1280_SetModulationParams(&modulationParams);
  SX1280_SetPacketParams(&packetParams);
  SX1280_SetRfFrequency(RF_FREQUENCY);
  SX1280_SetBufferBaseAddresses(0x00, 0x00);
  SX1280_SetTxParams(TX_OUTPUT_POWER, RADIO_RAMP_20_US);
}

void SX1280_setListen(void)
{
  SX1280_SetDioIrqParams(RX_IRQ_MASK, RX_IRQ_MASK, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
  SX1280_SetRx((TickTime_t){
      RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE});
}

void SX1280_setTransmit(uint8_t *payload, uint8_t size)
{
  SX1280_SetDioIrqParams(TX_IRQ_MASK, TX_IRQ_MASK, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
  SX1280_SendPayload(payload, size, (TickTime_t){RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE}, 0);
}

uint16_t generateEmergencySlot(void)
{
  return 300;
}

void SendPacket(void)
{
  payload[0] = DEVICE_ID;
  payload[1] = 0;
  AppState = UPLINK_STATE;
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8 *file, u32 line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
