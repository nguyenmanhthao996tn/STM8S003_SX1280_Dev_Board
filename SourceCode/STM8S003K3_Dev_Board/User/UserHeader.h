#ifndef __USER_HEADER_H__
#define __USER_HEADER_H__

/* Private defines -----------------------------------------------------------*/
#ifndef NULL
#define NULL 0
#endif /* NULL */

#define DEVICE_ID 0X01

#define I2C_BUFFER_SIZE 8 // Bytes
#define NUMBER_OF_TIMESLOT 300

#define RF_FREQUENCY 2400000000ul // Hz
#define TX_OUTPUT_POWER 13        // dBm
#define RX_TIMEOUT_TICK_SIZE RADIO_TICK_SIZE_1000_US
#define RX_TIMEOUT_VALUE 450 // ms
#define TX_TIMEOUT_VALUE 450 // ms

#define RX_IRQ_MASK (IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT)
#define TX_IRQ_MASK (IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT)

#define DS1307_I2C_ADDRESS 0xD0

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  uint8_t Address;
  I2C_Direction_TypeDef Direction;
  uint8_t numberOfByteToTransmit;
} I2CDataInfo_t;

typedef enum
{
  NONE_FLAG = 0X00,
  TX_DONE,
  RX_DONE,
  TX_TIMEOUT,
  RX_TIMEOUT
} SX1280Flag_t;

typedef enum
{
  NOT_SYNC_STATE = 0x00,
  IDLE_STATE,
  UPLINK_STATE,
  UPLINK_ACK_WAIT_STATE,
  DOWNLINK_STATE,
  ERROR_STATE
} AppState_t;

#endif /* __USER_HEADER_H__ */
