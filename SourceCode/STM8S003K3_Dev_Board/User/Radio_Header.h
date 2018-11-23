#ifndef __RADIO_HEADER_H__
#define __RADIO_HEADER_H__

#ifndef NULL
#define NULL 0
#endif /* NULL */

/*!
   \brief Provides the frequency of the chip running on the radio and the frequency step

   \remark These defines are used for computing the frequency divider to set the RF frequency
*/
#define XTAL_FREQ 52000000
#define FREQ_STEP ((double)(XTAL_FREQ / 262144.0))

/*!
   \brief The address of the register holding LORA packet parameters
*/
#define REG_LR_PACKETPARAMS 0x903

/*!
   \brief The address of the register holding payload length

   \remark Do NOT try to read it directly. Use GetRxBuffer( ) instead.
*/
#define REG_LR_PAYLOADLENGTH 0x901

/* RadioCommands_t */
#define RADIO_GET_STATUS 0xC0
#define RADIO_WRITE_REGISTER 0x18
#define RADIO_READ_REGISTER 0x19
#define RADIO_WRITE_BUFFER 0x1A
#define RADIO_READ_BUFFER 0x1B
#define RADIO_SET_SLEEP 0x84
#define RADIO_SET_STANDBY 0x80
#define RADIO_SET_FS 0xC1
#define RADIO_SET_TX 0x83
#define RADIO_SET_RX 0x82
#define RADIO_SET_RXDUTYCYCLE 0x94
#define RADIO_SET_CAD 0xC5
#define RADIO_SET_TXCONTINUOUSWAVE 0xD1
#define RADIO_SET_TXCONTINUOUSPREAMBLE 0xD2
#define RADIO_SET_PACKETTYPE 0x8A
#define RADIO_GET_PACKETTYPE 0x03
#define RADIO_SET_RFFREQUENCY 0x86
#define RADIO_SET_TXPARAMS 0x8E
#define RADIO_SET_CADPARAMS 0x88
#define RADIO_SET_BUFFERBASEADDRESS 0x8F
#define RADIO_SET_MODULATIONPARAMS 0x8B
#define RADIO_SET_PACKETPARAMS 0x8C
#define RADIO_GET_RXBUFFERSTATUS 0x17
#define RADIO_GET_PACKETSTATUS 0x1D
#define RADIO_GET_RSSIINST 0x1F
#define RADIO_SET_DIOIRQPARAMS 0x8D
#define RADIO_GET_IRQSTATUS 0x15
#define RADIO_CLR_IRQSTATUS 0x97
#define RADIO_CALIBRATE 0x89
#define RADIO_SET_REGULATORMODE 0x96
#define RADIO_SET_SAVECONTEXT 0xD5
#define RADIO_SET_AUTOTX 0x98
#define RADIO_SET_AUTOFS 0x9E
#define RADIO_SET_LONGPREAMBLE 0x9B
#define RADIO_SET_UARTSPEED 0x9D
#define RADIO_SET_RANGING_ROLE 0xA3

/* RadioRegulatorModes_t */
#define USE_LDO 0x00  //! Use LDO (default value)
#define USE_DCDC 0x01 //! Use DCDC

/* RadioStandbyModes_t */
#define STDBY_RC 0x00
#define STDBY_XOSC 0x01

/* RadioPacketTypes_t */
#define PACKET_TYPE_GFSK 0x00
#define PACKET_TYPE_LORA 0x01
#define PACKET_TYPE_RANGING 0x02
#define PACKET_TYPE_FLRC 0x03
#define PACKET_TYPE_BLE 0x04
#define PACKET_TYPE_NONE 0x0F

/* RadioRampTimes_t */
#define RADIO_RAMP_02_US 0x00
#define RADIO_RAMP_04_US 0x20
#define RADIO_RAMP_06_US 0x40
#define RADIO_RAMP_08_US 0x60
#define RADIO_RAMP_10_US 0x80
#define RADIO_RAMP_12_US 0xA0
#define RADIO_RAMP_16_US 0xC0
#define RADIO_RAMP_20_US 0xE0

/* RadioTickSizes_t */
#define RADIO_TICK_SIZE_0015_US 0x00
#define RADIO_TICK_SIZE_0062_US 0x01
#define RADIO_TICK_SIZE_1000_US 0x02
#define RADIO_TICK_SIZE_4000_US 0x03

/* RadioLoRaSpreadingFactors_t */
#define LORA_SF5 0x50
#define LORA_SF6 0x60
#define LORA_SF7 0x70
#define LORA_SF8 0x80
#define LORA_SF9 0x90
#define LORA_SF10 0xA0
#define LORA_SF11 0xB0
#define LORA_SF12 0xC0

/* RadioLoRaBandwidths_t */
#define LORA_BW_0200 0x34
#define LORA_BW_0400 0x26
#define LORA_BW_0800 0x18
#define LORA_BW_1600 0x0A

/* RadioLoRaCodingRates_t */
#define LORA_CR_4_5 0x01
#define LORA_CR_4_6 0x02
#define LORA_CR_4_7 0x03
#define LORA_CR_4_8 0x04
#define LORA_CR_LI_4_5 0x05
#define LORA_CR_LI_4_6 0x06
#define LORA_CR_LI_4_7 0x07

/* RadioLoRaIQModes_t */
#define LORA_IQ_NORMAL 0x40
#define LORA_IQ_INVERTED 0x00

/* RadioLoRaCrcModes_t */
#define LORA_CRC_ON 0x20  //!< CRC activated
#define LORA_CRC_OFF 0x00 //!< CRC not used

/* RadioLoRaPacketLengthsModes_t */
#define LORA_PACKET_VARIABLE_LENGTH 0x00 //!< The packet is on variable size, header included
#define LORA_PACKET_FIXED_LENGTH 0x80    //!< The packet is known on both sides, no header included in the packet
#define LORA_PACKET_EXPLICIT LORA_PACKET_VARIABLE_LENGTH
#define LORA_PACKET_IMPLICIT LORA_PACKET_FIXED_LENGTH

/* RadioOperatingModes_t */
#define MODE_SLEEP 0x00       //! The radio is in sleep mode
#define MODE_CALIBRATION 0x01 //! The radio is in calibration mode
#define MODE_STDBY_RC 0x02    //! The radio is in standby mode with RC oscillator
#define MODE_STDBY_XOSC 0x03  //! The radio is in standby mode with XOSC oscillator
#define MODE_FS 0x04          //! The radio is in frequency synthesis mode
#define MODE_RX 0x05          //! The radio is in receive mode
#define MODE_TX 0x06          //! The radio is in transmit mode
#define MODE_CAD 0x07         //! The radio is in channel activity detection mode

/* RadioIrqMasks_t */
#define IRQ_RADIO_NONE 0x0000
#define IRQ_TX_DONE 0x0001
#define IRQ_RX_DONE 0x0002
#define IRQ_SYNCWORD_VALID 0x0004
#define IRQ_SYNCWORD_ERROR 0x0008
#define IRQ_HEADER_VALID 0x0010
#define IRQ_HEADER_ERROR 0x0020
#define IRQ_CRC_ERROR 0x0040
#define IRQ_RANGING_SLAVE_RESPONSE_DONE 0x0080
#define IRQ_RANGING_SLAVE_REQUEST_DISCARDED 0x0100
#define IRQ_RANGING_MASTER_RESULT_VALID 0x0200
#define IRQ_RANGING_MASTER_TIMEOUT 0x0400
#define IRQ_RANGING_SLAVE_REQUEST_VALID 0x0800
#define IRQ_CAD_DONE 0x1000
#define IRQ_CAD_DETECTED 0x2000
#define IRQ_RX_TX_TIMEOUT 0x4000
#define IRQ_PREAMBLE_DETECTED 0x8000
#define IRQ_RADIO_ALL 0xFFFF

/* IrqErrorCode_t */
#define IRQ_HEADER_ERROR_CODE 0x00
#define IRQ_SYNCWORD_ERROR_CODE 0x01
#define IRQ_CRC_ERROR_CODE 0x02
#define IRQ_RANGING_ON_LORA_ERROR_CODE 0x03

/* IrqRangingCode_t */
#define IRQ_RANGING_SLAVE_ERROR_CODE 0x00
#define IRQ_RANGING_SLAVE_VALID_CODE 0x01
#define IRQ_RANGING_MASTER_ERROR_CODE 0x02
#define IRQ_RANGING_MASTER_VALID_CODE 0x03

/*!
   \brief The radio callbacks structure
   Holds function pointers to be called on radio interrupts
*/
typedef struct
{
    void (*txDone)(void);                      //!< Pointer to a function run on successful transmission
    void (*rxDone)(void);                      //!< Pointer to a function run on successful reception
    void (*rxSyncWordDone)(void);              //!< Pointer to a function run on successful SyncWord reception
    void (*rxHeaderDone)(void);                //!< Pointer to a function run on successful Header reception
    void (*txTimeout)(void);                   //!< Pointer to a function run on transmission timeout
    void (*rxTimeout)(void);                   //!< Pointer to a function run on reception timeout
    void (*rxError)(uint8_t IrqErrorCode);     //!< Pointer to a function run on reception error
    void (*rangingDone)(uint8_t IrqRangingCode); //!< Pointer to a function run on ranging terminated
    void (*cadDone)(bool cadFlag);             //!< Pointer to a function run on channel activity detected
} RadioCallbacks_t;

/*!
   \brief Represents an amount of time measurable by the radio clock

   @code
   Time = PeriodBase * PeriodBaseCount
   Example:
   PeriodBase = RADIO_TICK_SIZE_4000_US( 4 ms )
   PeriodBaseCount = 1000
   Time = 4e-3 * 1000 = 4 seconds
   @endcode
*/
typedef struct TickTime_s
{
    uint8_t PeriodBase; //!< The base time of ticktime, RadioTickSizes_t
    /*!
     \brief The number of periodBase for ticktime
     Special values are:
         - 0x0000 for single mode
         - 0xFFFF for continuous mode
  */
    uint16_t PeriodBaseCount;
} TickTime_t;

/*!
   \brief The type describing the modulation parameters for every packet types
*/
typedef struct
{
    uint8_t PacketType; //!< Packet to which the modulation parameters are referring to.
    struct
    {
        /*!
       \brief Holds the LORA modulation parameters

       LORA modulation is defined by Spreading Factor (SF), Bandwidth and Coding Rate
    */
        struct
        {
            uint8_t SpreadingFactor; //!< Spreading Factor for the LORA modulation, RadioLoRaSpreadingFactors_t
            uint8_t Bandwidth;       //!< Bandwidth for the LORA modulation, RadioLoRaBandwidths_t
            uint8_t CodingRate;      //!< Coding rate for the LORA modulation, RadioLoRaCodingRates_t
        } LoRa;
    } Params; //!< Holds the modulation parameters structure
} ModulationParams_t;

/*!
   \brief The type describing the packet parameters for every packet types
*/
typedef struct
{
    uint8_t PacketType; //!< Packet to which the packet parameters are referring to.
    struct
    {
        struct
        {
            uint8_t PreambleLength; //!< The preamble length is the number of LORA symbols in the preamble. To set it, use the following formula @code Number of symbols = PreambleLength[3:0] * ( 2^PreambleLength[7:4] ) @endcode
            uint8_t HeaderType;     //!< If the header is explicit, it will be transmitted in the LORA packet. If the header is implicit, it will not be transmitted, RadioLoRaPacketLengthsModes_t
            uint8_t PayloadLength;  //!< Size of the payload in the LORA packet
            uint8_t Crc;            //!< Size of CRC block in LORA packet, RadioLoRaCrcModes_t
            uint8_t InvertIQ;       //!< Allows to swap IQ for LORA packet, RadioLoRaIQModes_t
        } LoRa;
    } Params; //!< Holds the packet parameters structure
} PacketParams_t;

#endif /* __RADIO_HEADER_H__ */
