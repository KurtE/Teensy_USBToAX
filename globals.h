#ifndef _GLOBALS_H_
#define _GLOBALS_H_

//==================================================================
// Defines to include or not include options
//==================================================================
//#define USE_LSM9DS1

//==================================================================
// Defines 
//==================================================================
#define HWSERIAL Serial1
//#define DBGSerial Serial

#define HWSerial_TXPIN    8       // hack when we turn off TX pin turns to normal IO, try to set high...
#define PCSerial Serial   // Default to USB
#define PCSerial_USB    // Is the PCSerial going to USB?
#define SERVO_DIRECTION_PIN -1
#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif
#define   LED_PIN           11

#define BUFFER_TO_USB
#ifdef BUFFER_TO_USB
extern uint8_t g_abToUSBBuffer[256]; // way more than enough buffer space
extern uint8_t g_abToUSBCnt;
#endif


#define DEBUG_PIN_USB_INPUT           A1
#define DEBUG_PIN_SEND_STATUS_PACKET  A2
#define DEBUG_PIN_AX_INPUT            A3
#define DEBUG_PIN_BACKGROUND          A4

//#define USE_DEBUG_IOPINS
#ifdef USE_DEBUG_IOPINS
#define debug_digitalWrite(pin, state)  digitalWrite((pin), (state))
#define debug_digitalToggle(pin)  digitalWrite((pin), !digitalRead((pin)))
#else
#define debug_digitalWrite(pin, state)  
#define debug_digitalToggle(pin)  
#endif
  
#define   VOLTAGE_ANALOG_PIN    0   // Was 0 on V1

#define     USART_TIMEOUT_MIN   8   //  x 20us
#define     SEND_TIMEOUT_MIN    0   //  x 20us
#define     RECEIVE_TIMEOUT_MIN 10  //  x 20us
//default values, can be modified with write_data and are saved in EEPROM
#define     USART_TIMEOUT       50   //  x 20us
#define     SEND_TIMEOUT        4    //  x 20us
#define   RECEIVE_TIMEOUT       100  //  x 20us

// Define our voltage divider
// Warning: values directly like 10000 and 40200 will overflow math, so reduce. 
//  example 100 and 402... 
#define   VOLTAGE_DIVIDER_RES1  50 // 10K
#define   VOLTAGE_DIVIDER_RES2  201 // 40.2K
#define   LOW_VOLTAGE_SHUTOFF_DEFAULT 90  // 9 volts

// AX-Bus Read pass though mode
#define AX_PASSTHROUGH      1
#define AX_DIVERT           2

//Dynamixel device Control table
#define AX_ID_DEVICE        200    // Default ID
#define AX_ID_BROADCAST     0xfe
#define MODEL_NUMBER_L      0x041 // 'A' ...
#define MODEL_NUMBER_H      0x54  // 'T' arbitrary model number that should not collide with the ones from Robotis
#define FIRMWARE_VERSION    0x05  // Firmware version, needs to be updated with every new release
#define RETURN_LEVEL         2



//extern uint8_t regs[REG_TABLE_SIZE];
#define REG_TABLE_SIZE      (CM730_VOLTAGE+1)

// Define which IDs will saved to and restored from EEPROM
#define REG_EEPROM_FIRST    CM730_ID
#define REG_EEPROM_LAST     CM730_STATUS_RETURN_LEVEL

#define AX_CMD_SYNC_READ      0x84
#define SYNC_READ_START_ADDR  5
#define SYNC_READ_LENGTH      6

#define AX_BUFFER_SIZE              128
#define AX_SYNC_READ_MAX_DEVICES    120
#define AX_MAX_RETURN_PACKET_SIZE   235

enum {AX_SEARCH_FIRST_FF = 0, AX_SEARCH_SECOND_FF, PACKET_ID, PACKET_LENGTH,
      PACKET_INSTRUCTION, AX_SEARCH_RESET, AX_SEARCH_BOOTLOAD, AX_GET_PARAMETERS,
      AX_SEARCH_READ, AX_SEARCH_PING, AX_PASS_TO_SERVOS
     };

//==================================================================
// Registers - CM730(ish)
//==================================================================
// CM-730 address table
enum{
    CM730_MODEL_NUMBER_L              = 0,
    CM730_MODEL_NUMBER_H              = 1,
    CM730_FIRMWARE_VERSION            = 2,
    CM730_ID                          = 3,
    CM730_BAUD_RATE                   = 4,
    CM730_RETURN_DELAY_TIME           = 5,
    TA_DOWN_LIMIT_VOLTAGE              = 12,
    CM730_STATUS_RETURN_LEVEL         = 16,
    CM730_DXL_POWER                   = 24,
    CM730_LED_PANEL                   = 25, // Teensy D13 low bit. D12? for 2nd bit. 
    CM730_VOLTAGE                     = 50, // A0
};

#if 0
// Arbotix Pro stuff - Only showing those things that have been added.

// Arbotix-Pro added. 
//#define -     80
#define P_BUZZER_DATA0        81
#define P_BUZZER_DATA1        82
#endif
// 83-89 used maybe on pro...
//==================================================================
// EEPROM and Min/Max
//==================================================================
extern uint8_t g_controller_registers[REG_TABLE_SIZE];

extern uint8_t g_passthrough_mode;
extern unsigned long last_message_time;
extern uint8_t ax_state;
extern uint8_t ax_tohost_state;

extern uint8_t rxbyte[AX_SYNC_READ_MAX_DEVICES + 8]; // buffer where currently processed data are stored when looking for a Dynamixel packet, with enough space for longest possible sync read request
extern uint8_t rxbyte_count;   // number of used bytes in rxbyte buffer

//==================================================================
// function definitions
//==================================================================
extern void InitalizeRegisterTable(void);
extern void axStatusPacket(uint8_t err, uint8_t* data, uint8_t count_bytes);
extern void LocalRegistersRead(uint8_t register_id, uint8_t count_bytes);
extern void CheckBatteryVoltage(void);
extern void LocalRegistersWrite(uint8_t register_id, uint8_t* data, uint8_t count_bytes);
extern void sync_read(uint8_t id, uint8_t* params, uint8_t nb_params);
extern void setAXtoTX(bool fTX);
extern void MaybeFlushUSBOutputData(void);
extern void FlushUSBInputQueue(void);
extern void UpdateHardwareAfterLocalWrite(uint8_t register_id, uint8_t count_bytes);
extern void CheckHardwareForLocalReadRequest(uint8_t register_id, uint8_t count_bytes);

extern bool ProcessInputFromUSB(void);
extern bool ProcessInputFromAXBuss(void);

//==================================================================
// inline functions
//==================================================================
//-----------------------------------------------------------------------------
// setAXtoTX - Set the Tx buffer to input or output.
//-----------------------------------------------------------------------------
extern bool g_AX_IS_TX;

inline void  setAXtoTX()
{
  if (!g_AX_IS_TX) {
    g_AX_IS_TX = true;
    setTX(0);
  }
}

inline void  setAXtoRX()
{
  if (g_AX_IS_TX) {
    g_AX_IS_TX = false;
    setRX(0);
  }
}



#endif
