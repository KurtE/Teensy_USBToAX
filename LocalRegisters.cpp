//=============================================================================
// File: LocalRegisters.cpp
// Description: Definitions and code to handle the logical registers associated
//      with the actual controller
//=============================================================================

//=============================================================================
// Header Files
//=============================================================================
#include <EEPROM.h>
#include <ax12Serial.h>
#include <BioloidSerial.h>
#include "globals.h"

//-----------------------------------------------------------------------------
// Define Global variables
//-----------------------------------------------------------------------------
//                                                    0             1              2                   3            4             5              6              7
uint8_t g_controller_registers[REG_TABLE_SIZE] = {MODEL_NUMBER_L, MODEL_NUMBER_H, FIRMWARE_VERSION, AX_ID_DEVICE, USART_TIMEOUT, SEND_TIMEOUT, RECEIVE_TIMEOUT, 0,
                                                  //                                                      8  9  0  1  2                            3  4  5  6
                                                  0, 0, 0, 0, LOW_VOLTAGE_SHUTOFF_DEFAULT, 0, 0, 0, RETURN_LEVEL
                                                 };

const uint8_t g_controller_registers_ranges[][2] =
{
  {1, 0},   //MODEL_NUMBER_L        0
  {1, 0},   //MODEL_NUMBER_H        1
  {1, 0},   //VERSION               2
  {0, 253}, //ID                    3
  {1, 254}, //BAUD_RATE             4
  {0, 254}, //Return Delay time     5
  {0, 255}, {0, 255},  {0, 255},  {0, 255}, {0, 255}, {0, 255}, // 6-11
  {0, 250}, //DOWN_LIMIT_VOLTAGE   12
  {50, 250}, //UP_LIMIT_VOLTAGE 13
  {0, 255},  {0, 255},      // 14-15
  {0, 2}, //RETURN_LEVEL          16

  // Not saved to eeprom...
  {1, 0}, {1, 0},  {1, 0},  {1, 0}, {1, 0}, {1, 0}, {1, 0}, // 17-23

};


//-----------------------------------------------------------------------------
// Forward reference
//-----------------------------------------------------------------------------

// helper functions for read
extern void CheckHardwareForLocalReadRequest(uint8_t register_id, uint8_t count_bytes);

// Helper functions for write
extern uint8_t ValidateWriteData(uint8_t register_id, uint8_t* data, uint8_t count_bytes);
extern void SaveEEPromSectionsLocalRegisters(void);
extern void UpdateHardwareAfterLocalWrite(uint8_t register_id, uint8_t count_bytes);



//-----------------------------------------------------------------------------
// LocalRegistersRead
//-----------------------------------------------------------------------------
void LocalRegistersRead(uint8_t register_id, uint8_t count_bytes)
{
#ifdef DBGSerial
  DBGSerial.printf("LR: %d %d\n\r", register_id, count_bytes);
#endif

  // Several ranges of logical registers to process.
  uint16_t top = (uint16_t)register_id + count_bytes;
  if ( count_bytes == 0  || (top >= REG_TABLE_SIZE))
  {
    axStatusPacket( ERR_RANGE, NULL, 0 );
    return;
  }

  // See if we need to do any preprocesing.
  CheckHardwareForLocalReadRequest(register_id, count_bytes);

  axStatusPacket(ERR_NONE, g_controller_registers + register_id, count_bytes);
}


//-----------------------------------------------------------------------------
// LocalRegistersWrite: Update the local registers
//-----------------------------------------------------------------------------
void LocalRegistersWrite(uint8_t register_id, uint8_t* data, uint8_t count_bytes)
{
#ifdef DBGSerial
  DBGSerial.printf("LW: %d %d %x\n\r", register_id, count_bytes, *data);
#endif
  if ( ! ValidateWriteData(register_id, data, count_bytes) ) {
    axStatusPacket( ERR_RANGE, NULL, 0 );
  } else {
    memcpy(g_controller_registers + register_id, data, count_bytes);

    // If at least some of the registers set is in the EEPROM area, save updates
    if (register_id <= CM730_STATUS_RETURN_LEVEL)
      SaveEEPromSectionsLocalRegisters();
    axStatusPacket(ERR_NONE, NULL, 0 );

    // Check to see if we need to do anything to the hardware in response to the
    // changes
    UpdateHardwareAfterLocalWrite(register_id, count_bytes);

  }
}


//--------------------------------------------------------------------
// CheckBatteryVoltage - We will call this from main loop.
//    It does some averaging of voltage reads to get a better consisten
//    voltage.  It will also try to detec when the voltage goes to low
//    either due to the battery is getting low or turned off.  Likewise
//    may detect when battery is turned on...
//--------------------------------------------------------------------

// Warning may need to increase sizes if we go beyond 10bit analog reads
#define MAX_ANALOG_DELTA 50
uint16_t  g_awVoltages[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint16_t  g_wVoltageSum = 0;
uint8_t   g_iVoltages = 0;

void CheckBatteryVoltage(void)
{
  // Get the current voltage
  uint16_t cur_analog = analogRead(VOLTAGE_ANALOG_PIN);

  if (abs((int)cur_analog - (int)(g_wVoltageSum / 8)) > MAX_ANALOG_DELTA)
  {
    // Lets read 8 times and reset sum...
    g_wVoltageSum = 0;
    for (g_iVoltages = 0; g_iVoltages < 8; g_iVoltages++)
    {
      g_awVoltages[g_iVoltages] = analogRead(VOLTAGE_ANALOG_PIN);
      g_wVoltageSum += g_awVoltages[g_iVoltages];
    }
  }
  else
  {
    g_iVoltages++;
    g_iVoltages &= 0x7;  // setup index to our array...
    g_wVoltageSum -= g_awVoltages[g_iVoltages];
    g_awVoltages[g_iVoltages] = cur_analog;
    g_wVoltageSum += cur_analog;
  }
  // Warning - using resistor values like 10000 and 40200 will overflow 32 bit math, but simply need right ratio. so wuse 100 and 402
  g_controller_registers[CM730_VOLTAGE] = (uint32_t)(g_wVoltageSum * 33 * (uint32_t)(VOLTAGE_DIVIDER_RES1 + VOLTAGE_DIVIDER_RES2))
                                          / (uint32_t)(1024 * 8 * (uint32_t)VOLTAGE_DIVIDER_RES1);

  // Check to see if voltage went low and our servos are on and a low voltage value is set.
  if (g_controller_registers[CM730_DXL_POWER] && g_controller_registers[TA_DOWN_LIMIT_VOLTAGE]
      && (g_controller_registers[CM730_VOLTAGE] <  g_controller_registers[TA_DOWN_LIMIT_VOLTAGE] ))
  {
    // Power is too low to run servos so shut them off
    g_controller_registers[CM730_DXL_POWER] = 0;  // Turn it logically off.
    UpdateHardwareAfterLocalWrite(CM730_DXL_POWER, 1);  // use the update function to do the real work.
  }
}


//-----------------------------------------------------------------------------
// CheckHardwareForLocalReadRequest - Some of this will change later to probably
//        some background tasks...
//-----------------------------------------------------------------------------
void CheckHardwareForLocalReadRequest(uint8_t register_id, uint8_t count_bytes)
{
#if 0
#endif
}

//-----------------------------------------------------------------------------
// ValidateWriteData: is this a valid range of registers to update?
//-----------------------------------------------------------------------------
uint8_t ValidateWriteData(uint8_t register_id, uint8_t* data, uint8_t count_bytes)
{
  uint16_t top = (uint16_t)register_id + count_bytes;
  if (count_bytes == 0  || ( top >= REG_TABLE_SIZE)) {
    return false;
  }
  // Check that the value written are acceptable
  for (uint8_t i = 0 ; i < count_bytes; i++ ) {
    uint8_t val = data[i];
    if ((val < g_controller_registers_ranges[register_id][0] ) ||
        (val > g_controller_registers_ranges[register_id][1] ))
    {
      return false;
    }
  }
  return true;
}

//-----------------------------------------------------------------------------
// UpdateHardwareAfterLocalWrite
//-----------------------------------------------------------------------------
void UpdateHardwareAfterLocalWrite(uint8_t register_id, uint8_t count_bytes)
{
#if 0
  uint8_t i;
  uint8_t mask;
  while (count_bytes)
  {
    switch (register_id)
    {
    }
    }
  }
#endif  
}


//-----------------------------------------------------------------------------
// InitializeRegisterTable()
//-----------------------------------------------------------------------------
void InitalizeRegisterTable(void)
{
  uint8_t saved_reg_values[CM730_STATUS_RETURN_LEVEL + 1];
  uint8_t checksum = 0;

  // First check to see if valid version is stored in EEPROM...
  if (EEPROM.read(1) != FIRMWARE_VERSION)
    return;

  // Now read in the bytes from the EEPROM
  for (int i = CM730_FIRMWARE_VERSION; i <= CM730_STATUS_RETURN_LEVEL; i++)
  {
    uint8_t ch = EEPROM.read(1 + i - CM730_FIRMWARE_VERSION);
    checksum += ch;
    saved_reg_values[i] = ch;
  }

  // Now see if the checksum matches
  if (EEPROM.read(0) == checksum)
  {
    // Valid, so copy values into the working table
    for (int i = CM730_FIRMWARE_VERSION; i <= CM730_STATUS_RETURN_LEVEL; i++)
      g_controller_registers[i] = saved_reg_values[i];
  }
}

//-----------------------------------------------------------------------------
// SaveEEPromSectionsLocalRegisters - Save updated registers out to the Teensy EEProm.
//-----------------------------------------------------------------------------
void SaveEEPromSectionsLocalRegisters(void)
{
  // Prety stupid here. simply loop and write out data.  Will also keep a checksum...
  uint8_t checksum = 0;
  for (int i = CM730_FIRMWARE_VERSION; i <= CM730_STATUS_RETURN_LEVEL; i++)
  {
    EEPROM.write(1 + i - CM730_FIRMWARE_VERSION, g_controller_registers[i]);
    checksum += g_controller_registers[i];
  }
  // Lets write the Checksum
  EEPROM.write(0, checksum);
}



