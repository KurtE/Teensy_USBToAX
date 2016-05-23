//=============================================================================
// File: SyncRead.cpp
//  Handle the SyncRead command
//=============================================================================

//=============================================================================
// Header Files
//=============================================================================

#include <ax12Serial.h>
#include <BioloidSerial.h>
#include "globals.h"

//-----------------------------------------------------------------------------
// Define Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// sync_read: this handles the sync read message and loops through each of the
// servos and reads the specified registers and packs the data back up into
// one usb message.
// Note: we pass through the ID as the other side validation will look to
// make sure it matches...
//-----------------------------------------------------------------------------
void sync_read(uint8_t id, uint8_t* params, uint8_t nb_params) {

  // divert incoming data to a buffer for local processing
  g_passthrough_mode = AX_DIVERT;

  uint8_t addr = params[0];    // address to read in control table
  uint8_t nb_to_read = params[1];    // # of bytes to read from each servo
  uint8_t nb_servos = nb_params - 2;

#ifdef BUFFER_TO_USB
  g_abToUSBCnt = 0;
  g_abToUSBBuffer[g_abToUSBCnt++] = (0xff);
  g_abToUSBBuffer[g_abToUSBCnt++] = (0xff);
  g_abToUSBBuffer[g_abToUSBCnt++] = (id);
  g_abToUSBBuffer[g_abToUSBCnt++] = (2 + (nb_to_read * nb_servos));
  g_abToUSBBuffer[g_abToUSBCnt++] = ((uint8_t)0);  //error code
#else
  PCSerial.write(0xff);
  PCSerial.write(0xff);
  PCSerial.write(id);
  PCSerial.write(2 + (nb_to_read * nb_servos));
  PCSerial.write((uint8_t)0);  //error code
#endif
  // get ax data
  uint8_t checksum = id + (nb_to_read * nb_servos) + 2; // start accumulating the checksum
  uint8_t* servos = params + 2; // pointer to the ids of the servos to read from
  for (uint8_t servo_id = 0; servo_id < nb_servos; servo_id++) {
    uint8_t id = servos[servo_id];
    setTX(id);
    // 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM
    int checksum = ~((id + 6 + addr + nb_to_read) % 256);
    ax12writeB(0xFF);
    ax12writeB(0xFF);
    ax12writeB(id);
    ax12writeB(4);    // length
    ax12writeB(AX_READ_DATA);
    ax12writeB(addr);
    ax12writeB(nb_to_read);
    ax12writeB(checksum);

    setRX(id);
    if (ax12ReadPacket(nb_to_read + 6) > 0) {
      for (uint8_t i = 0; i < nb_to_read; i++) {
        uint8_t b = ax_rx_buffer[i + 5];
        checksum += b ;
#ifdef BUFFER_TO_USB
        g_abToUSBBuffer[g_abToUSBCnt++] = b;
#else
        PCSerial.write(b);
#endif
#ifdef DBGSerial
        DBGSerial.print(b , HEX);
        DBGSerial.print(" ");
#endif
      }
    } else {
      for (uint8_t i = 0; i < nb_to_read; i++) {
        checksum += 0xFF;
#ifdef BUFFER_TO_USB
        g_abToUSBBuffer[g_abToUSBCnt++] = 0xFF;
#else
        PCSerial.write(0xFF);
#endif
      }
    }
  }

#ifdef BUFFER_TO_USB
  g_abToUSBBuffer[g_abToUSBCnt++] = (255 - ((checksum) % 256));
  PCSerial.write(g_abToUSBBuffer, g_abToUSBCnt);

#else
  PCSerial.write(255 - ((checksum) % 256));
#endif
#ifdef DBGSerial
  DBGSerial.println("SF");
#endif
  PCSerial.flush();

  // allow data from USART to be sent directly to USB
  g_passthrough_mode = AX_PASSTHROUGH;
}


