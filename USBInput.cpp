//=============================================================================
// Header Files
//=============================================================================

#include <ax12Serial.h>
#include <BioloidSerial.h>
#include "globals.h"

//=============================================================================
//[CONSTANTS]
//=============================================================================

//-----------------------------------------------------------------------------
// Define Global variables
//-----------------------------------------------------------------------------
uint16_t ax_checksum = 0;

//-----------------------------------------------------------------------------
// Forward references
//-----------------------------------------------------------------------------
extern void pass_bytes(uint8_t nb_bytes);

//-----------------------------------------------------------------------------
// passBufferedDataToServos - take any data that we read in and now output the
// data over the AX Buss. 
//-----------------------------------------------------------------------------
void passBufferedDataToServos(void) {
  // if the last byte read is not what it's expected to be, but is a 0xFF, it could be the first 0xFF of an incoming command
  if (rxbyte[rxbyte_count - 1] == 0xFF) {
    // we trade the latest 0xFF received against the first one that is necessarily in the first position of the buffer
    pass_bytes(rxbyte_count - 1); // pass the discarded data, except the last 0xFF
    ax_state = AX_SEARCH_SECOND_FF;
    rxbyte_count = 1; // keep the first 0xFF in the buffer
    last_message_time = 0;
  } else {
    pass_bytes(rxbyte_count);
    ax_state = AX_SEARCH_FIRST_FF;
  }
}

//-----------------------------------------------------------------------------
// pass_bytes - Switch the AX Buss to output and output the number of bytes
//     from our buffered rx buffer. 
//-----------------------------------------------------------------------------
void pass_bytes(uint8_t nb_bytes) {
  if (nb_bytes) {
    setAXtoTX();
//    ax12write(rxbyte, nb_bytes);
    for (uint8_t i = 0; i < nb_bytes; i++) {
      ax12writeB(rxbyte[i]);
    }
  }
}
//-----------------------------------------------------------------------------
// ProcessInputFromUSB - Process all of the input bytes that are buffered up
//  from the USB
//-----------------------------------------------------------------------------
bool ProcessInputFromUSB(void)
{
  bool we_did_something = false;
  // Main loop, lets loop through reading any data that is coming in from the USB
  int ch;
  while ((ch = PCSerial.read()) != -1)
  {
    we_did_something = true;
    digitalWriteFast(LED_PIN, digitalReadFast(LED_PIN)? LOW : HIGH);
    last_message_time = micros();
    switch (ax_state) {
      case AX_SEARCH_FIRST_FF:
        rxbyte[0] = ch;
        if (ch == 0xFF) {
          ax_state = AX_SEARCH_SECOND_FF;
          rxbyte_count = 1;
        } else {
          setAXtoTX();
          ax12writeB(ch);
        }
        break;

      case AX_SEARCH_SECOND_FF:
        rxbyte[rxbyte_count++] = ch;
        if (ch == 0xFF) {
          ax_state = PACKET_ID;
        } else {
          passBufferedDataToServos();
        }
        break;

      case PACKET_ID:
        rxbyte[rxbyte_count++] = ch;
        if (ch == 0xFF) { // we've seen 3 consecutive 0xFF
          rxbyte_count--;
          pass_bytes(1); // let a 0xFF pass
        } else {
          ax_state = PACKET_LENGTH;

          // Check to see if we should start sending out the data here.  
          if (rxbyte[PACKET_ID] != g_controller_registers[CM730_ID] && rxbyte[PACKET_ID] != AX_ID_BROADCAST ) {
            pass_bytes(rxbyte_count);
          }
        }
        break;

      case PACKET_LENGTH:
        rxbyte[rxbyte_count++] = ch;
        if (rxbyte[PACKET_ID] == g_controller_registers[CM730_ID] || rxbyte[PACKET_ID] == AX_ID_BROADCAST ) {
          if (rxbyte[PACKET_LENGTH] > 1 && rxbyte[PACKET_LENGTH] < (AX_SYNC_READ_MAX_DEVICES + 4)) { // reject message if too short or too big for rxbyte buffer
            ax_state = PACKET_INSTRUCTION;
          } else {
            axStatusPacket(ERR_RANGE, NULL, 0);
            passBufferedDataToServos();
          }
        } else {
          setAXtoTX();
          ax12writeB(ch);
          ax_state = AX_PASS_TO_SERVOS;
        }
        break;

      case PACKET_INSTRUCTION:
        rxbyte[rxbyte_count++] = ch;
        if (rxbyte[PACKET_INSTRUCTION] == AX_CMD_SYNC_READ) {
          ax_state = AX_GET_PARAMETERS;
          ax_checksum =  rxbyte[PACKET_ID] + AX_CMD_SYNC_READ + rxbyte[PACKET_LENGTH];
        } else if (rxbyte[PACKET_ID] == g_controller_registers[CM730_ID]) {
          if (rxbyte[PACKET_INSTRUCTION] == AX_PING) {
            ax_state = AX_SEARCH_PING;
          } else if (rxbyte[PACKET_INSTRUCTION] == AX_READ_DATA) {
            ax_state = AX_GET_PARAMETERS;
            ax_checksum = g_controller_registers[CM730_ID] + AX_READ_DATA + rxbyte[PACKET_LENGTH];
          } else if (rxbyte[PACKET_INSTRUCTION] == AX_WRITE_DATA) {
            ax_state = AX_GET_PARAMETERS;
            ax_checksum = g_controller_registers[CM730_ID] + AX_WRITE_DATA + rxbyte[PACKET_LENGTH];
          } else {
            passBufferedDataToServos();
          }
        } else {
          passBufferedDataToServos();
        }
        break;

      case AX_SEARCH_PING:
        rxbyte[5] = ch;
        if (((g_controller_registers[CM730_ID] + 2 + AX_PING + rxbyte[5]) % 256) == 255) {
          axStatusPacket(ERR_NONE, NULL, 0);
          ax_state = AX_SEARCH_FIRST_FF;
        } else {
          passBufferedDataToServos();
        }
        break;

      case AX_GET_PARAMETERS:
        rxbyte[rxbyte_count] = ch;
        ax_checksum += rxbyte[rxbyte_count] ;
        rxbyte_count++;
        if (rxbyte_count >= (rxbyte[PACKET_LENGTH] + 4)) { // we have read all the data for the packet
          if ((ax_checksum % 256) != 255) { // ignore message if checksum is bad
            passBufferedDataToServos();
          } else {
            if (rxbyte[PACKET_INSTRUCTION] == AX_CMD_SYNC_READ) {
              uint8_t nb_servos_to_read = rxbyte[PACKET_LENGTH] - 4;
              uint8_t packet_overhead = 6;
              if ( (rxbyte[SYNC_READ_LENGTH] == 0)
                   || (rxbyte[SYNC_READ_LENGTH] > AX_BUFFER_SIZE - packet_overhead) // the return packets from the servos must fit the return buffer
                   || ( (int16_t)rxbyte[SYNC_READ_LENGTH] * nb_servos_to_read > AX_MAX_RETURN_PACKET_SIZE - packet_overhead )) { // and the return packet to the host must not be bigger either
                axStatusPacket(ERR_RANGE, NULL, 0);
              } else {
                sync_read(rxbyte[PACKET_ID], &rxbyte[SYNC_READ_START_ADDR], rxbyte[PACKET_LENGTH] - 2);
              }
            } else if (rxbyte[PACKET_INSTRUCTION] == AX_READ_DATA) {
              LocalRegistersRead(rxbyte[5], rxbyte[6]);
            } else if (rxbyte[PACKET_INSTRUCTION] == AX_WRITE_DATA) {
              LocalRegistersWrite(rxbyte[5], &rxbyte[6], rxbyte[PACKET_LENGTH] - 3);
            }
            ax_state = AX_SEARCH_FIRST_FF;
          }
        }
        break;

      case AX_PASS_TO_SERVOS:
        setAXtoTX();
        ax12writeB(ch);
        rxbyte_count++;
        if (rxbyte_count >= (rxbyte[PACKET_LENGTH] + 4)) { // we have read all the data for the packet // we have let the right number of bytes pass
          ax_state = AX_SEARCH_FIRST_FF;
        }
        break;

      default:
        break;
    }
  }
    // Timeout on state machine while waiting on further USB data
  if (ax_state != AX_SEARCH_FIRST_FF) {
    if ((micros() - last_message_time) > (20 * g_controller_registers[AX_RETURN_DELAY_TIME])) {
      pass_bytes(rxbyte_count);
      ax_state = AX_SEARCH_FIRST_FF;
    }
  }
  // If we are at the waiting for first ff and no more input, then set AX back to read
  if (ax_state == AX_SEARCH_FIRST_FF)
    setAXtoRX();

  return we_did_something;  

}  

//-----------------------------------------------------------------------------
// FlushUSBInutQueue - Flush all of the data out of the input queue...
//-----------------------------------------------------------------------------
void FlushUSBInputQueue(void)
{
  // Lets use internal Teensy function... 
#if defined(TEENSYDUINO)
  //usb_serial_flush_input();
  Serial.clear();
#else
  int ch;
  while (Serial.read() != -1)
    ;
#endif
}  

