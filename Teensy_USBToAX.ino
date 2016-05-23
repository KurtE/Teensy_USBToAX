#include <TimerOne.h>

//=============================================================================
//Project Teensy Arbotix Pro
//Description: To have an Teensy 3.1/3.2, semi emulate an Arbotix Pro or 
//      Robotis CM730
//=============================================================================

//=============================================================================
// Header Files
//=============================================================================

#include <EEPROM.h>

#include <ax12Serial.h>
#include <BioloidSerial.h>
#include "globals.h"


//=============================================================================
//[CONSTANTS]
//=============================================================================

//=============================================================================
//[Globals]
//=============================================================================
uint8_t ax_state = AX_SEARCH_FIRST_FF; // current state of the Dynamixel packet parser state machine

uint8_t rxbyte[AX_SYNC_READ_MAX_DEVICES + 8]; // buffer where currently processed data are stored when looking for a Dynamixel packet, with enough space for longest possible sync read request
uint8_t rxbyte_count = 0;   // number of used bytes in rxbyte buffer
unsigned long last_message_time;
uint8_t g_passthrough_mode;

//unsigned long baud = 1000000;
unsigned long baud = 1000000;
//IntervalTimer interval_timer_background_;


//-----------------------------------------------------------------------------
// setup - Main Arduino setup function
//-----------------------------------------------------------------------------
void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  // Temporary Debug stuff
#ifdef USE_DEBUG_IOPINS
  pinMode(DEBUG_PIN_USB_INPUT, OUTPUT);
  pinMode(DEBUG_PIN_SEND_STATUS_PACKET, OUTPUT);
  pinMode(DEBUG_PIN_AX_INPUT, OUTPUT);
  pinMode(DEBUG_PIN_BACKGROUND, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
#endif
#ifdef DBGSerial
  delay(2000);
  DBGSerial.begin(115200);
  delay(1000);
  DBGSerial.println("Teensy Arbotix Pro Start");
#endif  

  // Hack define which
#ifdef PCSerial_TXPIN
  pinMode(PCSerial_TXPIN, INPUT_PULLUP);
#endif
  PCSerial.begin(baud);	// USB, communication to PC or Mac
  ax12Init(1000000, &HWSERIAL, SERVO_DIRECTION_PIN);
  
  setAXtoTX();
  InitalizeRegisterTable(); 

  // clear out USB Input queue
  FlushUSBInputQueue();

  // Start up our background task
  //interval_timer_background_.begin(BackgroundTimerInterrupt, 100000);  //  for 100 times per second...
  //Timer1.initialize(100000);
  //Timer1.attachInterrupt(BackgroundTimerInterrupt); // Setup to run 100 times per second
  for (int i=0;i<2; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
  }
}

//-----------------------------------------------------------------------------
// loop - Main Arduino loop function. 
//-----------------------------------------------------------------------------
void loop()
{

  debug_digitalWrite( DEBUG_PIN_USB_INPUT,  HIGH);
  bool did_something = ProcessInputFromUSB();
  debug_digitalWrite( DEBUG_PIN_USB_INPUT,  LOW);
//  yield();  // Give a chance for other things to happen

  // Call off to process any input that we may have received from the AXBuss
  debug_digitalWrite( DEBUG_PIN_AX_INPUT,  HIGH);
  did_something |= ProcessInputFromAXBuss();
  debug_digitalWrite( DEBUG_PIN_AX_INPUT,  LOW);
//  yield();

  // If we did not process any data input from USB or from AX Buss, maybe we should flush anything we have 
  // pending to go back to main processor
#if 0
  if (!did_something) 
  {
    MaybeFlushUSBOutputData();
    debug_digitalWrite( DEBUG_PIN_BACKGROUND,  HIGH);
    CheckBatteryVoltage();
    debug_digitalWrite( DEBUG_PIN_BACKGROUND,  LOW);
  }
#endif  
}

uint8_t g_Timer_loop_count = 0;
void BackgroundTimerInterrupt()
{
    debug_digitalWrite( DEBUG_PIN_BACKGROUND,  HIGH);
    CheckBatteryVoltage();

    debug_digitalWrite( DEBUG_PIN_BACKGROUND,  LOW);
}





