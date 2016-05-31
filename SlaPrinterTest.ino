#include <SPI.h>
#include "MarlinSerial.h"
#include "SlaPrinterConstants.h"
#include "Boards.h"
#include "Language.h"
#include "WString.h"

#define  FORCE_INLINE __attribute__((always_inline)) inline

#define VERSION_STRING  "1.0.2"

//The ASCII buffer for receiving from the serial:
#define MAX_CMD_SIZE 96
#define BUFSIZE 4

// This determines the communication speed of the printer
#define BAUDRATE 250000

#define BLOCK_BUFFER_SIZE 16 // maximize block buffer

const char errormagic[] PROGMEM = "Error:";
const char echomagic[] PROGMEM = "echo:";

// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in 
// the source g-code and may never actually be reached if acceleration management is active.
typedef struct {
  // Fields used by the bresenham algorithm for tracing the line
  long steps_x, steps_y, steps_z, steps_e;  // Step count along each axis
  unsigned long step_event_count;           // The number of step events required to complete this block
  long accelerate_until;                    // The index of the step event on which to stop acceleration
  long decelerate_after;                    // The index of the step event on which to start decelerating
  long acceleration_rate;                   // The acceleration rate used for acceleration calculation
  unsigned char direction_bits;             // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  unsigned char active_extruder;            // Selects the active extruder
  #ifdef ADVANCE
    long advance_rate;
    volatile long initial_advance;
    volatile long final_advance;
    float advance;
  #endif

  // Fields used by the motion planner to manage acceleration
//  float speed_x, speed_y, speed_z, speed_e;        // Nominal mm/sec for each axis
  float nominal_speed;                               // The nominal speed for this block in mm/sec 
  float entry_speed;                                 // Entry speed at previous-current junction in mm/sec
  float max_entry_speed;                             // Maximum allowable junction entry speed in mm/sec
  float millimeters;                                 // The total travel of this block in mm
  float acceleration;                                // acceleration mm/sec^2
  unsigned char recalculate_flag;                    // Planner flag to recalculate trapezoids on entry junction
  unsigned char nominal_length_flag;                 // Planner flag for nominal speed always reached

  // Settings for the trapezoid generator
  unsigned long nominal_rate;                        // The nominal step rate for this block in step_events/sec 
  unsigned long initial_rate;                        // The jerk-adjusted step rate at start of block  
  unsigned long final_rate;                          // The minimal rate at exit
  unsigned long acceleration_st;                     // acceleration steps/sec^2
  unsigned long fan_speed;
  #ifdef BARICUDA
  unsigned long valve_pressure;
  unsigned long e_to_p_pressure;
  #endif
  volatile char busy;
} block_t;

// Buffer definition
//
static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static int bufindr = 0;       // 'read' buffer index (0 - 3)
static int bufindw = 0;       // 'write' buffer index (0 - 3)
static int buflen = 0;
static char serial_char;      // single character read from the serial port
static int serial_count = 0;  // index to on one of the 4 arrays in the buffer

const uint16_t k_uAConfigBits = 0x7000;
const uint16_t k_uBConfigBits = 0x8000;
const int chipASelectPin = 47;
//const int chipBSelectPin = 49; // This pin is not working. Just stays high.

static int g_iSerialState = 0;
// ILDA parameters
//
static long lNumRecords = 0;
static long lFrameColorNum = 0;
static long lTotalFrames = 0;
static long lRecordSize = 0;

// This is used by the ILDA parser to determine how many bytes to use in each buffer string based on the record size
//
static int iBufLimit = 0;

extern "C" {
  extern unsigned int __bss_end;
  extern unsigned int __heap_start;
  extern void *__brkval;

  int freeMemory() {
    int free_memory;

    if ((int)__brkval == 0)
      free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
      free_memory = ((int)&free_memory) - ((int)__brkval);

    return free_memory;
  }
}

void setup()
{
//  pinMode(chipASelectPin, OUTPUT);
//  digitalWrite(chipASelectPin, HIGH);
//
//  // Use the slowest
//  SPI.beginTransaction(SPISettings(1525000, MSBFIRST, SPI_MODE1));
//  SPI.begin();

  MSerial.begin(BAUDRATE);
  SERIAL_PROTOCOLLNPGM("start");
  SERIAL_ECHO_START;

  // Check startup - does nothing if bootloader sets MCUSR to 0
  byte mcu = MCUSR;
  if(mcu & 1) SERIAL_ECHOLNPGM(MSG_POWERUP);
  if(mcu & 2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
  if(mcu & 4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
  if(mcu & 8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
  if(mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
  MCUSR=0;

  SERIAL_ECHOPGM(MSG_MARLIN);
  SERIAL_ECHOLNPGM(VERSION_STRING);

  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_FREE_MEMORY);
  SERIAL_ECHO(freeMemory());
  SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
  SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);

  g_iSerialState = 0;
  lNumRecords = 0;
  lFrameColorNum = 0;
  lTotalFrames = 0;
  lRecordSize = 0;
  buflen = 0;
  iBufLimit = 0;
}

void loop()
{
  get_command();
  process_commands();
}

void get_command()
{
  while(MSerial.available() > 0 && buflen < BUFSIZE)
  {
    // Read 1 character from the serial peripheral port
    //
    serial_char = MSerial.read();
    MSerial.write(serial_char);
    switch(g_iSerialState)
    {
      // This state will determine if the file stream is an ILDA or a Gcode
      //
    case 0:
      if((0 == serial_count) && ('I' == serial_char))
      {
        // Set the state to process ILDA
        //
        g_iSerialState = 1;
        cmdbuffer[bufindw][serial_count] = serial_char;
        get_IldaHeaderInfo();
      }
      else
      {
        // Set the state to process Gcode
        //
        g_iSerialState = 10;
      }
      break;
    case 1:
      MSerial.write("get ILDA header\n");
      get_IldaHeaderInfo();
      break;
    case 2:
      MSerial.write("get ILDA records\n");
      get_IldaRecords();
      break;
    case 10:
      MSerial.write("Gcode processing\n");
      break;
    default:
      // Send error if the state is invalid
      //
      SERIAL_ERROR_START;
      SERIAL_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
      //SERIAL_ERRORLN(gcode_LastN);
      FlushSerialRequestResend();
      serial_count = 0;
      bufindw = 0;
      g_iSerialState = 0;
    }
  }
}

void get_IldaHeaderInfo()
{
  serial_count++;
  if(serial_count < 4)
  {
    // Save the character just in case this is not the ILDA header
    //
    cmdbuffer[bufindw][serial_count] = serial_char;
  }    
  switch(serial_count)
  {
  // The 'I' has already been detected therefore continue with a count of 1
  //
  case 1:
    if('L' != serial_char)
    {
      g_iSerialState = 0;
      MSerial.write("case 1 did not find ILDA\n");
    }
    break;
  case 2:
    if('D' != serial_char)
    {
      g_iSerialState = 0;
      MSerial.write("case 2 did not find ILDA\n");
    }
    break;
  case 3:
    if('A' != serial_char)
    {
      g_iSerialState = 0;
      MSerial.write("case 3 did not find ILDA\n");
    }
    else
    {
      MSerial.write("found ILDA\n");
    }
    break;
  // Retrieve the format code
  //
  case 7:
    if((0 == serial_char) || (5 == serial_char))
    {
      lRecordSize = 8;
    }
    else if(1 == serial_char)
    {
      lRecordSize = 6;
    }
    else if(2 == serial_char)
    {
      lRecordSize = 3;
    }
    else if(4 == serial_char)
    {
      lRecordSize = 10;
    }
    else
    {
      // Send error if the state is invalid
      //
      SERIAL_ERROR_START;
      SERIAL_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
      //SERIAL_ERRORLN(gcode_LastN);
      FlushSerialRequestResend();
      serial_count = 0;
      bufindw = 0;
      g_iSerialState = 0;
    }
    break;
  // Retrieve the record size(big endian format)
  //
  case 24:
    cmdbuffer[bufindw][0] = serial_char;
    break;
  case 25:
    cmdbuffer[bufindw][1] = serial_char;
    lNumRecords = (long)cmdbuffer[bufindw];
    break;
  // This is the end of the header
  //
  case 31:
    // Set the state to retrieve the records
    //
    g_iSerialState = 2;
    serial_count = 0;
    iBufLimit = MAX_CMD_SIZE / lRecordSize;
    break;
  }
}

void get_IldaRecords()
{
  cmdbuffer[bufindw][serial_count++] = serial_char;
  if(serial_count == iBufLimit)
  {
    serial_count = 0;
    bufindw = (bufindw + 1) % BUFSIZE;
  }
}

void process_commands()
{
}

void process_IldaCommands()
{
}

void FlushSerialRequestResend()
{
  MSerial.flush();
  SERIAL_PROTOCOLPGM(MSG_RESEND);
  SERIAL_PROTOCOLLNPGM(MSG_OK);
}

