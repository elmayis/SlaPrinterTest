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

//default stepper release if idle
#define DEFAULT_STEPPER_DEACTIVE_TIME 60

#define BLOCK_BUFFER_SIZE 16 // maximize block buffer

//// MOVEMENT SETTINGS
#define NUM_AXIS 4 // The axis order in all axis related arrays is X, Y, Z, E
#define HOMING_FEEDRATE {50*60, 50*60, 4*60, 0}  // set the homing speeds (mm/min)

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

static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;
static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static bool fromsd[BUFSIZE];
// Buffer used to determine if the data is ILDA
//
static char g_cBuf[5];
static int g_iBufIndex = 0;

static int bufindr = 0;
static int bufindw = 0;
static int buflen = 0;
static char serial_char;
static int serial_count = 0;
static boolean comment_mode = false;
static char *strchr_pointer; // just a pointer to find chars in the command string like X, Y, Z, E, etc

// Used for messages back to the monitor
//
static String sMsg;

//Inactivity shutdown variables
static unsigned long previous_millis_cmd = 0;
static unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;

unsigned long starttime=0;
unsigned long stoptime=0;

static uint8_t tmp_extruder;

bool Stopped=false;

const uint16_t k_uAConfigBits = 0x7000;
const uint16_t k_uBConfigBits = 0x8000;
const int chipASelectPin = 47;
//const int chipBSelectPin = 49; // This pin is not working. Just stays high.

int g_iSerialState = 0;
bool g_bIldaFormat = false;

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

void setup() {
//  pinMode(chipASelectPin, OUTPUT);
//  digitalWrite(chipASelectPin, HIGH);
//
//  // Use the slowest
//  SPI.beginTransaction(SPISettings(1525000, MSBFIRST, SPI_MODE1));
//  SPI.begin();

  setup_killpin();
  //mayis
//  setup_powerhold();
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
  for(int8_t i = 0; i < BUFSIZE; i++)
  {
    fromsd[i] = false;
  }

  //mayis
  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
//  Config_RetrieveSettings();

//  tp_init();    // Initialize temperature loop
//  plan_init();  // Initialize planner;
//  watchdog_init();
//  st_init();    // Initialize stepper, this enables interrupts!
//  setup_photpin();
//  servo_init();
  
//  lcd_init();
//  _delay_ms(1000);  // wait 1sec to display the splash screen

//  #if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1
//    SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan
//  #endif
//
//  #ifdef DIGIPOT_I2C
//    digipot_i2c_init();
//  #endif
//#ifdef Z_PROBE_SLED
//  pinMode(SERVO0_PIN, OUTPUT);
//  digitalWrite(SERVO0_PIN, LOW); // turn it off
//#endif // Z_PROBE_SLED
//  setup_homepin();

  g_iBufIndex = 0;
  g_iSerialState = 0;
  g_bIldaFormat = false;
}

void loop() {
//  for(uint16_t uIndex = 0; uIndex < 4096; ++uIndex)
//  {
//    digitalWrite(chipASelectPin, LOW);    
//    SPI.transfer16(k_uAConfigBits | uIndex);
//    //SPI.transfer16(k_uBConfigBits | (4095 - uIndex));
//    digitalWrite(chipASelectPin, HIGH);
//  }
//
//  for(uint16_t uIndex = 0; uIndex < 4096; ++uIndex)
//  {
//    digitalWrite(chipASelectPin, LOW);
//    SPI.transfer16(k_uAConfigBits | 0);
//    //SPI.transfer16(k_uAConfigBits | (4095 - uIndex));
//    //SPI.transfer16(k_uBConfigBits | uIndex);
//    digitalWrite(chipASelectPin, HIGH);
//  }

  if(buflen < (BUFSIZE-1))
    get_command();
    
  if(buflen)
  {
    process_commands();
    buflen = (buflen-1);
    bufindr = (bufindr + 1)%BUFSIZE;
  }
  //mayis
  //check heater every n milliseconds
//  manage_heater();
//  manage_inactivity();
//  checkHitEndstops();
//  lcd_update();
}

void get_command()
{
  while(MSerial.available() > 0  && buflen < BUFSIZE)
  {
    // Read 1 character from the serial peripheral
    //
    serial_char = MSerial.read();
    MSerial.write(serial_char);
    switch(g_iSerialState)
    {
    case 0:
      if(g_iBufIndex < 4)
      {
        g_cBuf[g_iBufIndex++] = serial_char;
        if(4 == g_iBufIndex)
        {
          g_cBuf[4] = 0;
          if(strcmp(g_cBuf, "ILDA") == 0)
          {
            g_iSerialState = 1;
            MSerial.write("found ILDA\n");
          }
          else
          {
            g_iSerialState = 2;
            g_iBufIndex = 0;
            while(g_iBufIndex < 4)
            {
              cmdbuffer[bufindw][serial_count++] = g_cBuf[g_iBufIndex++];
            }
            MSerial.write("did not find ILDA\n");
          }
          g_iBufIndex = 0;
        }
      }
      break;
    case 1:
      MSerial.write("case 1\n");
      g_iSerialState = 0;
      break;
    case 2:
      MSerial.write("case 2\n");
      cmdbuffer[bufindw][serial_count++] = 0;
      MSerial.write(cmdbuffer[bufindw]);
      bufindw = 0;
      serial_count = 0;
      g_iSerialState = 0;
      break;
    }
  }
}

void get_IldaCommands(void)
{
    cmdbuffer[bufindw][serial_count++] = serial_char;

    sMsg = "bufindw = ";
    sMsg += bufindw;
    sMsg += " serial_count = ";
    sMsg += serial_count;
    sMsg += "\n";
    MSerial.write(sMsg.c_str());

    MSerial.write("got it ");
    MSerial.write(serial_char);
    MSerial.write("\n");
}

void get_GcodeCommands(void)
{
  if(serial_char == '\n' ||
    serial_char == '\r' ||
    (serial_char == ':' && comment_mode == false) ||
    serial_count >= (MAX_CMD_SIZE - 1) )
  {
    if(!serial_count) { //if empty line
      comment_mode = false; //for new command
      return;
    }
    cmdbuffer[bufindw][serial_count] = 0; //terminate string
    if(!comment_mode)
    {
      comment_mode = false; //for new command
      fromsd[bufindw] = false;
      if(strchr(cmdbuffer[bufindw], 'N') != NULL)
      {
        strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
        gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
        if(gcode_N != gcode_LastN+1 && (strstr_P(cmdbuffer[bufindw], PSTR("M110")) == NULL) ) {
          SERIAL_ERROR_START;
          SERIAL_ERRORPGM(MSG_ERR_LINE_NO);
          SERIAL_ERRORLN(gcode_LastN);
          //Serial.println(gcode_N);
          FlushSerialRequestResend();
          serial_count = 0;
          return;
        }

        if(strchr(cmdbuffer[bufindw], '*') != NULL)
        {
          byte checksum = 0;
          byte count = 0;
          while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
          strchr_pointer = strchr(cmdbuffer[bufindw], '*');

          if( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum) {
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
            SERIAL_ERRORLN(gcode_LastN);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }
          //if no errors, continue parsing
        }
        else
        {
          SERIAL_ERROR_START;
          SERIAL_ERRORPGM(MSG_ERR_NO_CHECKSUM);
          SERIAL_ERRORLN(gcode_LastN);
          FlushSerialRequestResend();
          serial_count = 0;
          return;
        }

        gcode_LastN = gcode_N;
        //if no errors, continue parsing
      }
      else  // if we don't receive 'N' but still see '*'
      {
        if((strchr(cmdbuffer[bufindw], '*') != NULL))
        {
          SERIAL_ERROR_START;
          SERIAL_ERRORPGM(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
          SERIAL_ERRORLN(gcode_LastN);
          serial_count = 0;
          return;
        }
      }
      if((strchr(cmdbuffer[bufindw], 'G') != NULL)){
        strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
        switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)))){
        case 0:
        case 1:
        case 2:
        case 3:
          if (Stopped == true) {
            SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
            //mayis
            //LCD_MESSAGEPGM(MSG_STOPPED);
          }
          break;
        default:
          break;
        }

      }

      //If command was e-stop process now
      if(strcmp(cmdbuffer[bufindw], "M112") == 0)
        kill();
      
      bufindw = (bufindw + 1)%BUFSIZE;
      buflen += 1;
    }
    serial_count = 0; //clear buffer
  }
  else
  {
    if(serial_char == ';') comment_mode = true;
    if(!comment_mode)
    {
      cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
}

void process_commands()
{
  unsigned long codenum; //throw away variable
  char *starpos = NULL;
#ifdef ENABLE_AUTO_BED_LEVELING
  float x_tmp, y_tmp, z_tmp, real_z;
#endif

  
// mayis - start of code 'G'
//  if(code_seen('G'))
//  {
//    switch((int)code_value())
//    {
//    case 0: // G0 -> G1
//    case 1: // G1
//      if(Stopped == false) {
//        
//        get_coordinates(); // For X Y Z E F
//          #ifdef FWRETRACT
//            if(autoretract_enabled)
//            if( !(code_seen('X') || code_seen('Y') || code_seen('Z')) && code_seen('E')) {
//              float echange=destination[E_AXIS]-current_position[E_AXIS];
//              if((echange<-MIN_RETRACT && !retracted) || (echange>MIN_RETRACT && retracted)) { //move appears to be an attempt to retract or recover
//                  current_position[E_AXIS] = destination[E_AXIS]; //hide the slicer-generated retract/recover from calculations
//                  plan_set_e_position(current_position[E_AXIS]); //AND from the planner
//                  retract(!retracted);
//                  return;
//              }
//            }
//          #endif //FWRETRACT
//        prepare_move();
//        //ClearToSend();
//      }
//      break;
//#ifndef SCARA //disable arc support
//    case 2: // G2  - CW ARC
//      if(Stopped == false) {
//        get_arc_coordinates();
//        prepare_arc_move(true);
//      }
//      break;
//    case 3: // G3  - CCW ARC
//      if(Stopped == false) {
//        get_arc_coordinates();
//        prepare_arc_move(false);
//      }
//      break;
//#endif
//    case 4: // G4 dwell
//      LCD_MESSAGEPGM(MSG_DWELL);
//      codenum = 0;
//      if(code_seen('P')) codenum = code_value(); // milliseconds to wait
//      if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait
//
//      st_synchronize();
//      codenum += millis();  // keep track of when we started waiting
//      previous_millis_cmd = millis();
//      while(millis() < codenum) {
//        manage_heater();
//        manage_inactivity();
//        lcd_update();
//      }
//      break;
//      #ifdef FWRETRACT
//      case 10: // G10 retract
//       #if EXTRUDERS > 1
//        retracted_swap[active_extruder]=(code_seen('S') && code_value_long() == 1); // checks for swap retract argument
//        retract(true,retracted_swap[active_extruder]);
//       #else
//        retract(true);
//       #endif
//      break;
//      case 11: // G11 retract_recover
//       #if EXTRUDERS > 1
//        retract(false,retracted_swap[active_extruder]);
//       #else
//        retract(false);
//       #endif 
//      break;
//      #endif //FWRETRACT
//    case 28: //G28 Home all Axis one at a time
//#ifdef ENABLE_AUTO_BED_LEVELING
//      plan_bed_level_matrix.set_to_identity();  //Reset the plane ("erase" all leveling data)
//#endif //ENABLE_AUTO_BED_LEVELING
//
//      saved_feedrate = feedrate;
//      saved_feedmultiply = feedmultiply;
//      feedmultiply = 100;
//      previous_millis_cmd = millis();
//
//      enable_endstops(true);
//
//      for(int8_t i=0; i < NUM_AXIS; i++) {
//        destination[i] = current_position[i];
//      }
//      feedrate = 0.0;
//
//#ifdef DELTA
//          // A delta can only safely home all axis at the same time
//          // all axis have to home at the same time
//
//          // Move all carriages up together until the first endstop is hit.
//          current_position[X_AXIS] = 0;
//          current_position[Y_AXIS] = 0;
//          current_position[Z_AXIS] = 0;
//          plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
//
//          destination[X_AXIS] = 3 * Z_MAX_LENGTH;
//          destination[Y_AXIS] = 3 * Z_MAX_LENGTH;
//          destination[Z_AXIS] = 3 * Z_MAX_LENGTH;
//          feedrate = 1.732 * homing_feedrate[X_AXIS];
//          plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
//          st_synchronize();
//          endstops_hit_on_purpose();
//
//          current_position[X_AXIS] = destination[X_AXIS];
//          current_position[Y_AXIS] = destination[Y_AXIS];
//          current_position[Z_AXIS] = destination[Z_AXIS];
//
//          // take care of back off and rehome now we are all at the top
//          HOMEAXIS(X);
//          HOMEAXIS(Y);
//          HOMEAXIS(Z);
//
//          calculate_delta(current_position);
//          plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
//
//#else // NOT DELTA
//
//      home_all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS])));
//
//      #if Z_HOME_DIR > 0                      // If homing away from BED do Z first
//      if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
//        HOMEAXIS(Z);
//      }
//      #endif
//
//      #ifdef QUICK_HOME
//      if((home_all_axis)||( code_seen(axis_codes[X_AXIS]) && code_seen(axis_codes[Y_AXIS])) )  //first diagonal move
//      {
//        current_position[X_AXIS] = 0;current_position[Y_AXIS] = 0;
//
//       #ifndef DUAL_X_CARRIAGE
//        int x_axis_home_dir = home_dir(X_AXIS);
//       #else
//        int x_axis_home_dir = x_home_dir(active_extruder);
//        extruder_duplication_enabled = false;
//       #endif
//
//        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
//        destination[X_AXIS] = 1.5 * max_length(X_AXIS) * x_axis_home_dir;destination[Y_AXIS] = 1.5 * max_length(Y_AXIS) * home_dir(Y_AXIS);
//        feedrate = homing_feedrate[X_AXIS];
//        if(homing_feedrate[Y_AXIS]<feedrate)
//          feedrate = homing_feedrate[Y_AXIS];
//        if (max_length(X_AXIS) > max_length(Y_AXIS)) {
//          feedrate *= sqrt(pow(max_length(Y_AXIS) / max_length(X_AXIS), 2) + 1);
//        } else {
//          feedrate *= sqrt(pow(max_length(X_AXIS) / max_length(Y_AXIS), 2) + 1);
//        }
//        plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
//        st_synchronize();
//
//        axis_is_at_home(X_AXIS);
//        axis_is_at_home(Y_AXIS);
//        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
//        destination[X_AXIS] = current_position[X_AXIS];
//        destination[Y_AXIS] = current_position[Y_AXIS];
//        plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
//        feedrate = 0.0;
//        st_synchronize();
//        endstops_hit_on_purpose();
//
//        current_position[X_AXIS] = destination[X_AXIS];
//        current_position[Y_AXIS] = destination[Y_AXIS];
//    #ifndef SCARA
//        current_position[Z_AXIS] = destination[Z_AXIS];
//    #endif
//      }
//      #endif
//
//      if((home_all_axis) || (code_seen(axis_codes[X_AXIS])))
//      {
//      #ifdef DUAL_X_CARRIAGE
//        int tmp_extruder = active_extruder;
//        extruder_duplication_enabled = false;
//        active_extruder = !active_extruder;
//        HOMEAXIS(X);
//        inactive_extruder_x_pos = current_position[X_AXIS];
//        active_extruder = tmp_extruder;
//        HOMEAXIS(X);
//        // reset state used by the different modes
//        memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
//        delayed_move_time = 0;
//        active_extruder_parked = true;
//      #else
//        HOMEAXIS(X);
//      #endif
//      }
//
//      if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) {
//        HOMEAXIS(Y);
//      }
//
//      if(code_seen(axis_codes[X_AXIS]))
//      {
//        if(code_value_long() != 0) {
//    #ifdef SCARA
//       current_position[X_AXIS]=code_value();
//    #else
//       current_position[X_AXIS]=code_value()+add_homing[X_AXIS];
//    #endif
//        }
//      }
//
//      if(code_seen(axis_codes[Y_AXIS])) {
//        if(code_value_long() != 0) {
//         #ifdef SCARA
//       current_position[Y_AXIS]=code_value();
//    #else
//       current_position[Y_AXIS]=code_value()+add_homing[Y_AXIS];
//    #endif
//        }
//      }
//
//      #if Z_HOME_DIR < 0                      // If homing towards BED do Z last
//        #ifndef Z_SAFE_HOMING
//          if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
//            #if defined (Z_RAISE_BEFORE_HOMING) && (Z_RAISE_BEFORE_HOMING > 0)
//              destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
//              feedrate = max_feedrate[Z_AXIS];
//              plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
//              st_synchronize();
//            #endif
//            HOMEAXIS(Z);
//          }
//        #else                      // Z Safe mode activated.
//          if(home_all_axis) {
//            destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT - X_PROBE_OFFSET_FROM_EXTRUDER);
//            destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT - Y_PROBE_OFFSET_FROM_EXTRUDER);
//            destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
//            feedrate = XY_TRAVEL_SPEED/60;
//            current_position[Z_AXIS] = 0;
//
//            plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
//            plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
//            st_synchronize();
//            current_position[X_AXIS] = destination[X_AXIS];
//            current_position[Y_AXIS] = destination[Y_AXIS];
//
//            HOMEAXIS(Z);
//          }
//                                                // Let's see if X and Y are homed and probe is inside bed area.
//          if(code_seen(axis_codes[Z_AXIS])) {
//            if ( (axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]) \
//              && (current_position[X_AXIS]+X_PROBE_OFFSET_FROM_EXTRUDER >= X_MIN_POS) \
//              && (current_position[X_AXIS]+X_PROBE_OFFSET_FROM_EXTRUDER <= X_MAX_POS) \
//              && (current_position[Y_AXIS]+Y_PROBE_OFFSET_FROM_EXTRUDER >= Y_MIN_POS) \
//              && (current_position[Y_AXIS]+Y_PROBE_OFFSET_FROM_EXTRUDER <= Y_MAX_POS)) {
//
//              current_position[Z_AXIS] = 0;
//              plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
//              destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
//              feedrate = max_feedrate[Z_AXIS];
//              plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
//              st_synchronize();
//
//              HOMEAXIS(Z);
//            } else if (!((axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]))) {
//                LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
//                SERIAL_ECHO_START;
//                SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
//            } else {
//                LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
//                SERIAL_ECHO_START;
//                SERIAL_ECHOLNPGM(MSG_ZPROBE_OUT);
//            }
//          }
//        #endif
//      #endif
//
//
//
//      if(code_seen(axis_codes[Z_AXIS])) {
//        if(code_value_long() != 0) {
//          current_position[Z_AXIS]=code_value()+add_homing[Z_AXIS];
//        }
//      }
//      #ifdef ENABLE_AUTO_BED_LEVELING
//        if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
//          current_position[Z_AXIS] += zprobe_zoffset;  //Add Z_Probe offset (the distance is negative)
//        }
//      #endif
//      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
//#endif // else DELTA
//
//#ifdef SCARA
//    calculate_delta(current_position);
//      plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
//#endif // SCARA
//
//      #ifdef ENDSTOPS_ONLY_FOR_HOMING
//        enable_endstops(false);
//      #endif
//
//      feedrate = saved_feedrate;
//      feedmultiply = saved_feedmultiply;
//      previous_millis_cmd = millis();
//      endstops_hit_on_purpose();
//      break;
//
//#ifdef ENABLE_AUTO_BED_LEVELING
//    case 29: // G29 Detailed Z-Probe, probes the bed at 3 or more points.
//        {
//            #if Z_MIN_PIN == -1
//            #error "You must have a Z_MIN endstop in order to enable Auto Bed Leveling feature!!! Z_MIN_PIN must point to a valid hardware pin."
//            #endif
//
//            // Prevent user from running a G29 without first homing in X and Y
//            if (! (axis_known_position[X_AXIS] && axis_known_position[Y_AXIS]) )
//            {
//                LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
//                SERIAL_ECHO_START;
//                SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
//                break; // abort G29, since we don't know where we are
//            }
//
//#ifdef Z_PROBE_SLED
//            dock_sled(false);
//#endif // Z_PROBE_SLED
//            st_synchronize();
//            // make sure the bed_level_rotation_matrix is identity or the planner will get it incorectly
//            //vector_3 corrected_position = plan_get_position_mm();
//            //corrected_position.debug("position before G29");
//            plan_bed_level_matrix.set_to_identity();
//            vector_3 uncorrected_position = plan_get_position();
//            //uncorrected_position.debug("position durring G29");
//            current_position[X_AXIS] = uncorrected_position.x;
//            current_position[Y_AXIS] = uncorrected_position.y;
//            current_position[Z_AXIS] = uncorrected_position.z;
//            plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
//            setup_for_endstop_move();
//
//            feedrate = homing_feedrate[Z_AXIS];
//#ifdef AUTO_BED_LEVELING_GRID
//            // probe at the points of a lattice grid
//
//            int xGridSpacing = (RIGHT_PROBE_BED_POSITION - LEFT_PROBE_BED_POSITION) / (AUTO_BED_LEVELING_GRID_POINTS-1);
//            int yGridSpacing = (BACK_PROBE_BED_POSITION - FRONT_PROBE_BED_POSITION) / (AUTO_BED_LEVELING_GRID_POINTS-1);
//
//
//            // solve the plane equation ax + by + d = z
//            // A is the matrix with rows [x y 1] for all the probed points
//            // B is the vector of the Z positions
//            // the normal vector to the plane is formed by the coefficients of the plane equation in the standard form, which is Vx*x+Vy*y+Vz*z+d = 0
//            // so Vx = -a Vy = -b Vz = 1 (we want the vector facing towards positive Z
//
//            // "A" matrix of the linear system of equations
//            double eqnAMatrix[AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS*3];
//            // "B" vector of Z points
//            double eqnBVector[AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS];
//
//
//            int probePointCounter = 0;
//            bool zig = true;
//
//            for (int yProbe=FRONT_PROBE_BED_POSITION; yProbe <= BACK_PROBE_BED_POSITION; yProbe += yGridSpacing)
//            {
//              int xProbe, xInc;
//              if (zig)
//              {
//                xProbe = LEFT_PROBE_BED_POSITION;
//                //xEnd = RIGHT_PROBE_BED_POSITION;
//                xInc = xGridSpacing;
//                zig = false;
//              } else // zag
//              {
//                xProbe = RIGHT_PROBE_BED_POSITION;
//                //xEnd = LEFT_PROBE_BED_POSITION;
//                xInc = -xGridSpacing;
//                zig = true;
//              }
//
//              for (int xCount=0; xCount < AUTO_BED_LEVELING_GRID_POINTS; xCount++)
//              {
//                float z_before;
//                if (probePointCounter == 0)
//                {
//                  // raise before probing
//                  z_before = Z_RAISE_BEFORE_PROBING;
//                } else
//                {
//                  // raise extruder
//                  z_before = current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS;
//                }
//
//                float measured_z = probe_pt(xProbe, yProbe, z_before);
//
//                eqnBVector[probePointCounter] = measured_z;
//
//                eqnAMatrix[probePointCounter + 0*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = xProbe;
//                eqnAMatrix[probePointCounter + 1*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = yProbe;
//                eqnAMatrix[probePointCounter + 2*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = 1;
//                probePointCounter++;
//                xProbe += xInc;
//              }
//            }
//            clean_up_after_endstop_move();
//
//            // solve lsq problem
//            double *plane_equation_coefficients = qr_solve(AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS, 3, eqnAMatrix, eqnBVector);
//
//            SERIAL_PROTOCOLPGM("Eqn coefficients: a: ");
//            SERIAL_PROTOCOL(plane_equation_coefficients[0]);
//            SERIAL_PROTOCOLPGM(" b: ");
//            SERIAL_PROTOCOL(plane_equation_coefficients[1]);
//            SERIAL_PROTOCOLPGM(" d: ");
//            SERIAL_PROTOCOLLN(plane_equation_coefficients[2]);
//
//
//            set_bed_level_equation_lsq(plane_equation_coefficients);
//
//            free(plane_equation_coefficients);
//
//#else // AUTO_BED_LEVELING_GRID not defined
//
//            // Probe at 3 arbitrary points
//            // probe 1
//            float z_at_pt_1 = probe_pt(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, Z_RAISE_BEFORE_PROBING);
//
//            // probe 2
//            float z_at_pt_2 = probe_pt(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);
//
//            // probe 3
//            float z_at_pt_3 = probe_pt(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);
//
//            clean_up_after_endstop_move();
//
//            set_bed_level_equation_3pts(z_at_pt_1, z_at_pt_2, z_at_pt_3);
//
//
//#endif // AUTO_BED_LEVELING_GRID
//            st_synchronize();
//
//            // The following code correct the Z height difference from z-probe position and hotend tip position.
//            // The Z height on homing is measured by Z-Probe, but the probe is quite far from the hotend.
//            // When the bed is uneven, this height must be corrected.
//            real_z = float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS];  //get the real Z (since the auto bed leveling is already correcting the plane)
//            x_tmp = current_position[X_AXIS] + X_PROBE_OFFSET_FROM_EXTRUDER;
//            y_tmp = current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_EXTRUDER;
//            z_tmp = current_position[Z_AXIS];
//
//            apply_rotation_xyz(plan_bed_level_matrix, x_tmp, y_tmp, z_tmp);         //Apply the correction sending the probe offset
//            current_position[Z_AXIS] = z_tmp - real_z + current_position[Z_AXIS];   //The difference is added to current position and sent to planner.
//            plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
//#ifdef Z_PROBE_SLED
//            dock_sled(true, -SLED_DOCKING_OFFSET); // correct for over travel.
//#endif // Z_PROBE_SLED
//        }
//        break;
//#ifndef Z_PROBE_SLED
//    case 30: // G30 Single Z Probe
//        {
//            engage_z_probe(); // Engage Z Servo endstop if available
//            st_synchronize();
//            // TODO: make sure the bed_level_rotation_matrix is identity or the planner will get set incorectly
//            setup_for_endstop_move();
//
//            feedrate = homing_feedrate[Z_AXIS];
//
//            run_z_probe();
//            SERIAL_PROTOCOLPGM(MSG_BED);
//            SERIAL_PROTOCOLPGM(" X: ");
//            SERIAL_PROTOCOL(current_position[X_AXIS]);
//            SERIAL_PROTOCOLPGM(" Y: ");
//            SERIAL_PROTOCOL(current_position[Y_AXIS]);
//            SERIAL_PROTOCOLPGM(" Z: ");
//            SERIAL_PROTOCOL(current_position[Z_AXIS]);
//            SERIAL_PROTOCOLPGM("\n");
//
//            clean_up_after_endstop_move();
//            retract_z_probe(); // Retract Z Servo endstop if available
//        }
//        break;
//#else
//    case 31: // dock the sled
//        dock_sled(true);
//        break;
//    case 32: // undock the sled
//        dock_sled(false);
//        break;
//#endif // Z_PROBE_SLED
//#endif // ENABLE_AUTO_BED_LEVELING
//    case 90: // G90
//      relative_mode = false;
//      break;
//    case 91: // G91
//      relative_mode = true;
//      break;
//    case 92: // G92
//      if(!code_seen(axis_codes[E_AXIS]))
//        st_synchronize();
//      for(int8_t i=0; i < NUM_AXIS; i++) {
//        if(code_seen(axis_codes[i])) {
//           if(i == E_AXIS) {
//             current_position[i] = code_value();
//             plan_set_e_position(current_position[E_AXIS]);
//           }
//           else {
//#ifdef SCARA
//    if (i == X_AXIS || i == Y_AXIS) {
//                  current_position[i] = code_value();  
//    }
//    else {
//                current_position[i] = code_value()+add_homing[i];  
//              }  
//#else
//    current_position[i] = code_value()+add_homing[i];
//#endif
//            plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
//           }
//        }
//      }
//      break;
//    }
//  } // mayis - end of code 'G'

// mayis - start of code 'M'
//  else if(code_seen('M'))
//  {
//    switch( (int)code_value() )
//    {
//#ifdef ULTIPANEL
//    case 0: // M0 - Unconditional stop - Wait for user button press on LCD
//    case 1: // M1 - Conditional stop - Wait for user button press on LCD
//    {
//      char *src = strchr_pointer + 2;
//
//      codenum = 0;
//
//      bool hasP = false, hasS = false;
//      if (code_seen('P')) {
//        codenum = code_value(); // milliseconds to wait
//        hasP = codenum > 0;
//      }
//      if (code_seen('S')) {
//        codenum = code_value() * 1000; // seconds to wait
//        hasS = codenum > 0;
//      }
//      starpos = strchr(src, '*');
//      if (starpos != NULL) *(starpos) = '\0';
//      while (*src == ' ') ++src;
//      if (!hasP && !hasS && *src != '\0') {
//        lcd_setstatus(src);
//      } else {
//        LCD_MESSAGEPGM(MSG_USERWAIT);
//      }
//
//      lcd_ignore_click();
//      st_synchronize();
//      previous_millis_cmd = millis();
//      if (codenum > 0){
//        codenum += millis();  // keep track of when we started waiting
//        while(millis() < codenum && !lcd_clicked()){
//          manage_heater();
//          manage_inactivity();
//          lcd_update();
//        }
//        lcd_ignore_click(false);
//      }else{
//          if (!lcd_detected())
//            break;
//        while(!lcd_clicked()){
//          manage_heater();
//          manage_inactivity();
//          lcd_update();
//        }
//      }
//      if (IS_SD_PRINTING)
//        LCD_MESSAGEPGM(MSG_RESUMING);
//      else
//        LCD_MESSAGEPGM(WELCOME_MSG);
//    }
//    break;
//#endif
//    case 17:
//        LCD_MESSAGEPGM(MSG_NO_MOVE);
//        enable_x();
//        enable_y();
//        enable_z();
//        enable_e0();
//        enable_e1();
//        enable_e2();
//      break;
//
//#ifdef SDSUPPORT
//    case 20: // M20 - list SD card
//      SERIAL_PROTOCOLLNPGM(MSG_BEGIN_FILE_LIST);
//      card.ls();
//      SERIAL_PROTOCOLLNPGM(MSG_END_FILE_LIST);
//      break;
//    case 21: // M21 - init SD card
//
//      card.initsd();
//
//      break;
//    case 22: //M22 - release SD card
//      card.release();
//
//      break;
//    case 23: //M23 - Select file
//      starpos = (strchr(strchr_pointer + 4,'*'));
//      if(starpos!=NULL)
//        *(starpos)='\0';
//      card.openFile(strchr_pointer + 4,true);
//      break;
//    case 24: //M24 - Start SD print
//      card.startFileprint();
//      starttime=millis();
//      break;
//    case 25: //M25 - Pause SD print
//      card.pauseSDPrint();
//      break;
//    case 26: //M26 - Set SD index
//      if(card.cardOK && code_seen('S')) {
//        card.setIndex(code_value_long());
//      }
//      break;
//    case 27: //M27 - Get SD status
//      card.getStatus();
//      break;
//    case 28: //M28 - Start SD write
//      starpos = (strchr(strchr_pointer + 4,'*'));
//      if(starpos != NULL){
//        char* npos = strchr(cmdbuffer[bufindr], 'N');
//        strchr_pointer = strchr(npos,' ') + 1;
//        *(starpos) = '\0';
//      }
//      card.openFile(strchr_pointer+4,false);
//      break;
//    case 29: //M29 - Stop SD write
//      //processed in write to file routine above
//      //card,saving = false;
//      break;
//    case 30: //M30 <filename> Delete File
//      if (card.cardOK){
//        card.closefile();
//        starpos = (strchr(strchr_pointer + 4,'*'));
//        if(starpos != NULL){
//          char* npos = strchr(cmdbuffer[bufindr], 'N');
//          strchr_pointer = strchr(npos,' ') + 1;
//          *(starpos) = '\0';
//        }
//        card.removeFile(strchr_pointer + 4);
//      }
//      break;
//    case 32: //M32 - Select file and start SD print
//    {
//      if(card.sdprinting) {
//        st_synchronize();
//
//      }
//      starpos = (strchr(strchr_pointer + 4,'*'));
//
//      char* namestartpos = (strchr(strchr_pointer + 4,'!'));   //find ! to indicate filename string start.
//      if(namestartpos==NULL)
//      {
//        namestartpos=strchr_pointer + 4; //default name position, 4 letters after the M
//      }
//      else
//        namestartpos++; //to skip the '!'
//
//      if(starpos!=NULL)
//        *(starpos)='\0';
//
//      bool call_procedure=(code_seen('P'));
//
//      if(strchr_pointer>namestartpos)
//        call_procedure=false;  //false alert, 'P' found within filename
//
//      if( card.cardOK )
//      {
//        card.openFile(namestartpos,true,!call_procedure);
//        if(code_seen('S'))
//          if(strchr_pointer<namestartpos) //only if "S" is occuring _before_ the filename
//            card.setIndex(code_value_long());
//        card.startFileprint();
//        if(!call_procedure)
//          starttime=millis(); //procedure calls count as normal print time.
//      }
//    } break;
//    case 928: //M928 - Start SD write
//      starpos = (strchr(strchr_pointer + 5,'*'));
//      if(starpos != NULL){
//        char* npos = strchr(cmdbuffer[bufindr], 'N');
//        strchr_pointer = strchr(npos,' ') + 1;
//        *(starpos) = '\0';
//      }
//      card.openLogFile(strchr_pointer+5);
//      break;
//
//#endif //SDSUPPORT
//
//    case 31: //M31 take time since the start of the SD print or an M109 command
//      {
//      stoptime=millis();
//      char time[30];
//      unsigned long t=(stoptime-starttime)/1000;
//      int sec,min;
//      min=t/60;
//      sec=t%60;
//      sprintf_P(time, PSTR("%i min, %i sec"), min, sec);
//      SERIAL_ECHO_START;
//      SERIAL_ECHOLN(time);
//      lcd_setstatus(time);
//      autotempShutdown();
//      }
//      break;
//    case 42: //M42 -Change pin status via gcode
//      if (code_seen('S'))
//      {
//        int pin_status = code_value();
//        int pin_number = LED_PIN;
//        if (code_seen('P') && pin_status >= 0 && pin_status <= 255)
//          pin_number = code_value();
//        for(int8_t i = 0; i < (int8_t)(sizeof(sensitive_pins)/sizeof(int)); i++)
//        {
//          if (sensitive_pins[i] == pin_number)
//          {
//            pin_number = -1;
//            break;
//          }
//        }
//      #if defined(FAN_PIN) && FAN_PIN > -1
//        if (pin_number == FAN_PIN)
//          fanSpeed = pin_status;
//      #endif
//        if (pin_number > -1)
//        {
//          pinMode(pin_number, OUTPUT);
//          digitalWrite(pin_number, pin_status);
//          analogWrite(pin_number, pin_status);
//        }
//      }
//     break;
//
//// M48 Z-Probe repeatability measurement function.
////
//// Usage:   M48 <n #_samples> <X X_position_for_samples> <Y Y_position_for_samples> <V Verbose_Level> <Engage_probe_for_each_reading> <L legs_of_movement_prior_to_doing_probe>
////  
//// This function assumes the bed has been homed.  Specificaly, that a G28 command
//// as been issued prior to invoking the M48 Z-Probe repeatability measurement function.
//// Any information generated by a prior G29 Bed leveling command will be lost and need to be
//// regenerated.
////
//// The number of samples will default to 10 if not specified.  You can use upper or lower case
//// letters for any of the options EXCEPT n.  n must be in lower case because Marlin uses a capital
//// N for its communication protocol and will get horribly confused if you send it a capital N.
////
//
//#ifdef ENABLE_AUTO_BED_LEVELING
//#ifdef Z_PROBE_REPEATABILITY_TEST 
//
//    case 48: // M48 Z-Probe repeatability
//        {
//            #if Z_MIN_PIN == -1
//            #error "You must have a Z_MIN endstop in order to enable calculation of Z-Probe repeatability."
//            #endif
//
//  double sum=0.0; 
//  double mean=0.0; 
//  double sigma=0.0;
//  double sample_set[50];
//  int verbose_level=1, n=0, j, n_samples = 10, n_legs=0, engage_probe_for_each_reading=0 ;
//  double X_current, Y_current, Z_current;
//  double X_probe_location, Y_probe_location, Z_start_location, ext_position;
//  
//  if (code_seen('V') || code_seen('v')) {
//          verbose_level = code_value();
//    if (verbose_level<0 || verbose_level>4 ) {
//      SERIAL_PROTOCOLPGM("?Verbose Level not plausable.\n");
//      goto Sigma_Exit;
//    }
//  }
//
//  if (verbose_level > 0)   {
//    SERIAL_PROTOCOLPGM("M48 Z-Probe Repeatability test.   Version 2.00\n");
//    SERIAL_PROTOCOLPGM("Full support at: http://3dprintboard.com/forum.php\n");
//  }
//
//  if (code_seen('n')) {
//          n_samples = code_value();
//    if (n_samples<4 || n_samples>50 ) {
//      SERIAL_PROTOCOLPGM("?Specified sample size not plausable.\n");
//      goto Sigma_Exit;
//    }
//  }
//
//  X_current = X_probe_location = st_get_position_mm(X_AXIS);
//  Y_current = Y_probe_location = st_get_position_mm(Y_AXIS);
//  Z_current = st_get_position_mm(Z_AXIS);
//  Z_start_location = st_get_position_mm(Z_AXIS) + Z_RAISE_BEFORE_PROBING;
//  ext_position   = st_get_position_mm(E_AXIS);
//
//  if (code_seen('E') || code_seen('e') ) 
//    engage_probe_for_each_reading++;
//
//  if (code_seen('X') || code_seen('x') ) {
//          X_probe_location = code_value() -  X_PROBE_OFFSET_FROM_EXTRUDER;
//    if (X_probe_location<X_MIN_POS || X_probe_location>X_MAX_POS ) {
//      SERIAL_PROTOCOLPGM("?Specified X position out of range.\n");
//      goto Sigma_Exit;
//    }
//  }
//
//  if (code_seen('Y') || code_seen('y') ) {
//          Y_probe_location = code_value() -  Y_PROBE_OFFSET_FROM_EXTRUDER;
//    if (Y_probe_location<Y_MIN_POS || Y_probe_location>Y_MAX_POS ) {
//      SERIAL_PROTOCOLPGM("?Specified Y position out of range.\n");
//      goto Sigma_Exit;
//    }
//  }
//
//  if (code_seen('L') || code_seen('l') ) {
//          n_legs = code_value();
//    if ( n_legs==1 ) 
//      n_legs = 2;
//    if ( n_legs<0 || n_legs>15 ) {
//      SERIAL_PROTOCOLPGM("?Specified number of legs in movement not plausable.\n");
//      goto Sigma_Exit;
//    }
//  }
//
////
//// Do all the preliminary setup work.   First raise the probe.
////
//
//        st_synchronize();
//        plan_bed_level_matrix.set_to_identity();
//  plan_buffer_line( X_current, Y_current, Z_start_location,
//      ext_position,
//          homing_feedrate[Z_AXIS]/60,
//      active_extruder);
//        st_synchronize();
//
////
//// Now get everything to the specified probe point So we can safely do a probe to
//// get us close to the bed.  If the Z-Axis is far from the bed, we don't want to 
//// use that as a starting point for each probe.
////
//  if (verbose_level > 2) 
//    SERIAL_PROTOCOL("Positioning probe for the test.\n");
//
//  plan_buffer_line( X_probe_location, Y_probe_location, Z_start_location,
//      ext_position,
//          homing_feedrate[X_AXIS]/60,
//      active_extruder);
//        st_synchronize();
//
//  current_position[X_AXIS] = X_current = st_get_position_mm(X_AXIS);
//  current_position[Y_AXIS] = Y_current = st_get_position_mm(Y_AXIS);
//  current_position[Z_AXIS] = Z_current = st_get_position_mm(Z_AXIS);
//  current_position[E_AXIS] = ext_position = st_get_position_mm(E_AXIS);
//
//// 
//// OK, do the inital probe to get us close to the bed.
//// Then retrace the right amount and use that in subsequent probes
////
//
//        engage_z_probe(); 
//
//  setup_for_endstop_move();
//  run_z_probe();
//
//  current_position[Z_AXIS] = Z_current = st_get_position_mm(Z_AXIS);
//  Z_start_location = st_get_position_mm(Z_AXIS) + Z_RAISE_BEFORE_PROBING;
//
//  plan_buffer_line( X_probe_location, Y_probe_location, Z_start_location,
//      ext_position,
//          homing_feedrate[X_AXIS]/60,
//      active_extruder);
//        st_synchronize();
//  current_position[Z_AXIS] = Z_current = st_get_position_mm(Z_AXIS);
//
//  if (engage_probe_for_each_reading)
//          retract_z_probe();
//
//        for( n=0; n<n_samples; n++) {
//
//    do_blocking_move_to( X_probe_location, Y_probe_location, Z_start_location); // Make sure we are at the probe location
//
//    if ( n_legs)  {
//    double radius=0.0, theta=0.0, x_sweep, y_sweep;
//    int rotational_direction, l;
//
//      rotational_direction = (unsigned long) millis() & 0x0001;     // clockwise or counter clockwise
//      radius = (unsigned long) millis() % (long) (X_MAX_LENGTH/4);      // limit how far out to go 
//      theta = (float) ((unsigned long) millis() % (long) 360) / (360./(2*3.1415926)); // turn into radians
//
////SERIAL_ECHOPAIR("starting radius: ",radius);
////SERIAL_ECHOPAIR("   theta: ",theta);
////SERIAL_ECHOPAIR("   direction: ",rotational_direction);
////SERIAL_PROTOCOLLNPGM("");
//
//      for( l=0; l<n_legs-1; l++) {
//        if (rotational_direction==1)
//          theta += (float) ((unsigned long) millis() % (long) 20) / (360.0/(2*3.1415926)); // turn into radians
//        else
//          theta -= (float) ((unsigned long) millis() % (long) 20) / (360.0/(2*3.1415926)); // turn into radians
//
//        radius += (float) ( ((long) ((unsigned long) millis() % (long) 10)) - 5);
//        if ( radius<0.0 )
//          radius = -radius;
//
//        X_current = X_probe_location + cos(theta) * radius;
//        Y_current = Y_probe_location + sin(theta) * radius;
//
//        if ( X_current<X_MIN_POS)   // Make sure our X & Y are sane
//           X_current = X_MIN_POS;
//        if ( X_current>X_MAX_POS)
//           X_current = X_MAX_POS;
//
//        if ( Y_current<Y_MIN_POS)   // Make sure our X & Y are sane
//           Y_current = Y_MIN_POS;
//        if ( Y_current>Y_MAX_POS)
//           Y_current = Y_MAX_POS;
//
//        if (verbose_level>3 ) {
//          SERIAL_ECHOPAIR("x: ", X_current);
//          SERIAL_ECHOPAIR("y: ", Y_current);
//          SERIAL_PROTOCOLLNPGM("");
//        }
//
//        do_blocking_move_to( X_current, Y_current, Z_current );
//      }
//      do_blocking_move_to( X_probe_location, Y_probe_location, Z_start_location); // Go back to the probe location
//    }
//
//    if (engage_probe_for_each_reading)  {
//            engage_z_probe(); 
//              delay(1000);
//    }
//
//    setup_for_endstop_move();
//                run_z_probe();
//
//    sample_set[n] = current_position[Z_AXIS];
//
////
//// Get the current mean for the data points we have so far
////
//    sum=0.0; 
//    for( j=0; j<=n; j++) {
//      sum = sum + sample_set[j];
//    }
//    mean = sum / (double (n+1));
////
//// Now, use that mean to calculate the standard deviation for the
//// data points we have so far
////
//
//    sum=0.0; 
//    for( j=0; j<=n; j++) {
//      sum = sum + (sample_set[j]-mean) * (sample_set[j]-mean);
//    }
//    sigma = sqrt( sum / (double (n+1)) );
//
//    if (verbose_level > 1) {
//      SERIAL_PROTOCOL(n+1);
//      SERIAL_PROTOCOL(" of ");
//      SERIAL_PROTOCOL(n_samples);
//      SERIAL_PROTOCOLPGM("   z: ");
//      SERIAL_PROTOCOL_F(current_position[Z_AXIS], 6);
//    }
//
//    if (verbose_level > 2) {
//      SERIAL_PROTOCOL(" mean: ");
//      SERIAL_PROTOCOL_F(mean,6);
//
//      SERIAL_PROTOCOL("   sigma: ");
//      SERIAL_PROTOCOL_F(sigma,6);
//    }
//
//    if (verbose_level > 0) 
//      SERIAL_PROTOCOLPGM("\n");
//
//    plan_buffer_line( X_probe_location, Y_probe_location, Z_start_location, 
//          current_position[E_AXIS], homing_feedrate[Z_AXIS]/60, active_extruder);
//          st_synchronize();
//
//    if (engage_probe_for_each_reading)  {
//            retract_z_probe();  
//              delay(1000);
//    }
//  }
//
//        retract_z_probe();
//  delay(1000);
//
//        clean_up_after_endstop_move();
//
////      enable_endstops(true);
//
//  if (verbose_level > 0) {
//    SERIAL_PROTOCOLPGM("Mean: ");
//    SERIAL_PROTOCOL_F(mean, 6);
//    SERIAL_PROTOCOLPGM("\n");
//  }
//
//SERIAL_PROTOCOLPGM("Standard Deviation: ");
//SERIAL_PROTOCOL_F(sigma, 6);
//SERIAL_PROTOCOLPGM("\n\n");
//
//Sigma_Exit:
//        break;
//  }
//#endif    // Z_PROBE_REPEATABILITY_TEST 
//#endif    // ENABLE_AUTO_BED_LEVELING
//
//    case 104: // M104
//      if(setTargetedHotend(104)){
//        break;
//      }
//      if (code_seen('S')) setTargetHotend(code_value(), tmp_extruder);
//#ifdef DUAL_X_CARRIAGE
//      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && tmp_extruder == 0)
//        setTargetHotend1(code_value() == 0.0 ? 0.0 : code_value() + duplicate_extruder_temp_offset);
//#endif
//      setWatch();
//      break;
//    case 112: //  M112 -Emergency Stop
//      kill();
//      break;
//    case 140: // M140 set bed temp
//      if (code_seen('S')) setTargetBed(code_value());
//      break;
//    case 105 : // M105
//      if(setTargetedHotend(105)){
//        break;
//        }
//      #if defined(TEMP_0_PIN) && TEMP_0_PIN > -1
//        SERIAL_PROTOCOLPGM("ok T:");
//        SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
//        SERIAL_PROTOCOLPGM(" /");
//        SERIAL_PROTOCOL_F(degTargetHotend(tmp_extruder),1);
//        #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
//          SERIAL_PROTOCOLPGM(" B:");
//          SERIAL_PROTOCOL_F(degBed(),1);
//          SERIAL_PROTOCOLPGM(" /");
//          SERIAL_PROTOCOL_F(degTargetBed(),1);
//        #endif //TEMP_BED_PIN
//        for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
//          SERIAL_PROTOCOLPGM(" T");
//          SERIAL_PROTOCOL(cur_extruder);
//          SERIAL_PROTOCOLPGM(":");
//          SERIAL_PROTOCOL_F(degHotend(cur_extruder),1);
//          SERIAL_PROTOCOLPGM(" /");
//          SERIAL_PROTOCOL_F(degTargetHotend(cur_extruder),1);
//        }
//      #else
//        SERIAL_ERROR_START;
//        SERIAL_ERRORLNPGM(MSG_ERR_NO_THERMISTORS);
//      #endif
//
//        SERIAL_PROTOCOLPGM(" @:");
//      #ifdef EXTRUDER_WATTS
//        SERIAL_PROTOCOL((EXTRUDER_WATTS * getHeaterPower(tmp_extruder))/127);
//        SERIAL_PROTOCOLPGM("W");
//      #else
//        SERIAL_PROTOCOL(getHeaterPower(tmp_extruder));
//      #endif
//
//        SERIAL_PROTOCOLPGM(" B@:");
//      #ifdef BED_WATTS
//        SERIAL_PROTOCOL((BED_WATTS * getHeaterPower(-1))/127);
//        SERIAL_PROTOCOLPGM("W");
//      #else
//        SERIAL_PROTOCOL(getHeaterPower(-1));
//      #endif
//
//        #ifdef SHOW_TEMP_ADC_VALUES
//          #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
//            SERIAL_PROTOCOLPGM("    ADC B:");
//            SERIAL_PROTOCOL_F(degBed(),1);
//            SERIAL_PROTOCOLPGM("C->");
//            SERIAL_PROTOCOL_F(rawBedTemp()/OVERSAMPLENR,0);
//          #endif
//          for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
//            SERIAL_PROTOCOLPGM("  T");
//            SERIAL_PROTOCOL(cur_extruder);
//            SERIAL_PROTOCOLPGM(":");
//            SERIAL_PROTOCOL_F(degHotend(cur_extruder),1);
//            SERIAL_PROTOCOLPGM("C->");
//            SERIAL_PROTOCOL_F(rawHotendTemp(cur_extruder)/OVERSAMPLENR,0);
//          }
//        #endif
//
//        SERIAL_PROTOCOLLN("");
//      return;
//      break;
//    case 109:
//    {// M109 - Wait for extruder heater to reach target.
//      if(setTargetedHotend(109)){
//        break;
//      }
//      LCD_MESSAGEPGM(MSG_HEATING);
//      #ifdef AUTOTEMP
//        autotemp_enabled=false;
//      #endif
//      if (code_seen('S')) {
//        setTargetHotend(code_value(), tmp_extruder);
//#ifdef DUAL_X_CARRIAGE
//        if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && tmp_extruder == 0)
//          setTargetHotend1(code_value() == 0.0 ? 0.0 : code_value() + duplicate_extruder_temp_offset);
//#endif
//        CooldownNoWait = true;
//      } else if (code_seen('R')) {
//        setTargetHotend(code_value(), tmp_extruder);
//#ifdef DUAL_X_CARRIAGE
//        if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && tmp_extruder == 0)
//          setTargetHotend1(code_value() == 0.0 ? 0.0 : code_value() + duplicate_extruder_temp_offset);
//#endif
//        CooldownNoWait = false;
//      }
//      #ifdef AUTOTEMP
//        if (code_seen('S')) autotemp_min=code_value();
//        if (code_seen('B')) autotemp_max=code_value();
//        if (code_seen('F'))
//        {
//          autotemp_factor=code_value();
//          autotemp_enabled=true;
//        }
//      #endif
//
//      setWatch();
//      codenum = millis();
//
//      /* See if we are heating up or cooling down */
//      target_direction = isHeatingHotend(tmp_extruder); // true if heating, false if cooling
//
//      cancel_heatup = false;
//
//      #ifdef TEMP_RESIDENCY_TIME
//        long residencyStart;
//        residencyStart = -1;
//        /* continue to loop until we have reached the target temp
//          _and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
//        while((!cancel_heatup)&&((residencyStart == -1) ||
//              (residencyStart >= 0 && (((unsigned int) (millis() - residencyStart)) < (TEMP_RESIDENCY_TIME * 1000UL)))) ) {
//      #else
//        while ( target_direction ? (isHeatingHotend(tmp_extruder)) : (isCoolingHotend(tmp_extruder)&&(CooldownNoWait==false)) ) {
//      #endif //TEMP_RESIDENCY_TIME
//          if( (millis() - codenum) > 1000UL )
//          { //Print Temp Reading and remaining time every 1 second while heating up/cooling down
//            SERIAL_PROTOCOLPGM("T:");
//            SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
//            SERIAL_PROTOCOLPGM(" E:");
//            SERIAL_PROTOCOL((int)tmp_extruder);
//            #ifdef TEMP_RESIDENCY_TIME
//              SERIAL_PROTOCOLPGM(" W:");
//              if(residencyStart > -1)
//              {
//                 codenum = ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residencyStart)) / 1000UL;
//                 SERIAL_PROTOCOLLN( codenum );
//              }
//              else
//              {
//                 SERIAL_PROTOCOLLN( "?" );
//              }
//            #else
//              SERIAL_PROTOCOLLN("");
//            #endif
//            codenum = millis();
//          }
//          manage_heater();
//          manage_inactivity();
//          lcd_update();
//        #ifdef TEMP_RESIDENCY_TIME
//            /* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
//              or when current temp falls outside the hysteresis after target temp was reached */
//          if ((residencyStart == -1 &&  target_direction && (degHotend(tmp_extruder) >= (degTargetHotend(tmp_extruder)-TEMP_WINDOW))) ||
//              (residencyStart == -1 && !target_direction && (degHotend(tmp_extruder) <= (degTargetHotend(tmp_extruder)+TEMP_WINDOW))) ||
//              (residencyStart > -1 && labs(degHotend(tmp_extruder) - degTargetHotend(tmp_extruder)) > TEMP_HYSTERESIS) )
//          {
//            residencyStart = millis();
//          }
//        #endif //TEMP_RESIDENCY_TIME
//        }
//        LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
//        starttime=millis();
//        previous_millis_cmd = millis();
//      }
//      break;
//    case 190: // M190 - Wait for bed heater to reach target.
//    #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
//        LCD_MESSAGEPGM(MSG_BED_HEATING);
//        if (code_seen('S')) {
//          setTargetBed(code_value());
//          CooldownNoWait = true;
//        } else if (code_seen('R')) {
//          setTargetBed(code_value());
//          CooldownNoWait = false;
//        }
//        codenum = millis();
//        
//        cancel_heatup = false;
//        target_direction = isHeatingBed(); // true if heating, false if cooling
//
//        while ( (target_direction)&&(!cancel_heatup) ? (isHeatingBed()) : (isCoolingBed()&&(CooldownNoWait==false)) )
//        {
//          if(( millis() - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up.
//          {
//            float tt=degHotend(active_extruder);
//            SERIAL_PROTOCOLPGM("T:");
//            SERIAL_PROTOCOL(tt);
//            SERIAL_PROTOCOLPGM(" E:");
//            SERIAL_PROTOCOL((int)active_extruder);
//            SERIAL_PROTOCOLPGM(" B:");
//            SERIAL_PROTOCOL_F(degBed(),1);
//            SERIAL_PROTOCOLLN("");
//            codenum = millis();
//          }
//          manage_heater();
//          manage_inactivity();
//          lcd_update();
//        }
//        LCD_MESSAGEPGM(MSG_BED_DONE);
//        previous_millis_cmd = millis();
//    #endif
//        break;
//
//    #if defined(FAN_PIN) && FAN_PIN > -1
//      case 106: //M106 Fan On
//        if (code_seen('S')){
//           fanSpeed=constrain(code_value(),0,255);
//        }
//        else {
//          fanSpeed=255;
//        }
//        break;
//      case 107: //M107 Fan Off
//        fanSpeed = 0;
//        break;
//    #endif //FAN_PIN
//    #ifdef BARICUDA
//      // PWM for HEATER_1_PIN
//      #if defined(HEATER_1_PIN) && HEATER_1_PIN > -1
//        case 126: //M126 valve open
//          if (code_seen('S')){
//             ValvePressure=constrain(code_value(),0,255);
//          }
//          else {
//            ValvePressure=255;
//          }
//          break;
//        case 127: //M127 valve closed
//          ValvePressure = 0;
//          break;
//      #endif //HEATER_1_PIN
//
//      // PWM for HEATER_2_PIN
//      #if defined(HEATER_2_PIN) && HEATER_2_PIN > -1
//        case 128: //M128 valve open
//          if (code_seen('S')){
//             EtoPPressure=constrain(code_value(),0,255);
//          }
//          else {
//            EtoPPressure=255;
//          }
//          break;
//        case 129: //M129 valve closed
//          EtoPPressure = 0;
//          break;
//      #endif //HEATER_2_PIN
//    #endif
//
//    #if defined(PS_ON_PIN) && PS_ON_PIN > -1
//      case 80: // M80 - Turn on Power Supply
//        SET_OUTPUT(PS_ON_PIN); //GND
//        WRITE(PS_ON_PIN, PS_ON_AWAKE);
//
//        // If you have a switch on suicide pin, this is useful
//        // if you want to start another print with suicide feature after
//        // a print without suicide...
//        #if defined SUICIDE_PIN && SUICIDE_PIN > -1
//            SET_OUTPUT(SUICIDE_PIN);
//            WRITE(SUICIDE_PIN, HIGH);
//        #endif
//
//        #ifdef ULTIPANEL
//          powersupply = true;
//          LCD_MESSAGEPGM(WELCOME_MSG);
//          lcd_update();
//        #endif
//        break;
//      #endif
//
//      case 81: // M81 - Turn off Power Supply
//        disable_heater();
//        st_synchronize();
//        disable_e0();
//        disable_e1();
//        disable_e2();
//        finishAndDisableSteppers();
//        fanSpeed = 0;
//        delay(1000); // Wait a little before to switch off
//      #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
//        st_synchronize();
//        suicide();
//      #elif defined(PS_ON_PIN) && PS_ON_PIN > -1
//        SET_OUTPUT(PS_ON_PIN);
//        WRITE(PS_ON_PIN, PS_ON_ASLEEP);
//      #endif
//      #ifdef ULTIPANEL
//        powersupply = false;
//        LCD_MESSAGEPGM(MACHINE_NAME" "MSG_OFF".");
//        lcd_update();
//      #endif
//    break;
//
//    case 82:
//      axis_relative_modes[3] = false;
//      break;
//    case 83:
//      axis_relative_modes[3] = true;
//      break;
//    case 18: //compatibility
//    case 84: // M84
//      if(code_seen('S')){
//        stepper_inactive_time = code_value() * 1000;
//      }
//      else
//      {
//        bool all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS]))|| (code_seen(axis_codes[E_AXIS])));
//        if(all_axis)
//        {
//          st_synchronize();
//          disable_e0();
//          disable_e1();
//          disable_e2();
//          finishAndDisableSteppers();
//        }
//        else
//        {
//          st_synchronize();
//          if(code_seen('X')) disable_x();
//          if(code_seen('Y')) disable_y();
//          if(code_seen('Z')) disable_z();
//          #if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
//            if(code_seen('E')) {
//              disable_e0();
//              disable_e1();
//              disable_e2();
//            }
//          #endif
//        }
//      }
//      break;
//    case 85: // M85
//      if(code_seen('S')) {
//        max_inactive_time = code_value() * 1000;
//      }
//      break;
//    case 92: // M92
//      for(int8_t i=0; i < NUM_AXIS; i++)
//      {
//        if(code_seen(axis_codes[i]))
//        {
//          if(i == 3) { // E
//            float value = code_value();
//            if(value < 20.0) {
//              float factor = axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
//              max_e_jerk *= factor;
//              max_feedrate[i] *= factor;
//              axis_steps_per_sqr_second[i] *= factor;
//            }
//            axis_steps_per_unit[i] = value;
//          }
//          else {
//            axis_steps_per_unit[i] = code_value();
//          }
//        }
//      }
//      break;
//    case 115: // M115
//      SERIAL_PROTOCOLPGM(MSG_M115_REPORT);
//      break;
//    case 117: // M117 display message
//      starpos = (strchr(strchr_pointer + 5,'*'));
//      if(starpos!=NULL)
//        *(starpos)='\0';
//      lcd_setstatus(strchr_pointer + 5);
//      break;
//    case 114: // M114
//      SERIAL_PROTOCOLPGM("X:");
//      SERIAL_PROTOCOL(current_position[X_AXIS]);
//      SERIAL_PROTOCOLPGM(" Y:");
//      SERIAL_PROTOCOL(current_position[Y_AXIS]);
//      SERIAL_PROTOCOLPGM(" Z:");
//      SERIAL_PROTOCOL(current_position[Z_AXIS]);
//      SERIAL_PROTOCOLPGM(" E:");
//      SERIAL_PROTOCOL(current_position[E_AXIS]);
//
//      SERIAL_PROTOCOLPGM(MSG_COUNT_X);
//      SERIAL_PROTOCOL(float(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
//      SERIAL_PROTOCOLPGM(" Y:");
//      SERIAL_PROTOCOL(float(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
//      SERIAL_PROTOCOLPGM(" Z:");
//      SERIAL_PROTOCOL(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);
//
//      SERIAL_PROTOCOLLN("");
//#ifdef SCARA
//    SERIAL_PROTOCOLPGM("SCARA Theta:");
//      SERIAL_PROTOCOL(delta[X_AXIS]);
//      SERIAL_PROTOCOLPGM("   Psi+Theta:");
//      SERIAL_PROTOCOL(delta[Y_AXIS]);
//      SERIAL_PROTOCOLLN("");
//      
//      SERIAL_PROTOCOLPGM("SCARA Cal - Theta:");
//      SERIAL_PROTOCOL(delta[X_AXIS]+add_homing[X_AXIS]);
//      SERIAL_PROTOCOLPGM("   Psi+Theta (90):");
//      SERIAL_PROTOCOL(delta[Y_AXIS]-delta[X_AXIS]-90+add_homing[Y_AXIS]);
//      SERIAL_PROTOCOLLN("");
//      
//      SERIAL_PROTOCOLPGM("SCARA step Cal - Theta:");
//      SERIAL_PROTOCOL(delta[X_AXIS]/90*axis_steps_per_unit[X_AXIS]);
//      SERIAL_PROTOCOLPGM("   Psi+Theta:");
//      SERIAL_PROTOCOL((delta[Y_AXIS]-delta[X_AXIS])/90*axis_steps_per_unit[Y_AXIS]);
//      SERIAL_PROTOCOLLN("");
//      SERIAL_PROTOCOLLN("");
//#endif
//      break;
//    case 120: // M120
//      enable_endstops(false) ;
//      break;
//    case 121: // M121
//      enable_endstops(true) ;
//      break;
//    case 119: // M119
//    SERIAL_PROTOCOLLN(MSG_M119_REPORT);
//      #if defined(X_MIN_PIN) && X_MIN_PIN > -1
//        SERIAL_PROTOCOLPGM(MSG_X_MIN);
//        SERIAL_PROTOCOLLN(((READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
//      #endif
//      #if defined(X_MAX_PIN) && X_MAX_PIN > -1
//        SERIAL_PROTOCOLPGM(MSG_X_MAX);
//        SERIAL_PROTOCOLLN(((READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
//      #endif
//      #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
//        SERIAL_PROTOCOLPGM(MSG_Y_MIN);
//        SERIAL_PROTOCOLLN(((READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
//      #endif
//      #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
//        SERIAL_PROTOCOLPGM(MSG_Y_MAX);
//        SERIAL_PROTOCOLLN(((READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
//      #endif
//      #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
//        SERIAL_PROTOCOLPGM(MSG_Z_MIN);
//        SERIAL_PROTOCOLLN(((READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
//      #endif
//      #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
//        SERIAL_PROTOCOLPGM(MSG_Z_MAX);
//        SERIAL_PROTOCOLLN(((READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
//      #endif
//      break;
//      //TODO: update for all axis, use for loop
//    #ifdef BLINKM
//    case 150: // M150
//      {
//        byte red;
//        byte grn;
//        byte blu;
//
//        if(code_seen('R')) red = code_value();
//        if(code_seen('U')) grn = code_value();
//        if(code_seen('B')) blu = code_value();
//
//        SendColors(red,grn,blu);
//      }
//      break;
//    #endif //BLINKM
//    case 200: // M200 D<millimeters> set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).
//      {
//
//        tmp_extruder = active_extruder;
//        if(code_seen('T')) {
//          tmp_extruder = code_value();
//          if(tmp_extruder >= EXTRUDERS) {
//            SERIAL_ECHO_START;
//            SERIAL_ECHO(MSG_M200_INVALID_EXTRUDER);
//            break;
//          }
//        }
//
//        float area = .0;
//        if(code_seen('D')) {
//      float diameter = (float)code_value();
//      if (diameter == 0.0) {
//      // setting any extruder filament size disables volumetric on the assumption that
//      // slicers either generate in extruder values as cubic mm or as as filament feeds
//      // for all extruders
//        volumetric_enabled = false;
//      } else {
//            filament_size[tmp_extruder] = (float)code_value();
//      // make sure all extruders have some sane value for the filament size
//      filament_size[0] = (filament_size[0] == 0.0 ? DEFAULT_NOMINAL_FILAMENT_DIA : filament_size[0]);
//            #if EXTRUDERS > 1
//      filament_size[1] = (filament_size[1] == 0.0 ? DEFAULT_NOMINAL_FILAMENT_DIA : filament_size[1]);
//            #if EXTRUDERS > 2
//      filament_size[2] = (filament_size[2] == 0.0 ? DEFAULT_NOMINAL_FILAMENT_DIA : filament_size[2]);
//            #endif
//            #endif
//      volumetric_enabled = true;
//      }
//        } else {
//          //reserved for setting filament diameter via UFID or filament measuring device
//          break;
//        }
//    calculate_volumetric_multipliers();
//      }
//      break;
//    case 201: // M201
//      for(int8_t i=0; i < NUM_AXIS; i++)
//      {
//        if(code_seen(axis_codes[i]))
//        {
//          max_acceleration_units_per_sq_second[i] = code_value();
//        }
//      }
//      // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
//      reset_acceleration_rates();
//      break;
//    #if 0 // Not used for Sprinter/grbl gen6
//    case 202: // M202
//      for(int8_t i=0; i < NUM_AXIS; i++) {
//        if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
//      }
//      break;
//    #endif
//    case 203: // M203 max feedrate mm/sec
//      for(int8_t i=0; i < NUM_AXIS; i++) {
//        if(code_seen(axis_codes[i])) max_feedrate[i] = code_value();
//      }
//      break;
//    case 204: // M204 acclereration S normal moves T filmanent only moves
//      {
//        if(code_seen('S')) acceleration = code_value() ;
//        if(code_seen('T')) retract_acceleration = code_value() ;
//      }
//      break;
//    case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
//    {
//      if(code_seen('S')) minimumfeedrate = code_value();
//      if(code_seen('T')) mintravelfeedrate = code_value();
//      if(code_seen('B')) minsegmenttime = code_value() ;
//      if(code_seen('X')) max_xy_jerk = code_value() ;
//      if(code_seen('Z')) max_z_jerk = code_value() ;
//      if(code_seen('E')) max_e_jerk = code_value() ;
//    }
//    break;
//    case 206: // M206 additional homing offset
//      for(int8_t i=0; i < 3; i++)
//      {
//        if(code_seen(axis_codes[i])) add_homing[i] = code_value();
//      }
//    #ifdef SCARA
//     if(code_seen('T'))       // Theta
//      {
//        add_homing[X_AXIS] = code_value() ;
//      }
//      if(code_seen('P'))       // Psi
//      {
//        add_homing[Y_AXIS] = code_value() ;
//      }
//    #endif
//      break;
//    #ifdef DELTA
//  case 665: // M665 set delta configurations L<diagonal_rod> R<delta_radius> S<segments_per_sec>
//    if(code_seen('L')) {
//      delta_diagonal_rod= code_value();
//    }
//    if(code_seen('R')) {
//      delta_radius= code_value();
//    }
//    if(code_seen('S')) {
//      delta_segments_per_second= code_value();
//    }
//    
//    recalc_delta_settings(delta_radius, delta_diagonal_rod);
//    break;
//    case 666: // M666 set delta endstop adjustemnt
//      for(int8_t i=0; i < 3; i++)
//      {
//        if(code_seen(axis_codes[i])) endstop_adj[i] = code_value();
//      }
//      break;
//    #endif
//    #ifdef FWRETRACT
//    case 207: //M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop]
//    {
//      if(code_seen('S'))
//      {
//        retract_length = code_value() ;
//      }
//      if(code_seen('F'))
//      {
//        retract_feedrate = code_value()/60 ;
//      }
//      if(code_seen('Z'))
//      {
//        retract_zlift = code_value() ;
//      }
//    }break;
//    case 208: // M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/min]
//    {
//      if(code_seen('S'))
//      {
//        retract_recover_length = code_value() ;
//      }
//      if(code_seen('F'))
//      {
//        retract_recover_feedrate = code_value()/60 ;
//      }
//    }break;
//    case 209: // M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
//    {
//      if(code_seen('S'))
//      {
//        int t= code_value() ;
//        switch(t)
//        {
//          case 0: 
//          {
//            autoretract_enabled=false;
//            retracted[0]=false;
//            #if EXTRUDERS > 1
//              retracted[1]=false;
//            #endif
//            #if EXTRUDERS > 2
//              retracted[2]=false;
//            #endif
//          }break;
//          case 1: 
//          {
//            autoretract_enabled=true;
//            retracted[0]=false;
//            #if EXTRUDERS > 1
//              retracted[1]=false;
//            #endif
//            #if EXTRUDERS > 2
//              retracted[2]=false;
//            #endif
//          }break;
//          default:
//            SERIAL_ECHO_START;
//            SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
//            SERIAL_ECHO(cmdbuffer[bufindr]);
//            SERIAL_ECHOLNPGM("\"");
//        }
//      }
//
//    }break;
//    #endif // FWRETRACT
//    #if EXTRUDERS > 1
//    case 218: // M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
//    {
//      if(setTargetedHotend(218)){
//        break;
//      }
//      if(code_seen('X'))
//      {
//        extruder_offset[X_AXIS][tmp_extruder] = code_value();
//      }
//      if(code_seen('Y'))
//      {
//        extruder_offset[Y_AXIS][tmp_extruder] = code_value();
//      }
//      #ifdef DUAL_X_CARRIAGE
//      if(code_seen('Z'))
//      {
//        extruder_offset[Z_AXIS][tmp_extruder] = code_value();
//      }
//      #endif
//      SERIAL_ECHO_START;
//      SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
//      for(tmp_extruder = 0; tmp_extruder < EXTRUDERS; tmp_extruder++)
//      {
//         SERIAL_ECHO(" ");
//         SERIAL_ECHO(extruder_offset[X_AXIS][tmp_extruder]);
//         SERIAL_ECHO(",");
//         SERIAL_ECHO(extruder_offset[Y_AXIS][tmp_extruder]);
//      #ifdef DUAL_X_CARRIAGE
//         SERIAL_ECHO(",");
//         SERIAL_ECHO(extruder_offset[Z_AXIS][tmp_extruder]);
//      #endif
//      }
//      SERIAL_ECHOLN("");
//    }break;
//    #endif
//    case 220: // M220 S<factor in percent>- set speed factor override percentage
//    {
//      if(code_seen('S'))
//      {
//        feedmultiply = code_value() ;
//      }
//    }
//    break;
//    case 221: // M221 S<factor in percent>- set extrude factor override percentage
//    {
//      if(code_seen('S'))
//      {
//        int tmp_code = code_value();
//        if (code_seen('T'))
//        {
//          if(setTargetedHotend(221)){
//            break;
//          }
//          extruder_multiply[tmp_extruder] = tmp_code;
//        }
//        else
//        {
//          extrudemultiply = tmp_code ;
//        }
//      }
//    }
//    break;
//
//  case 226: // M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
//  {
//      if(code_seen('P')){
//        int pin_number = code_value(); // pin number
//        int pin_state = -1; // required pin state - default is inverted
//
//        if(code_seen('S')) pin_state = code_value(); // required pin state
//
//        if(pin_state >= -1 && pin_state <= 1){
//
//          for(int8_t i = 0; i < (int8_t)(sizeof(sensitive_pins)/sizeof(int)); i++)
//          {
//            if (sensitive_pins[i] == pin_number)
//            {
//              pin_number = -1;
//              break;
//            }
//          }
//
//          if (pin_number > -1)
//          {
//            int target = LOW;
//
//            st_synchronize();
//
//            pinMode(pin_number, INPUT);
//
//            switch(pin_state){
//            case 1:
//              target = HIGH;
//              break;
//
//            case 0:
//              target = LOW;
//              break;
//
//            case -1:
//              target = !digitalRead(pin_number);
//              break;
//            }
//
//            while(digitalRead(pin_number) != target){
//              manage_heater();
//              manage_inactivity();
//              lcd_update();
//            }
//          }
//        }
//      }
//    }
//    break;
//
//    #if NUM_SERVOS > 0
//    case 280: // M280 - set servo position absolute. P: servo index, S: angle or microseconds
//      {
//        int servo_index = -1;
//        int servo_position = 0;
//        if (code_seen('P'))
//          servo_index = code_value();
//        if (code_seen('S')) {
//          servo_position = code_value();
//          if ((servo_index >= 0) && (servo_index < NUM_SERVOS)) {
//#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
//          servos[servo_index].attach(0);
//#endif
//            servos[servo_index].write(servo_position);
//#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
//              delay(PROBE_SERVO_DEACTIVATION_DELAY);
//              servos[servo_index].detach();
//#endif
//          }
//          else {
//            SERIAL_ECHO_START;
//            SERIAL_ECHO("Servo ");
//            SERIAL_ECHO(servo_index);
//            SERIAL_ECHOLN(" out of range");
//          }
//        }
//        else if (servo_index >= 0) {
//          SERIAL_PROTOCOL(MSG_OK);
//          SERIAL_PROTOCOL(" Servo ");
//          SERIAL_PROTOCOL(servo_index);
//          SERIAL_PROTOCOL(": ");
//          SERIAL_PROTOCOL(servos[servo_index].read());
//          SERIAL_PROTOCOLLN("");
//        }
//      }
//      break;
//    #endif // NUM_SERVOS > 0
//
//    #if (LARGE_FLASH == true && ( BEEPER > 0 || defined(ULTRALCD) || defined(LCD_USE_I2C_BUZZER)))
//    case 300: // M300
//    {
//      int beepS = code_seen('S') ? code_value() : 110;
//      int beepP = code_seen('P') ? code_value() : 1000;
//      if (beepS > 0)
//      {
//        #if BEEPER > 0
//          tone(BEEPER, beepS);
//          delay(beepP);
//          noTone(BEEPER);
//        #elif defined(ULTRALCD)
//      lcd_buzz(beepS, beepP);
//    #elif defined(LCD_USE_I2C_BUZZER)
//      lcd_buzz(beepP, beepS);
//        #endif
//      }
//      else
//      {
//        delay(beepP);
//      }
//    }
//    break;
//    #endif // M300
//
//    #ifdef PIDTEMP
//    case 301: // M301
//      {
//        if(code_seen('P')) Kp = code_value();
//        if(code_seen('I')) Ki = scalePID_i(code_value());
//        if(code_seen('D')) Kd = scalePID_d(code_value());
//
//        #ifdef PID_ADD_EXTRUSION_RATE
//        if(code_seen('C')) Kc = code_value();
//        #endif
//
//        updatePID();
//        SERIAL_PROTOCOL(MSG_OK);
//        SERIAL_PROTOCOL(" p:");
//        SERIAL_PROTOCOL(Kp);
//        SERIAL_PROTOCOL(" i:");
//        SERIAL_PROTOCOL(unscalePID_i(Ki));
//        SERIAL_PROTOCOL(" d:");
//        SERIAL_PROTOCOL(unscalePID_d(Kd));
//        #ifdef PID_ADD_EXTRUSION_RATE
//        SERIAL_PROTOCOL(" c:");
//        //Kc does not have scaling applied above, or in resetting defaults
//        SERIAL_PROTOCOL(Kc);
//        #endif
//        SERIAL_PROTOCOLLN("");
//      }
//      break;
//    #endif //PIDTEMP
//    #ifdef PIDTEMPBED
//    case 304: // M304
//      {
//        if(code_seen('P')) bedKp = code_value();
//        if(code_seen('I')) bedKi = scalePID_i(code_value());
//        if(code_seen('D')) bedKd = scalePID_d(code_value());
//
//        updatePID();
//        SERIAL_PROTOCOL(MSG_OK);
//        SERIAL_PROTOCOL(" p:");
//        SERIAL_PROTOCOL(bedKp);
//        SERIAL_PROTOCOL(" i:");
//        SERIAL_PROTOCOL(unscalePID_i(bedKi));
//        SERIAL_PROTOCOL(" d:");
//        SERIAL_PROTOCOL(unscalePID_d(bedKd));
//        SERIAL_PROTOCOLLN("");
//      }
//      break;
//    #endif //PIDTEMP
//    case 240: // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
//     {
//      #ifdef CHDK
//       
//         SET_OUTPUT(CHDK);
//         WRITE(CHDK, HIGH);
//         chdkHigh = millis();
//         chdkActive = true;
//       
//       #else
//      
//        #if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
//  const uint8_t NUM_PULSES=16;
//  const float PULSE_LENGTH=0.01524;
//  for(int i=0; i < NUM_PULSES; i++) {
//        WRITE(PHOTOGRAPH_PIN, HIGH);
//        _delay_ms(PULSE_LENGTH);
//        WRITE(PHOTOGRAPH_PIN, LOW);
//        _delay_ms(PULSE_LENGTH);
//        }
//        delay(7.33);
//        for(int i=0; i < NUM_PULSES; i++) {
//        WRITE(PHOTOGRAPH_PIN, HIGH);
//        _delay_ms(PULSE_LENGTH);
//        WRITE(PHOTOGRAPH_PIN, LOW);
//        _delay_ms(PULSE_LENGTH);
//        }
//        #endif
//      #endif //chdk end if
//     }
//    break;
//#ifdef DOGLCD
//    case 250: // M250  Set LCD contrast value: C<value> (value 0..63)
//     {
//    if (code_seen('C')) {
//     lcd_setcontrast( ((int)code_value())&63 );
//          }
//          SERIAL_PROTOCOLPGM("lcd contrast value: ");
//          SERIAL_PROTOCOL(lcd_contrast);
//          SERIAL_PROTOCOLLN("");
//     }
//    break;
//#endif
//    #ifdef PREVENT_DANGEROUS_EXTRUDE
//    case 302: // allow cold extrudes, or set the minimum extrude temperature
//    {
//    float temp = .0;
//    if (code_seen('S')) temp=code_value();
//      set_extrude_min_temp(temp);
//    }
//    break;
//  #endif
//    case 303: // M303 PID autotune
//    {
//      float temp = 150.0;
//      int e=0;
//      int c=5;
//      if (code_seen('E')) e=code_value();
//        if (e<0)
//          temp=70;
//      if (code_seen('S')) temp=code_value();
//      if (code_seen('C')) c=code_value();
//      PID_autotune(temp, e, c);
//    }
//    break;
//  #ifdef SCARA
//  case 360:  // M360 SCARA Theta pos1
//      SERIAL_ECHOLN(" Cal: Theta 0 ");
//      //SoftEndsEnabled = false;              // Ignore soft endstops during calibration
//      //SERIAL_ECHOLN(" Soft endstops disabled ");
//      if(Stopped == false) {
//        //get_coordinates(); // For X Y Z E F
//        delta[X_AXIS] = 0;
//        delta[Y_AXIS] = 120;
//        calculate_SCARA_forward_Transform(delta);
//        destination[X_AXIS] = delta[X_AXIS]/axis_scaling[X_AXIS];
//        destination[Y_AXIS] = delta[Y_AXIS]/axis_scaling[Y_AXIS];
//        
//        prepare_move();
//        //ClearToSend();
//        return;
//      }
//    break;
//
//    case 361:  // SCARA Theta pos2
//      SERIAL_ECHOLN(" Cal: Theta 90 ");
//      //SoftEndsEnabled = false;              // Ignore soft endstops during calibration
//      //SERIAL_ECHOLN(" Soft endstops disabled ");
//      if(Stopped == false) {
//        //get_coordinates(); // For X Y Z E F
//        delta[X_AXIS] = 90;
//        delta[Y_AXIS] = 130;
//        calculate_SCARA_forward_Transform(delta);
//        destination[X_AXIS] = delta[X_AXIS]/axis_scaling[X_AXIS];
//        destination[Y_AXIS] = delta[Y_AXIS]/axis_scaling[Y_AXIS];
//        
//        prepare_move();
//        //ClearToSend();
//        return;
//      }
//    break;
//    case 362:  // SCARA Psi pos1
//      SERIAL_ECHOLN(" Cal: Psi 0 ");
//      //SoftEndsEnabled = false;              // Ignore soft endstops during calibration
//      //SERIAL_ECHOLN(" Soft endstops disabled ");
//      if(Stopped == false) {
//        //get_coordinates(); // For X Y Z E F
//        delta[X_AXIS] = 60;
//        delta[Y_AXIS] = 180;
//        calculate_SCARA_forward_Transform(delta);
//        destination[X_AXIS] = delta[X_AXIS]/axis_scaling[X_AXIS];
//        destination[Y_AXIS] = delta[Y_AXIS]/axis_scaling[Y_AXIS];
//        
//        prepare_move();
//        //ClearToSend();
//        return;
//      }
//    break;
//    case 363:  // SCARA Psi pos2
//      SERIAL_ECHOLN(" Cal: Psi 90 ");
//      //SoftEndsEnabled = false;              // Ignore soft endstops during calibration
//      //SERIAL_ECHOLN(" Soft endstops disabled ");
//      if(Stopped == false) {
//        //get_coordinates(); // For X Y Z E F
//        delta[X_AXIS] = 50;
//        delta[Y_AXIS] = 90;
//        calculate_SCARA_forward_Transform(delta);
//        destination[X_AXIS] = delta[X_AXIS]/axis_scaling[X_AXIS];
//        destination[Y_AXIS] = delta[Y_AXIS]/axis_scaling[Y_AXIS];
//        
//        prepare_move();
//        //ClearToSend();
//        return;
//      }
//    break;
//    case 364:  // SCARA Psi pos3 (90 deg to Theta)
//      SERIAL_ECHOLN(" Cal: Theta-Psi 90 ");
//     // SoftEndsEnabled = false;              // Ignore soft endstops during calibration
//      //SERIAL_ECHOLN(" Soft endstops disabled ");
//      if(Stopped == false) {
//        //get_coordinates(); // For X Y Z E F
//        delta[X_AXIS] = 45;
//        delta[Y_AXIS] = 135;
//        calculate_SCARA_forward_Transform(delta);
//        destination[X_AXIS] = delta[X_AXIS]/axis_scaling[X_AXIS];
//        destination[Y_AXIS] = delta[Y_AXIS]/axis_scaling[Y_AXIS]; 
//        
//        prepare_move();
//        //ClearToSend();
//        return;
//      }
//    break;
//    case 365: // M364  Set SCARA scaling for X Y Z
//      for(int8_t i=0; i < 3; i++) 
//      {
//        if(code_seen(axis_codes[i])) 
//        {
//          
//            axis_scaling[i] = code_value();
//          
//        }
//      }
//      break;
//  #endif
//    case 400: // M400 finish all moves
//    {
//      st_synchronize();
//    }
//    break;
//#if defined(ENABLE_AUTO_BED_LEVELING) && defined(SERVO_ENDSTOPS) && not defined(Z_PROBE_SLED)
//    case 401:
//    {
//        engage_z_probe();    // Engage Z Servo endstop if available
//    }
//    break;
//
//    case 402:
//    {
//        retract_z_probe();    // Retract Z Servo endstop if enabled
//    }
//    break;
//#endif
//
//#ifdef FILAMENT_SENSOR
//case 404:  //M404 Enter the nominal filament width (3mm, 1.75mm ) N<3.0> or display nominal filament width 
//    {
//    #if (FILWIDTH_PIN > -1) 
//    if(code_seen('N')) filament_width_nominal=code_value();
//    else{
//    SERIAL_PROTOCOLPGM("Filament dia (nominal mm):"); 
//    SERIAL_PROTOCOLLN(filament_width_nominal); 
//    }
//    #endif
//    }
//    break; 
//    
//    case 405:  //M405 Turn on filament sensor for control 
//    {
//    
//    
//    if(code_seen('D')) meas_delay_cm=code_value();
//       
//       if(meas_delay_cm> MAX_MEASUREMENT_DELAY)
//        meas_delay_cm = MAX_MEASUREMENT_DELAY;
//    
//       if(delay_index2 == -1)  //initialize the ring buffer if it has not been done since startup
//         {
//         int temp_ratio = widthFil_to_size_ratio(); 
//            
//            for (delay_index1=0; delay_index1<(MAX_MEASUREMENT_DELAY+1); ++delay_index1 ){
//                      measurement_delay[delay_index1]=temp_ratio-100;  //subtract 100 to scale within a signed byte
//                }
//            delay_index1=0;
//            delay_index2=0; 
//         }
//    
//    filament_sensor = true ; 
//    
//    //SERIAL_PROTOCOLPGM("Filament dia (measured mm):"); 
//    //SERIAL_PROTOCOL(filament_width_meas); 
//    //SERIAL_PROTOCOLPGM("Extrusion ratio(%):"); 
//    //SERIAL_PROTOCOL(extrudemultiply); 
//    } 
//    break; 
//    
//    case 406:  //M406 Turn off filament sensor for control 
//    {      
//    filament_sensor = false ; 
//    } 
//    break; 
//  
//    case 407:   //M407 Display measured filament diameter 
//    { 
//     
//    
//    
//    SERIAL_PROTOCOLPGM("Filament dia (measured mm):"); 
//    SERIAL_PROTOCOLLN(filament_width_meas);   
//    } 
//    break; 
//    #endif
//    
//
//
//
//
//    case 500: // M500 Store settings in EEPROM
//    {
//        Config_StoreSettings();
//    }
//    break;
//    case 501: // M501 Read settings from EEPROM
//    {
//        Config_RetrieveSettings();
//    }
//    break;
//    case 502: // M502 Revert to default settings
//    {
//        Config_ResetDefault();
//    }
//    break;
//    case 503: // M503 print settings currently in memory
//    {
//        Config_PrintSettings();
//    }
//    break;
//    #ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
//    case 540:
//    {
//        if(code_seen('S')) abort_on_endstop_hit = code_value() > 0;
//    }
//    break;
//    #endif
//
//    #ifdef CUSTOM_M_CODE_SET_Z_PROBE_OFFSET
//    case CUSTOM_M_CODE_SET_Z_PROBE_OFFSET:
//    {
//      float value;
//      if (code_seen('Z'))
//      {
//        value = code_value();
//        if ((Z_PROBE_OFFSET_RANGE_MIN <= value) && (value <= Z_PROBE_OFFSET_RANGE_MAX))
//        {
//          zprobe_zoffset = -value; // compare w/ line 278 of ConfigurationStore.cpp
//          SERIAL_ECHO_START;
//          SERIAL_ECHOLNPGM(MSG_ZPROBE_ZOFFSET " " MSG_OK);
//          SERIAL_PROTOCOLLN("");
//        }
//        else
//        {
//          SERIAL_ECHO_START;
//          SERIAL_ECHOPGM(MSG_ZPROBE_ZOFFSET);
//          SERIAL_ECHOPGM(MSG_Z_MIN);
//          SERIAL_ECHO(Z_PROBE_OFFSET_RANGE_MIN);
//          SERIAL_ECHOPGM(MSG_Z_MAX);
//          SERIAL_ECHO(Z_PROBE_OFFSET_RANGE_MAX);
//          SERIAL_PROTOCOLLN("");
//        }
//      }
//      else
//      {
//          SERIAL_ECHO_START;
//          SERIAL_ECHOLNPGM(MSG_ZPROBE_ZOFFSET " : ");
//          SERIAL_ECHO(-zprobe_zoffset);
//          SERIAL_PROTOCOLLN("");
//      }
//      break;
//    }
//    #endif // CUSTOM_M_CODE_SET_Z_PROBE_OFFSET
//
//    #ifdef FILAMENTCHANGEENABLE
//    case 600: //Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
//    {
//        float target[4];
//        float lastpos[4];
//        target[X_AXIS]=current_position[X_AXIS];
//        target[Y_AXIS]=current_position[Y_AXIS];
//        target[Z_AXIS]=current_position[Z_AXIS];
//        target[E_AXIS]=current_position[E_AXIS];
//        lastpos[X_AXIS]=current_position[X_AXIS];
//        lastpos[Y_AXIS]=current_position[Y_AXIS];
//        lastpos[Z_AXIS]=current_position[Z_AXIS];
//        lastpos[E_AXIS]=current_position[E_AXIS];
//        //retract by E
//        if(code_seen('E'))
//        {
//          target[E_AXIS]+= code_value();
//        }
//        else
//        {
//          #ifdef FILAMENTCHANGE_FIRSTRETRACT
//            target[E_AXIS]+= FILAMENTCHANGE_FIRSTRETRACT ;
//          #endif
//        }
//        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);
//
//        //lift Z
//        if(code_seen('Z'))
//        {
//          target[Z_AXIS]+= code_value();
//        }
//        else
//        {
//          #ifdef FILAMENTCHANGE_ZADD
//            target[Z_AXIS]+= FILAMENTCHANGE_ZADD ;
//          #endif
//        }
//        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);
//
//        //move xy
//        if(code_seen('X'))
//        {
//          target[X_AXIS]+= code_value();
//        }
//        else
//        {
//          #ifdef FILAMENTCHANGE_XPOS
//            target[X_AXIS]= FILAMENTCHANGE_XPOS ;
//          #endif
//        }
//        if(code_seen('Y'))
//        {
//          target[Y_AXIS]= code_value();
//        }
//        else
//        {
//          #ifdef FILAMENTCHANGE_YPOS
//            target[Y_AXIS]= FILAMENTCHANGE_YPOS ;
//          #endif
//        }
//
//        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);
//
//        if(code_seen('L'))
//        {
//          target[E_AXIS]+= code_value();
//        }
//        else
//        {
//          #ifdef FILAMENTCHANGE_FINALRETRACT
//            target[E_AXIS]+= FILAMENTCHANGE_FINALRETRACT ;
//          #endif
//        }
//
//        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);
//
//        //finish moves
//        st_synchronize();
//        //disable extruder steppers so filament can be removed
//        disable_e0();
//        disable_e1();
//        disable_e2();
//        delay(100);
//        LCD_ALERTMESSAGEPGM(MSG_FILAMENTCHANGE);
//        uint8_t cnt=0;
//        while(!lcd_clicked()){
//          cnt++;
//          manage_heater();
//          manage_inactivity(true);
//          lcd_update();
//          if(cnt==0)
//          {
//          #if BEEPER > 0
//            SET_OUTPUT(BEEPER);
//
//            WRITE(BEEPER,HIGH);
//            delay(3);
//            WRITE(BEEPER,LOW);
//            delay(3);
//          #else
//      #if !defined(LCD_FEEDBACK_FREQUENCY_HZ) || !defined(LCD_FEEDBACK_FREQUENCY_DURATION_MS)
//              lcd_buzz(1000/6,100);
//      #else
//        lcd_buzz(LCD_FEEDBACK_FREQUENCY_DURATION_MS,LCD_FEEDBACK_FREQUENCY_HZ);
//      #endif
//          #endif
//          }
//        }
//
//        //return to normal
//        if(code_seen('L'))
//        {
//          target[E_AXIS]+= -code_value();
//        }
//        else
//        {
//          #ifdef FILAMENTCHANGE_FINALRETRACT
//            target[E_AXIS]+=(-1)*FILAMENTCHANGE_FINALRETRACT ;
//          #endif
//        }
//        current_position[E_AXIS]=target[E_AXIS]; //the long retract of L is compensated by manual filament feeding
//        plan_set_e_position(current_position[E_AXIS]);
//        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //should do nothing
//        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //move xy back
//        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //move z back
//        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], lastpos[E_AXIS], feedrate/60, active_extruder); //final untretract
//    }
//    break;
//    #endif //FILAMENTCHANGEENABLE
//    #ifdef DUAL_X_CARRIAGE
//    case 605: // Set dual x-carriage movement mode:
//              //    M605 S0: Full control mode. The slicer has full control over x-carriage movement
//              //    M605 S1: Auto-park mode. The inactive head will auto park/unpark without slicer involvement
//              //    M605 S2 [Xnnn] [Rmmm]: Duplication mode. The second extruder will duplicate the first with nnn
//              //                         millimeters x-offset and an optional differential hotend temperature of
//              //                         mmm degrees. E.g., with "M605 S2 X100 R2" the second extruder will duplicate
//              //                         the first with a spacing of 100mm in the x direction and 2 degrees hotter.
//              //
//              //    Note: the X axis should be homed after changing dual x-carriage mode.
//    {
//        st_synchronize();
//
//        if (code_seen('S'))
//          dual_x_carriage_mode = code_value();
//
//        if (dual_x_carriage_mode == DXC_DUPLICATION_MODE)
//        {
//          if (code_seen('X'))
//            duplicate_extruder_x_offset = max(code_value(),X2_MIN_POS - x_home_pos(0));
//
//          if (code_seen('R'))
//            duplicate_extruder_temp_offset = code_value();
//
//          SERIAL_ECHO_START;
//          SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
//          SERIAL_ECHO(" ");
//          SERIAL_ECHO(extruder_offset[X_AXIS][0]);
//          SERIAL_ECHO(",");
//          SERIAL_ECHO(extruder_offset[Y_AXIS][0]);
//          SERIAL_ECHO(" ");
//          SERIAL_ECHO(duplicate_extruder_x_offset);
//          SERIAL_ECHO(",");
//          SERIAL_ECHOLN(extruder_offset[Y_AXIS][1]);
//        }
//        else if (dual_x_carriage_mode != DXC_FULL_CONTROL_MODE && dual_x_carriage_mode != DXC_AUTO_PARK_MODE)
//        {
//          dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
//        }
//
//        active_extruder_parked = false;
//        extruder_duplication_enabled = false;
//        delayed_move_time = 0;
//    }
//    break;
//    #endif //DUAL_X_CARRIAGE
//
//    case 907: // M907 Set digital trimpot motor current using axis codes.
//    {
//      #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
//        for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) digipot_current(i,code_value());
//        if(code_seen('B')) digipot_current(4,code_value());
//        if(code_seen('S')) for(int i=0;i<=4;i++) digipot_current(i,code_value());
//      #endif
//      #ifdef MOTOR_CURRENT_PWM_XY_PIN
//        if(code_seen('X')) digipot_current(0, code_value());
//      #endif
//      #ifdef MOTOR_CURRENT_PWM_Z_PIN
//        if(code_seen('Z')) digipot_current(1, code_value());
//      #endif
//      #ifdef MOTOR_CURRENT_PWM_E_PIN
//        if(code_seen('E')) digipot_current(2, code_value());
//      #endif
//      #ifdef DIGIPOT_I2C
//        // this one uses actual amps in floating point
//        for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) digipot_i2c_set_current(i, code_value());
//        // for each additional extruder (named B,C,D,E..., channels 4,5,6,7...)
//        for(int i=NUM_AXIS;i<DIGIPOT_I2C_NUM_CHANNELS;i++) if(code_seen('B'+i-NUM_AXIS)) digipot_i2c_set_current(i, code_value());
//      #endif
//    }
//    break;
//    case 908: // M908 Control digital trimpot directly.
//    {
//      #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
//        uint8_t channel,current;
//        if(code_seen('P')) channel=code_value();
//        if(code_seen('S')) current=code_value();
//        digitalPotWrite(channel, current);
//      #endif
//    }
//    break;
//    case 350: // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
//    {
//      #if defined(X_MS1_PIN) && X_MS1_PIN > -1
//        if(code_seen('S')) for(int i=0;i<=4;i++) microstep_mode(i,code_value());
//        for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_mode(i,(uint8_t)code_value());
//        if(code_seen('B')) microstep_mode(4,code_value());
//        microstep_readings();
//      #endif
//    }
//    break;
//    case 351: // M351 Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
//    {
//      #if defined(X_MS1_PIN) && X_MS1_PIN > -1
//      if(code_seen('S')) switch((int)code_value())
//      {
//        case 1:
//          for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,code_value(),-1);
//          if(code_seen('B')) microstep_ms(4,code_value(),-1);
//          break;
//        case 2:
//          for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,-1,code_value());
//          if(code_seen('B')) microstep_ms(4,-1,code_value());
//          break;
//      }
//      microstep_readings();
//      #endif
//    }
//    break;
//    case 999: // M999: Restart after being stopped
//      Stopped = false;
//      lcd_reset_alert_level();
//      gcode_LastN = Stopped_gcode_LastN;
//      FlushSerialRequestResend();
//    break;
//    }
//  }
//
//  else if(code_seen('T'))
//  {
//    tmp_extruder = code_value();
//    if(tmp_extruder >= EXTRUDERS) {
//      SERIAL_ECHO_START;
//      SERIAL_ECHO("T");
//      SERIAL_ECHO(tmp_extruder);
//      SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
//    }
//    else {
//      boolean make_move = false;
//      if(code_seen('F')) {
//        make_move = true;
//        next_feedrate = code_value();
//        if(next_feedrate > 0.0) {
//          feedrate = next_feedrate;
//        }
//      }
//      #if EXTRUDERS > 1
//      if(tmp_extruder != active_extruder) {
//        // Save current position to return to after applying extruder offset
//        memcpy(destination, current_position, sizeof(destination));
//      #ifdef DUAL_X_CARRIAGE
//        if (dual_x_carriage_mode == DXC_AUTO_PARK_MODE && Stopped == false &&
//            (delayed_move_time != 0 || current_position[X_AXIS] != x_home_pos(active_extruder)))
//        {
//          // Park old head: 1) raise 2) move to park position 3) lower
//          plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT,
//                current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
//          plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT,
//                current_position[E_AXIS], max_feedrate[X_AXIS], active_extruder);
//          plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS],
//                current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
//          st_synchronize();
//        }
//
//        // apply Y & Z extruder offset (x offset is already used in determining home pos)
//        current_position[Y_AXIS] = current_position[Y_AXIS] -
//                     extruder_offset[Y_AXIS][active_extruder] +
//                     extruder_offset[Y_AXIS][tmp_extruder];
//        current_position[Z_AXIS] = current_position[Z_AXIS] -
//                     extruder_offset[Z_AXIS][active_extruder] +
//                     extruder_offset[Z_AXIS][tmp_extruder];
//
//        active_extruder = tmp_extruder;
//
//        // This function resets the max/min values - the current position may be overwritten below.
//        axis_is_at_home(X_AXIS);
//
//        if (dual_x_carriage_mode == DXC_FULL_CONTROL_MODE)
//        {
//          current_position[X_AXIS] = inactive_extruder_x_pos;
//          inactive_extruder_x_pos = destination[X_AXIS];
//        }
//        else if (dual_x_carriage_mode == DXC_DUPLICATION_MODE)
//        {
//          active_extruder_parked = (active_extruder == 0); // this triggers the second extruder to move into the duplication position
//          if (active_extruder == 0 || active_extruder_parked)
//            current_position[X_AXIS] = inactive_extruder_x_pos;
//          else
//            current_position[X_AXIS] = destination[X_AXIS] + duplicate_extruder_x_offset;
//          inactive_extruder_x_pos = destination[X_AXIS];
//          extruder_duplication_enabled = false;
//        }
//        else
//        {
//          // record raised toolhead position for use by unpark
//          memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
//          raised_parked_position[Z_AXIS] += TOOLCHANGE_UNPARK_ZLIFT;
//          active_extruder_parked = true;
//          delayed_move_time = 0;
//        }
//      #else
//        // Offset extruder (only by XY)
//        int i;
//        for(i = 0; i < 2; i++) {
//           current_position[i] = current_position[i] -
//                                 extruder_offset[i][active_extruder] +
//                                 extruder_offset[i][tmp_extruder];
//        }
//        // Set the new active extruder and position
//        active_extruder = tmp_extruder;
//      #endif //else DUAL_X_CARRIAGE
//#ifdef DELTA 
//
//  calculate_delta(current_position); // change cartesian kinematic  to  delta kinematic;
//   //sent position to plan_set_position();
//  plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS],current_position[E_AXIS]);
//            
//#else
//        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
//
//#endif
//        // Move to the old position if 'F' was in the parameters
//        if(make_move && Stopped == false) {
//           prepare_move();
//        }
//      }
//      #endif
//      SERIAL_ECHO_START;
//      SERIAL_ECHO(MSG_ACTIVE_EXTRUDER);
//      SERIAL_PROTOCOLLN((int)active_extruder);
//    }
//  }
//
//  else
//  {
//    SERIAL_ECHO_START;
//    SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
//    SERIAL_ECHO(cmdbuffer[bufindr]);
//    SERIAL_ECHOLNPGM("\"");
//  }

  ClearToSend();
}

void FlushSerialRequestResend()
{
  //char cmdbuffer[bufindr][100]="Resend:";
  MSerial.flush();
  SERIAL_PROTOCOLPGM(MSG_RESEND);
  SERIAL_PROTOCOLLN(gcode_LastN + 1);
  ClearToSend();
}

void ClearToSend()
{
  previous_millis_cmd = millis();
//  #ifdef SDSUPPORT
//  if(fromsd[bufindr])
//    return;
//  #endif //SDSUPPORT
  SERIAL_PROTOCOLLNPGM(MSG_OK);
}

void kill()
{
  cli(); // Stop interrupts
  //mayis
//  disable_heater();
//
//  disable_x();
//  disable_y();
//  disable_z();
//  disable_e0();
//  disable_e1();
//  disable_e2();

#if defined(PS_ON_PIN) && PS_ON_PIN > -1
  pinMode(PS_ON_PIN,INPUT);
#endif
  SERIAL_ERROR_START;
  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);
  //mayis
  //LCD_ALERTMESSAGEPGM(MSG_KILLED);
  
  // FMC small patch to update the LCD before ending
  sei();   // enable interrupts
  //mayis
//  for ( int i=5; i--; lcd_update())
//  {
//     delay(200);  
//  }
  cli();   // disable interrupts
  suicide();
  while(1) { /* Intentionally left empty */ } // Wait for reset
}

void Stop()
{
  //mayis
  //disable_heater();
  
  if(Stopped == false) {
    Stopped = true;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
    //mayis
    //LCD_MESSAGEPGM(MSG_STOPPED);
  }
}

bool IsStopped() { return Stopped; };

void suicide()
{
  #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
    SET_OUTPUT(SUICIDE_PIN);
    WRITE(SUICIDE_PIN, LOW);
  #endif
}

void setup_killpin()
{
  #if defined(KILL_PIN) && KILL_PIN > -1
    SET_INPUT(KILL_PIN);
    WRITE(KILL_PIN,HIGH);
  #endif
}

float code_value()
{
  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

long code_value_long()
{
  return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10));
}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

void get_coordinates()
{
  //mayis
//  bool seen[4]={false,false,false,false};
//  for(int8_t i=0; i < NUM_AXIS; i++) {
//    if(code_seen(axis_codes[i]))
//    {
//      destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
//      seen[i]=true;
//    }
//    else destination[i] = current_position[i]; //Are these else lines really needed?
//  }
//  if(code_seen('F')) {
//    next_feedrate = code_value();
//    if(next_feedrate > 0.0) feedrate = next_feedrate;
//  }
}

void get_arc_coordinates()
{
  //mayis
//#ifdef SF_ARC_FIX
//   bool relative_mode_backup = relative_mode;
//   relative_mode = true;
//#endif
//   get_coordinates();
//#ifdef SF_ARC_FIX
//   relative_mode=relative_mode_backup;
//#endif
//
//   if(code_seen('I')) {
//     offset[0] = code_value();
//   }
//   else {
//     offset[0] = 0.0;
//   }
//   if(code_seen('J')) {
//     offset[1] = code_value();
//   }
//   else {
//     offset[1] = 0.0;
//   }
}

