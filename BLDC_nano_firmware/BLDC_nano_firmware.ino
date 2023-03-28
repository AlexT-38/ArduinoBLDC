/*
 * 
  Simple BLDC controller using available parts
  emphasis on torque control

  the purpose of the controller is the start the ICE and
  then provide regenerative braking to charge the starter battery
  and power loads or provide a controlled load on the engine.

  2000VA target capacity
  48V bus voltage
  42A nominal motor current
  6000rpm absolute maximum angular speed
  hall sensor input
  no. of poles is unkown, presumed to be 4

  waveform output frequency is rpm/60 * poles/2
  max Fwave = 200Hz
  min Twave = 5000us

  there are six steps in each rotation
  min Tstep = 833us
  
  with Fpwm = 3921.6Hz,
  there are 3.27 pwm updates per step
  
  with Fpwm = 31kHz,
  there are 26.14 pwm updates per step
  
  
  OC1A, OC1B and OC2A are grouped, so these will be the pwm outputs

  OC2B will set the current limit reference voltage

  current measurment will be by resistors in each leg of the inverter
  each resistor 's voltage will be offset and amlipfied
  
  use ir2112 high side fet drivers

  torque control by adjusting output phase
  this first requires that the output waveform
  is synchronised to the back emf waveform
 */




#define setBIT(port, pin) port|=_BV(pin)
#define clrBIT(port, pin) port&=~_BV(pin)
#define togBIT(port, pin) port^=_BV(pin)

/* serial report flags */
#define REPORT_TIME_ms 100

//#define PERF_REPORT_TOGETHER
//#define PERF_REPORT

//#define DO_REPORT_SIN

/* operation control flags */
#define SYNC_WAVE //syncronise the output waveform to the input sensor edge
#define PWM_DITHER //count the wave position at a higher resolution than the wave table

#include "tables.h"
#include "pwm.h"
#include "sensors.h"



static byte led = 0;
static byte button = 0;
static byte do_sim = false;
static byte do_sin = true;
static unsigned int sin_hold_pos = SEG_SIZE;
static unsigned int sin_pos1 = 0;
static unsigned int sin_pos2;
static unsigned int sin_pos3;
static byte sin_rate = 1;
static unsigned int pwm1 = 0; 
static unsigned int pwm2 = 0;
static unsigned int pwm3 = 0;
static byte do_report_sin = false;
static byte pwm_max = 255;
static unsigned int sim_interval_us = STEP_RATE_MAXIMUM_us;
static unsigned int tmr1_time_tck = 0;


void setup() 
{

#define pin_LED 13    //on board LED
#define LED_port PORTB
#define LED_pin PORTB5
#define setLED() setBIT(LED_port, LED_pin)
#define clrLED() clrBIT(LED_port, LED_pin)
#define togLED() togBIT(LED_port, LED_pin)
  pinMode(pin_LED,OUTPUT);

#define pin_DBG 12    //output pin for dbg, isr timing, etc
#define DBG_port PORTB
#define DBG_pin PORTB4
#define setDBG() setBIT(DBG_port, DBG_pin)
#define clrDBG() clrBIT(DBG_port, DBG_pin)
#define togDBG() togBIT(DBG_port, DBG_pin)
  pinMode(pin_DBG,OUTPUT);
  digitalWrite(pin_DBG, HIGH);
  
#define pin_SD 8    //inverter shut down pin
#define SD_port PORTB
#define SD_pin PORTB0
#define DRIVE_EN() clrBIT(SD_port, SD_pin)
#define DRIVE_DIS() setBIT(SD_port, SD_pin)
#define DRIVE_TOG() togBIT(SD_port, SD_pin)
#define DRIVE       ((SD_port&SD_pin) != SD_pin) 
  pinMode(pin_SD,OUTPUT);
  DRIVE_DIS();

#define pin_Button 2
  pinMode(pin_Button, INPUT_PULLUP); //button input to simulate over current event
  digitalWrite(pin_Button, HIGH);

  Serial.begin(1000000);
  Serial.setTimeout(1);
  Serial.println("boop");

  sensors_init();
  pwmInit();

  
}

static byte do_report = true;
byte report()
{
    //togLED();
    //togDBG();
    static unsigned int interval_tck_;
    static unsigned int interval_us_;
    static unsigned int sin_pos1_;
    static byte sin_rate_;
    #ifdef PWM_DITHER
    static unsigned int sin_pos1_frac_;
    #endif
    static byte pwm1_;
    static byte pwm2_;
    static byte pwm3_;
    static byte pwm_max_;
    static byte sensor_position_;

    static byte report_state = 0;

    switch (report_state++)
    {
      case 0:
        interval_tck_ = interval_tck;
        interval_us_ = interval_us;
        sin_pos1_ = sin_pos1;
        sin_rate_ = sin_rate;
        #ifdef PWM_DITHER
        sin_pos1_frac_ = sin_pos1_frac;
        #endif
        pwm1_ = pwm1;
        pwm2_ = pwm2;
        pwm3_ = pwm3;
        pwm_max_ = pwm_max;
        sensor_position_ = sensor_position;
        break;
      case 1:
        Serial.println();
        Serial.println("-------------------");
        break;
      case 2:
        Serial.print("sim interval(us): ");Serial.println(sim_interval_us);
        Serial.print("interval(tck): ");Serial.println(interval_tck_);
        break;
      case 3:
        Serial.print("interval(us): ");Serial.println(interval_us_);
        Serial.print("RPM: ");Serial.println(RPM(interval_tck_));
        Serial.println();
        break;
      case 4:
        Serial.print("sin_pos1: ");Serial.println(sin_pos1_);
        Serial.print("sin_rate: ");Serial.println(sin_rate_);      
        break;
      case 5:
        #ifdef PWM_DITHER
        //Serial.print("sin_pos1 (whole): "); Serial.println(sin_pos1_frac_ >> SIN_RATE_SCALE_BITS);
        Serial.print("sin_pos1 (frac): "); Serial.println(sin_pos1_frac_ & SIN_RATE_MASK);
        Serial.print("sin_rate (whole): ");Serial.println(sin_rate_ >> SIN_RATE_SCALE_BITS);
        Serial.print("sin_rate (frac): "); Serial.println(sin_rate_ & SIN_RATE_MASK);
        #endif
        break;
      case 6:
        Serial.print("pwm_max: ");Serial.println(pwm_max_);
        Serial.print("pwm1: ");Serial.println(pwm1_);      
        break;
      case 7:
        Serial.print("pwm2: ");Serial.println(pwm2_);
        Serial.print("pwm3: ");Serial.println(pwm3_);      
        break;
      case 8:
        {
          byte sensor_bits = sensor_seq[sensor_position_];
          Serial.print("spos1: ");Serial.println((sensor_bits&1)!=0);
          Serial.print("spos2: ");Serial.println((sensor_bits&2)!=0); 
          Serial.print("spos3: ");Serial.println((sensor_bits&4)!=0);
        }
        break;
      case 9:
        Serial.print("sensor_position: ");Serial.println(sensor_position_);
        Serial.println();
        break;
      case 10:
        #ifdef PERF_REPORT
         #ifdef PERF_REPORT_TOGETHER
          if(performance_timer2 > 127) performance_timer2 = 256-performance_timer2;
          Serial.print("Perf1 (us): ");  Serial.println( TMR1_TCK_TIME_us(performance_timer)  );
          Serial.print("Perf2 (us): ");  Serial.println( TMR1_TCK_TIME_us(performance_timer2) );
          Serial.println();
         #endif
        #endif      
        break;
      case 11:
      default:
        report_state = 0;
        return 1;
    }

  return 0;
}


void loop() {


  /* simulate sensor signals */
  if(do_sim)
  {
    sensor_sim(sim_interval_us);
  }

  /* check for simulated overcurrent */
  if(digitalRead(pin_Button) == LOW)
  {
    //make sure we get the on press event
    if(button == HIGH)
    {
      button = LOW;
    }
  }
  else
  {
    button=HIGH;
  }

  /* check for serial input */
  #define RESPONSE_SIZE 32
  static char response[RESPONSE_SIZE] = ">";
  static byte response_size = 1;
  #define RESPONSE response[response_size++]
  #define RESPONSE_NEXT &response[response_size]
  static byte respond = false;
  #define NO_COMMAND '\0'
  static char command = NO_COMMAND;
  static byte parse_param = false;
  static int parameter = 0;
  static byte parse_command = false;
  enum SS
  {
    SS_IDLE,      // check for serial input and perform serial reports
    SS_COMMAND,   // fetch the command from the input stream
    SS_PARAM,     // fetch the parameter from the input stream
    SS_RESPOND   // send the response to the input
  };
  static enum SS ser_stat = SS_IDLE;
  /* we'll need to define some commands
   *  we want this to be expediant
   *  so commands will be a single letter followed immediately by an integer parameter,
   *  terminated with linefeed or carriage return
   *  we check the first symbol, and if the symbol has a parameter, scan for it
   *  the command and parameter will be echoed back once recieved in full
   */
  switch (ser_stat)
  {
    case SS_IDLE:                           // check for serial input and perform serial reports
      if (Serial.available() > 0)       
      { ser_stat = SS_COMMAND;      }
      else
      {
        /* print report periodically */
        static int report_counter = 0;
        if(!report_counter && do_report)
        {
          report_counter = report();
        }
        else
        {
          report_counter++;
        }
        /* check for debug and performance reports */
  
        if (do_report_sin)
        {
          static unsigned int last_time_ms = 0;
          unsigned int this_time_s = millis();
          if (this_time_s-last_time_ms > 100){
            do_report_sin =false;
            //togLED();
            Serial.print("sin(idx1): [");
            Serial.print(sin_pos1);
            Serial.print("] ");
            Serial.println(pwm1);
            Serial.print("sin(idx2): [");
            Serial.print(sin_pos2);
            Serial.print("] ");
            Serial.println(pwm2);
            Serial.print("sin(idx3): [");
            Serial.print(sin_pos3);
            Serial.print("] ");
            Serial.println(pwm3);
            Serial.println();
          }
        }
        #ifdef PERF_REPORT
        #ifndef PERF_REPORT_TOGETHER
        print_perf_timer();
        print_perf_timer2();  
        #endif
        #endif
      }
      break;
      
    case SS_COMMAND:                  // fetch the command from the input stream
      command = Serial.read();
      RESPONSE = command;
      
      switch (command)
      {
        case 'R': //toggle printing reports
          do_report = !do_report;
          RESPONSE = do_report?'1':'0';
          ser_stat = SS_RESPOND;
          break;
        case 'S': //toggle simulation
          do_sim = !do_sim;
          RESPONSE = do_sim?'1':'0';
          ser_stat = SS_RESPOND;
          break;
        case 'W': //toggle waveform generation
          tog_sin();
          RESPONSE = do_sin?'1':'0';
          ser_stat = SS_RESPOND;
          break;
        case 'M': //toggle motor drive
          DRIVE_TOG();
          RESPONSE = DRIVE?'1':'0';
          ser_stat = SS_RESPOND;
          break;
        case 's': //set the simulation rate (us)
        case 'p': //set the phase offset
          ser_stat = SS_PARAM;
          break;
        default:
          if(response_size > (RESPONSE_SIZE-3))
          {
            RESPONSE = '?';
            ser_stat = SS_RESPOND;
          }
          break;
        case '\r':
        case '\n':
          if(response_size > 2)
          {
            response_size--;
            RESPONSE = '?';
            ser_stat = SS_RESPOND;
          }
          else
          {
            ser_stat = SS_IDLE;
            response_size = 0;
          }
          break;
      }
      break;
      
    case SS_PARAM:                              // fetch the parameter from the input stream
      parameter = Serial.parseInt(SKIP_NONE);
      #define RESPOND_PARAM(parameter) itoa(parameter, RESPONSE_NEXT, 10); response_size = strlen(response)
      RESPOND_PARAM(parameter);
      switch (command)
      {
        case 's': //simulation rate (us)
          if(parameter > (2*TMR1_OVF_TIME_us)) //simulation rate limited to 2x the pwm update rate minimum
          {
            sim_interval_us = parameter;
            RESPONSE = 'O';
            RESPONSE = 'K';
          }
          else
          {
            RESPONSE = '?';
          }
          break;
        case 'p': //phase offset in units of (360/sin_table_size)degrees (ie 0.09375)
          if (parameter < -(int)sin_table_size)
          {
            RESPONSE = 'v';
          }
          else if (parameter >= sin_table_size)
          {
            RESPONSE = '^';
          }
          else 
          {
            RESPONSE = 'O';
            RESPONSE = 'K';
            RESPONSE = '\n';
            if(parameter < 0)
            {
              parameter += sin_table_size;
            }
            RESPOND_PARAM(parameter);
            RESPONSE = '/';
            RESPOND_PARAM(sin_table_size);
            phase_offset = parameter;
          }
      }
      ser_stat = SS_RESPOND;
      break;
      
    case SS_RESPOND:                // send the response to the input
      RESPONSE = '\0';
      Serial.println(response);
      response_size = 0;
      ser_stat = SS_IDLE;
      RESPONSE = '>';
      break;
  }
}
