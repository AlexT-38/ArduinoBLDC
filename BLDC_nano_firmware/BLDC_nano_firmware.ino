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


#include "tables.h"
#include "pwm.h"

#define setBIT(port, pin) port|=_BV(pin)
#define clrBIT(port, pin) port&=~_BV(pin)
#define togBIT(port, pin) port^=_BV(pin)

#define REPORT_TIME_ms 100

//#define PERF_REPORT_TOGETHER
//#define PERF_REPORT


static byte led = 0;
static byte button = 0;

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
static unsigned int sim_interval_us = 20000;
static unsigned int tmr1_time_us = 0;


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
  pinMode(pin_SD,OUTPUT);
  DRIVE_DIS();

#define pin_Button 2
  pinMode(pin_Button, INPUT_PULLUP); //button input to simulate over current event
  digitalWrite(pin_Button, HIGH);

  Serial.begin(1000000);
  Serial.println("boop");

  sensors_init();
  pwmInit();

  
}
extern unsigned int interval_us;
extern const byte sensor_seq[];
extern byte sensor_position;
extern byte pwm_max;

#ifdef PERF_REPORT
extern byte performance_timer2, performance_timer;
#endif
static byte do_report = true;

void loop() {
  static int led_counter = 0;
  if(!++led_counter && do_report)
  {
    //togLED();
    //togDBG();
    Serial.println();
    Serial.println("-------------------");
    Serial.print("sim interval(us): ");Serial.println(sim_interval_us);
    Serial.print("interval(us): ");Serial.println(interval_us);
    Serial.print("do_sin: ");Serial.println(do_sin);
    Serial.print("sin_pos1: ");Serial.println(sin_pos1);
    Serial.print("sin_rate: ");Serial.println(sin_rate);
    Serial.print("pwm_max: ");Serial.println(pwm_max);
    Serial.print("pwm1: ");Serial.println(pwm1);
    Serial.print("pwm2: ");Serial.println(pwm2);
    Serial.print("pwm3: ");Serial.println(pwm3);
    byte sensor_bits = sensor_seq[sensor_position];
    Serial.print("spos1: ");Serial.println((sensor_bits&1)!=0);
    Serial.print("spos2: ");Serial.println((sensor_bits&2)!=0);
    Serial.print("spos3: ");Serial.println((sensor_bits&4)!=0);
    Serial.print("sensor_position: ");Serial.println(sensor_position);
    //sanity check sensor:
    //sensor_bits = PORTC&7;
    //Serial.print("spin1: ");Serial.println((sensor_bits&1)!=0);
    //Serial.print("spin2: ");Serial.println((sensor_bits&2)!=0);
    //Serial.print("spin3: ");Serial.println((sensor_bits&4)!=0);
    
    Serial.println();
#ifdef PERF_REPORT
    #ifdef PERF_REPORT_TOGETHER
    if(performance_timer2 > 127) performance_timer2 = 256-performance_timer2;
    Serial.print("Perf1 (us): ");  Serial.println( TMR1_TCK_TIME_us(performance_timer)  );
    Serial.print("Perf2 (us): ");  Serial.println( TMR1_TCK_TIME_us(performance_timer2) );
    Serial.println();
    #endif
#endif    
  }

  /* simulate sensor signals */
  sensor_sim(sim_interval_us);

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
  if (Serial.available() > 0) {
    unsigned int new_rate_us = Serial.parseInt();
    Serial.print('>'); Serial.println(new_rate_us);
    if(new_rate_us > 0)
    {
      sim_interval_us = new_rate_us;
      Serial.print("simulated sensor interval: ");Serial.println(sim_interval_us, DEC);
      sin_en();
    }
    else
    {
      sin_dis();
    }

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
