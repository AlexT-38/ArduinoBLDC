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
  6000rpm maximum angular speed
  hall sensor input
  no. of poles is unkown, presumed to be 4

  maximum step rate, max rpm * poles/2 * 6
  100hz * 2 * 6 = 1200Hz, 832us

  pwm frequency needs to be at least 4x this, ie >=9600Hz
  8 bit pwm control should be sufficient for this simple system
  giving a base timer frequency of >=2.46MHz

  atmega328p timer0 output compare overlaps with analog comparataor inputand i s used for millis()

  OC1A, OC1B and OC2A are grouped, so these will be the pwm outputs

  OC2B will set the current limit reference voltage

  current measurment will be by resistors in each leg of the inverter
  each resistor 's voltage will be offset and amlipfied
  an opamp with a minimum common mode voltage of Vss/0v is required
  the KA258 meets this requirement

  use ir2112 high side fet drivers

  torque control by current limiting
  current limiting using the controller's built in analog comparator
  when measured current exceeds a threshold, set by a spare pwm channel
  pwm is forced to switch, alternatively the non pwm phase stops conducting
  so long as rpm, and thus back emf is kept below the supply bus voltage
  so that the body diode doesnt conduct
  otherwise, the fet should be left on and pwm set to max
 */


#include "tables.h"
#include "pwm.h"

#define setBIT(port, pin) port|=_BV(pin)
#define clrBIT(port, pin) port&=~_BV(pin)
#define togBIT(port, pin) port^=_BV(pin)

#define REPORT_TIME_ms 100

static byte led = 0;
static byte button = 0;

static byte do_sin = true;
static unsigned int sin_hold_pos = SEG_SIZE;
static unsigned int sin_pos1 = 0;
static unsigned int sin_pos2;
static unsigned int sin_pos3;
static byte sin_rate = 1;
static byte pwm1_state = 1;
static byte pwm2_state = 1;
static byte pwm3_state = 1;
static unsigned int pwm1 = 0; 
static unsigned int pwm2 = 0;
static unsigned int pwm3 = 0;
static byte do_report_sin = false;
static byte pwm_max = 255;
static unsigned int sim_interval_us = 20000;


void setup() 
{

#define pin_LED 13    //on board LED
#define LED_port PORTB
#define LED_pin PORTB5
#define setLED() setBIT(LED_port, LED_pin)
#define clrLED() clrBIT(LED_port, LED_pin)
#define togLED() togBIT(LED_port, LED_pin)
  pinMode(pin_LED,OUTPUT);
  
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
//  Serial.setTimeout(1); 
//  timeout probably wont work without TMR0 doing it's thing
//  but all the vital code should be interrupt based
//  so no timeout is needed

  sensors_init();
  pwmInit();
}

void loop() {

  static byte led_counter = 0;
  if(!++led_counter)
  {
    togLED();
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
    sim_interval_us = new_rate_us;
    Serial.print("simulated sensor interval: ");Serial.println(sim_interval_us, DEC);
    /*
    if(do_sin)
    {
      // read the incoming byte:
      
      if(new_rate < 0)
      {
        sin_dis();
        Serial.println("setting pwm");
      }
      else
      {
        if (new_rate > sizeof(sin_table)/24) new_rate = sizeof(sin_table)/24;
        sin_rate = new_rate;
        sin_en();
      }
      // say what you got:
      Serial.println(new_rate, DEC);
    }
    else
    {
      if(new_rate < 0)
      {
        Serial.println("setting phase rate");
        sin_en();
      }
      else
      {
        //set the maximum sinusoidal pwm with the upper byte
        pwm_max = new_rate>>8;
        new_rate &= 255;
        
        pwmWrite(0, new_rate);
        //pwmWrite(1, new_rate);
        //pwmWrite(2, new_rate);
        //pwmWrite(3, new_rate);
      }
      
    }
    */
  }

  /* check for debug and performance reports */

  if (do_report_sin)
  {
    static unsigned int last_time_ms = 0;
    unsigned int this_time_s = millis();
    if (this_time_s-last_time_ms > 100){
      do_report_sin =false;
      togLED();
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
  
  print_perf_timer();
  print_perf_timer2();

  
}
