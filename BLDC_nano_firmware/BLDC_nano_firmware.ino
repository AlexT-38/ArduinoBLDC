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

  maximum step rate, max rpm * poles * 6
  100hz * 4 * 6 = 2400Hz, 416us

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


void setup() {
  // put your setup code here, to run once:
#define setBIT(port, pin) port|=_BV(pin)
#define clrBIT(port, pin) port&=~_BV(pin)
#define togBIT(port, pin) port^=_BV(pin)

#define pin_LED 13
#define LED_port PORTB
#define LED_pin PORTB5
#define setLED() setBIT(LED_port, LED_pin)
#define clrLED() clrBIT(LED_port, LED_pin)
#define togLED() togBIT(LED_port, LED_pin)


  pinMode(pin_LED,OUTPUT);

  Serial.begin(1000000);
  Serial.println("boop");
  Serial.setTimeout(1);
  /* configure timer1 and timer2 for 8bit pwm at a frequency of 10kHz or greater
   *  (base clock 2.4MHz or greater)
   *  based on a 16Mhz clock (CH340 based Nano, was expecting it to be 12Mhz, but is infact 16)
   *  
   *  Timer1 prescaler is 0,1,8,64,256,1024,ext.v, ext.^
   *  Timer2 prescaler is 0,1,8,32,64,128,256,1024
   *  these aren't all that useful
   *  1 gives 62500Hz PWM frequncy
   *  8 gives  7813Hz PWM frequncy, which is double the highest expected operating frequency
   *  
   *  base frequency can be reduced by 2 using phase correct pwm
   *  assuming FETs are PSMN3R4-30PL
   *  typ switching time, full cycle (delay, rise/fall), 40+73+59+28 = 200ns, 5Mhz
   *  max pwm freq is clk/2, 6Mhz
   *  so the fets are only just capable of turning on before turning off again
   *  when running at 12Mhz
   *  
   *  and the fet drivers, ir2112?
   *  typ rise/fall times are 80 and 40
   *  max rise/fall times are 130 and 65
   *  prop delays dont count here, because its not a single transistor
   *  and the input can be toggled before the previous toggle has propagated
   *  or so I assume, as the turn on delay for a fet is time taken to charge the gate
   *  to the threshold voltage, and the rise time is that taken to charge the gate to saturation
   */
//these frequencies are for phase correct pwm
#define TMR1_PRESCALE_31kHz   1
#define TMR1_PRESCALE_3906Hz  2
#define TMR1_PRESCALE_977Hz   3
#define TMR1_PRESCALE_122Hz   4
#define TMR1_PRESCALE_31Hz    5
#define TMR1_PRESCALE_EXTF    6 //T1 is D5
#define TMR1_PRESCALE_EXTR    7 //T1 is D5

#define TMR2_PRESCALE_31kHz   1
#define TMR2_PRESCALE_3906Hz  2
#define TMR2_PRESCALE_1953Hz  3
#define TMR2_PRESCALE_977Hz   4
#define TMR2_PRESCALE_244Hz   5
#define TMR2_PRESCALE_122Hz   6
#define TMR2_PRESCALE_31Hz    7

#define TMR_FREQ_31kHz    31250.0
#define TMR_FREQ_3906Hz   3906.25
#define TMR_FREQ_1953Hz   1953.125
#define TMR_FREQ_977Hz    976.5625
#define TMR_FREQ_244Hz    244.140625
#define TMR_FREQ_122Hz    122.0703125
#define TMR_FREQ_31Hz     30.517578125

#define TMR_INTus_31kHz    32
#define TMR_INTus_3906Hz   256
#define TMR_INTus_1953Hz   1024
#define TMR_INTus_977Hz    2048
#define TMR_INTus_244Hz    4096
#define TMR_INTus_122Hz    8192
#define TMR_INTus_31Hz     32768

#define TMR1_PRESCALE       TMR1_PRESCALE_3906Hz
#define TMR2_PRESCALE       TMR2_PRESCALE_3906Hz

  // mask bit b in int m
  #define MASK(b,m) ((m)&_BV(b))
  // select bit r  if bit b of int m is set
  #define SELECT(r,b,m) ((_BV(r))*(MASK(b,m)!=0))

  #define T1M(r,b)  SELECT(r,b,TMR1_MODE)
  #define T2M(r,b)  SELECT(r,b,TMR2_MODE)


/*  timer 2
 *  WGM:
 *  0 Normal
 *  1 PWM phase correct
 *  2 Clear Timer on compare OCR2A
 *  3 Fast PWM
 *  4 res.
 *  5 PWM phase correct, top OCR2A
 *  6 Res.
 *  7 Fast PWM, top OCR2A
 */
#define TMR2_MODE 1

/* timer 1
 *  WGM:
 *  0 Normal
 *  1-3 phase correct pwm 8-10bit
 *  4 Clear Timer on compare OCR1A
 *  5-7 fast PWM 8-10bit
 *  ...
 */

 #define TMR1_MODE 1
/*
 *  COM:
 *  0 disabled
 *  1 reserved
 *  2 clear on up count, set on down count
 *  3 set on up count, clear on down count
 *  
 *  logic high drives the low fet ('off')
 */

 #define TMR_COM 3
 #define TMR_COM_VREF 2

 #define TCP(r,b)  SELECT(r,b,TMR_COM)
 #define TCV(r,b)  SELECT(r,b,TMR_COM_VREF)
 
 #define TMR1_COMA ( TCP(COM1A1,1) | TCP(COM1A0,0) )
 #define TMR1_COMB ( TCP(COM1B1,1) | TCP(COM1B0,0) )
 #define TMR2_COMA ( TCP(COM2A1,1) | TCP(COM2A0,0) )
 #define TMR2_COMB ( TCV(COM2B1,1) | TCV(COM2B0,0) )
 
/* configure pins first */

  /* 
   *  OC1A is PB1/D9
   *  OC1B is PB3/D11
   *  OC2A is PB2/D10
   *  OC2B is PD3/D3
   */
   #define pin_OC1A 9
   #define pin_OC1B 11
   #define pin_OC2A 10
   #define pin_OC2B 3
   #define OC1A_port PORTB
   #define OC1A_pin  PORTB1
   #define OC1B_port PORTB
   #define OC1B_pin  PORTB3
   #define OC2A_port PORTB
   #define OC2A_pin  PORTB2
   #define OC2B_port PORTD
   #define OC2B_pin  PORTD3
   
   
  pinMode(pin_OC1A, OUTPUT);
  pinMode(pin_OC1B, OUTPUT);
  pinMode(pin_OC2A, OUTPUT);
  pinMode(pin_OC2B, OUTPUT);

  //set the state of the otput pins when pwm is 0, which is LOW when using non inverted output
  digitalWrite(pin_OC1A, TMR_COM&1);
  digitalWrite(pin_OC1B, TMR_COM&1);
  digitalWrite(pin_OC2A, TMR_COM&1);
  digitalWrite(pin_OC2A, TMR_COM_VREF&1);

  /* for dev purposes,
   *  we can clock Timer1 by soft toggling it's input pin
   *  so that we can visually inspct the output waveform at very low frequencies
   */
  #define pin_T1 5
  pinMode(pin_T1, OUTPUT); //T1 input
  digitalWrite(pin_T1, LOW);

  #define pin_Button 14
  pinMode(pin_Button, INPUT_PULLUP); //button input to simulate over current event
  digitalWrite(pin_Button, HIGH);

#define OC1A_EN() TCCR1A |= (TMR1_COMA)
#define OC1A_DIS() TCCR1A &= ~(TMR1_COMA)
#define OC1B_EN() TCCR1A |= (TMR1_COMB)
#define OC1B_DIS() TCCR1A &= ~(TMR1_COMB)

#define OC2A_EN() TCCR2A |= (TMR2_COMA)
#define OC2A_DIS() TCCR2A &= ~(TMR2_COMA)
#define OC2B_EN() TCCR2A |= (TMR2_COMB)
#define OC2B_DIS() TCCR2A &= ~(TMR2_COMB)

  TCCR1A = T1M(WGM11,1) | T1M(WGM10,0);
  TCCR1B = T1M(WGM13,3) | T1M(WGM12,2);

  TCCR2A = T2M(WGM21,1) | T2M(WGM20,0);
  TCCR2B = T2M(WGM22,2);

#define TMR1_OFFSET 2  //set this ensure TMR1 and TMR2 overflow by the same amount - measure by having one tof set the led, and one clear it, adjust until it switches from mostly on to mostly off, with timer2 having the off command

  TCNT1 = TMR1_OFFSET;
  TCNT2 = 0;
  /* start timers */
  TCCR2B |= (TMR2_PRESCALE);
  TCCR1B |= (TMR1_PRESCALE);

  sin_en(); //enable sin wave player

  OC1A_EN();
  OC1B_EN();
  OC2A_EN();
  OC2B_EN();
}
static byte led = 0;
static byte button = 0;

static byte do_sin = true;
static unsigned int sin_pos1 = 0;
static unsigned int sin_pos2 = SEG_OFFSET;
static unsigned int sin_pos3 = SEG_OFFSET*2;
static byte sin_rate = 1;
static byte pwm1_state = 1;
static byte pwm2_state = 1;
static byte pwm3_state = 1;
static unsigned int pwm1 = 0; //these are unsigned so we can multiply and left shift in place
static unsigned int pwm2 = 0;
static unsigned int pwm3 = 0;
static byte do_report_sin = true;
static byte pwm_max = 255; //this needs its own table to match frequency to peak voltage. current is minimised by having this match back emf in amplitude and frequency

void loop() {
  // put your main code here, to run repeatedly:

  /* clock at 50 hz to get 1 pwm cycle every 10 seconds */ 
  delay(50);
  pwm_max++;

  /* tick the timer */
  //digitalWrite(pin_T1,HIGH); 
  //digitalWrite(pin_T1,LOW);

  static byte led_counter = 0;
  if(++led_counter>=25)
  {
//    togLED();
    led_counter = 0;
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

  if (Serial.available() > 0) {
    int new_rate = Serial.parseInt();
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
    
  }

 

  if (do_report_sin)
  {
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

/* writes a value to the given pwm channel
 *  this isnt something I expect to use for actual motor control, since Im going for sinusoidal control, after all.
 */
void pwmWrite(byte channel, byte value)
{
  
  
  /* sometimes switching the pwm back on will cause a single cycle at 100%
   *  not sure what causes this 
   *  its not the print command
   *  going to try syncronising the enable with TOF interupt
   *  no, that's not it - it's when the old PWM value has been left high, and then a low value is requested
   *  the pwm runs up to one cycle before updating the register
   *  setting ocr1a to zero should fix this, but doesnt.
   *  setting ocr1a to zero AND using the overflow int to reneable output also doesnt work
   *  trying compare match interupt to enable output
   *  no idea what's going on at this point
   *  
   *  ok, it's working
   *  all registers are updated at the same time in the TOV interupt
   *  if the pwm is set to zero on the same tick as the timer overflows,
   *  there will be a momentary on pulse up to the length of time it takes to set the registers
   *  it's possible at full speed, this might result in the pwm being set the next cycle
   *  so long as it doesnt give a pul;se longer than the previous pmc value, I'll take that as a win.
   *  
   *  no, it is.
   *  try the comp interupt
   *  nope, tried comp for wone and tof for the other, both ways
   *  with 'next flip to' at 33, turn_on glitches, but only around that value?
   */
  switch(channel)
  {
    case 0: //OC2B, current limit reference
      OCR2B = value;
      OC2B_EN();
      break;
    case 1: //OC1A, Phase A
      if(do_sin) break; //no write whem pwm running
      pwm1 = value;
      TIMSK1 |= _BV(TOIE1);//enable the overflow interupt to update the pwm
      break;
    case 2: //OC1B, Phase B
      if(do_sin) break; //no write whem pwm running
      pwm2 = value;
      TIMSK1 |= _BV(TOIE1);//enable the overflow interupt to update the pwm
      break;
    case 3: //OC2A, Phase C
      if(do_sin) break; //no write whem pwm running
      pwm3 = value;
      TIMSK1 |= _BV(TOIE1);//enable the overflow interupt to update the pwm
      break;
    
  }
}

/* enable the sin wave generator */
void sin_en()
{
  do_sin=true;
  TIMSK1 |= _BV(TOIE1);//enable the overflow interupt
}
/* disable the sin wave generator */
void sin_dis()
{
  do_sin=false;
}

/* timer 1 overflow
 *  Updates the pwm values. 
 *  does quite a lot of stuff in an isr, so keep things as simple as posible.
 *  do not use local variables so we can use 'naked' mode
 */
ISR(TIMER1_OVF_vect)//,ISR_NAKED) //naked = nothing uses the stack
{
  // write the pwm values
  OCR1A = pwm1;
  OCR1B = pwm2;
  OCR2A = pwm3;

  // disable outputs where pwm value is zero
  if(pwm1 != 0)  {    OC1A_EN();  }
  else            {    OC1A_DIS();  }
  //if(pwm2 != 0)  {    OC1B_EN();  }
  //else            {    OC1B_DIS();  }
  if(pwm3 != 0)  {    OC2A_EN();  }
  else            {    OC2A_DIS();  }
  
  //step through the sin tables for each phase
  if(do_sin)
  {
    /* we need some way of syncing with the sensor state changes
     *  this basically means jumping ahead if we are lagging behind
     *  or stopping the sinwave until the rotor catches up
     *  the indices in the sin table where sensor state changes should occur
     *  can be specified to give the correct phase advance for efficient running
     * 
     */
    
    //increment by rate
    #define SIN_INC(x) x=x+sin_rate
    SIN_INC(sin_pos1);
    SIN_INC(sin_pos2);
    SIN_INC(sin_pos3);

    //loop back to the start on overflow
    #define SIN_MOD(x) if(x>=sin_table_size) x-=sin_table_size
    SIN_MOD(sin_pos1);
    SIN_MOD(sin_pos2);
    SIN_MOD(sin_pos3);

    // fetch the next pwm values from the table
    pwm1 = SIN(sin_pos1);
    pwm2 = SIN(sin_pos2);
    pwm3 = SIN(sin_pos3);

    //multiply each pwm value by the maximum
    pwm1 = ((pwm1 * pwm_max) + pwm_max)>>8;
    pwm2 = ((pwm2 * pwm_max) + pwm_max)>>8;
    pwm3 = ((pwm3 * pwm_max) + pwm_max)>>8;

    // set the next pwm states
    pwm1_state = pwm1 != 0;
    pwm2_state = pwm2 != 0;
    pwm3_state = pwm3 != 0;
    
    //do_report_sin = true;


  }
  else
  {
    TIMSK1 &= ~_BV(TOIE1); //disable interrupt
  }
  
  //reti(); //required with ISR_NAKED
}
