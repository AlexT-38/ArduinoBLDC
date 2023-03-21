/***************************************
 * handles reading the position sensors
 * also reconfigures TMR0 for measuring intervals
 */

#define INTERVAL_MAX (((UINT16_MAX/6)&0xFF00)-0x100)

unsigned int intervals_lsb[6] = {INTERVAL_MAX,INTERVAL_MAX,INTERVAL_MAX,INTERVAL_MAX,INTERVAL_MAX,INTERVAL_MAX};
unsigned int interval_lsb = INTERVAL_MAX*6;
byte interval_idx;
byte sensor_state;
byte sensor_state_lt;
byte tmr0_overflows;

#define SENSOR_DDR      DDRC
#define SENSOR_PORT     PINC
#define SENSOR_PULL     PORTC
#define SENSOR_PIN0      PORTC0
#define SENSOR_MASK     (_BV(SENSOR_PIN0)|_BV(SENSOR_PIN0+1)|_BV(SENSOR_PIN0+2))

#define SENSOR_vect     PCINT0_vect
#define SENSOR_PCMSK    PCMSK1
#define SENSOR_PCIE     PCIE1

#define SENSOR_EN()     (PCICR |= SENSOR_PCIE)
#define SENSOR_DIS()    (PCICR &= ~SENSOR_PCIE)

#define GET_SENSOR()    ((SENSOR_PORT&SENSOR_MASK) >> SENSOR_PIN0)
/* the base resolution will be 1/clk, with clk= 16Mhz/prescale
 *  16Mhz: 
 */
#define TMR0_PRESCALE_63NS   1
#define TMR0_PRESCALE_500NS  2
#define TMR0_PRESCALE_4US    3
#define TMR0_PRESCALE_16US   4
#define TMR0_PRESCALE_64US   5

//closest to 32us is 16us
#define TMR0_PRESCALE       TMR0_PRESCALE_16US
 
#define TMR0_DIS()  TCCR0A = 0
#define TMR0_EN()   TCCR0A = TMR0_PRESCALE
#define TMR0_RST()  TCNT0 = 0;  tmr0_overflows = 0

void sensors_init()
{
  //set the TMR0 registers here
  TMR0_RST(); 
  TMR0_EN();
  //enable overflow interrupt
  TIMSK0 = _BV(TOIE0); 

  //set the sensor input pin config
  SENSOR_DDR &= ~SENSOR_MASK;
  SENSOR_PULL |= SENSOR_MASK;
  
  //get the initial sensor state
  sensor_state = GET_SENSOR();
  sensor_state_lt = 0;
  
  //set the PCINT interrupt registers here
  SENSOR_PCMSK = SENSOR_MASK;
  //enable sensor interrupt
  SENSOR_EN();
}

/* we need to measure the time bewteen each interrupt
 *  averaged over the last full electrical cycle
 *  it may turn out this is overly complex
 *  and simply using the segment time without averaging
 *  may turn out to be required
 *  to keep the isr tight
 */
ISR(SENSOR_vect)//, ISR_NAKED)
{
  /* stop the timer */
  //TMR0_DIS();
  /* deduct the oldest time */
  interval_lsb -= intervals_lsb[interval_idx];
  /* store the newest time */
  intervals_lsb[interval_idx] = TCNT0 + tmr0_overflows;
  /* clamp the value to the minimum */
  intervals_lsb[interval_idx] = intervals_lsb[interval_idx] > STEP_RATE_MINIMUM_lsb ? intervals_lsb[interval_idx] : STEP_RATE_MINIMUM_lsb;
  /* reset the timer */
  TMR0_RST();            //we could offset TCNT0 to account for the ISR overhead, but with a resoultion of 16us, ISR should always complete before this can be an issue
  TMR0_EN();
  /* fetch the latest sensor value */
  //sensor_state_lt = sensor_state;
  sensor_state = GET_SENSOR();
  /* add the new time to the total */
  interval_lsb += intervals_lsb[interval_idx];
  /* increment the buffer index */
  if(++interval_idx>=6) interval_idx =0;

  /* fetch new pwm values */
  sin_rate = STEP_RATE_lsb(interval_lsb);
  pwm_max = PWM_RATE_lsb(interval_lsb);
  
  //reti();   //required with ISR_NAKED
}
//overwriting TMR0_OVF_vect requires altering wiring.h, see https://forum.arduino.cc/t/overriding-arduino-interrupt-handlers/15416/4
/* we use tmr0 to measure time in the same resolution as the lookup tables */
ISR(TIMER0_OVF_vect, ISR_NAKED)
{
  /* count up to the maximum then turn off */
  if(++tmr0_overflows==(INTERVAL_MAX>>8)) TMR0_DIS();
  reti();   //required with ISR_NAKED
}
