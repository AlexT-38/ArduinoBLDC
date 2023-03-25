/***************************************
 * handles reading the position sensors
 * also reconfigures TMR0 for measuring intervals
 */
#define SESNOR_SIM

const byte sensor_seq[] = { 0b001, 0b011, 0b010, 0b110, 0b100, 0b101};
static const byte sensor_decode[] = { 0xFF, 0, 2, 1, 4, 5, 3, 0xFF };
byte sensor_position;

unsigned int interval_us;
//unsigned int last_sensor_change_time_us = 0;

// map sensor position to sin wave positions for each phase
#define SPHASE(n)  (sin_table_size*n/6)
unsigned int start_phase1_step[6] = {SPHASE(0),SPHASE(1),SPHASE(2),SPHASE(3),SPHASE(4),SPHASE(5)};

/* adjust phase angle to control the motor */
unsigned int phase_offset = 0;

#define SENSOR_PIN      A0
#define SENSOR_DDR      DDRC
#define SENSOR_PINS     PINC
#define SENSOR_PORT     PORTC
#define SENSOR_PIN0     PORTC0
#define SENSOR_MASK     (_BV(SENSOR_PIN0)|_BV(SENSOR_PIN0+1)|_BV(SENSOR_PIN0+2))

#define SENSOR_vect     PCINT1_vect
#define SENSOR_PCMSK    PCMSK1
#define SENSOR_PCIE     _BV(PCIE1)

#define SENSOR_EN()     (PCICR |= SENSOR_PCIE)
#define SENSOR_DIS()    (PCICR &= ~SENSOR_PCIE)

#define GET_SENSOR()    ((SENSOR_PINS&SENSOR_MASK) >> SENSOR_PIN0)

#define STEPS_PER_CYCLE 6
void sensors_init()
{
  //set the sensor input pin config
#ifndef SESNOR_SIM
  SENSOR_DDR &= ~SENSOR_MASK;
  SENSOR_PORT |= SENSOR_MASK;
#else
  SENSOR_DDR |= SENSOR_MASK;
  SENSOR_PORT &= ~SENSOR_MASK;

#endif  

  //get the initial sensor state
  sensor_position = sensor_decode[GET_SENSOR()];
  
  //set the PCINT interrupt registers here
  SENSOR_PCMSK = SENSOR_MASK;
  //enable sensor interrupt
  SENSOR_EN();

}
static byte sim_processed = true;
void sensor_sim(unsigned int set_interval_us)
{
  if((set_interval_us == 0) | !sim_processed) return;
  #ifdef SESNOR_SIM
  static unsigned int last_time_us = 0;
  static byte sequence_idx = 0;
  unsigned int this_time_us = micros();
  
  //this_time_us = tmr1_time_tck; //use the measured time to time the simulation
  if((sensor_position == 0xff) | ((this_time_us-last_time_us) > set_interval_us))
  {
    sim_processed = false;
    
    byte port = SENSOR_PORT;
    port &= ~SENSOR_MASK;
    port |= (sensor_seq[sequence_idx])<<SENSOR_PIN0;
    SENSOR_PORT = port;

    //Serial.print(" SQ: "); Serial.println(0x10|GET_SENSOR(),BIN);
    if(++sequence_idx >= 6) sequence_idx = 0;
    last_time_us = this_time_us;
    //togLED();
    
  }
  #endif 
}
/* we need to measure the time bewteen each interrupt
 *  averaged over the last full electrical cycle
 *  it may turn out this is overly complex
 *  and simply using the segment time without averaging
 *  may turn out to be required
 *  to keep the isr tight
 *  
 *  ISR measured as 8.67us
 */
 #ifdef PERF_REPORT

 byte performance_timer2 = 0;
static bool do_perf_timer2_print = false;
void print_perf_timer2()
{
  if(do_perf_timer2_print)
  {
    static unsigned int last_time_ms;
    unsigned int this_time_ms=millis();
    if((this_time_ms- last_time_ms) >REPORT_TIME_ms )
    {
      last_time_ms = 0;
      last_time_ms= this_time_ms;
      Serial.print("Perf2(us): ");    Serial.println(TMR1_TCK_TIME_us(performance_timer2));
      do_perf_timer2_print = false;
    }
  }
}
#endif

ISR(PCINT1_vect, ISR_ALIASOF(PCINT0_vect));
ISR(PCINT2_vect, ISR_ALIASOF(PCINT0_vect));
ISR(PCINT0_vect)
{
  #ifdef PERF_REPORT
  performance_timer2 = TCNT1;
  #endif
  
  //setDBG();
  togLED();
  
  interval_us = tmr1_time_tck * (TMR1_OVF_TIME_us);
  /* fetch the latest sensor value */
  sensor_position = sensor_decode[GET_SENSOR()];
  /* check for a valid state */
  if(sensor_position != 0xff)
  {
 
    //unsigned int time_now_us = micros();
    //interval_us = time_now_us - last_sensor_change_time_us;
    //last_sensor_change_time_us = time_now_us;
    
    //tmr1_time_tck = 0;
    #ifdef SESNOR_SIM
    sim_processed = true;
    #endif
    /* clamp to range */
    //interval_us = interval_us>STEP_RATE_MINIMUM_us?interval_us:STEP_RATE_MINIMUM_us;
    //interval_us = interval_us<STEP_RATE_MAXIMUM_us?interval_us:STEP_RATE_MAXIMUM_us;
    /* fetch new pwm values */
    //sin_rate = STEP_RATE_us((interval_us >> 2) + (interval_us >> 1)); //pwm appears to be advancing about 5x too quick... 
    //6x infact. because time is now per sensor step, not per waveform cycle
    //if(sin_rate == 0) sin_rate=1;
    //pwm_max = PWM_RATE_us(interval_us);

    if(tmr1_time_tck > STEP_RATE_MINIMUM_tck)
    {
      tmr1_time_tck -= STEP_RATE_MINIMUM_tck;
    }
    else
    {
      tmr1_time_tck = 0;
    }
    
    sin_rate = STEP_RATE(tmr1_time_tck); 
    pwm_max = PWM_RATE(tmr1_time_tck);
    tmr1_time_tck = 0;
    
    //synchronise the sin wave generators with the sensor position
    sin_pos1 = start_phase1_step[sensor_position] + phase_offset;
    
    //wrap phase around
    sin_pos1 = sin_pos1>sin_table_size?sin_pos1-sin_table_size:sin_pos1;

    //next seg position. note no wrap around
    sin_hold_pos = sin_pos1 + SEG_SIZE;

    DRIVE_EN(); //ensure drive is enabled
    sin_en(); //ensure waveform generator is enabled

    #ifdef PERF_REPORT
    performance_timer2 = TCNT1 - performance_timer2;
    do_perf_timer2_print = true;
    #endif
  }
  else
  {
    /* shut down the drive when the sensor inputs are invalid */
    DRIVE_DIS();
    sin_dis();
  }
  //clrDBG();
}
