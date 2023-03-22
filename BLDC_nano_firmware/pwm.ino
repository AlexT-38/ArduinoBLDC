/***************************************
 * handles the generation of pwm signals
 */
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
 *  
 *  at the desired 31kHz base pwm rate, this interrupt would be called every 32us
 *  this is 512 instruction cycles (62.5ns each)
 *  
 *  this function should be able to complete in significantly less time,
 *  to account for other interrupts taking up cycles.
 *  infact, all interrupts must be capable of executing within this time
 *  
 *  presently typical timeforTIMER1_OVF is 14.5us
 *  which may be acceptable, but ideally would be lower
 *  
 *  it is occasionally 20.5us, which is probably TMR0 ISR,
 *  as we are measuring from actual TMR1 ovf, not start of the ISR
 *  
 */
static byte performance_timer = 0;
static bool do_perf_timer_print = false;
void print_perf_timer()
{
  if(do_perf_timer_print & true)
  {
    static unsigned int last_time_ms;
    unsigned int this_time_ms=millis();
    if((this_time_ms- last_time_ms) >REPORT_TIME_ms )
    {
      last_time_ms= this_time_ms;
      Serial.print("Perf1: ");
      Serial.println( performance_timer);
      do_perf_timer_print = false;
    }
  }
}
ISR(TIMER1_OVF_vect)//,ISR_NAKED) //naked = nothing uses the stack
{
  
  // write the pwm values
  OCR1A = pwm1;
  OCR1B = pwm2;
  OCR2A = pwm3;

  // disable outputs where pwm value is zero
  if(pwm1 != 0)  {    OC1A_EN();  }
  else            {    OC1A_DIS();  }
  if(pwm2 != 0)  {    OC1B_EN();  }
  else            {    OC1B_DIS();  }
  if(pwm3 != 0)  {    OC2A_EN();  }
  else            {    OC2A_DIS();  }
  
  //step through the sin tables for each phase, hold if the next step would take us past the next seg
  if(do_sin && ((sin_pos1+sin_rate) > sin_hold_pos))
  {
    /* we need some way of syncing with the sensor state changes
     *  this basically means jumping ahead if we are lagging behind
     *  or stopping the sinwave until the rotor catches up
     *  the indices in the sin table where sensor state changes should occur
     *  can be specified to give the correct phase advance for efficient running
     * 
     *  it might be better to change the frequency gently, like a PLL
     *  
     *  this is handled in sensors tab
     *  
     */


    
    //increment by rate
    /*
    #define SIN_INC(x) x=x+sin_rate
    SIN_INC(sin_pos1);
    SIN_INC(sin_pos2);
    SIN_INC(sin_pos3);
    */
    sin_pos1 = sin_pos1 + sin_rate;
    sin_pos2 = sin_pos1 + SEG_OFFSET;
    sin_pos3 = sin_pos2 + SEG_OFFSET;

    /* we can increase performance slightly by tabulating 2 whole sine waves,
     * this would mean only needing to check for loop back 
     */
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
  performance_timer = TCNT1;
  do_perf_timer_print = true;
}

void pwmInit()
{
   
  pinMode(pin_OC1A, OUTPUT);
  pinMode(pin_OC1B, OUTPUT);
  pinMode(pin_OC2A, OUTPUT);
  pinMode(pin_OC2B, OUTPUT);

  //set the state of the otput pins when pwm is 0, which is LOW when using non inverted output
  digitalWrite(pin_OC1A, TMR_COM&1);
  digitalWrite(pin_OC1B, TMR_COM&1);
  digitalWrite(pin_OC2A, TMR_COM&1);
  digitalWrite(pin_OC2A, TMR_COM_VREF&1);





  TCCR1A = T1M(WGM11,1) | T1M(WGM10,0);
  TCCR1B = T1M(WGM13,3) | T1M(WGM12,2);

  TCCR2A = T2M(WGM21,1) | T2M(WGM20,0);
  TCCR2B = T2M(WGM22,2);


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
   *  nope, tried comp for one and tof for the other, both ways
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
