void testpwm(byte tcnt1)
{
   /* to better understand the problem with a cycle being on too long,
   *  toggle between low and off every other cycle,
   *  and each time, change the time at which the command is issueed
   *  results: the problem is worse when updating just after TCNT=0
   *  and get progressively bettwer towards TCNT=255
   */
   
   static byte flip_state = 0;   
   #define FLIP_AT 32
  static byte flip_at = FLIP_AT;
  static byte pwm_high = 1;
  #define PWM_INC 8
  static byte skip_count = 0;
  #define SKIP 4
  enum{
    WAIT_OFF,
    TURN_OFF,
    WAIT_ON,
    TURN_ON
  };

  
  switch(flip_state)
  {
    default:
    case WAIT_OFF: //0
      if (tcnt1 == 0)
      {
        if(skip_count++ >= SKIP) 
        {
          flip_state = TURN_OFF;
          skip_count = 0;
          
        }
      }

      break;
    case TURN_OFF: //1
      if (tcnt1 == flip_at)
      {
        
        pwmWrite(0,0);
        flip_state = WAIT_ON;
        Serial.print("turned on at: ");
        Serial.println(flip_at);
      }
      break;
    case WAIT_ON: //2
      if (tcnt1 == 0)
      {
        if(skip_count++ >= SKIP) 
        {
          flip_state = TURN_ON;
          skip_count = 0;
        }
      }
      break;
    case TURN_ON://3
      if (tcnt1 == flip_at)
      {
        pwmWrite(0, pwm_high);
        flip_state = WAIT_OFF;
        
        Serial.print("turned off at: ");
        Serial.println(flip_at);
        
        flip_at+=FLIP_AT;
        if(!flip_at)
        {
          pwm_high += PWM_INC;
          Serial.print("next flip to: ");
          Serial.println(pwm_high);
          
        }
      }
      break;
  }
}
