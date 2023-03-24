#ifndef __PWM_H__
#define __PWM_H__

  /* configure timer1 and timer2 for 8bit pwm at a frequency of 10kHz or greater
   *  (base clock 2.4MHz or greater)
   *  based on a 16Mhz clock (CH340 based Nano, was expecting it to be 12Mhz, but is infact 16)
   *  
   *  Timer1 prescaler is 0,1,8,64,256,1024,ext.v, ext.^
   *  Timer2 prescaler is 0,1,8,32,64,128,256,1024
   *   using phase correct pwm is prefered for motor control 
   *   and also adds an extra x2 scale factor
   *  
   *  in phase correct 8bit pwm, frequency is given by Fclk(io)/(N*510) 
   *  rather than by Fclk(io)/(N*512)
   *  so frequencies are 0.4% higher than the 'correct' frequencies
   *  phase and frequency correct pwm is not available on TMR2
   *  
   *  (looking only at relavent settings...)
   *  
   *  Frequencies (Hz):
   *  prescale  Tclk    fast      phase correct   phase & freq. correct
   *  1         62.5ns  62500.0   31372.5         31250.0
   *  8         0.5us   7812.5    3921.6          3906.3
   *  64        4.0us   976.6     490.2           488.3
   *  256       16.0us  244.1     122.5           122.1
   *  
   *  
   *  PWM period (us):
   *  fast pwm        phase correct   phase and freq correct
   *  16.0            31.9            32.0
   *  128.0           255.0           256.0
   *  1024.0          2040.0          2048.0
   *  4096.0          8160.0          8192.0
   *  
   *  PWM freq must be at least the max motor frequency 
   *  and should be at least 4x, preferably 12x
   *  to accurately describe the waveform.
   *  
   *  there is also a 1/2 pwm cycle delay between
   *  setting the pwm and the output changing,
   *  but this can be compensated for (in theory) by 
   *  using a negative phase offset (more tables needed for this),
   *  or by switching to direct control of the output pins by the
   *  sensor state, but then may as well go for the simpler
   *  control.
   *  
   *  the tables are based on 5600 rpm being the motor's higest.
   *  so the maximum sensor step frequency we could see is 
   *  maximum step rate, max rpm * poles/2 * 6
   *  93.3hz * 2 * 6 = 1120Hz, 893us
   *  
   *  that's 3.5 pwm cycles per motor cycle at /8
   *  or 28 cycles at /1
   *  
   *  
   *  
   *  at the other end of things, too high a frequency would mean
   *  the shortest possible pulse time must be greater than the
   *  switch on and switch off times of the fets
   *  
   *  assuming FETs are PSMN3R4-30PL
   *  typ switching time, full cycle (delay, rise/fall), 40+73+59+28 = 200ns, 5Mhz
   *  shortest pwm pulse is  2/clk = 125ns, 8Mhz
   *  these fets may not be fast enough, but they're also old and better fets are probably available
   *  
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

#define TMR_TCKus_31kHz(t)    (t>>3)
#define TMR_TCKus_3906Hz(t)   (t)
#define TMR_TCKus_1953Hz(t)   (t<<1)
#define TMR_TCKus_977Hz(t)    (t<<2)
#define TMR_TCKus_244Hz(t)    (t<<3)
#define TMR_TCKus_122Hz(t)    (t<<4)
#define TMR_TCKus_31Hz(t)     (t<<7)

#define TMR1_PRESCALE       TMR1_PRESCALE_3906Hz
#define TMR2_PRESCALE       TMR2_PRESCALE_3906Hz

#define TMR1_OVF_TIME_us    TMR_INTus_3906Hz
#define TMR1_TCK_TIME_us(tk)    TMR_TCKus_3906Hz(tk)

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

 #define TMR_COM 2
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

  //to set or unset a bit mask in a register, multiply the bit mask by the set value, xor with target, and again, but masking the target bit
  //Register, bit Mask, State
  #define SET_TO(r, m, s) r ^= ((r) ^ ((m)*(s))) & (m)
  
     #define OC1A_EN() TCCR1A |= (TMR1_COMA)
  #define OC1A_DIS() TCCR1A &= ~(TMR1_COMA)
  #define OC1A_SET(s) SET_TO(TCCR1A, TMR1_COMA, s)
  
  #define OC1B_EN() TCCR1A |= (TMR1_COMB)
  #define OC1B_DIS() TCCR1A &= ~(TMR1_COMB)
  #define OC1B_SET(s) SET_TO(TCCR1A, TMR1_COMB, s)

  #define OC2A_EN() TCCR2A |= (TMR2_COMA)
  #define OC2A_DIS() TCCR2A &= ~(TMR2_COMA)
  #define OC2A_SET(s) SET_TO(TCCR2A, TMR2_COMA, s)
  
  #define OC2B_EN() TCCR2A |= (TMR2_COMB)
  #define OC2B_DIS() TCCR2A &= ~(TMR2_COMB)
  #define OC2B_SET(s) SET_TO(TCCR2A, TMR2_COMB, s)

    #define TMR1_OFFSET 2  //set this ensure TMR1 and TMR2 overflow by the same amount - measure by having one tof set the led, and one clear it, adjust until it switches from mostly on to mostly off, with timer2 having the off command

#endif //__PWM_H__
