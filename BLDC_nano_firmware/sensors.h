#ifndef __SENSORS_H_
#define __SENSORS_H_

extern unsigned int interval_us;
extern unsigned int interval_tck;
extern const byte sensor_seq[];
extern byte sensor_position;
extern unsigned int phase_offset;
extern byte pwm_max;
extern unsigned int sin_pos1_frac;

#ifdef PERF_REPORT
extern byte performance_timer2, performance_timer;
#endif

#endif __SENSORS_H_
