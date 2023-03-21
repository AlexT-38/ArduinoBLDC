# -*- coding: utf-8 -*-
"""
Created on Sun Mar 19 19:22:11 2023

@author: AlexT-38
"""
import numpy as np
from matplotlib import pyplot as plt


""" 
    SIN TABLE
    just a sin table with a multpile of thre indices, and overall length chosen to give
    reasonable steps per update for the operating frequency range
"""


N_SAMPS = 768 * 5

time = np.linspace(0,1,N_SAMPS,0)

table_sin = np.sin(time*2*np.pi)
table_sin = (table_sin+1)*255/2
table_sin = table_sin.round()

plt.step(time, table_sin)
plt.show()
print(table_sin[0],table_sin.min(),table_sin.max())

#print(table_to_c_array(table_sin))

def table_to_c_array(table):
    string = "{ "
    for n, t in enumerate(table):
        if n%256 == 0:    #maximum 256 entries to a line
            string = string + "\n"
        string = string + str(round(t)) +", "
    string = string + "};"
    return string

def print_table(y,x):
    print(table_to_c_array(y))
    print(y.size)
    print(round(x[0]), "->", round(x[-1]))


"""
    STEP RATE FROM INTERVAL TABLE
    this is a more complex table to derive,
    but at it's heart it is just a 1/x table
"""

RESOLUTION_us  = 32   # number of microseconds per table index
MINIMUM_us     = 4000   # offset microseconds for table index 0

POLE_PAIRS      = 2  #number of electrical cyles per shaft revolution
STEPS           = 6  #number of phase steps per electrical cycle, this is always 6

MIN_RPM         = 1000 #minimum rpm we can expect to handle
MAX_RPM         = 5600 #maximum rpm we can expect to handle

UPDATE_RATE_Hz  = 31250

MIN_FREQ_Hz        = POLE_PAIRS*MIN_RPM/60
MAX_FREQ_Hz        = POLE_PAIRS*MAX_RPM/60

MIN_INT_us      = 1000000/MAX_FREQ_Hz
MAX_INT_us      = 1000000/MIN_FREQ_Hz

# we can simplify matters by ommiting values for higher speeds
MINIMUM_us      = round(MIN_INT_us)

INT_RANGE_VALID_us    = MAX_INT_us-MIN_INT_us
INT_RANGE_TOTAL_us    = MAX_INT_us-MINIMUM_us

def int2idx(interval):
    return round((interval-MINIMUM_us)/RESOLUTION_us)

N_INTS = 1+round(INT_RANGE_TOTAL_us / RESOLUTION_us)

#input axis
intervals = np.linspace(MINIMUM_us,MAX_INT_us,N_INTS)
intervals = np.round(intervals/RESOLUTION_us)*RESOLUTION_us
Hz2MHz = 1000000
#number of sin table indices to advance each pwm update for each cycle interval
table_rate = Hz2MHz * N_SAMPS / (UPDATE_RATE_Hz * intervals)


points = [int2idx(MIN_INT_us),int2idx(MAX_INT_us)]
plt.step(intervals, table_rate)
plt.plot(intervals[points],table_rate[points],'x')
plt.show()

#print_table(intervals, table_rate)

"""
    MAX PWM FROM INTERVAL TABLE
    this is basically the same table as above
    but with pwm values instead of step values
"""

table_pwm = 255 * intervals.min()/intervals

plt.step(intervals, table_pwm)
plt.show()

print_table(table_pwm, intervals)