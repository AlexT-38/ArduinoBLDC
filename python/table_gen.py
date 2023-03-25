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
print("SINE SAMPLES: ", N_SAMPS)

time = np.linspace(0,1,N_SAMPS,0)

table_sin = np.sin(time*2*np.pi)
table_sin = (table_sin+1)*255/2
table_sin = table_sin.round()

plt.title(f"Sine Wave ({N_SAMPS})")
plt.step(time, table_sin)
plt.show()
#print(table_sin[0],table_sin.min(),table_sin.max())

#print(table_to_c_array(table_sin))

def table_to_c_array(table):
    string = "{ "
    for n, t in enumerate(table):
        if n%256 == 0:    #maximum 256 entries to a line
            string = string + "\n"
        string = string + str(round(t)) +", "
    string = string + "};"
    return string

def print_table(x,y):
    print(table_to_c_array(y))
    print(y.size)
    print(round(x[0]), "->", round(x[-1]))


"""
    STEP RATE FROM INTERVAL TABLE
    this is a more complex table to derive,
    but at it's heart it is just a 1/x table
    
    interval is the time from one sensor position to the next
    resolution should match the PWM period.
"""
MEG = 1000000
MINUTE = 60
CLOCK_MHz = 16

POLE_PAIRS       = 2  #number of electrical cyles per shaft revolution
STEPS            = 6  #number of phase steps per electrical cycle, this is always 6
print("Pole pairs (n):", POLE_PAIRS)

MIN_RPM          = 600 #minimum rpm we can expect to handle
MAX_RPM          = 6000 #maximum rpm we can expect to handle
print("RPM range (rpm):", MIN_RPM, " - ",MAX_RPM)

MIN_FREQ_Hz        = POLE_PAIRS*MIN_RPM/MINUTE      #minimum output frequency
MAX_FREQ_Hz        = POLE_PAIRS*MAX_RPM/MINUTE      #maximum output frequency
print("FREQ range (Hz):", MIN_FREQ_Hz, " - ",MAX_FREQ_Hz)

MIN_INT_us      = MEG/(MAX_FREQ_Hz*STEPS)   #minimum sensor step interval
MAX_INT_us      = MEG/(MIN_FREQ_Hz*STEPS)   #maximum sensor step interval
print("INT range (us):", MIN_INT_us, " - ",MAX_INT_us)

UPDATE_RATE_Hz  = CLOCK_MHz*MEG/510
print("PWM Freq(Hz):", UPDATE_RATE_Hz)
RESOLUTION_us    = MEG/UPDATE_RATE_Hz  # number of microseconds per table index
print("Resolution(us):", RESOLUTION_us)


TABLE_MIN_tck= int(MIN_INT_us/RESOLUTION_us) #table offset (indcies prior to this will be omitted)
TABLE_MIN_us = TABLE_MIN_tck*RESOLUTION_us
print("Table MIN (tck/us):", TABLE_MIN_tck, ", ",TABLE_MIN_us)

TABLE_MAX_tck= int(MAX_INT_us/RESOLUTION_us) #table offset (indcies prior to this will be omitted)
TABLE_MAX_us = TABLE_MAX_tck*RESOLUTION_us
print("Table MAX (tck/us):", TABLE_MAX_tck, ", ",TABLE_MAX_us)


TABLE_RANGE_us    = TABLE_MAX_us-TABLE_MIN_us
TABLE_SIZE_tck = TABLE_MAX_tck-TABLE_MIN_tck

print("Table Range(us), Size(tck):", TABLE_RANGE_us, ", ",TABLE_SIZE_tck)


def int2idx(interval):
    return ((interval-TABLE_MIN_us)/RESOLUTION_us)

#input axis
intervals_us = np.linspace(TABLE_MIN_us, TABLE_MAX_us, TABLE_SIZE_tck)
#intervals_us = np.round(intervals_us/RESOLUTION_us)*RESOLUTION_us
#print("Intervals: ", intervals_us)

#number of sin table indices to advance each pwm update for each cycle interval
"""
it takes N_SAMPS steps to perform one cycle
steps are taken once every UPDATE_RATE_Hz
table_rate is the number of steps to be taken each update such that
one cycle takes interval_us * STEPS

"""
cycle_us = intervals_us * STEPS

table_rate = np.round(MEG * N_SAMPS / (UPDATE_RATE_Hz * cycle_us))
#print("Rates: ", table_rate)

#points = [int2idx(MIN_INT_us),int2idx(MAX_INT_us)]
plt.title("sin rate table")
plt.step(intervals_us, table_rate)
#plt.plot(intervals_us[points],table_rate[points],'x')
plt.show()

print_table(intervals_us, table_rate)

"""
    MAX PWM FROM INTERVAL TABLE
    this is basically the same table as above
    but with pwm values instead of step values
    
    this would ideally be matched to the motor and minimum Vbus
    and then be multiplied by the ratio of Vbus(min):Vbus
    by the controller
    
    but at least the max rpm should be set independantly of the
    table size
    
"""
print()
# battery pack details
VBAT_CELL_MIN = 3 #3 for lipo, 2.5 for lifepo
VBAT_CELL_FULL = 4.2 #4.2 for lipo, 3.4 for lifepo 
VBUS_NOMINAL = 48 #voltage the motor is rated at
VBUS_N_CELLS = round(VBUS_NOMINAL/VBAT_CELL_FULL)   #number of cells needed in series to make the full voltage
VBUS_MIN = VBUS_N_CELLS*VBAT_CELL_MIN
VBUS_FULL = VBUS_N_CELLS*VBAT_CELL_FULL
print("Battery (cells, Vfull, Vmin): ", VBUS_N_CELLS, ", ",VBUS_FULL,", ", VBUS_MIN)

PWM_VBUS_RATIO = VBUS_MIN/VBUS_FULL
print("Vbus Ratio (min:full) : ", PWM_VBUS_RATIO)

PWM_MAX_RPM = 5600 #this is the rpm at which Vbemf == Vbus(nom)
PWM_RPM_RATIO = MAX_RPM/PWM_MAX_RPM
table_pwm = 255 * (intervals_us.min()/intervals_us) * PWM_RPM_RATIO #/ PWM_VBUS_RATIO
table_pwm = np.minimum(table_pwm, 255)

plt.title("pwm table")
plt.step(intervals_us, table_pwm)
plt.show()

print_table(intervals_us, table_pwm)

#FOR REFERENCE ONLY:
table_rpm = MEG*MINUTE/(intervals_us*STEPS*POLE_PAIRS)
plt.title("rpm table")
plt.step(intervals_us, table_rpm)
plt.show()
#print("rpm table: ", table_rpm)