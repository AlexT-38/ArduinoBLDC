# -*- coding: utf-8 -*-
"""
Created on Sun Mar 19 19:22:11 2023

@author: AlexT-38
"""
import numpy as np
from matplotlib import pyplot as plt

def print_uint_name(base_name):
    print(f"const uint PROGMEM {base_name}[] = ")
    
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
    SIN TABLE
    just a sin table with a multpile of thre indices, and overall length chosen to give
    reasonable steps per update for the operating frequency range
"""


N_SAMPS = 768 * 5
print("SINE SAMPLES: ", N_SAMPS)

do_sin_lookup = False
if do_sin_lookup:
    time = np.linspace(0,1,N_SAMPS,0)
    
    table_sin = np.sin(time*2*np.pi)
    table_sin = (table_sin+1)*255/2
    table_sin = table_sin.round()
    
    plt.title(f"Sine Wave ({N_SAMPS})")
    plt.step(time, table_sin)
    plt.show()
    #print(table_sin[0],table_sin.min(),table_sin.max())
    
    #print(table_to_c_array(table_sin))
    print_table(time, table_sin)



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
print("PWM Freq(Hz):", round(UPDATE_RATE_Hz))
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

# dithering limits the maximum resolution of the sine table
# sine table must be less than 65536/PWM_DITHER_SCALE long
PWM_DITHER_SCALE  =  8
print("Dither scale:", PWM_DITHER_SCALE)

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

do_sin_rate = False
if do_sin_rate:
    table_rate = np.round(PWM_DITHER_SCALE * MEG * N_SAMPS / (UPDATE_RATE_Hz * cycle_us))
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
do_bat_pwm = False
if do_bat_pwm:
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

#print_table(intervals_us, table_pwm)

#FOR REFERENCE ONLY:
do_rpm = False
if do_rpm:
    table_rpm = MEG*MINUTE/(intervals_us*STEPS*POLE_PAIRS)
    plt.title("rpm table")
    plt.step(intervals_us, table_rpm)
    plt.show()
    print_table(intervals_us, table_rpm)
    p#rint("rpm table: ", table_rpm)



"""
phase shift table
this table compensates for phase shift of the motor current caused by the reactance of the coil windings

the winding incutance L and resistance R form a 1st order low pass filter
 with a 3dB cutoff frequency of 1/(2.Pi.RL)
 phase shift is -arctan(2.Pi.f.RL)
"""

MILLI=0.001
MICRO=MILLI*MILLI

CoilR = 1
CoilL = 1*MILLI

CoilFq = 1/(2*np.pi*CoilR*CoilL)

def CoilPhase(freq):
    return np.degrees(-np.arctan(freq/CoilFq))

def int2frq(interval_us):
    return 1/(cycle_us)

FLASH_LEFT = 10000
#how many tables can we fit into this much flash?
NO_PHASE_TABLES = FLASH_LEFT//(2*TABLE_SIZE_tck)
F0_STEP = (MAX_FREQ_Hz-MIN_FREQ_Hz)/NO_PHASE_TABLES

f0_range = np.arange(MIN_FREQ_Hz,MAX_FREQ_Hz,F0_STEP)

#concatenate all phase tables 
print_uint_name("phase_tables")

phase_tables = np.array([])
for f0 in f0_range:
    
    CoilFq = f0
    
    freq_table = int2frq(intervals_us)*MEG
    phase_table = -CoilPhase(freq_table)*N_SAMPS/360
    
    #plt.title("freq vs interval")
    #plt.step(intervals_us, freq_table)
    #plt.show()
    
    #plt.title("phase vs freq")
    #plt.step(freq_range, phase)
    #plt.show()
    
    plt.title(f"phase table {f0}")
    plt.step(intervals_us, phase_table)
    plt.show()
    
    phase_tables = np.append(phase_tables, phase_table.round())

print(table_to_c_array(phase_tables.astype(np.int16)))
    
#finally the table of f0s
print(f"#define NO_PHASE_TABLES {NO_PHASE_TABLES}")
print("const byte PROGMEM phase_table_f0[NO_PHASE_TABLES+1] =")

f0_table =np.append(f0_range, np.array([0]))
print(table_to_c_array(f0_range.round().astype(np.int16)))

