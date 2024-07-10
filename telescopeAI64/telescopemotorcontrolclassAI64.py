import ctypes
from ctypes import *
import subprocess
import os
import mmap
import time
import board
import adafruit_lsm303_accel
import adafruit_lis2mdl
import adafruit_mma8451
import busio
import serial
import math
import numpy
import gpiod
from gpiod.line import Direction, Value


def set_enable_(chip_path, line_offset,enab):
    value_str = {Value.ACTIVE: "Active", Value.INACTIVE: "Inactive"}
    value = Value.ACTIVE

    with gpiod.request_lines(
        chip_path,
        consumer="set enable",
        config={
            line_offset: gpiod.LineSettings(
                direction=Direction.OUTPUT, output_value=value
            )
        },
    ) as request:
        
            if enab == 0:
                request.set_value(line_offset, Value.ACTIVE)
            else:
                request.set_value(line_offset, Value.INACTIVE) 




tsleep=.3

#I2C COMS TO Sensors

#i2c = board.I2C()
#time.sleep(tsleep)
#accel = adafruit_mma8451.MMA8451(i2c)
#time.sleep(tsleep)
#mag = adafruit_lis2mdl.LIS2MDL(i2c)
#time.sleep(tsleep)
#sensor = adafruit_lsm303_accel.LSM303_Accel(i2c) # not used yet could check base level
#time.sleep(tsleep)

# globals

Task= ['getbit','setbit', 'clearbit']
#acess pru/arm shared memory   
SRAM_Offset=0x0000000  #pru0 SRAM offset
PRU0_Address=0x0b000000  #pru0 base address AI64
Address=(SRAM_Offset+PRU0_Address)

fd = os.open("/dev/mem", os.O_SYNC | os.O_RDWR)
mem = mmap.mmap(fd, mmap.PAGESIZE, flags=mmap.MAP_SHARED, offset=Address)
os.close(fd)
command_bits=0
command_bits0=0
command_bits1=0
command_bits2=0

#Axis 1 - azimuth axis
Axis1_microsteps=16
Axis1_ratio=50*10  #gearbox*gear ratio
Axis1_motor_deg_per_step=1.8
Axis1_direction_bit=3
Axis1_command_bit=0
Axis1_deg_per_step=Axis1_motor_deg_per_step/Axis1_microsteps/Axis1_ratio

#Axis 2 - alt axis
Axis2_microsteps=16
Axis2_ratio=50*10  #gearbox*gear ratio
Axis2_motor_deg_per_step=1.8
Axis2_direction_bit=4
Axis2_command_bit=1
Axis2_deg_per_step=Axis2_motor_deg_per_step/Axis2_microsteps/Axis2_ratio

#Axis 3 - focuser axis
Axis3_microsteps=16
Axis3_motor_deg_per_step=1.8
Axis3_max_turns=3.25
Axis3_direction_bit=5
Axis3_command_bit=2
Axis3_deg_per_step=Axis3_motor_deg_per_step/Axis3_microsteps

pruclockfreq=200000000*5.969  # rate the prus porgram cycles 200MHz*No instructions approximate



def bitmagic(value, bitnumber,task):
    if task == 'getbit':
        return ((value>>bitnumber)&1)
    if task == 'setbit':
        return (value | (1<<bitnumber))
    if task == 'clearbit':
        return (value & ~(1<<bitnumber))
    
    if task =='TogleBit':
        return (value ^((value>>bitnumber)&1))

class motorcontrol(object):
    #pru 0
    
    def __init__(self):
             
        subprocess.run("echo 'stop' > /sys/class/remoteproc/remoteproc0/state", shell=True, check=False)
        subprocess.run("sudo cp /home/debian/Steppercontrolpru2.out /lib/firmware/Steppercontrolpru2.out", shell=True, check=True)
        subprocess.run("echo 'Steppercontrolpru2.out' > /sys/class/remoteproc/remoteproc0/firmware", shell=True, check=True)
        subprocess.run("echo 'start' > /sys/class/remoteproc/remoteproc0/state", shell=True, check=True)
    
     
        
        
    def __del__(self):
        chip_path="/dev/gpiochip1"
        line_offset=74 #P8_30_Line_Offset=74  # output stepper high disable
        set_enable_(chip_path, line_offset,0)
        #pin P8-30 enable steppers high disables
        
    def disable_stepper_drivers(self):
        
        chip_path="/dev/gpiochip1"
        line_offset=74
        set_enable_(chip_path, line_offset,0)
        #pin P8-30 enable steppers high disables

    def enable_stepper_drivers(self):
        chip_path="/dev/gpiochip1"
        line_offset=74
        set_enable_(chip_path, line_offset,1)
        #pin P8-30 enable steppers high disables

    def AZ_rotate(self,pru0_command_bits,a1degrees,a1degreespersec,a1acceltime):
        pru0_command_bits=0
        a1steps=int((abs(a1degrees))/Axis1_deg_per_step) #run the 
        print ('steps', a1steps)
        a1pulselength=int((1/a1degreespersec)*Axis1_deg_per_step*pruclockfreq) #set frequency
        print('pulselength',a1pulselength)
        a1adsteps=int(math.sqrt( (8*(a1acceltime*1000000000)+a1pulselength)/(4*a1pulselength))+.5) #no accel /decel
        print('accel steps ',a1adsteps)
        ctypes.c_uint32.from_buffer(mem, 0x100).value = a1steps
        ctypes.c_uint32.from_buffer(mem, 0x104).value = a1pulselength 
        ctypes.c_uint32.from_buffer(mem, 0x118).value = a1adsteps
        if (a1degrees<0):
            pru0_command_bits=bitmagic(pru0_command_bits,Axis1_direction_bit,'setbit') # 
        else:
             pru0_command_bits=bitmagic(pru0_command_bits,Axis1_direction_bit,'clearbit')
        
        pru0_command_bits=bitmagic(pru0_command_bits,Axis1_command_bit,'setbit') # 
        print(hex(pru0_command_bits))
        time.sleep(.1)
        return (pru0_command_bits)


    

    def ALT_move(self,pru0_command_bits,a2degrees,a2degreespersec,a2acceltime):
        pru0_command_bits=0
        a2steps=int((abs(a2degrees))/Axis2_deg_per_step) #run the 
        print ('steps', a2steps)
        a2pulselength=int((1/a2degreespersec)*Axis2_deg_per_step*pruclockfreq) #set frequency
        print('pulselength',a2pulselength)
        a2adsteps=int(math.sqrt( (8*(a2acceltime*1000000000)+a2pulselength)/(4*a2pulselength))+.5) #no accel /decel
        print('accel steps ',a2adsteps)
        ctypes.c_uint32.from_buffer(mem, 0x108).value = a2steps
        ctypes.c_uint32.from_buffer(mem, 0x10C).value = a2pulselength 
        ctypes.c_uint32.from_buffer(mem, 0x11C).value = a2adsteps
        if (a2degrees<0):
            pru0_command_bits=bitmagic(pru0_command_bits,Axis2_direction_bit,'setbit') # 
        else:
             pru0_command_bits=bitmagic(pru0_command_bits,Axis2_direction_bit,'clearbit')
        
        pru0_command_bits=bitmagic(pru0_command_bits,Axis2_command_bit,'setbit') # 
        print(hex(pru0_command_bits))
        time.sleep(.1)
        return (pru0_command_bits)

    def focuser_home(self,pru0_command_bits):
        pru0_command_bits=0
        a3steps=int((Axis3_microsteps*Axis3_max_turns*360)/Axis3_motor_deg_per_step) #run the focuser in until it cycles the max turns (the focuser)
        print ('steps', a3steps)
        time_home=3*(10000000000/5) #homing run time
        print('time home',time_home)
        a3pulselength=int(time_home/a3steps) #set frequency
        a3adsteps=1 #no accel /decel
        ctypes.c_uint32.from_buffer(mem, 0x110).value = a3steps
        ctypes.c_uint32.from_buffer(mem, 0x114).value = a3pulselength 
        ctypes.c_uint32.from_buffer(mem, 0x120).value = a3adsteps
        pru0_command_bits=bitmagic(pru0_command_bits,Axis3_direction_bit,'setbit') # 
        pru0_command_bits=bitmagic(pru0_command_bits,Axis3_command_bit,'setbit') # 
        print(hex(pru0_command_bits))
        time.sleep(.1)
        return (pru0_command_bits)

    def focuser_move(self,pru0_command_bits, a3degrees,a3degreespersec,a3accelrate):
        pru0_command_bits=0
        a3steps=int((Axis3_microsteps*abs(a3degrees))/Axis3_motor_deg_per_step) #run the focuser in until it cycles the max turns (the focuser)
        print ('steps', a3steps)
        a3pulselength=int((1/a3degreespersec)*Axis3_deg_per_step*pruclockfreq) #set frequency
        a3adsteps=a3accelrate #no accel /decel
        ctypes.c_uint32.from_buffer(mem, 0x110).value = a3steps
        ctypes.c_uint32.from_buffer(mem, 0x114).value = a3pulselength 
        ctypes.c_uint32.from_buffer(mem, 0x120).value = a3adsteps
        if (a3degrees<0):
            pru0_command_bits=bitmagic(pru0_command_bits,Axis3_direction_bit,'setbit') # 
        else:
             pru0_command_bits=bitmagic(pru0_command_bits,Axis3_direction_bit,'clearbit')
        
        pru0_command_bits=bitmagic(pru0_command_bits,Axis3_command_bit,'setbit') # 
        print(hex(pru0_command_bits))
        time.sleep(.1)
        return (pru0_command_bits)





