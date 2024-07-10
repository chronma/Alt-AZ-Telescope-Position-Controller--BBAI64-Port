import board
import adafruit_lsm303_accel
import adafruit_lis2mdl
import adafruit_mma8451
import time 
import busio
import serial
import subprocess
import math
import numpy
#!/usr/bin/env python


i2c = board.I2C()
sensor = adafruit_lsm303_accel.LSM303_Accel(i2c)
mag = adafruit_lis2mdl.LIS2MDL(i2c)
accel = adafruit_mma8451.MMA8451(i2c)
accel.data_rate=adafruit_mma8451.DATARATE_100HZ  #  100Hz
print ("range ",accel.range)
accel.range= adafruit_mma8451.RANGE_2G  # +/- 2G
RX = 'P9.24'
TX = 'P9.26'

gps = serial.Serial(port = "/dev/ttyO1", baudrate=9600)
gps.close()
gps.open()

hardiron_calibration = [[-67.05, -20.849999999999998], [-12.45, 26.55], [-38.85, -32.25]]
def normalize(_magvals):

    ret = [0, 0, 0]

    for i, axis in enumerate(_magvals):

        minv, maxv = hardiron_calibration[i]

        axis = min(max(minv, axis), maxv)  # keep within min/max calibration

        ret[i] = (axis - minv) * 200 / (maxv - minv) + -100

    return ret

accel.range=1
accel.mode=1

#mag.reset
#mag.x_offset=-26.4
#mag.y_offset=-35.1
#mag.z_offset=-19


def main():
    while 1:
        print("tube angle: Acceleration (m/s^2): X=%0.5f Y=%0.5f Z=%0.5f"%accel.acceleration)
        Accx,Accy,Accz=accel.acceleration # tupple from sensor
        angletohome=math.degrees(math.atan2(round(Accz,2),-round(Accx,2)))
        print("angletohome az",angletohome)
        
        print("Base: Acceleration (m/s^2): X=%0.3f Y=%0.3f Z=%0.3f"%sensor.acceleration)
        Accx,Accy,Accz=sensor.acceleration # tupple from sensor
        angletohome=math.degrees(math.atan2(round(Accz,2),-round(Accx,2)))
        print("angletohome az",angletohome)
        absvector=math.sqrt(Accx*Accx+Accy*Accy+Accz*Accz)
        print("vector magnitude: ",absvector)
        Axn,Ayn,Azn=Accx/absvector,Accy/absvector,Accz/absvector
        Anormal=(Axn,Ayn,Azn)   
        print("Base: Normal X=%0.5f Y=%0.5f Z=%0.5f"%Anormal)
        Aup=(0,0,1)
        Ay=(0,1,0)
        
        vectordotproduct=numpy.dot(Anormal,Aup)
        vectorcrossproduct=numpy.cross(Aup,Anormal)
        
        print("dotproduct, crossproduct base @ up: ",vectordotproduct,vectorcrossproduct)
        anglebetweenvectors=math.degrees(math.acos(vectordotproduct/(1*1)))
        print("angle between vectors: ",anglebetweenvectors)
        
        planevectordotproduct=numpy.dot(vectorcrossproduct,Ay)
        planeabsvector=math.sqrt(vectorcrossproduct[0]*vectorcrossproduct[0]+vectorcrossproduct[1]*vectorcrossproduct[1]+vectorcrossproduct[2]*vectorcrossproduct[2])
        vectorprojectionangle=math.degrees(math.acos(planevectordotproduct/(1*planeabsvector))) #angle between x axis and shadow of base z axis 
        print("ground plane angle between vectors: ",vectorprojectionangle)
        
        magvals = mag.magnetic
    
        normvals = normalize(magvals)
    
        print("magnetometer: %s -> %s" % (magvals, normvals))
    
    
        # we will only use X and Y for the compass calculations, so hold it level!
    
        compass_heading = int(math.atan2(normvals[1], normvals[0]) * 180.0 / math.pi)
        #compass_heading = int(math.atan2(magvals[0], magvals[2]) * 180.0 / math.pi)
        # compass_heading is between -180 and +180 since atan2 returns -pi to +pi
    
        # this translates it to be between 0 and 360
    
        compass_heading += 180
        
    
        print("Heading:", compass_heading)
        print("gps", gps.readline())
        #print("gps", gps.readline())
        #print("gps", gps.readline())
        #print("gps", gps.readline())
        time.sleep(1) 
        subprocess.run("clear", shell=True, check=True)
        



main()
   