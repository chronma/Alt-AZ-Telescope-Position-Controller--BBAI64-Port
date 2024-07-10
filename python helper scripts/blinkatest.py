import board # uses adafruit BLinka
import adafruit_lsm303_accel
import adafruit_lis2mdl 
import adafruit_mma8451 #ADAFRUIT Accelerometer I2C coms
import os
import re
import time
import digitalio

#led = digitalio.DigitalInOut(board.P8_17)
#led.direction = digitalio.Direction.OUTPUT
 
#while True:
#    led.value = True
#    time.sleep(0.0005)
#    led.value = False
#    time.sleep(0.0005)

#f=os.open("/sys/bus/nvmem/devices/2-00500/nvmem", os.O_RDONLY|os.O_NONBLOCK) 
#print (f)
#strg=os.read(f,64)
#print(strg)
#os.close(f) 

i2c = board.I2C()  #Adafruit blinka create board
print("I2C devices found: ", [hex(i) for i in i2c.scan()])

accel = adafruit_mma8451.MMA8451(i2c, address=0x1d) # start tube accelerometer sensor
accel.data_rate=adafruit_mma8451.DATARATE_800HZ 
sensor = adafruit_lsm303_accel.LSM303_Accel(i2c) # start base accelerometer sensor
mag = adafruit_lis2mdl.LIS2MDL(i2c)

print("one",end='\r')
print("two",end='\r')


while True:
    Accx,Accy,Accz=accel.acceleration # tupple from sensor
    print (f'{800:.0f}%',end="",flush=True)
    print("Acceleration (m/s^2): X=%0.3f Y=%0.3f Z=%0.3f"%accel.acceleration, end=' ')
    print (f'{800:.0f}%',end="",flush=True)
    print("mag: X=%0.3f Y=%0.3f Z=%0.3f"%mag.magnetic, end='\r')
    
    time.sleep(.2)


