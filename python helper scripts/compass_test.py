import board
import adafruit_lsm303_accel
import adafruit_lis2mdl
import time
import math

#         [        SM303AGR          ]
#         [          - X +       -   ]
#         [                      Y   ] 
#         [           Z(up+)     +   ] 
#         [                          ]
#         [  VIN 3V GND SCL SCA INT  ]
#         [   o   o  o   o   o   o   ]                  

# N  =0deg  E=90deg S=180deg W=270deg
i2c = board.I2C()
accel = adafruit_lsm303_accel.LSM303_Accel(i2c)
mag = adafruit_lis2mdl.LIS2MDL(i2c)
hardiron_calibration = [[-16.5, 9.45], [9.0, 37.199999999999996], [-19.8, -14.25]]

def normalize(_magvals,hi_calibration):

    ret = [0, 0, 0]

    for i, axis in enumerate(_magvals):

        minv, maxv = hi_calibration[i]

        axis = min(max(minv, axis), maxv)  # keep within min/max calibration

        ret[i] = (axis - minv) * 200 / (maxv - minv) + -100

    return ret

max_X=-100
min_X=100
max_Y=-100
min_Y=100


while 1:
    #print("Acceleration (m/s^2): X=%0.3f Y=%0.3f Z=%0.3f"%accel.acceleration)
    print("Magnetometer (micro-Teslas)): X=%0.3f Y=%0.3f Z=%0.3f"%mag.magnetic)
    magvals = mag.magnetic
    normvals = normalize(magvals,hardiron_calibration)
    print ("normals: ",normvals )
    max_X=max(max_X,magvals[0])
    min_X=min(min_X,magvals[0])
    max_Y=max(max_Y,magvals[1])
    min_Y=min(min_Y,magvals[1])
    extremes=[max_X,min_X,max_Y,min_Y]
    print("extremes: ",extremes)
    
    compass_heading = int(math.atan2(-normvals[0], -normvals[1]) * 180.0 / math.pi)
    compass_heading += 180
    print("compass_heading: ", compass_heading)
    
    time.sleep(2)