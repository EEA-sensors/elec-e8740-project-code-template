from altimu10v5.lsm6ds33 import LSM6DS33
from altimu10v5.lis3mdl import LIS3MDL
from altimu10v5.lps25h import LPS25H
import calendar
import numpy as np
import time
from time import sleep
import datetime

dt = str(datetime.datetime.now())
#format of saving file name 

lsm6ds33 = LSM6DS33()
lsm6ds33.enable()
#enabling above IC which responsible for accelerometer and gyroscope.

lis3mdl = LIS3MDL()
lis3mdl.enable()
#enabling above IC which responsible for magnetometer 


     
while True:
    
    timestamped_imu_readings = np.ndarray((0,), np.float32)
    #creating and assigning an array to store readings
    timestamp= time.time()*1000
    #millisecond timestamping
    
    timestamped_imu_readings = np.append(timestamped_imu_readings, lsm6ds33.get_accelerometer_g_forces())
    timestamped_imu_readings = np.append(timestamped_imu_readings, lsm6ds33.get_accelerometer_angles())
    timestamped_imu_readings = np.append(timestamped_imu_readings, lsm6ds33.get_gyroscope_raw())
    timestamped_imu_readings = np.append(timestamped_imu_readings, lis3mdl.get_magnetometer_raw())
    #appending IMU readings to the array
    
    
    timestamped_imu_readings = np.append(float(timestamp),timestamped_imu_readings)
    #adding sensor readings in a row
    
   # with open("IMU_Readings_muh.csv", "ab") as f:
        #np.savetxt(f, np.expand_dims(timestamped_imu_readings, axis=0),  fmt="%4.8f" , delimiter=",")
        
    with open("Readings_IMU {}.csv".format(dt), "ab") as ff:
        np.savetxt(ff, np.expand_dims(timestamped_imu_readings, axis=0),fmt='%4.8f' , delimiter=",")
        
        #saving file in name of current time and in format of XXXX.XXXXXXXX seperated by , 
        
        

    #sleep(0.2)
    sleep(0.05)
        

    

    
    


