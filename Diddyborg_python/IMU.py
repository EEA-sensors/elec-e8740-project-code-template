from altimu10v5.lsm6ds33 import LSM6DS33
from altimu10v5.lis3mdl import LIS3MDL
from altimu10v5.lps25h import LPS25H
import datetime
import numpy as np
import time
from time import sleep
# import cv2
import argparse

from pynput.keyboard import Key, Listener
import key_help as kh

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-o", "--output", type=str, default="IMU Record-{}.csv".format(datetime.datetime.now()),
	help="path to output CSV file containing IMU scan")
ap.add_argument("-s", "--sampling", type=float, default=0.05,
	help="time sampling in seconds, default=0.05")
args = vars(ap.parse_args())
time_sampling = args["sampling"]

#format of saving file name 

lsm6ds33 = LSM6DS33()
lsm6ds33.enable()
#enabling above IC which responsible for accelerometer and gyroscope.

lis3mdl = LIS3MDL()
lis3mdl.enable()
#enabling above IC which responsible for magnetometer 

MAGNETOMETER_LSB = 6842# for full scale -+4 Gauss
GAUSS_TO_MICRO_TESLA = 100
timestamped_imu_readings = np.zeros(12)

# open the output CSV file for writing
# csv = open(args["output"], "w")
listener = Listener(on_release=kh.on_release)
listener.start()
listener.wait()
# loop over until escape is pressed
while (listener.running):
# while True:
    #creating and assigning an array to store readings

    #timestamp= time.time()#
    #second timestamping
    timestamped_imu_readings[0] = np.array([time.time()])
    timestamped_imu_readings[1:4] = np.array(lsm6ds33.get_accelerometer_g_forces())
    timestamped_imu_readings[4:6] = np.array(lsm6ds33.get_accelerometer_angles())
    timestamped_imu_readings[6:9] = np.array(lsm6ds33.get_gyro_angular_velocity())
    timestamped_imu_readings[9:] = np.array(lis3mdl.get_magnetometer_raw())/MAGNETOMETER_LSB
    #appending IMU readings to the array
    
    
        
    with open(args["output"], "ab") as ff:
        np.savetxt(ff, np.expand_dims(timestamped_imu_readings, axis=0),fmt='%4.8f' , delimiter=",")

    #sleep(0.2)
    sleep(time_sampling)
    
    #check for termination
    # key = cv2.waitKey(1) & 0xFF
    # if the `q` key was pressed, break from the loop
    # if key == ord("q"):
    #     # close the output CSV file do a bit of cleanup
print("[INFO] Stopping IMU and cleaning up...")
# csv.close()
    #     break

    

    
    


