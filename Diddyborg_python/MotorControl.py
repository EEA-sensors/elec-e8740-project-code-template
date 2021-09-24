import RPi.GPIO as GPIO
from time import sleep
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
import time
import datetime
import numpy as np
import ThunderBorg3 as ThunderBorg
# import cv2
import argparse
import parser_help as ph

from pynput.keyboard import Key, Listener
import key_help as kh

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-o", "--output", type=str, default="MotorControl-{}.csv".format(datetime.datetime.now()),
	help="path to output CSV file containing barcodes")
ap.add_argument("-m", "--maxspeed", type=float, default=0.3,
	help="maximum motor speed, between 0 and 1, default 0.3")
ph.add_boolean_argument(ap,'direct',default=False,messages='Record pwm applied to motor, Default=False')
ph.add_boolean_argument(ap,'calibration',default=False,messages='Enter Calibration Mode, Default=False')


args = vars(ap.parse_args())

# open the output CSV file for writing
csv = open(args["output"], "w")
calibration_mode = args["calibration"]
direct_re = args["direct"]

TB = ThunderBorg.ThunderBorg()
TB.Init()


GPIO.setup(11, GPIO.IN)    #SO1
GPIO.setup(9, GPIO.IN)     #SO2
GPIO.setup(10, GPIO.IN)    #S03
GPIO.setup(22, GPIO.IN)    #SO4
GPIO.setup(27, GPIO.IN)    #SO5
sleep(1)


# speed=0.3
speed = args["maxspeed"]
# dt = str(datetime.datetime.now())

listener = Listener(on_release=kh.on_release)
listener.start()
listener.wait()

#Calibration Mode
# calibration_loop_n = 1000
if calibration_mode:
  # for _ in range(calibration_loop_n):
  while(listener.running):
    TB.SetMotor2(speed)
    TB.SetMotor1(speed)
    print("input 1 {}, input 2 {}".format(TB.GetMotor1(),TB.GetMotor2()))
    sleep(0.5)
  #set to zero
  TB.SetMotor2(0)
  TB.SetMotor1(0)
      
      
else:
  #on = black, off = white
  timestamped_motor_command = np.zeros(3)
  input_1 = 0
  input_2 = 0
  # try:
  
  # loop over until escape is pressed
  while (listener.running):
    s1=GPIO.input(11)
    s2=GPIO.input(9)
    s3=GPIO.input(10)
    s4=GPIO.input(22)
    s5=GPIO.input(27)
    
  #10000
    if (s1==1)& (s2==0)&(s3==0)&(s4==0)&(s5==0):
      #print ("left1")
      input_2 = 1*speed
      input_1 = 0*speed
      # TB.SetMotor2(input_1)
      # TB.SetMotor1(input_2)
  #11000
    elif (s1==1)&(s2==1)&(s3==0)&(s4==0)&(s5==0):
      #print ("left2")
      input_2 = 1*speed
      input_1 = 0.125*speed
      # TB.SetMotor2(1*speed)
      # TB.SetMotor1(0.125*speed)
  #11100
    elif (s1==1)&(s2==1)&(s3==1)&(s4==0)&(s5==0):
      #print ("left3")
      input_2 = 1*speed
      input_1 = 0.375*speed
      # TB.SetMotor2(1*speed)
      # TB.SetMotor1(0.375*speed)
  #01000    
    elif (s1==0)&(s2==1)&(s3==0)&(s4==0)&(s5==0):
      #print ("left4")
      input_2 = 1*speed
      input_1 = 0.45*speed
      # TB.SetMotor2(1*speed)
      # TB.SetMotor1(0.45*speed)
  #01100
    elif (s1==0)&(s2==1)&(s3==1)&(s4==0)&(s5==0):
      #print ("left5")
      input_2 = 1*speed
      input_1 = 0.6*speed
      # TB.SetMotor2(1*speed)
      # TB.SetMotor1(0.6*speed)
  #00001
    elif (s1==0)&(s2==0)&(s3==0)&(s4==0)&(s5==1):
      #print ("right1")
      input_2 = 0*speed
      input_1 = 1*speed
      # TB.SetMotor2(0*speed)
      # TB.SetMotor1(1*speed)
  #00011
    elif (s1==0)&(s2==0)&(s3==0)&(s4==1)&(s5==1):
      #print ("right2")
      input_2 = 0.125*speed
      input_1 = 1*speed
      # TB.SetMotor2(0.125*speed)
      # TB.SetMotor1(1*speed)
  #00111
    elif (s1==0)&(s2==0)&(s3==1)&(s4==1)&(s5==1):
      #print ("right3")
      input_2 = 0.375*speed
      input_1 = 1*speed
      # TB.SetMotor2(0.375*speed)
      # TB.SetMotor1(1*speed)
  #00010
    elif (s1==0)&(s2==0)&(s3==0)&(s4==1)&(s5==0):
      #print ("right4")
      input_2 = 0.48*speed
      input_1 = 1*speed
      # TB.SetMotor2(0.48*speed)
      # TB.SetMotor1(1*speed)
  #00110
    elif (s1==0)&(s2==0)&(s3==1)&(s4==1)&(s5==0):
      #print ("right5")
      input_2 = 0.6*speed
      input_1 = 1*speed
      # TB.SetMotor2(0.6*speed)
      # TB.SetMotor1(1*speed)
  #00100
    elif (s1==0)&(s2==0)&(s3==1)&(s4==0)&(s5==0):
      #print ("straight1")
      input_2 = 1*speed
      input_1 = 1*speed
      # TB.SetMotor2(1*sp eed)
      # TB.SetMotor1(1*speed)
  #01110
    elif (s1==0)&(s2==1)&(s3==1)&(s4==1)&(s5==0):
      #print ("straight2")
      input_2 = 1*speed
      input_1 = 1*speed
      # TB.SetMotor1(1*speed)
      # TB.SetMotor2(1*speed)
    #01111
    elif (s1==0)&(s2==1)&(s3==1)&(s4==1)&(s5==1):
      input_2 = 0.8*speed
      input_1 = 1*speed
    #11110
    elif (s1==1)&(s2==1)&(s3==1)&(s4==1)&(s5==0):
      input_2 = 1*speed
      input_1 = 0.8*speed
  #11111
    elif (s1==1)&(s2==1)&(s3==1)&(s4==1)&(s5==1):
      #print ("STOP")
      input_2 = 0*speed
      input_1 = 0*speed
      # TB.SetMotor1(0)
      # TB.SetMotor2(0)
      #sleep(0.5)
    
    TB.SetMotor2(input_2)
    TB.SetMotor1(input_1)
    sleep(0.5)
    timestamp= time.time()#
    # input_1 =  TB.GetMotor1()
    # input_2 = TB.GetMotor2()
    # if input_1 is None:
    #   input_1 = -99999.
    # if input_2 is None:
    #   input_2 = -99999.

    # timestamped_motor_command = np.array([[timestamp,input_1,input_2]])
    # #print(timestamped_motor_command)
    # with open("Motor_Input{0}.csv".format(dt), "ab") as f:
    #       # np.savetxt(f, np.expand_dims(timestamped_motor_command, axis=0),fmt='%4.8f' , delimiter=",")
    #       np.savetxt(f, timestamped_motor_command,fmt='%4.8f' , delimiter=",")

    csv.write("{},{},{}\n".format(timestamp,input_1,input_2))
    csv.flush()

    #check for termination
    # key = cv2.waitKey(1) & 0xFF
    # # if the `q` key was pressed, break from the loop
    # if key == ord("q"):
    #   # close the output CSV file do a bit of cleanup
  print("[INFO] Stopping motors and cleaning up...")
  TB.SetMotor2(0)
  TB.SetMotor1(0)
  csv.close()
    #   break
  