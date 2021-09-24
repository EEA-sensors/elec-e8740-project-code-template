import cv2
import numpy as np
import time
import calendar
import csv
from time import sleep
import datetime

#current date for file name
dt = str(datetime.datetime.now())

#video capture object of openCV
cap = cv2.VideoCapture(0)

#font
font = cv2.FONT_HERSHEY_COMPLEX

#each row of the recorded
timestamped_camera = np.zeros(4) #time_stamp, id_object, distance, angle

low_H = 0   #black
low_S = 0
low_V = 70
high_H = 255
high_S = 255
high_V = 255

lower_red = np.array([low_H, low_S, low_V])
upper_red = np.array([high_H, high_S, high_V])

kernel = np.ones((5, 5), np.uint8)

while True:
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lower_red, upper_red)
    mask = cv2.erode(mask, kernel)

    # Contours detection
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
