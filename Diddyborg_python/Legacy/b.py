import cv2
import numpy as np
import time
import calendar
import csv
from time import sleep
import datetime



#date_time = now.strftime("%m/%d/%Y, %H:%M:%S")

dt = str(datetime.datetime.now())

cap = cv2.VideoCapture(0)

font = cv2.FONT_HERSHEY_COMPLEX





while True:
    
    
    
    #assigning distance (camera to shape ) variables to each shape
    Distance_triangle=0
    Distance_square=0
    Distance_pentagone=0
    Distance_hexagone=0
    Distance_7agone=0
    
    timestamped_camera_readings = np.ndarray((0,), np.float32)
    timestamp = calendar.timegm(time.gmtime())
    timestamp_ms= time.time()
    
    
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #low_H = 0     #orage color filter values
    #low_S = 0
    #low_V = 0
    #high_H = 180
    #high_S = 136
    #high_V = 255
    
    #low_H = 95    # blue color filter
    #low_S = 0
    #low_V = 0
    #high_H = 126
    #high_S = 255
    #high_V = 255
    
    
    #low_H = 123     #red
    #low_S = 84
    #low_V = 0
    #high_H = 180
    #high_S = 255
    #high_V = 145
    
    low_H = 0   #black
    low_S = 0
    low_V = 70
    high_H = 255
    high_S = 255
    high_V = 255
    

    lower_red = np.array([low_H, low_S, low_V])
    upper_red = np.array([high_H, high_S, high_V])

    mask = cv2.inRange(hsv, lower_red, upper_red)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel)
    

    # Contours detection
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    
    
    for cnt in contours[1:]:
        area = cv2.contourArea(cnt)
        approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
        x = approx.ravel()[0]
        y = approx.ravel()[1]
        perimeter = cv2.arcLength(cnt,True)
        g = float("{0:.2f}".format(perimeter))
        

        if area > 200:
         # we consider contour if it has area above 200 pixel*pixel to eliminate small detections
            cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)
            #drawing predicted contour in frame 

            if len(approx) == 3:
            #when approx contour's length is 3 we decide it's triangle
                cv2.putText(frame, "triangle " , (x, y), font, 1, (0, 0, 0))
                # displaying name 
                Distance_triangle=(((223*58)/(perimeter/3))*0.2645)
                print("At",timestamp_ms,"timestamp" , "Triangle found in :",Distance_triangle,"cm")
                
                
            elif len(approx) == 4:
                cv2.putText(frame, "square", (x, y), font, 1, (0, 0, 0))                
                Distance_square=(((223*58)/(perimeter/4))*0.2645)
                print("At",timestamp_ms,"timestamp" , "Square found in :" ,Distance_square,"cm")
                
            elif len(approx) == 5:
                cv2.putText(frame, "Pentogon", (x, y), font, 1, (0, 0, 0))                           
                Distance_pentagone=(((223*58)/(perimeter/5))*0.2645)
                print("At",timestamp_ms,"timestamp" , "Pentagone found in :" ,Distance_pentagone,"cm")
                
                
            elif len(approx) == 6:
                cv2.putText(frame, "6angle", (x, y), font, 1, (0, 0, 0))             
                Distance_hexagone=(((223*58)/(perimeter/6))*0.2645)               
                print("At",timestamp_ms,"timestamp" , "Hexogone found in :" ,Distance_hexagone,"cm")
                
            elif len(approx) == 7:
                cv2.putText(frame, "7angle", (x, y), font, 1, (0, 0, 0))             
                Distance_7agone=(((223*58)/(perimeter/7))*0.2645)               
                print("At",timestamp_ms,"timestamp" , "7ogone found in :" ,Distance_7agone,"cm")
                
                
            elif len(approx) > 6 :
                cv2.putText(frame, "Random Contour", (x, y), font, 1, (0, 0, 0))
                pass
                
            

    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)
    
    
    #write_fmtt = " ".join("%4.2f" for _ in timestamped_camera_readings)
    # append time
    
    #write_fmtt = " ".join("%4.8f" for _ in timestamped_camera_readings)
    timestamped_camera_readings = np.append(float(timestamp_ms),[Distance_triangle , Distance_square , Distance_pentagone , Distance_hexagone,Distance_7agone])
    #logging format of the results
    


    
    #with open("sensor reading {}.txt".format(datetime.datetime.now())) as ff:
              #np.savetxt(ff, np.expand_dims(timestamped_camera_readings, axis=0),fmt='%f')
        
        
        
        

    key = cv2.waitKey(1)
    if key == 0:
        break
    
    with open("Readings_Camera {}.csv".format(dt), "ab") as ff:
        np.savetxt(ff, np.expand_dims(timestamped_camera_readings, axis=0),fmt="%4.8f",delimiter=',')

cap.release()
cv2.destroyAllWindows()
















































































































































































































































































































































































































