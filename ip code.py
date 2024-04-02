#Importing Libraries
import cv2
import numpy as np

import serial
import time

cap = cv2.VideoCapture(0) #Declaring Camera Frame

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
out = cv2.VideoWriter('22-05-2022first.avi',cv2.VideoWriter_fourcc('X','V','I','D'), 14, (frame_width,frame_height))

#Declaring Serial Communication
ser = serial.Serial('/dev/ttyUSB0', 115200)  #By USB
#ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=.1)  #By GPIO
#ser.flush()

#Taking Colour Input
print('Violet = , Indigo =  , Blue = 3, Green = 4  , Yellow = 5  , Red = 7, Orange=8') 
m = int(input('Enter Value: ')) 

#Main loop. Image processing for each frame captured
while(True):
    
    #Reading each frame
    ret, frame = cap.read()
    cv2.imshow('Video', frame)
    out.write(frame)
    
    #Declaring HSV limits for different colours

    if m == 1:      #Violet
        low_H = 132
        low_S = 49
        low_V = 120
        high_H = 168
        high_S = 240
        high_V = 255

    if m == 2:      #Indigo
        low_H = 100
        low_S = 32
        low_V = 135
        high_H = 145
        high_S = 255
        high_V = 255

    if m == 3:      #Blue
        low_H = 85
        low_S = 46
        low_V = 100
        high_H = 131
        high_S = 255
        high_V = 255

    if m == 4:      #GreenFINAL
        low_H = 46
        low_S = 130
        low_V = 214
        high_H = 66
        high_S = 208
        high_V = 248

    if m==5:      #YellowFINAL
        low_H = 34
        low_S = 139
        low_V = 178
        high_H = 37
        high_S = 247
        high_V = 253  

    if m == 7:      #Red
        low_H = 168
        low_S = 75
        low_V = 80
        high_H = 179
        high_S = 255
        high_V = 255
        
    if m==8:     #ORANGEFINAL
        low_H = 0
        low_S = 103
        low_V = 43
        high_H = 13
        high_S = 200
        high_V = 190

    #Converting RGB frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)     

    #Declaring arrays of Lower and Higher HSV value
    lower_hsv = np.array([low_H, low_S, low_V])
    higher_hsv = np.array([high_H, high_S, high_V])

    #Defining mask of the selected colour
    mask = cv2.inRange(hsv, lower_hsv, higher_hsv)

    #Bitwise_and of the mask and og frame to get only the desired colour
    frame = cv2.bitwise_and(frame, frame, mask=mask)

    #Finding the regions of the mask on screen; Finding contours
    #contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    img, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

    #Defining Region of Interest(ROI) {Red Bounding Box}
    #Smaller rectangle-> (195,167) X1,Y1 ; (430,300) X2,Y2
    #Larger rectangle-> (130,113) X1,Y1 ; (505,367) X2Y2
       
    cv2.rectangle(frame,(34,35),(597,318),(0,0,255),2)

    #Run the loop only if you find the countour in the frame
    if len(contours) != 0:

        # find the biggest countour (c) by the area
        c = max(contours, key = cv2.contourArea)
        area = cv2.contourArea(c)
        if (area>200):
            x,y,w,h = cv2.boundingRect(c)  
        
            x2=x+w #Diagonal ends of the contour
            y2=y+h

            #Defining center of the contour
            x_c=(x+x2)/2
            y_c=(y+y2)/2

            #Display the biggest contour in green
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)

            #Checking if the center of the contour is in the ROC
            if (x_c>34 and y_c>35 and x_c<597 and y_c<318):
                print('Object is inside ROC')
                #cv2.rectangle(frame,(x_c,y_c),((x_c+1),(y_c+1)),(0,255,0),2 )
                ser.write(b"1\n")
            else:
                print('no')
                ser.write(b"0\n")