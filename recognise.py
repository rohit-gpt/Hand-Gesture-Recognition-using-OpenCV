#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  8 20:27:48 2018

@author: rohitgupta
"""

import cv2
import imutils
import numpy as np
from sklearn.metrics import pairwise
import serial
import time

arduino = serial.Serial("/dev/cu.usbmodem1411", 9600)
time.sleep(2)

bg = None

#Function to find the running average over the background
def run_avg(image, aWeight):
    global bg

    if bg is None:
        bg = image.copy().astype("float")
        return
    

    cv2.accumulateWeighted(image, bg, aWeight)
    
#Function to segment the region of hand in the image
def segment(image, threshold=25):
    global bg

    diff = cv2.absdiff(bg.astype("uint8"), image)
    

    thresholded = cv2.threshold(diff,
                                threshold,
                                255,
                                cv2.THRESH_BINARY)[1]

    (_,cnts,_) = cv2.findContours(thresholded.copy(),
                                 cv2.RETR_EXTERNAL,
                                 cv2.CHAIN_APPROX_SIMPLE)
    

    if len(cnts) == 0:
        return
    else:
        #based on contour area, get the maximum contour which is the hand
        segmented = max(cnts, key=cv2.contourArea)
        return (thresholded, segmented)

#Function to count the number of fingers in the segmented hand region
def count(thresholded, segmented):

    chull = cv2.convexHull(segmented)
    

    extreme_top = tuple(chull[chull[:,:, 1].argmin()][0])
    extreme_bottom = tuple(chull[chull[:,:, 1].argmax()][0])
    extreme_left = tuple(chull[chull[:,:, 0].argmin()][0])
    extreme_right = tuple(chull[chull[:,:, 0].argmax()][0])
    
    #find the center of the palm
    cX = int((extreme_left[0] + extreme_right[0])/2)
    cY = int((extreme_top[1] + extreme_bottom[1])/2)
  
    distance = pairwise.euclidean_distances([(cX, cY)], 
                                            Y=[extreme_left, extreme_right, extreme_top, extreme_bottom])[0]
    maximum_distance = distance[distance.argmax()]
    

    radius = int(0.8 * maximum_distance)
    
    #find the circumference of the circle
    circumference = 2 * np.pi * radius
    
    #take out the circular region of interest which has the palm and the fingers
    circular_roi = np.zeros(thresholded.shape[:2], dtype="uint8")
    

    cv2.circle(circular_roi, (cX, cY), radius, 255, 1)
    
    circular_roi = cv2.bitwise_and(thresholded, thresholded, mask=circular_roi)
    

    (_, cnts, _) = cv2.findContours(circular_roi.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    

    count = 0
    

    for c in cnts:

        (x,y,w,h) = cv2.boundingRect(c)

        if((cY + (cY * 0.25)) > (y+h)) and ((circumference * 0.25) > c.shape[0]):
            count += 1
            
    return count


#Main function
if __name__ == "__main__":

    aWeight = 0.5
    
    camera = cv2.VideoCapture(-1)
    
    top, right, bottom, left = 10, 350, 225, 590
    
    num_frames = 0
    
    calibrated = False
    
    while(True):
        #get the current frame
        (grabbed, frame) = camera.read()
        
        frame = imutils.resize(frame, width=700)
        
        frame = cv2.flip(frame, 1)
        
        #clone the frame
        clone = frame.copy()
        
        (height, width) = frame.shape[:2]
        
        roi = frame[top:bottom, right:left]
        

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7,7), 0)
        
        if num_frames < 30:
            run_avg(gray, aWeight)
            if num_frames == 1:
                print("[STATUS]Please wait, calibrating...!!!")
            elif num_frames == 29:
                print("[STATUS]Calibration Successfull!")
        else:

            hand = segment(gray)
            

            if hand is not None:

                (thresholded, segmented) = hand
                
                #draw the segmented region and display the frame
                cv2.drawContours(clone, [segmented + (right, top)], -1, (0,0,255))
                
                fingers = count(thresholded, segmented)
                
                arduino.write((str(fingers)).encode())
                time.sleep(1)
                
                print(fingers)
                
                cv2.putText(clone, str(fingers), (70, 45), cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
                
                cv2.imshow("Thresholded", thresholded)
                
        cv2.rectangle(clone, (left, top), (right, bottom), (0, 255, 0), 2)
        
        num_frames += 1
        
        cv2.imshow("Video feed", clone)
        
        keypress = cv2.waitKey(1) & 0xFF
        
        if keypress == ord("q"):
            break
        
camera.release()
cv2.distroyAllWindows()
    
    

            
    