#!/usr/bin/python3

# Object detection code which tracks the blue playground ball
# provided in the lab.
import numpy as np
import cv2

cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    # Our operations on the frame come here
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_hsv = np.array([48, 69, 71])
    higher_hsv = np.array([145, 212, 214])
    # Apply the cv2.inrange method to create a mask
    mask = cv2.inRange(hsv, lower_hsv, higher_hsv)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    element = cv2.getStructuringElement(cv2.MORPH_RECT,(7,7))
    
    mask = cv2.GaussianBlur(mask,(5,5),0)

    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 200, param1=50, param2=20,
minRadius=20, maxRadius=800)
    # circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 200, param1=50, param2=20,minRadius=20, maxRadius=800)
    if not (circles is None):
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            # draw bounding circle
            cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
            # draw center
            cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)
            print("Center coordinates: x = ", i[0], ", y = ", i[1])


    # Display the resulting frame
    cv2.imshow('Object Detection',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()