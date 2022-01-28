import cv2
import imutils
import numpy as np


cap = cv2.VideoCapture(0)

lower_color = (20, 100, 100)
upper_color = (40, 255, 255)

while 1:
    ret, frame = cap.read()

    blurred = cv2.GaussianBlur(frame, (11, 11), 0) # Blur


    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV) # HSV Filter

    mask = cv2.inRange(hsv, lower_color, upper_color) # Mask colors inbetween range
    mask = cv2.erode(mask, None, iterations=2)  # Get rid of small blobs
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)


    if len(cnts) > 0: # Should probably tune

        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if radius > 10:

            cv2.circle(frame, (int(x), int(y)), int(radius),
                       (0, 255, 255), 2)


    #cv2.imshow("Threshold", thresh)
    cv2.imshow("Original", frame)

    cv2.waitKey(1)
