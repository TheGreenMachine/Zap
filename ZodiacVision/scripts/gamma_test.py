import cv2
import numpy as np


def gammaCorrection(src, gamma):
    invGamma = 1 / gamma

    table = [((i / 255) ** invGamma) * 255 for i in range(256)]
    table = np.array(table, np.uint8)

    return cv2.LUT(src, table)


cap = cv2.VideoCapture('right.mp4')

cv2.namedWindow("TrackedBars")
cv2.resizeWindow("TrackedBars", 640, 240)


def on_trackbar(val):
    hue_min = cv2.getTrackbarPos("Hue Min", "TrackedBars")
    hue_max = cv2.getTrackbarPos("Hue Max", "TrackedBars")
    sat_min = cv2.getTrackbarPos("Sat Min", "TrackedBars")
    sat_max = cv2.getTrackbarPos("Sat Max", "TrackedBars")
    val_min = cv2.getTrackbarPos("Val Min", "TrackedBars")
    val_max = cv2.getTrackbarPos("Val Max", "TrackedBars")

    lower = np.array([hue_min, sat_min, val_min])
    upper = np.array([hue_max, sat_max, val_max])

    imgMASK = cv2.inRange(imgHSV, lower, upper)

    cv2.imshow("Output1", img)
    cv2.imshow("Output2", imgHSV)
    cv2.imshow("Mask", imgMASK)


cv2.createTrackbar("Hue Min", "TrackedBars", 0, 179, on_trackbar)
cv2.createTrackbar("Hue Max", "TrackedBars", 179, 179, on_trackbar)
cv2.createTrackbar("Sat Min", "TrackedBars", 0, 255, on_trackbar)
cv2.createTrackbar("Sat Max", "TrackedBars", 255, 255, on_trackbar)
cv2.createTrackbar("Val Min", "TrackedBars", 0, 255, on_trackbar)
cv2.createTrackbar("Val Max", "TrackedBars", 255, 255, on_trackbar)

img = cv2.imread('../truetest.png')
imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Show some stuff
on_trackbar(0)
cv2.waitKey()

# while True:
#     _, frame = cap.read()
#     kernel = np.ones((4, 4), np.uint8)
#     kernel1 = np.ones((5, 5), np.uint8)
#     img_erosion = cv2.erode(frame, kernel, iterations=1)
#     img_erosion = cv2.dilate(img_erosion, kernel, iterations=1)
#     cv2.imshow('erode', img_erosion)
#     frame = img_erosion
#     frame = gammaCorrection(frame, .1)
#     cv2.imshow('Frame1', frame)
#     brightness = 63
#     contrast = 100
#     img = np.int16(frame)
#     img = img * (contrast / 127 + 1) - contrast + brightness
#     img = np.clip(img, 0, 255)
#     img = np.uint8(img)
#
#     cv2.imshow('Frame', img)
#     if cv2.waitKey(1) == ord('a'):
#         cv2.imwrite('sample.jpg', frame)
#     cv2.waitKey(1)

