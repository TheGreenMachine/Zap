import cv2
import yaml
import math


class Detector:
    def __init__(self, nt, camera):
        self.frame = 0
        self.nt = nt
        self.camera = camera


    def preProcessFrame(self, frame):
        lower = self.nt.yml_data['color']['lower']
        upper = self.nt.yml_data['color']['upper']
        # Preprocess
        lower_color = (lower['H'], lower['S'], lower['V'])
        upper_color = (upper['H'], upper['S'], upper['V'])
        h, w, _ = frame.shape
        image = frame[0:int(h / 2), 0:w]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_color, upper_color)
        return mask
    def findTarget(self, mask):
        # Returns contour
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) != 0:
            c = max(contours, key=cv2.contourArea)
            perimeter = cv2.arcLength(c, True)
            rect = cv2.boundingRect(c)
            area = cv2.contourArea(c)
            if area != 0:
                ratio = perimeter / area
            else:
                ratio = 0
        else:
            self.nt.clearTable()
            return -1, -2, -2
        if ratio > .2:
            cx = rect[0] + (rect[2] * .5)
            cy = rect[1]
            self.nt.putValue('center_x', cx)
            self.nt.putValue('center_y', cy)
            return c,cx,cy

        self.nt.clearTable()
        return -1, -2, -2
    def postProcess(self, frame, target):
        if target is -1:
            return frame
        drawnimage = cv2.drawContours(frame, [target], -1, (0, 255, 255), 2)
        return drawnimage

