import cv2
import yaml
import sys
import numpy as np
path = 'vision.yml'
with open(path, 'r') as file:
    data = yaml.safe_load(file)
print(data)
if data['zed']:
    import pyzed.sl as sl
import math
def midpoint(x1, y1, x2, y2):
    return (x1 + x2)/2, (y1 + y2)/2

class Detector:
    def __init__(self, vs):
        self.frame = 0
        self.vs = vs

    def preProcessFrame(self, frame):
        lower = self.vs.yml_data['color']['lower']
        upper = self.vs.yml_data['color']['upper']
        # Preprocess
        lower_color = (lower['H'], lower['S'], lower['V'])
        upper_color = (upper['H'], upper['S'], upper['V'])
        h, w, _ = frame.shape
        frame = frame[0:h, 0:w]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_color, upper_color)
        return mask
    def findTargetZED(self, mask, zed, point_cloud, frame):
        # Returns contour
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 2:
            cnts = sorted(contours, key=cv2.contourArea)
            largest = cnts[-1]
            second = cnts[-2]
            third = cnts[-3]
            x, y, w, h = cv2.boundingRect(largest)
            #cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 255), 2)
            x1, y1, w1, h1 = cv2.boundingRect(second)
            #cv2.rectangle(frame, (x1, y1), (x1 + w1, y1 + h1), (255, 0, 255), 2)
            x2, y2, w2, h2 = cv2.boundingRect(third)
            cv2.rectangle(frame, (x2, y2), (x2 + w2, y2 + h2), (255, 0, 255), 2)
            if w < 10 or w1 < 10:
                self.vs.updateSavedDistance(-1)
                self.vs.updateSavedCenter(-1, -1)
                return -1, -1
            cx1 = (int(x + (w / 2)))
            cy1 = (int(y + (h / 2)))

            cx2 = (int(x1 + (w1 / 2)))
            cy2 = (int(y1 + (h1 / 2)))

            cx3 = (int(x2 + (w2 / 2)))
            cy3 = (int(y2 + (h2 / 2)))

            print("Test Vision:" + str(abs(cx1-cx2)), file=sys.stdout)
            if abs(cx1-cx2) > 150:
                self.vs.updateSavedDistance(-1)
                self.vs.updateSavedCenter(-1, -1)
                return -1, -1
            cx_real, cy_real = (cx1 + cx2 + cx3) /3, (cy1+cy2+cy3)/3
            cx_real, cy_real = int(cx_real), int(cy_real)
            err, point3D = point_cloud.get_value(cx1, cy1)
            distance = math.sqrt(point3D[0] * point3D[0] + point3D[1] * point3D[1] + point3D[2] * point3D[2])
           # err, point3D = point_cloud.get_value(cx2, cy2)
           # distance2 = math.sqrt(point3D[0] * point3D[0] + point3D[1] * point3D[1] + point3D[2] * point3D[2])
           # distance = (distance1 + distance2) / 2
            if math.isnan(distance) or math.isinf(distance):
                self.vs.updateSavedDistance(-1)
                self.vs.updateSavedCenter(cx_real, cy_real)
                return largest, second
            #with open('/home/jetson/ZodiacVision/distanceout.txt', 'a') as f:
            #    f.write(str(round(distance)) + '\n')
            self.vs.updateSavedDistance(round(distance))
            self.vs.updateSavedCenter(cx_real, cy_real)
            return largest, second
        self.vs.updateSavedDistance(-1)
        self.vs.updateSavedCenter(-1, -1)
        return -1, -1
    def findTarget(self, mask):
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 1:
            cnts = sorted(contours, key=cv2.contourArea)
            largest = cnts[-1]
            second = cnts[-2]
            return largest, second
        return -1, -1
    def postProcess(self, frame, largest, second_largest):
        drawn_frame = cv2.line(frame, (int(1280/2), 0),  (int(1280/2), 720), color=(255, 255, 0), thickness=1)
        if second_largest is -1 or largest is -1:
            return frame
        x, y, w, h = cv2.boundingRect(largest)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 255), 2)
        x1, y1, w1, h1 = cv2.boundingRect(second_largest)
        cv2.rectangle(frame, (x1, y1), (x1 + w1, y1 + h1), (255, 0, 255), 2)
        cx1 = (int(x + (w / 2)))
        cy1 = (int(y + (h / 2)))
        
        cx2 = (int(x1 + (w1 / 2)))
        cy2 = (int(y1 + (h1 / 2)))

        cx_real, cy_real = midpoint(cx1, cy1, cx2, cy2)

        drawn_frame = cv2.circle(drawn_frame, (int(cx_real), int(cy_real)), radius=0, color=(255, 0, 255), thickness=5)
        return drawn_frame
