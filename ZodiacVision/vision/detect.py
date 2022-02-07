import cv2
import yaml
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
    def __init__(self, nt, vs):
        self.frame = 0
        self.nt = nt
        self.vs = vs

    def preProcessFrame(self, frame):
        lower = self.nt.yml_data['color']['lower']
        upper = self.nt.yml_data['color']['upper']
        # Preprocess
        lower_color = (lower['H'], lower['S'], lower['V'])
        upper_color = (upper['H'], upper['S'], upper['V'])
        h, w, _ = frame.shape
        # image = frame[0:int(0.7 * h), 0:w]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_color, upper_color)
        return mask
    def findTargetZED(self, mask, zed, point_cloud, frame):
        # Returns contour
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 1:
            cnts = sorted(contours, key=cv2.contourArea)
            largest = cnts[-1]
            second = cnts[-2]
            x, y, w, h = cv2.boundingRect(largest)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 255), 2)
            x1, y1, w1, h1 = cv2.boundingRect(second)
            cv2.rectangle(frame, (x1, y1), (x1 + w1, y1 + h1), (255, 0, 255), 2)
            cx1 = (int(x + (w / 2)))
            cy1 = (int(y + (h / 2)))

            cx2 = (int(x1 + (w1 / 2)))
            cy2 = (int(y1 + (h1 / 2)))

            cx_real, cy_real = midpoint(cx1, cy1, cx2, cy2)
            cx_real, cy_real = int(cx_real), int(cy_real)
            err, point3D = point_cloud.get_value(cx_real, cy_real)
            distance = math.sqrt(point3D[0] * point3D[0] + point3D[1] * point3D[1] + point3D[2] * point3D[2])
            if math.isnan(distance) or math.isinf(distance):
                self.nt.putValue('distance', -1)
                self.vs.updateSavedDistance(-1)
            self.nt.putValue('distance', round(distance))
            self.vs.updateSavedDistance(round(distance))
            self.vs.updateSavedCenter(cx_real, cy_real)
            return largest, second
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

        drawn_frame = cv2.circle(frame, (int(cx_real), int(cy_real)), radius=0, color=(255, 0, 255), thickness=5)
        return drawn_frame
