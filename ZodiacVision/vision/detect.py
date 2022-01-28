import cv2
import yaml
import pyzed.sl as sl
import math


class Detector:
    def __init__(self, nt):
        self.frame = 0
        self.nt = nt

    def preProcessFrame(self, frame):
        lower = self.nt.yml_data['color']['lower']
        upper = self.nt.yml_data['color']['upper']
        # Preprocess
        lower_color = (lower['H'], lower['S'], lower['V'])
        upper_color = (upper['H'], upper['S'], upper['V'])
        h, w, _ = frame.shape
        image = frame[0:int(0.7 * h), 0:w]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_color, upper_color)
        return mask
    def findTarget(self, mask, zed, point_cloud, frame):
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
            h, w, _ = frame.shape
            err, point3D = point_cloud.get_value(h / 2, w / 2)
            distance = math.sqrt(point3D[0] * point3D[0] + point3D[1] * point3D[1] + point3D[2] * point3D[2])
            if math.isnan(distance) or math.isinf(distance):
                self.nt.putValue('distance', -1)
                return -1
            self.nt.putValue('distance', round(distance))
            return -1
        if ratio > .2:
            cx = rect[0] + (rect[2] * .5)
            cy = rect[1]
            self.nt.putValue('center_x', cx)
            self.nt.putValue('center_y', cy)

            err, point3D = point_cloud.get_value(cx, cy)
            distance = math.sqrt(point3D[0] * point3D[0] + point3D[1] * point3D[1] + point3D[2] * point3D[2])
            if math.isnan(distance) or math.isinf(distance):
                distance = -1
            self.nt.putValue('distance', round(distance))
            return c
        self.nt.clearTable()
        return -1
    def postProcess(self, frame, target):
        if target is -1:
            return frame
        drawnimage = cv2.drawContours(frame, [target], -1, (0, 255, 255), 2)
        return drawnimage
