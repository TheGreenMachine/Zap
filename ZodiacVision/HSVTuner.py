import cv2
# import pyzed.sl as sl
import math
import numpy as np


# Team 1816's 2020 Vision Processing Code
# Used with OpenCV and Gstreamer
def make_half_hex_shape():
    pts = []
    for ang in [0, 60, 120, 180]:
        ang_r = math.radians(ang)
        x1 = int(100.0 * math.cos(ang_r) + 100.5)
        y1 = int(100.0 * math.sin(ang_r) + 100.5)
        pts.append([x1, y1])

    for ang in [180, 120, 60, 0]:
        ang_r = math.radians(ang)
        x1 = int(92.0 * math.cos(ang_r) + 100.5)
        y1 = int(92.0 * math.sin(ang_r) + 100.5)
        pts.append([x1, y1])
    shape_np = np.array(pts, np.int32)
    shape_np = np.reshape(shape_np, (-1, 1, 2))
    return shape_np


def preProcess(image):
    # lower_color = (60, 106, 150)
    # upper_color = (130, 255, 255)
    lower_color = (edge_params['min_h'], edge_params['min_s'], edge_params['min_v'])
    upper_color = (edge_params['max_h'], edge_params['max_s'], edge_params['max_v'])
    h, w, _ = image.shape
    # image = image[0:int(h / 2), 0:w]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)

    return mask


edge_params = {
    'min_h': 59,
    'min_s': 55,
    'min_v': 120,
    'max_h': 111,
    'max_s': 255,
    'max_v': 255,
}


def min_vchange(new_val):
    change_params('min_v', new_val)


def max_vchange(new_val):
    change_params('max_v', new_val)


def min_schange(new_val):
    change_params('min_s', new_val)


def max_schange(new_val):
    change_params('max_s', new_val)


def min_hchange(new_val):
    change_params('min_h', new_val)


def max_hchange(new_val):
    change_params('max_h', new_val)


def change_params(name, value):
    global edge_params
    edge_params[name] = value
    print(edge_params)


def findTarget(image, shape):
    contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    hexes = []
    largest = None
    firstLoop = True
    for cont in contours:
        if firstLoop:
            largest = cont
            firstLoop = False
            continue
        if cont.size > largest.size:
            largest = cont

    if largest is not None:
        rect = cv2.boundingRect(largest)
    # only process larger areas with at least 5 points in the contour
    if True or (len(largest) > 4 and rect[2] > 40 and rect[3] > 40):
        match = cv2.matchShapes(largest, shape, cv2.CONTOURS_MATCH_I2, 0.0)
        print(match)
        if match < 10:
            return largest
    return -1


def postProcess(image, target):
    if target is -1:
        return image
    drawnimage = cv2.drawContours(image, [target], -1, (0, 255, 255), 2)
    return drawnimage


if __name__ == '__main__':
    cap = cv2.VideoCapture("video2_Trim.mp4")
    cv2.namedWindow("PreProcessed Image")
    cv2.createTrackbar('Hmin', 'PreProcessed Image', edge_params['min_h'], 255, min_hchange)
    cv2.createTrackbar('Smin', 'PreProcessed Image', edge_params['min_s'], 255, min_schange)
    cv2.createTrackbar('Vmin', 'PreProcessed Image', edge_params['min_v'], 255, min_vchange)
    cv2.createTrackbar('Hmax', 'PreProcessed Image', edge_params['max_h'], 255, max_hchange)
    cv2.createTrackbar('Smax', 'PreProcessed Image', edge_params['max_s'], 255, max_schange)
    cv2.createTrackbar('Vmax', 'PreProcessed Image', edge_params['max_v'], 255, max_vchange)

    while 1:
        _, frame = cap.read()
        preProcessImage = preProcess(frame)
        stream_image = cv2.bitwise_and(frame, frame, mask=preProcessImage)
        cv2.line(stream_image, (0, 272), (672, 272), (0, 255, 255), 3)
        cv2.imshow("PreProcessed Image", stream_image)
        #cv2.imshow("PostProcessed Image", postProcessImage)
        cv2.imshow("Raw Image", frame)
        if cv2.waitKey(30) == ord('p'):
            while 1:
                preProcessImage = preProcess(frame)
                stream_image = cv2.bitwise_and(frame, frame, mask=preProcessImage)
                cv2.line(stream_image, (0, 272), (672, 272), (0, 255, 255), 3)
                cv2.imshow("PreProcessed Image", stream_image)
                if cv2.waitKey(1) == ord('r'):
                    break
