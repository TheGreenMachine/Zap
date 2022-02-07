import cv2
import heapq

import cv2
def midpoint(x1, y1, x2, y2):
    return (x1 + x2)/2, (y1 + y2)/2


def zoom(img, zoom_factor=2):
    return cv2.resize(img, None, fx=zoom_factor, fy=zoom_factor)

if __name__ == '__main__':
    # writer = cv2.VideoWriter('target.mp4',
    #                          cv2.VideoWriter_fourcc(*'MP4V'),
    #                          100, (1344, 752))
    cap = cv2.VideoCapture('scripts/right.mp4')
    while 1:
        _, frame = cap.read()
        # frame = zoom(frame, 2)
        lower_color = (60,80, 150)
        upper_color = (180, 255, 255)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_color, upper_color)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 1:
            cnts = sorted(contours, key=cv2.contourArea)
            largest = cnts[-1]
            second = cnts[-2]
            x,y,w,h = cv2.boundingRect(largest)
            area = cv2.contourArea(largest)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 255), 2)
            x1,y1,w1,h1 = cv2.boundingRect(second)
            area = cv2.contourArea(second)
            cv2.rectangle(frame, (x1, y1), (x1 + w1, y1 + h1), (255, 0, 255), 2)
            cx1 = (int(x+(w/2)))
            cy1 = (int(y+(h/2)))

            cx2 = (int(x1+(w1/2)))
            cy2 = (int(y1+(h1/2)))

            cx_real, cy_real = midpoint(cx1, cy1, cx2, cy2)



            frame = cv2.circle(frame, (int(cx_real),  int(cy_real)), radius=0, color=(255, 0, 255), thickness=5)
            # cv2.drawContours(frame, , -1, (255, 0, 255), 3)
            print(area)
            cx = x + (w * .5)
            cy = y
            # cv2.line(frame, (int(cx), int(cx)), (int(cy), int(cy)), (0, 255, 0), 3)
            print(cx, cy)
        # writer.write(frame)
        cv2.imshow('Frame', frame)
        cv2.waitKey(1)
