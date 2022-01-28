import cv2
import numpy as np
import math
import argparse
import pyzed.sl as sl
res = np.array([0])
# the following will:
# - find a green blob
# - find a contour around it
# - find a rectangle around the contour
# - extract a portion of the image for that rectangle
# - split the rectangle into three parts
# - find the contour and minAreaRect of each part
#
# The left part usually has an angle of around -30 degrees
# The middle part should be close to 0 degrees, but often isn't
# The right part usually has an angle around -60 degrees
#
def proc_part(part):
    contours2, hier2 = cv2.findContours(part, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours2) > 0:
        cont2 = contours2[0]

        rect = cv2.minAreaRect(cont2)
        (x1, y1), (x2, y2), angle = rect
        xa = x1 + x2 / 2

        # angle = math.atan2(y1 - y2, x1-x2)
        # math.degrees(angle)
        # print(f"RECT: {rect}")
        print(angle)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        # print(box)
        return box, angle

    return False, False


def proc_img(mask, img):
    global res
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    hexes = []

    res = cv2.bitwise_and(img, img, mask=mask)

    # debug code to show some contours for frame 100
    # print(f"Got contours, count: {len(contours)}")
    # print(contours)
    # img = cv2.drawContours(img, contours, -1, (0,255,255), 2)
    cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
    top_3_contours = []
    if len(cntsSorted) > 0:
        top_3_contours.append(cntsSorted[0])
    # if len(cntsSorted) > 1:
    #     top_3_contours.append(cntsSorted[1])
    # if len(cntsSorted) > 2:
    #     top_3_contours.append(cntsSorted[2])
    for cont in top_3_contours:
        x, y, w, h = cv2.boundingRect(cont)
        # only process larger areas with at least 5 points in the contour
        if len(cont) > 4:
            quarter_w = int(w * 0.25)
            x0 = x
            y0 = y
            x1 = x + quarter_w
            y1 = y + h
            x2 = x + w - quarter_w
            x3 = x + w
            #
            # cv2.rectangle(img, (x0, y0), (x1, y1), (255, 255, 0), 2)
            # cv2.rectangle(img, (x1, y0), (x2, y1), (255, 255, 0), 2)
            # cv2.rectangle(img, (x2, y0), (x3, y1), (255, 255, 0), 2)

            part1 = mask[y0:y1, x0:x1]
            print('RECT1')
            box1, angle1 = proc_part(part1)
            # x1 + x2
            if box1 is False or angle1 > -5 or angle1 < -40:
                continue
            box1a = box1 + [x0, y0]
            # print(f"{x0:3d}/{y0:3d}  box1: {box1a}")
            # img = cv2.drawContours(img, [box1a], -1, (0, 0, 255), 2)

            part2 = mask[y0:y1, x1:x2]
            # print('RECT2')
            box2, angle2 = proc_part(part2)
            if box2 is False or -75 < angle2 < -30:
                continue
            box2a = box2 + [x1, y0]
            # print(f"box2: {box2a}")
            # img = cv2.drawContours(img, [box2a], -1, (0, 0, 255), 2)

            part3 = mask[y0:y1, x2:x3]
            print('RECT3')
            box3, angle3 = proc_part(part3)

            if box3 is False or angle3 > -30:
                continue

            box3a = box3 + [x2, y0]
            print(box1a)
            centerX = int((box3a[2][0] + box1a[2][0]) / 2)
            centerY = int((box3a[3][1] + box1a[2][1]) /2)
            # print(f"box3: {box3a}")
            # img = cv2.drawContours(img, [box3a], -1, (0, 0, 255), 2)

            hexes.append(cont)
            img = cv2.line(img, (centerX, centerY), (centerX, centerY), (255, 255, 0), 5)
            img = cv2.rectangle(img, (box1a[1][0], box1a[0][1]), (box3a[3][0],int((box3a[2][1] / 2))), (255, 255, 0), 2)
            # imgx = cv2.resize(img[y:y+h, x:x+w], None, fx=4.0, fy=4.0)
            # cv2.imshow('imgx', imgx)
        # else:
        #     print(f"ELSE: bounding rect x y w h: {x:3d} {y:3d} {w:3d} {h:3d}")
    # print(len(hexes))

    # img = cv2.drawContours(img, hexes, -1, (0, 0, 255), 1)

    return img


def show_frame(img, res, mask):
    cv2.imshow('orig', img)
    cv2.imshow('res', res)
    cv2.imshow('mask', mask)
    img2 = cv2.resize(img, None, fx=2.0, fy=2.0)
    # cv2.imwrite('testx.jpg', img2)
    ch = cv2.waitKey(1) & 0xff


def process_pic(frame):
    upper_green = np.array([100, 255, 255])
    lower_green = np.array([60, 125, 110])
    cv2.namedWindow('orig')
    cv2.namedWindow('res')
    cv2.namedWindow('mask')
    cv2.moveWindow('orig', 0, 0)
    cv2.moveWindow('res', 600, 0)
    cv2.moveWindow('mask', 300, 500)
    quit_flag = False
    ratio = -1
    img = frame

    if ratio < 0:
        ratio = 640.0 / img.shape[1]
        # print(f"ratio = {ratio}")

    img = cv2.resize(img, None, fx=ratio, fy=ratio)
    # blur to get try to remove missing parts
    img = cv2.GaussianBlur(img, (5, 5), 0)
    hsvimg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsvimg, lower_green, upper_green)

    # mask out the bottom of the image
    mask[250:, :] = 0

    img = proc_img(mask, img)

    show_frame(img, res, mask)


def main():
    zed = sl.Camera()
    point_cloud = sl.Mat()
    # Set configuration parameters
    input_type = sl.InputType()
    input_type.set_from_svo_file('/home/ianmcvann/Documents/ZED/recent.svo')
    init_params = sl.InitParameters(input_t=input_type, svo_real_time_mode=False)
    # init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.UNIT.INCH  # Use milliliter units (for depth measurements)
    init_params.camera_resolution = sl.RESOLUTION.VGA
    init_params.camera_fps = 100
    init_params.depth_maximum_distance = 400
    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(-1)
    image = sl.Mat()
    zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 15)
    runtime_parameters = sl.RuntimeParameters()
    while True:
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
            # A new image is available if grab() returns SUCCESS
            zed.retrieve_image(image, sl.VIEW.RIGHT)  # Retrieve the left image
            frame = image.get_data()
            process_pic(frame)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
