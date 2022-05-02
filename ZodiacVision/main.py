import threading

import yaml
import cv2
import math
path = 'vision.yml'
with open(path, 'r') as file:
    data = yaml.safe_load(file)
print(data)
from vision import *
import time
import numpy as np

isZed = False
isGstreamer = False
isCalib = False
if data['zed']:
    import pyzed.sl as sl
    isZed = True
if data['gstreamer']:
    isGstreamer = True
if data['camera']['debug'] == "001":
    isCalib = True

#net = networktables.NetworkTables(data, path)
if isGstreamer:
    gst_str = "appsrc ! shmsink socket-path=/tmp/foo1 sync=true wait-for-connection=false shm-size=10000000"
    out = cv2.VideoWriter(gst_str, 0, 60, (1280,720), True)
else:
    streamer = stream.Streamer(data['stream']['port'])
#net.setupCalib()

vs = visionserver.ThreadedVisionServer('', 5802, path, data)
server_thread = threading.Thread(target=vs.listen)
# Exit the server thread when the main thread terminates
server_thread.daemon = True
server_thread.start()

cap = 0
if isZed:
    # Set configuration parameters
    zed = sl.Camera()
    point_cloud = sl.Mat()
    # Set configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.UNIT.INCH  # Use milliliter units (for depth measurements)
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 60
    #init_params.depth_maximum_distance = 400
    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(-1)
    image = sl.Mat()
    runtime_parameters = sl.RuntimeParameters()
    zed.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO, -1)
    zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, data['camera']['exposure'])
else:
    cap = cv2.VideoCapture(0)

detector = detect.Detector(vs)
width = 640
fpsCounter = fps.FPS()
lastDist = -1
while True:
    fpsCounter.reset()
    fpsCounter.start()
    if vs.update_exposure:
        zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, vs.yml_data['camera']['exposure'])
        #with open('/home/jetson/ZodiacVision/log.txt', 'a') as f:
        #    f.write(str(vs.yml_data['camera']['exposure']) + '\n')
        vs.update_exposure = False
    if vs.update_debug:
        if vs.yml_data['camera']['debug'] == "001":
            isCalib = True
        else:
            isCalib = False
        vs.update_debug = False
    if isZed:
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
            # A new image is available if grab() returns SUCCESS
            zed.retrieve_image(image, sl.VIEW.LEFT)  # Retrieve the left image
            frame = image.get_data()
            mask = detector.preProcessFrame(frame)
            if mask.all() == -1:
                continue
            largest, second_largest = detector.findTargetZED(mask, zed, point_cloud, frame)
            stream_image = detector.postProcess(frame, largest, second_largest)
            fpsCounter.update()
            fpsCounter.stop()
            #stream_image = fps.putIterationsPerSec(stream_image, fpsCounter.fps())
            #stream_image = cv2.bitwise_and(stream_image, stream_image, mask=mask)
            if vs.distance >= 0:
                lastDist = vs.distance
            stream_image = cv2.putText(stream_image, f"{lastDist} in",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0))
            offset = abs(640 -  int(vs.cx))
            if 640 < int(vs.cx):
                offset = -1 * offset
            stream_image = cv2.putText(stream_image, f"{offset} x",
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0))
            stream_image = cv2.line(stream_image, (int(vs.cx), int(vs.cy) + 30), (int(vs.cx), int(vs.cy) - 30), (255, 0, 255), 3)
            #stream_image = cv2.line(stream_image, (width, 0), (width, int(stream_image.shape[0])), (0, 255, 0), 3)
            if isGstreamer:
                if isZed:
                    if isCalib:
                        out.write(cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR))
                    else:
                        out.write(stream_image[:, :, :3])
                else:
                    out.write(stream_image)
            else:
                streamer.write(stream_image)
    else:
        _, frame = cap.read()
        if frame is None:
            continue
        mask = detector.preProcessFrame(frame)
        if mask.all() == -1:
            continue
        largest, second_largest = detector.findTarget(mask)
        stream_image = detector.postProcess(frame, largest, second_largest)
        fpsCounter.update()
        fpsCounter.stop()
        stream_image = fps.putIterationsPerSec(stream_image, fpsCounter.fps())

        stream_image = cv2.line(stream_image, (width, 0), (width, int(stream_image.shape[0])), (0, 255, 0), 3)
        if isGstreamer:
            if isZed:
                out.write(stream_image[:,:,:3])
            else:
                out.write(stream_image)
        else:
            streamer.write(stream_image)
