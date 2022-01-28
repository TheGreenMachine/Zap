import cv2
import yaml
import pyzed.sl as sl
from vision import *
import time
import numpy as np
time.sleep(1)
path = 'vision.yml'
with open(path, 'r') as file:
    data = yaml.safe_load(file)
print(data)
net = networktables.NetworkTables(data, path)
gst_str = "appsrc ! shmsink socket-path=/tmp/foo1 sync=true wait-for-connection=false shm-size=10000000"
out = cv2.VideoWriter(gst_str, 0, 100, (672,376), True)
net.setupCalib()
# Set configuration parameters
zed = sl.Camera()
point_cloud = sl.Mat()
# Set configuration parameters
init_params = sl.InitParameters()
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
#zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, data['camera']['exposure'])
runtime_parameters = sl.RuntimeParameters()
detector = detect.Detector(net)
width = int(net.yml_data['stream']['line'])
fpsCounter = fps.FPS()
while True:
    fpsCounter.reset()
    fpsCounter.start()
    if net.update_exposure:
        #zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, net.yml_data['camera']['exposure'])
        net.update_exposure = False
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
        # A new image is available if grab() returns SUCCESS
        zed.retrieve_image(image, sl.VIEW.RIGHT)  # Retrieve the left image
        frame = image.get_data()
        if net.vision_use:
            cv2.imwrite('/home/jetson/ZodiacVision/capture/' + time.strftime("%Y%m%d-%H%M%S") + '.png', frame)
            net.vision_use = False
        mask = detector.preProcessFrame(frame)
        if mask.all() == -1:
            continue
        contour = detector.findTarget(mask, zed, point_cloud, frame)
        stream_image = detector.postProcess(frame, contour)
        fpsCounter.update()
        fpsCounter.stop()
        stream_image = fps.putIterationsPerSec(stream_image, fpsCounter.fps())

        if net.line:
            width = int(net.yml_data['stream']['line'])
            net.line = False
        stream_image = cv2.line(stream_image, (width, 0), (width, int(stream_image.shape[0])), (0, 255, 0), 3)
        if net.calib_camera:
            #edit this
            out.write(mask)
        else:
            out.write(stream_image[:,:,:3])
