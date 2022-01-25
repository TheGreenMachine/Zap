import cv2
import yaml

from vision import networktables, camera, detect, stream, fps, socketserver
import time
path = 'vision.yml'
with open(path, 'r') as file:
    data = yaml.safe_load(file)
# print(data)
net = networktables.NetworkTables(data, path)
net.setupCalib()

visionCamera = camera.Camera()
socket = socketserver.SocketServer(data, path, 1180)

detector = detect.Detector(net, visionCamera)
#streamer = stream.Streamer(data['stream']['port'])
width = int(net.yml_data['stream']['line'])
fpsCounter = fps.FPS()

socket.startServer()





while True:
    fpsCounter.reset()
    fpsCounter.start()
    if net.update_exposure:
        visionCamera.updateExposure(net)
        net.update_exposure = False
    # A new image is available if grab() returns SUCCESS
    frame = visionCamera.read()
    if net.vision_use:
        cv2.imwrite('/home/jetson/ZodiacVision/capture/' + time.strftime("%Y%m%d-%H%M%S") + '.png', frame)
        net.vision_use = False
    mask = detector.preProcessFrame(frame)
    if mask.all() == -1:
        continue
    contour, center_x, center_y = detector.findTarget(mask)

    socket.listenMessage(1816 ,1, 1)

    stream_image = detector.postProcess(frame, contour)
    fpsCounter.update()
    fpsCounter.stop()
    # stream_image = fps.putIterationsPerSec(stream_image, fpsCounter.fps())
    #
    # if net.line:
    #     width = int(net.yml_data['stream']['line'])
    #     net.line = False
    # stream_image = cv2.line(stream_image, (width, 0), (width, int(stream_image.shape[0])), (0, 255, 0), 3)
    # if net.calib_camera:
    #     streamer.write(mask)
    # else:
    #     streamer.write(stream_image)
