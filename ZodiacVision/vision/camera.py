from . import hwcheck
import yaml
import cv2
import math

class Camera:
    def __init__(self):
        self.hw = hwcheck.HWCheck()
        self.isZed = self.hw.CheckZed()
        path = 'vision.yml'
        with open(path, 'r') as file:
            data = yaml.safe_load(file)
        if self.isZed:
            import pyzed.sl as sl
            # Set configuration parameters
            self.zed = sl.Camera()
            self.point_cloud = sl.Mat()
            # Set configuration parameters
            self.init_params = sl.InitParameters()
            self.init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use PERFORMANCE depth mode
            self.init_params.coordinate_units = sl.UNIT.INCH  # Use milliliter units (for depth measurements)
            self.init_params.camera_resolution = sl.RESOLUTION.VGA
            self.init_params.camera_fps = 100
            self.init_params.depth_maximum_distance = 400
            # Open the camera
            err = self.zed.open(self.init_params)
            if err != sl.ERROR_CODE.SUCCESS:
                exit(-1)
            self.image = sl.Mat()
            self.zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, data['camera']['exposure'])
            runtime_parameters = sl.RuntimeParameters()
        else:
            self.cap = cv2.VideoCapture(0)
    def read(self):
        if self.isZed:
            import pyzed.sl as sl
            if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
                # A new image is available if grab() returns SUCCESS
                self.zed.retrieve_image(self.image, sl.VIEW.RIGHT)  # Retrieve the left image
                frame = self.image.get_data()
                return frame
        else:
            _, frame = self.cap.read()
            return frame

    def updateExposure(self, net):
        if self.isZed:
            import pyzed.sl as sl
            self.zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, net.yml_data['camera']['exposure'])
    def findTargetDistance(self, cx, cy):
        if self.isZed:
            err, point3D = self.point_cloud.get_value(cx, cy)
            distance = math.sqrt(point3D[0] * point3D[0] + point3D[1] * point3D[1] + point3D[2] * point3D[2])
            if math.isnan(distance) or math.isinf(distance):
                return -1
            return round(distance)
        else:
            return -1
