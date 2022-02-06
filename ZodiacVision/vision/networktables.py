import yaml
import networktables as nt


class NetworkTables:
    def __init__(self, yml_data, yml_path, ip=None):
        self.yml_data = yml_data
        self.update_exposure = False
        self.yml_path = yml_path
        self.calib_camera = False
        self.vision_use = False
        self.line = False
        if ip is None:
            self.ip = self.yml_data['network']['nt-ip']
        print(self.ip)
        nt.NetworkTables.initialize(server=self.ip)
        self.table = nt.NetworkTables.getTable('SmartDashboard')
        camera_table = nt.NetworkTables.getTable("CameraPublisher")
        table = camera_table.getSubTable("Camera")
        table.getEntry("streams").setStringArray([f"mjpg:http://{self.yml_data['main']['device-ip']}:1180/video_feed"])

    def clearTable(self):
        for item in self.yml_data['network']['variables']:
            self.table.putNumber(item, -1)

    def setupCalib(self):
        def dumpYML():
            with open(self.yml_path, "w") as f:
                yaml.dump(self.yml_data, f)

        def valueChanged(table, key, value, isNew):
            if key == "VISION":
                self.vision_use = value
            if key == "HMIN":
                self.yml_data['color']['lower']['H'] = value
                dumpYML()
            elif key == "SMIN":
                self.yml_data['color']['lower']['S'] = value
                dumpYML()
            elif key == "VMIN":
                self.yml_data['color']['lower']['V'] = value
                dumpYML()
            elif key == "HMAX":
                self.yml_data['color']['upper']['H'] = value
                dumpYML()
            elif key == "SMAX":
                self.yml_data['color']['upper']['S'] = value
                dumpYML()
            elif key == "VMAX":
                self.yml_data['color']['upper']['V'] = value
                dumpYML()
            elif key == "EXPOSURE":
                self.yml_data['camera']['exposure'] = value
                dumpYML()
                self.update_exposure = True
            elif key == "LINE":
                self.yml_data['stream']['line'] = value
                dumpYML()
                self.line = True
            elif key == "CalibrationCamera":
                self.calib_camera = value

        lower = self.yml_data['color']['lower']
        upper = self.yml_data['color']['upper']
        calibTable = nt.NetworkTables.getTable('SmartDashboard/Calibration')
        calibTable.putNumber("HMIN", lower['H'])
        calibTable.putNumber("HMAX", upper['H'])
        calibTable.putNumber("SMIN", lower['S'])
        calibTable.putNumber("SMAX", upper['S'])
        calibTable.putNumber("VMIN", lower['V'])
        calibTable.putNumber("VMAX", upper['V'])
        calibTable.putNumber("EXPOSURE", self.yml_data['camera']['exposure'])
        calibTable.putNumber("LINE", self.yml_data['stream']['line'])
        calibTable.putBoolean("CalibrationCamera", False)
        calibTable.addEntryListener(valueChanged)

    def putValue(self, key, value):
        if isinstance(value, str):
            self.table.putString(key, value)
            return True
        if isinstance(value, float) or isinstance(value, int):
            self.table.putNumber(key, value)
            return True
        if isinstance(value, bool):
            self.table.putBoolean(key, value)
            return True
        return False
