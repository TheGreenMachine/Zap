import socket
import threading

import yaml


class ThreadedVisionServer(object):
    def __init__(self, host, port, yml_path, yaml_data):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.cx = bytes('-1', 'utf-8')
        self.cy = bytes('-1', 'utf-8')
        self.distance = bytes('-1', 'utf-8')
        self.yml_data = yaml_data
        self.yml_path = yml_path
        self.update_exposure = False
        self.calib_camera = False

    def listen(self):
        self.sock.listen(5)
        while True:
            client, address = self.sock.accept()
            client.settimeout(300)
            threading.Thread(target=self.listenToClient, args=(client, address)).start()

    def listenToClient(self, client, address):
        size = 14
        while True:
            try:
                data = client.recv(size)
                if data:
                    print(data)
                    # Set the response to echo back the recieved data
                    response = data
                    self.dispatchResponse(client, response)
            except Exception as e:
                print(e)
                client.close()
                return False

    def dispatchResponse(self, client, msg):
        if msg == b'distance':
            client.send(b"distance|" + self.distance)
        elif msg == b'center_x':
            client.send(b"center_x|" + self.cx)
        elif msg == b'center_y':
            client.send(b"center_y|" + self.cy)
        elif msg == b'point':
            client.send(self.cx+b'|'+self.cy+b'|' + self.distance)
        elif b'calib' in msg:
            print(str(msg))
            msg = str(msg).replace("'", "").split('|')
            self.calibChange(msg[1], msg[2])

    def updateSavedCenter(self, cx, cy):
        self.cx = bytes(str(cx), 'utf-8')
        self.cy = bytes(str(cy), 'utf-8')

    def updateSavedDistance(self, d):
        self.distance = bytes(str(d), 'utf-8')
    def calibChange(self, key, value):
        print(value)
        def dumpYML():
            with open(self.yml_path, "w") as f:
                yaml.dump(self.yml_data, f)
        # if key == "VISION":
        #     self.vision_use = value
        if key == "HMIN":
            value = float(value)
            self.yml_data['color']['lower']['H'] = value
            dumpYML()
        elif key == "SMIN":
            value = float(value)
            self.yml_data['color']['lower']['S'] = value
            dumpYML()
        elif key == "VMIN":
            value = float(value)
            self.yml_data['color']['lower']['V'] = value
            dumpYML()
        elif key == "HMAX":
            value = float(value)
            self.yml_data['color']['upper']['H'] = value
            dumpYML()
        elif key == "SMAX":
            value = float(value)
            self.yml_data['color']['upper']['S'] = value
            dumpYML()
        elif key == "VMAX":
            value = float(value)
            self.yml_data['color']['upper']['V'] = value
            dumpYML()
        elif key == "EXPS":
            value = float(value)
            self.yml_data['camera']['exposure'] = value
            dumpYML()
            self.update_exposure = True
        elif key == "LINE":
            self.yml_data['stream']['line'] = value
            dumpYML()
            self.line = True
        elif key == "CalibrationCamera":
            self.calib_camera = value
        print(self.yml_data)

