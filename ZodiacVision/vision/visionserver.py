import socket
import threading
import copy
import yaml
import json


class ThreadedVisionServer(object):
    def __init__(self, host, port, yml_path, yaml_data):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.cx = '-1'
        self.cy = '-1'
        self.distance = '-1'
        self.yml_data = yaml_data
        self.original_yml = copy.deepcopy(yaml_data)
        self.yml_path = yml_path
        self.update_exposure = False
        self.update_debug = False
        self.calib_camera = False

    def listen(self):
        self.sock.listen(5)
        while True:
            client, address = self.sock.accept()
            client.settimeout(300)
            threading.Thread(target=self.listenToClient, args=(client, address)).start()

    def listenToClient(self, client, address):
        print(address)
        read = client.makefile('r')
        write = client.makefile('w')
        with client, read, write:
            while True:
                try:
                    data = read.readline()
                    if data:
                        # Set the response to echo back the recieved data
                        response = data.strip()
                        print(response)
                        self.dispatchResponse(write, response)
                except Exception as e:
                    client.close()
                    return False

    def dispatchResponse(self, writer, msg):
        if msg == 'distance':
            writer.write("distance|" + str(self.distance) + "\n")
            writer.flush()
        elif msg == 'center_x':
            writer.write("center_x|" + str(self.cx) + "\n")
            writer.flush()
        elif msg == 'center_y':
            writer.write("center_y|" + str(self.cy) + "\n")
            writer.flush()
        elif msg == 'point':
            writer.write("point|"+str(self.cx)+'|'+str(self.cy)+'|' + str(self.distance) + "\n")
            writer.flush()
        elif msg == 'settings':
            writer.write(json.dumps(self.yml_data) + "\n")
            writer.flush()
        elif 'calib' in msg:
            print(str(msg))
            msg = str(msg).replace("'", "").split('|')
            self.calibChange(msg[1], msg[2])

    def updateSavedCenter(self, cx, cy):
        self.cx = cx
        self.cy = cy

    def updateSavedDistance(self, d):
        self.distance = d
    def calibChange(self, key, value):
        def dumpYML():
            with open(self.yml_path, "w") as f:
                yaml.dump(self.yml_data, f)
        # if key == "VISION":
        #     self.vision_use = value
        if key == "HMIN":
            value = float(value)
            self.yml_data['color']['lower']['H'] = value
        elif key == "SMIN":
            value = float(value)
            self.yml_data['color']['lower']['S'] = value
        elif key == "VMIN":
            value = float(value)
            self.yml_data['color']['lower']['V'] = value
        elif key == "HMAX":
            value = float(value)
            self.yml_data['color']['upper']['H'] = value
        elif key == "SMAX":
            value = float(value)
            self.yml_data['color']['upper']['S'] = value
        elif key == "VMAX":
            value = float(value)
            self.yml_data['color']['upper']['V'] = value
        elif key == "EXPS":
            value = float(value)
            self.yml_data['camera']['exposure'] = value
            self.update_exposure = True
        elif key == "LINE":
            self.yml_data['stream']['line'] = value
            self.line = True
        elif key == "DEBG":
            if value == 1:
                self.calib_camera = True
                self.update_debug = True
            else:
                self.calib_camera = False
                self.update_debug = True
            self.yml_data['camera']['debug'] = value
        elif key == "RESET":
            self.yml_data.update(self.original_yml)
            dumpYML()
        elif key == "SAVE":
            dumpYML()
        print(self.yml_data)
        dumpYML()

