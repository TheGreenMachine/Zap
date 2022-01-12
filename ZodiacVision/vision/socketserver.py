import socket


class SocketServer:
    def __init__(self, yml_data, yml_path,port):
        self.yml_data = yml_data
        self.update_exposure = False
        self.yml_path = yml_path
        self.calib_camera = False
        self.vision_use = False
        self.line = False
        self.port = port
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def startServer(self):
        self.server.bind(('',self.port))
        self.server.listen(10)
        self.conn, self.addr = self.server.accept()

    def sendValues(self, msg, dist, Cx,Cy):
        if msg == 'distance':
            self.server.send(bytes(""+dist))
        elif msg == 'center_x':
            self.server.send(bytes(""+Cx+","+Cy))

    def listenMessage(self,Cy,Cx, distance):
        data = self.conn.recv(1024)
        if data != '':
            self.sendValues(data, distance,Cy, Cx)


















