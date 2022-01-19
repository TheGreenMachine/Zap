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

    def sendValues(self, msg, dist, cx,cy):
        if msg == 'distance':
            self.server.send(bytes("distance|"+dist+"\n"))
        elif msg == 'center_x':
            self.server.send(bytes("center_x|"+cx+"\n"))
        elif msg == 'cxy':
            self.server.send(bytes(""+cx+","+cy))
        elif msg == 'cxd':
            self.server.send(bytes(""+cx+","+cy+","+dist))

    def listenMessage(self,Cy,Cx, distance):
        data = self.conn.recv(1024)
        if data != '':
            self.sendValues(data, distance,Cy, Cx)


















