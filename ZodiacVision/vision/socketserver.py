import socket
import threading


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
        self.server.bind(('172.20.99.92', self.port))
        self.server.listen(5)


    def sendValues(self, dist, cx,cy,msg):



        if msg == 'distance':
            self.server.send(("distance|"+dist+"\n").encode())
        elif msg == 'center_x':
            self.server.send(("center_x|"+cx+"\n").encode())
        elif msg == 'cxy':
            self.server.send((""+cx+","+cy).encode())
        elif msg == 'cxd':
            self.server.send((""+cx+","+cy+","+dist).encode())
        else:
            self.server.send('spel rite dum poo poo head')

    def listenMessage(self,distance,Cy,Cx ):

        self.conn, self.addr = self.server.accept()

        print("Server started")

        data = self.conn.recv(1024)

        print(data)

        self.sendValues(distance,Cy, Cx,data)


















