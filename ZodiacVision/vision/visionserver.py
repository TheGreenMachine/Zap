import socket
import threading


class ThreadedVisionServer(object):
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.cx = bytes('-1', 'utf-8')
        self.cy = bytes('-1', 'utf-8')
        self.distance = bytes('-1', 'utf-8')

    def listen(self):
        self.sock.listen(5)
        while True:
            client, address = self.sock.accept()
            client.settimeout(300)
            threading.Thread(target=self.listenToClient, args=(client, address)).start()

    def listenToClient(self, client, address):
        size = 1024
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

    def updateSavedCenter(self, cx, cy):
        self.cx = bytes(str(cx), 'utf-8')
        self.cy = bytes(str(cy), 'utf-8')

    def updateSavedDistance(self, d):
        self.distance = bytes(str(d), 'utf-8')
