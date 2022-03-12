import socket
import sys

address = ('10.18.16.16', 5802)
data = ''.join(sys.argv[1:])
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect(address)
    s.send(b'distance' + b'\n')
    received = s.recv(20000)
    print(received)
