import socket

Vision = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
Vision.connect(('192.168.68.118', 1180))

while True:
    from_client = ''
    from_server = b''
    send = b''
    send += input('Send Command: ').encode()
    Vision.sendall(send)
    from_server = Vision.recv(1024)
    print(repr(from_server))
