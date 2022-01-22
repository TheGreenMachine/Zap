import socket

Vision = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
Vision.connect(('172.20.99.92', 5802))

while True:
    from_client = ''
    from_server = ''
    send = (input("Send Command: "))
    Vision.send(send.encode())
    from_server = Vision.recv(1024)
    print(from_server)

Vision.close()






















