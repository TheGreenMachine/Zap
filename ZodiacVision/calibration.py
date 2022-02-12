import cv2
import socket
import sys

if __name__ == '__main__':
    print("Enter socket IP")
    ip = input()
    if ip == "":
        ip = "localhost"
    address = (ip, 5802)
    data = ''.join(sys.argv[1:])
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


    def updateHmin(value):
        s.send(b'calib|HMIN|' + str(value).zfill(3).encode())


    def updateSmin(value):
        s.send(b'calib|SMIN|' + str(value).zfill(3).encode())


    def updateVmin(value):
        s.send(b'calib|VMIN|' +  str(value).zfill(3).encode())

    def updateHmax(value):
        s.send(b'calib|HMAX|' +  str(value).zfill(3).encode())


    def updateSmax(value):
        s.send(b'calib|SMAX|' +  str(value).zfill(3).encode())


    def updateVmax(value):
        s.send(b'calib|VMAX|' +  str(value).zfill(3).encode())


    def updateExposure(value):
        s.send(b'calib|EXPS|' +  str(value).zfill(3).encode())


    s.connect(address)
    cv2.namedWindow('config')
    cv2.resizeWindow('config', 500, 400)
    cv2.createTrackbar('HMIN', 'config', 0, 180, updateHmin)
    cv2.createTrackbar('SMIN', 'config', 0, 255, updateSmin)
    cv2.createTrackbar('VMIN', 'config', 0, 255, updateVmin)
    cv2.createTrackbar('HMAX', 'config', 0, 180, updateHmax)
    cv2.createTrackbar('SMAX', 'config', 0, 255, updateSmax)
    cv2.createTrackbar('VMAX', 'config', 0, 255, updateVmax)
    cv2.createTrackbar('EXPOSURE', 'config', 0, 50, updateExposure)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
