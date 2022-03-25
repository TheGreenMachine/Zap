import cv2
import socket
import sys
import argparse

# Initialize parser
parser = argparse.ArgumentParser()

# Adding optional argument
parser.add_argument("-i", "--ip", help="Socket server ip to connect to (If not passed, IP is set to 10.18.16.16).")

# Read arguments from command line
args = parser.parse_args()


if __name__ == '__main__':
    ip = "10.18.16.16"
    if args.ip:
        print("IP set to: % s" % args.ip)
        ip = args.ip
    address = (ip, 5802)
    data = ''.join(sys.argv[1:])
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


    def updateHmin(value):
        s.send(b'calib|HMIN|' + str(value).zfill(3).encode() + b'\n')


    def updateSmin(value):
        s.send(b'calib|SMIN|' + str(value).zfill(3).encode() + b'\n')


    def updateVmin(value):
        s.send(b'calib|VMIN|' +  str(value).zfill(3).encode() + b'\n')

    def updateHmax(value):
        s.send(b'calib|HMAX|' +  str(value).zfill(3).encode() + b'\n')


    def updateSmax(value):
        s.send(b'calib|SMAX|' +  str(value).zfill(3).encode() + b'\n')


    def updateVmax(value):
        s.send(b'calib|VMAX|' +  str(value).zfill(3).encode() + b'\n')


    def updateExposure(value):
        s.send(b'calib|EXPS|' +  str(value).zfill(3).encode() + b'\n')


    s.connect(address)
    cv2.namedWindow('config')
    cv2.resizeWindow('config', 500, 400)
    cv2.createTrackbar('HMIN', 'config', 36, 180, updateHmin)
    cv2.createTrackbar('SMIN', 'config', 80, 255, updateSmin)
    cv2.createTrackbar('VMIN', 'config', 150, 255, updateVmin)
    cv2.createTrackbar('HMAX', 'config', 180, 180, updateHmax)
    cv2.createTrackbar('SMAX', 'config', 255, 255, updateSmax)
    cv2.createTrackbar('VMAX', 'config', 255, 255, updateVmax)
    cv2.createTrackbar('EXPOSURE', 'config', 10, 50, updateExposure)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
