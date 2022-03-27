import cv2
import socket
import sys
import argparse
import json

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

    def update(type):
        type = type.encode() 
        return lambda value : (
            s.send(b'calib|' + type + b'|' + str(value).zfill(3).encode() + b'\n')
        )

    s.connect(address)
    reader = s.makefile('r')
    s.send(b"settings\n")
    raw_data = reader.readline()

    print("read settings: " + raw_data)
    data = json.loads(raw_data)
    colors = data['color'];
    cv2.namedWindow('config')
    cv2.resizeWindow('config', 500, 400)
    cv2.createTrackbar('HMIN', 'config', int(colors['lower']['H']), 180, update("HMIN"))
    cv2.createTrackbar('SMIN', 'config', int(colors['lower']['S']), 255, update("SMIN"))
    cv2.createTrackbar('VMIN', 'config', int(colors['lower']['V']), 255, update("VMIN"))
    cv2.createTrackbar('HMAX', 'config', int(colors['upper']['H']), 180, update("HMAX"))
    cv2.createTrackbar('SMAX', 'config', int(colors['upper']['S']), 255, update("SMAX"))
    cv2.createTrackbar('VMAX', 'config', int(colors['upper']['V']), 255, update("VMAX"))
    cv2.createTrackbar('EXPOSURE', 'config', int(data['camera']['exposure']), 50, update("EXPS"))
    # todo: fix this - using createButton throws saying the library was compiled 
    # without it
    
    # cv2.createButton("Save", lambda : s.send(b'calib|SAVE\n'))
    # cv2.createButton("Reset", lambda : s.send(b'calib|RESET\n'))
    cv2.waitKey(0)
    cv2.destroyAllWindows()
