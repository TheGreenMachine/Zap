import cv2

class Record():
    def __init__(self):
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter('output.avi', fourcc, 30, (640, 480))
    def write(self, frame):
        self.out.write(frame)
    def release(self):
        self.out.release()
