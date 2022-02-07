import cv2

def crop_half(image):
    height, width, channels = image.shape
    # cropped_img = image[0:height, 0:int(width/2)]
    cropped_img = image[0:height,int(width/2):width]
    return cropped_img

cap = cv2.VideoCapture('out1.avi')
writer = cv2.VideoWriter('right.mp4',
                         cv2.VideoWriter_fourcc(*'MP4V'),
                         100, (672, 376))
while 1:
    ret, frame = cap.read()
    cv2.imshow('Frame', crop_half(frame))
    writer.write(crop_half(frame))
    cv2.waitKey(1)
