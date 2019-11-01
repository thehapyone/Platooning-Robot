import cv2 as cv

import numpy as np
from time import gmtime, strftime, sleep, time


# NVIDIA jetson ues the gstreamer pipeline for getting it's data
def gstreamer_pipeline (capture_width=800, capture_height=640, display_width=800, display_height=640, framerate=60, flip_method=0) :   
    return ('nvarguscamerasrc ! ' 
    'video/x-raw(memory:NVMM), '
    'width=(int)%d, height=(int)%d, '
    'format=(string)NV12, framerate=(fraction)%d/1 ! '
    'nvvidconv flip-method=%d ! '
    'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
    'videoconvert ! '
    'video/x-raw, format=(string)BGR ! appsink'  % (capture_width,capture_height,framerate,flip_method,display_width,display_height))

# define a range of black color in HSV

lower_black = np.array([0, 0, 0])
upper_black = np.array([227, 100, 70])


if __name__=='__main__':

    # capture the video stream data
    capture = cv.VideoCapture(gstreamer_pipeline(flip_method=0, framerate=60), cv.CAP_GSTREAMER)

    if not capture.isOpened:
        print('Unable to open: Camera interface')
        exit(0)

    print('Started')
    print ("Beginning Transmitting to channel: Happy_Robots")
    now = time()
    
    # commencing subtraction
    while True:
        try:
            # fetching each frame
            ret, frame = capture.read()

            if frame is None:
                break

            # here we convert to the HSV colorspace
            hsv_image = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

            # apply color threshold to the HSV image to get only black colors
            thres_1 = cv.inRange(hsv_image, lower_black, upper_black)

            '''
            mask_yellow = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
            mask_white = cv2.inRange(gray_image, 200, 255)
            mask_yw = cv2.bitwise_or(mask_white, mask_yellow)
            mask_yw_image = cv2.bitwise_and(gray_image, mask_yw)
            '''

            # dilate the the threshold image
            thresh = cv.dilate(thres_1, None, iterations=1)

            # display both the current frame and the fg masks
            cv.imshow('Frame', frame)
            cv.imshow('FG Mask', thresh)

            keyboard = cv.waitKey(30)
            if keyboard == 'q' or keyboard == 27:
                break
        except KeyboardInterrupt:
            break

    # cleanup
    capture.release()
    cv.destroyAllWindows()
    del capture
    print('Stopped')
    
