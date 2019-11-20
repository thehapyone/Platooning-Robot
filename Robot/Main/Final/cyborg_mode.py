import jetson.inference
import jetson.utils
import cv2 as cv
import copy
import numpy as np

import lcm
from bumblee import cyborg_detection


system_parameters = ['cyborg_mode.py', '--prototxt=/home/ayo/deis-model/deploy.prototxt', '--model=/home/ayo/deis-model/snapshot_iter_33300.caffemodel',
                     '--class_labléls=insect,traffic', '--threshold=0.5', '--width=640', '--height=420']
network = 'ssd-mobilenet-v2'

detect_threshold = 0.5
print ('System Parameters - ', system_parameters)


image_width = 640
image_height = 480

# load the object detection model
net = jetson.inference.detectNet("ssd-mobilenet-v2", system_parameters, threshold=detect_threshold)

# create the camera
camera = jetson.utils.gstCamera(image_width, image_height, "0")

# set the display
# display = jetson.utils.glDisplay()

'''
Detection object parameters
   -- ClassID: 1
   -- Confidence: 0.825449
   -- Left:    340.048
   -- Top:     69.4844
   -- Right:   368.161
   -- Bottom:  141.75
   -- Width:   28.113
   -- Height:  72.2656
   -- Area:    2031.6
   -- Center:  (354.105, 105.617)
'''

# notes
# objects with area above 10000, won't be an insect, so discard the result from those.

target_class = 0
detectedObjects = []
# image FPS
detection_rate = ""
# focal length
focal_length = 380.4 # used for traffic light
focal_length1 = 264.66
focal_length2 = 423.465

# real object width
real_width = 2.5 # for traffic light
real_width1 = 16
real_width2 = 10

# save the object detections to a list
insect_detections = list()

# initialize the LCM variables
insect_results = cyborg_detection()
insect_results.detectSize = 1
insect_results.data = [(0,0,0)]

# initialize LCM
lc = lcm.LCM()


# pixels set point
setpoint_xy = [int(image_width/2), int(image_height)]

def find_angle_vision(start, target):
    angle = np.arctan2((start[1] - target[1]), (start[0] - target[0]))
    return np.rad2deg((np.pi / 2) - angle)

# process frames until the user exits
while True:
    try:
        # main loop will go here
        # capture the image
        img, width, height = camera.CaptureRGBA(zeroCopy = True)
        jetson.utils.cudaDeviceSynchronize()

        # trasform the image to be used in Opencv
        aimg = jetson.utils.cudaToNumpy(img, width, height, 4)
        aimg1 = cv.cvtColor(aimg.astype(np.uint8), cv.COLOR_RGBA2BGR)

        # show the center of the image
        cv.circle(aimg1, (setpoint_xy[0], setpoint_xy[1]), 5, (111, 0, 255), -1)
        #to draw a rectangle, you need top-left corner and bottom-right corner of rectangle.
        # parameters are: (left, top) and (right, bottom)

        # detect objects in the image (with overlay)
        detections = net.Detect(img, width, height, "box,labels,conf")

        # print the detections
        print("detected {:d} objects in image".format(len(detections)))

        insect_detections = []
        detected_id = 0
        
        for detection in detections:
            # only get detections for the specified class
            if detection.ClassID == target_class:
                #print(detection)
                # draw a bounding box for that detections
                object_left = int(detection.Left)
                object_right = int(detection.Right)
                object_top = int(detection.Top)
                object_bottom = int(detection.Bottom)
                object_center = (int(detection.Center[0]), int(detection.Center[1]))
                object_area = int(detection.Area)
                object_width = detection.Width
                object_height = detection.Height
                # check if object_area is within the area
                # for the insect a good value is 500
                if object_area < 50000:                        
                    print ('Object area: ',object_area, object_width) 
                    cv.circle(aimg1, object_center, 5, (255, 0, 0), -1)
                    cv.rectangle(aimg1,(object_left,object_top),(object_right,object_bottom),(0,255,0),3)
                    if  object_width > object_height:
                        object_distance = (focal_length1 * real_width1)/ object_width
                        # compensate for the tails or head
                        object_distance = object_distance - 7
                    else:
                        object_distance = (focal_length2 * real_width2)/object_width
                        
                    print ('Distance to object: ',object_distance)
                    object_angle = find_angle_vision(setpoint_xy, object_center)
                    print ('Angle to object: ', object_angle)
                    insect_detections.append((detected_id, object_angle, object_distance))
                    detected_id = detected_id + 1

        detection_rate = str(net.GetNetworkFPS()) + ' FPS'
        #print ('Frame rate: ',detection_rate)

        # here we will transmit the object detections
        if len(insect_detections) > 0:
            # publish the result to LCM
            insect_results.detectSize = len(insect_detections)
            insect_results.data = insect_detections
            lc.publish("BumbleBee_Insects", insect_results.encode())
        else:
            # we have to send -1
            insect_results.detectSize = 1
            insect_results.data = [(0,-1,-1)]
            lc.publish("BumbleBee_Insects", insect_results.encode())
        
        # print out performance info
        # net.PrintProfilerTimes()
        # display both the current frame and the fg masks
        #cv.imshow('Frame', raw_image)
        cv.imshow('Detections', aimg1)

        keyboard = cv.waitKey(30)
        if keyboard == 'q' or keyboard == 27:
            break
            
    except KeyboardInterrupt:
        break

cv.destroyAllWindows()
del camera
print ('stopped')
