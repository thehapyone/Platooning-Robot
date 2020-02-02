import cv2 as cv

import numpy as np
from time import gmtime, strftime, sleep, time
import logging
import nanocamera as nano

def weighted_img(img, initial_img, α=0.8, β=1., λ=0.):
    """
    `img` is the output of the hough_lines(), An image with lines drawn on it.
    Should be a blank image (all black) with lines drawn on it.
    `initial_img` should be the image before any processing.
    The result image is computed as follows:
    initial_img * α + img * β + λ
    NOTE: initial_img and img must be the same shape!
    """
    return cv.addWeighted(initial_img, α, img, β, λ)


def detect_line_segments(cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
                                    np.array([]), minLineLength=10, maxLineGap=15)

    return line_segments

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]

def average_slope_intercept(frame, line_segments):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        logging.info('No line_segment segments detected')
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/3
    left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

    return lane_lines

def display_lines(frame, lines, line_color=(0, 255, 255), line_width=20):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image

def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # only focus bottom half of the screen
    polygon = np.array([[
        (0, height * 1 / 2),
        (width, height * 1 / 2),
        (width, height),
        (0, height),
    ]], np.int32)

    cv.fillPoly(mask, polygon, 255)
    cropped_edges = cv.bitwise_and(edges, mask)
    return cropped_edges

# define a range of black color in HSV

lower_black = np.array([0, 0, 0])
upper_black = np.array([227, 100, 70])

# Rectangular Kernel
rectKernel = cv.getStructuringElement(cv.MORPH_RECT,(7,7))

# Create the Camera instance for 640 by 480
camera = nano.Camera()

if __name__=='__main__':

    print('Started')
    print ("Beginning Transmitting to channel: Happy_Robots")
    now = time()
    
    # commencing subtraction
    while True:
        try:
            # fetching each frame
            frame = camera.read()

            if frame is None:
                break

            # apply some gaussian blur to the image
            kenerl_size = (3, 3)
            gauss_image = cv.GaussianBlur(frame, kenerl_size, 0)
            #gauss_image =  cv.bilateralFilter(frame,9,75,75)

            # here we convert to the HSV colorspace
            hsv_image = cv.cvtColor(gauss_image, cv.COLOR_BGR2HSV)
           
            # apply color threshold to the HSV image to get only black colors
            thres_1 = cv.inRange(hsv_image, lower_black, upper_black)


            # dilate the the threshold image
            thresh = cv.dilate(thres_1, rectKernel, iterations=1)

            # apply canny edge detection
            low_threshold = 200
            high_threshold = 400
            canny_edges = cv.Canny(gauss_image, low_threshold, high_threshold)
            # get a region of interest
            roi_image = region_of_interest(canny_edges)

            line_segments = detect_line_segments(roi_image)
            lane_lines = average_slope_intercept(frame, line_segments)
            # overlay the line image on the main frame
            line_image = display_lines(frame, lane_lines)

            # display both the current frame and the fg masks
            cv.imshow('Frame', frame)
            cv.imshow('New Image', roi_image)
            cv.imshow('Line Image', line_image)
            
            keyboard = cv.waitKey(30)
            if keyboard == 'q' or keyboard == 27:
                break
        except KeyboardInterrupt:
            break

    # cleanup
    camera.release()
    cv.destroyAllWindows()
    del camera
    print('Stopped')
    
