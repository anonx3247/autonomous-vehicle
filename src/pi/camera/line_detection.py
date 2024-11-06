from __future__ import division
from .perception_students import width, height
#from sklearn.linear_model import LinearRegression

import cv2
import numpy as np
mid_x = width // 2
tread_length = 0.1
error_amplitude = 3

def image_to_white_points(image):
    blur = cv2.blur(image,(5,5))
    ret,thresh1 = cv2.threshold(blur,168,255,cv2.THRESH_BINARY)
    hsv = cv2.cvtColor(thresh1, cv2.COLOR_RGB2HSV)
    lower_white = np.array([0, 0, 168])
    upper_white = np.array([172, 111, 255])
    mask = cv2.inRange(hsv, lower_white, upper_white)
    kernel_erode = np.ones((6,6), np.uint8)
    eroded_mask = cv2.erode(mask, kernel_erode, iterations=1)
    kernel_dilate = np.ones((4,4), np.uint8)
    dilated_mask = cv2.dilate(eroded_mask, kernel_dilate, iterations=1)
    return dilated_mask

def find_centroid(image):
    # Input Image
    h, w = image.shape[:2]
    print (w,h)

    # Convert to HSV color space
    dilated_mask = image_to_white_points(image)

    # Find the different contours
    contours, hierarchy = cv2.findContours(dilated_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # Sort by area (keep only the biggest one)
    #im2 = cv2.drawContours(image,contours,-1, (0,255,0), 3)


    #print (len(contours))
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]

    if len(contours) > 0:
        M = cv2.moments(contours[0])
        # Centroid
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        #print("Centroid of the biggest area: ({}, {})".format(cx, cy))
        #cv2.circle(im2, (cx, cy),3,255,-1)
        #cv2.imshow('image', im2)
        return (cx, cy)
    else:
        #print("No Centroid Found")
        return None
    #cv2.imwrite('out_test.png', im2)


def orientation_error(image,bias=0):
    centroid = find_centroid(image)
    if centroid is None:
        return 0
    c = (centroid[0]-mid_x)
    if abs(c) - bias <= 0:
        c = 0
    return c

def motor_speeds_from_image_centroid(image,v, L, R, max_speed=450):
    error = orientation_error(image,bias=0)
    error = error ** 2
    left_speed = R*(v - error*L/2)
    right_speed = R*(v + error*L/2)
    
    # if (abs(left_speed) > max_speed):
    #     left_speed = max_speed if left_speed > 0 else -max_speed
    # if (abs(right_speed) > max_speed):
    #     right_speed = max_speed if right_speed > 0 else -max_speed
    print('left:', left_speed, 'right:', right_speed)
    return (left_speed,right_speed)

def find_direction(image):
    image = cv2.resize(image, (width // 3, height // 3), fx=0.1, fy=0.1)
    dilated_mask = image_to_white_points(image)
    white_points = np.column_stack(np.where(dilated_mask > 0))

    if len(white_points) == 0:
        print("No white points found")
        return 0
    # Run linear regression on the white points
    X = white_points[:, 1]  # x-coordinates
    y = white_points[:, 0]  # y-coordinates
    #reg = LinearRegression().fit(X, y)
    reg = np.polyfit(X, y, 1)
    # Calculate the angle of the line with the vertical
    angle = np.arctan(reg[0]) * 180 / np.pi
    print("Angle with vertical: {:.2f} degrees".format(angle))

    return -angle

def motor_speeds_from_image_direction(image,v, error_weight, speed_factor):
    angle = find_direction(image)
    left_speed = speed_factor * (v - angle * error_weight/2)
    right_speed = speed_factor * (v + angle * error_weight/2)
    return (left_speed, right_speed)
