from __future__ import division
import cv2
import numpy as np

from picamera import PiCamera
from picamera.array import PiRGBArray
width, height = 3280 // 2, 2464 // 2
resolution_target = (width, height)

camera = PiCamera(sensor_mode = 2)
camera.resolution = resolution_target
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=camera.resolution)

frame_source = camera.capture_continuous(rawCapture, format="bgr", use_video_port=True)


def perception(feedback = True):
    global frame_source
    global rawCapture

    # Input Image
    image = next(frame_source).array

    if feedback: cv2.imshow("Image non traitée", image)

    # Clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    
    if feedback: 
        cv2.imshow("Image traitée", image)
        cv2.waitKey(1)
    
    return image


if __name__ == "__main__":
    while True:
        perception(feedback = True)