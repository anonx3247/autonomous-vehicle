from __future__ import division
import cv2

from picamera import PiCamera
from picamera.array import PiRGBArray
full_width, full_height = 3280, 2464
width, height = full_width // 10, full_height // 10
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
    image_to_show = cv2.resize(image, (width // 10, height // 10))

    # Clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    
    if feedback: 
        show_image(image_to_show)
    
    return image


if __name__ == "__main__":
    while True:
        perception(feedback = True)

def show_image(image):
    cv2.imshow("Image traitée", image)
    cv2.waitKey(1)
