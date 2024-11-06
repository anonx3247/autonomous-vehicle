from serial_communication.serial_utils import connect_arduino, set_speed, obstacle_detected, reset_obstacle_detected
from camera.perception_students import perception
from camera.line_detection import motor_speeds_from_image_direction, motor_speeds_from_image_centroid
from utils import wait, floor
from camera.perception_students import show_image

arduino = connect_arduino(protection=True)

def follow_line():
    speed = int(input("Enter speed: "))
    error_weight = float(input("Enter error weight (L): "))
    speed_factor = float(input("Enter speed factor (1/R): "))
    while True:
        image = perception(feedback=False)
        if image is None:
            print("No image")
            continue
        if obstacle_detected(arduino):
            set_speed(arduino, 0, 0)
            print("Obstacle detected")
            wait(0.5)
            reset_obstacle_detected(arduino)
        else:
            (left, right) = motor_speeds_from_image_centroid(image, speed, error_weight, speed_factor)
            left, right = floor(left, right)
            set_speed(arduino, left=left, right=right)
