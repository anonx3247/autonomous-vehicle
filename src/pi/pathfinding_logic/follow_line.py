from serial_communication.serial_utils import connect_arduino, set_prog_speed, set_speed, obstacle_detected, reset_obstacle_detected
from camera.perception_students import perception
from camera.line_detection import motor_speeds_from_image_direction, motor_speeds_from_image_centroid
from utils import wait, floor
from camera.perception_students import show_image
from camera.corner_detection import corner_detection

arduino = connect_arduino(protection=True)
def follow_line(expected_corners=4):
    intersection_detected = False
    detections = 0
    speed = input("Enter speed: ")
    error_weight = input("Enter error weight (L): ")
    speed_factor = input("Enter speed factor (1/R): ")
    quality = input("Enter quality (0-1): ")
    speed = int(speed) if speed.isdigit() else 50
    error_weight = float(error_weight) if error_weight.replace('.', '', 1).isdigit() else 0.7
    speed_factor = float(speed_factor) if speed_factor.replace('.', '', 1).isdigit() else 2
    quality = float(quality) if quality.replace('.', '', 1).isdigit() else 0.99
    while True:
        image = perception(feedback=False)
        if image is None:
            print("No image")
            continue
        detected, _ = corner_detection(image, quality=quality, expected_corners=expected_corners)
        if detected:
            print("Intersection detected")
            detections += 1
            if detections >= 3:
                intersection_detected = True
                detections = 0
        elif intersection_detected:
            set_speed(arduino, 0, 0)
            wait(1)
            set_speed(arduino, -500, 500)
            wait(1.5)
            set_speed(arduino, 0, 0)
            wait(1)
            intersection_detected = False
            continue
        if obstacle_detected(arduino):
            set_speed(arduino, 0, 0)
            print("Obstacle detected")
            wait(0.5)
            reset_obstacle_detected(arduino)
        else:
            (left, right) = motor_speeds_from_image_centroid(image, speed, error_weight, speed_factor)
            left, right = floor(left, right)
            set_speed(arduino, left, right)
