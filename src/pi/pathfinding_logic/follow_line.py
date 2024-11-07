from serial_communication.serial_utils import Arduino, set_prog_speed, set_speed, obstacle_detected, reset_obstacle_detected
from camera.perception_students import perception
from camera.line_detection import motor_speeds_from_image_direction, motor_speeds_from_image_centroid
from utils import wait, floor
from camera.perception_students import show_image
from camera.corner_detection import detect_intersection

arduino = Arduino()
def follow_line(use_default_parameters=True, expected_corners=4, on_intersection_callback=None):
    intersection_detected = False
    detections = 0
    if use_default_parameters:
        speed = 50
        error_weight = 0.7
        speed_factor = 2
        width_threshold = 0.5
    else:
        speed = input("Enter speed: ")
        error_weight = input("Enter error weight (L): ")
        speed_factor = input("Enter speed factor (1/R): ")
        width_threshold = input("Enter width threshold (0-1): ")
        speed = int(speed) if speed.isdigit() else 50
        error_weight = float(error_weight) if error_weight.replace('.', '', 1).isdigit() else 0.7
        speed_factor = float(speed_factor) if speed_factor.replace('.', '', 1).isdigit() else 2
        width_threshold = float(width_threshold) if width_threshold.replace('.', '', 1).isdigit() else 0.5
    while True:
        image = perception(feedback=False)
        if image is None:
            print("No image")
            continue
        detected = detect_intersection(image, width_threshold=width_threshold, expected_corners=expected_corners)
        if detected:
            print("Intersection detected", detections)
            detections += 1
            if detections >= 3:
                intersection_detected = True
                detections = 0
        elif intersection_detected:
            arduino.set_speed(0, 0)
            wait(2)
            intersection_detected = False
            if on_intersection_callback is not None:
                on_intersection_callback()
            continue
        if arduino.obstacle_detected():
            arduino.set_speed(0, 0)
            print("Obstacle detected")
            wait(0.5)
            arduino.reset_obstacle_detected()
            arduino.turn_degrees(180)
            wait(0.5)
            arduino.reset_obstacle_detected()
        else:
            (left, right) = motor_speeds_from_image_centroid(image, speed, error_weight, speed_factor)
            left, right = floor(left, right)
            arduino.set_speed(left, right)
