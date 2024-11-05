from serial_communication.serial_utils import process_command, connect_arduino, set_servo, set_speed, set_protection
from camera.perception_students import perception
from camera.line_detection import motor_speeds_from_image
import time

arduino = connect_arduino(protection=False)

def main():
    speed = int(input("Enter speed: "))
    while True:
        time.sleep(0.1)
        image = perception()
        if image is None:
            print("No image")
            continue
        left, right = motor_speeds_from_image(image, speed)
        left, right = int(left), int(right)
        set_speed(arduino, left=left, right=right)

if __name__ == "__main__":
    main()