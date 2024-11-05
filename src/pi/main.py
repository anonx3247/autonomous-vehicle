from serial_communication.serial_utils import process_command, connect_arduino, set_servo, set_speed, set_protection
from camera.perception_students import perception
from camera.line_detection import motor_speeds_from_image

arduino = connect_arduino()

def main():
    while True:
        image = perception()
        if image is None:
            print("No image")
            continue
        left, right = motor_speeds_from_image(image, 100)
        set_speed(arduino, right, left)

if __name__ == "__main__":
    main()