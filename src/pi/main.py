from serial_communication.serial_test import process_command, connect_arduino
from camera.perception_students import perception, motor_speeds

arduino = connect_arduino()

while True:
    perception()
    motor_speeds()
