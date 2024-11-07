from serial_communication.serial_utils import Arduino
from utils import wait
from pathfinding_logic.follow_line import follow_line
from pathfinding_logic.pathfinder import Pathfinder
pathfinder = Pathfinder()
address = int(input('address:'))

def callback(arduino):
    global pathfinder       
    rotation = pathfinder.decision(address)
    print('rotation',   rotation)
    if type(rotation) == str:
        print(rotation)
        exit()
    elif rotation != 0:
        arduino.turn_degrees(-rotation, right_angle_factor=150)
    wait(0.5)

def main():
    follow_line(width_threshold=0.3, on_intersection_callback=callback)

if __name__ == "__main__":
    main()
    