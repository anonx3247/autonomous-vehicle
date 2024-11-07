from serial_communication.serial_utils import Arduino
from utils import wait
from pathfinding_logic.follow_line import follow_line
from pi.pathfinding_logic.pathfinder import Pathfinder
pathfinder = Pathfinder()
address = 6
pathfinder.direction(address)

def callback(arduino):
    global pathfinder       
    rotation = pathfinder.decision()
    arduino.turn_degrees(rotation)
    wait(0.5)

def main():
    follow_line(callback)

if __name__ == "__main__":
    main()
    