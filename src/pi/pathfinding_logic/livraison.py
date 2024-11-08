from serial_communication.serial_utils import Arduino
from utils import wait
from pathfinding_logic.follow_line import follow_line
from pathfinding_logic.pathfinder import Pathfinder
from camera.line_detection import find_centroid
from camera.perception_students import perception
pathfinder = Pathfinder()

#addresses = []
idx = 0
#address = addresses[idx]

def callback(arduino,addresses):
    global pathfinder, idx  
    address = addresses[idx]
    rotation = pathfinder.decision(address)
        
    print('rotation',   rotation)
    if type(rotation) == str:
        if rotation == 'arrived':
            pathfinder.pos = address
            if idx == len(addresses)-1:
                idx += 1
                return 'arrived'
            wait(1)
            idx += 1
            address = addresses[idx]
            return callback(arduino, addresses)
        
    elif rotation != 0:
        arduino.turn_degrees(-rotation, right_angle_factor=150)
    wait(0.5)
    image = perception(feedback=False)
    c = find_centroid(image)
    if c is None:
        print('enlevement d\'arrete')
        pathfinder.enleve_arrete_en_face()
        callback(arduino,addresses)

def obstacle_line():
    global pathfinder, address
    pathfinder.orientation = (pathfinder.orientation - 2) % 4
    pathfinder.enleve(pathfinder.prev, pathfinder.pos)
    pathfinder.pos = pathfinder.prev
    pathfinder.djikstra(pathfinder.pos, address)
    
    

def obstacle_int(arduino, addresses):
    global pathfinder, address
    pathfinder.enleve_arrete_en_face()
    pathfinder.pos = pathfinder.prev
    pathfinder.djikstra(pathfinder.pos, address)
    callback(arduino, addresses)

def main():
    follow_line(
        use_default_parameters=True,
        width_threshold=0.3, 
        on_intersection_callback=callback, 
        on_obstacle_line=obstacle_line, 
        on_obstacle_intersection=obstacle_int)

if __name__ == "__main__":
    main()
    