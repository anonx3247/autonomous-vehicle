import serial
import time
import sys
import glob
from utils import wait

def connect_arduino(protection=True, port_selection=True):
    
    ports = get_serial_ports()
    print(ports)
    while True:
        port = input('Enter port: ')
        arduino = serial.Serial(ports[int(port)], 115200, timeout=0.1)
        arduino.write(bytes('A', 'utf-8'))
        arduino.write(0) # padding
        arduino.write(0) # padding  
        value = ''
        for _ in range(3):
            arduino.write(bytes('N', 'utf-8')) # Send a command to check if connected
            value = arduino.readline().decode('utf-8').rstrip()
            if value != '':
                break
            print('Disconnected from Arduino, retrying...')
        if value != '':
            print('Connection successful')
            break
        print('Failed to connect after 3 attempts. Please enter a new port number.')
    if not protection:
        set_protection(arduino, False)
    return arduino

def get_serial_ports():
    """
    Lists serial ports.
    :return: ([str]) A list of available serial ports
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    results = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            results.append(port)
        except (OSError, serial.SerialException):
            pass
    return results

def process_command(arduino, command):
    if command == 'A':
        print('Connecting to Arduino...')
        arduino.write(bytes(command, 'utf-8'))
        arduino.write(bytes('N', 'utf-8')) # Send a command to check if connected
        value = arduino.readline().decode('utf-8').rstrip()
        if value != '':
            print('Connected to Arduino')
    elif command == 'a':
        print('Disconnecting from Arduino...')
        arduino.write(bytes(command, 'utf-8'))
        arduino.write(bytes('N', 'utf-8')) # Send a command to check if connected
        value = arduino.readline().decode('utf-8').rstrip()
        if value == '':
            print('Disconnected from Arduino')
    elif command == 'N':
        print('Getting encoder values...')
        arduino.write(bytes(command, 'utf-8'))
        value = arduino.readline().decode('utf-8').rstrip()
        print(value)
    elif command == 'T':
        print('Getting motor voltages...')
        arduino.write(bytes(command, 'utf-8'))
        value = arduino.readline().decode('utf-8').rstrip()
        print(value)
    elif command[0] == 'F':
        print('Moving forward...')
        t = time.time()
        arduino.write(bytes('C500', 'utf-8'))
        while time.time() - t < float(command[1:]) / 2:
            pass
        arduino.write(bytes('C0', 'utf-8'))
    elif command[0] == 'B':
        print('Moving backward...')
        t = time.time()
        arduino.write(bytes('C-500', 'utf-8'))
        while time.time() - t < float(command[1:]) / 2:
            pass
        arduino.write(bytes('C0', 'utf-8'))
    elif command[0] == 'L':
        print('Turning left...')
        t = time.time()
        arduino.write(bytes('C500 -500', 'utf-8'))
        while time.time() - t < float(command[1:]) / 2:
            pass
        arduino.write(bytes('C0', 'utf-8'))
    elif command[0] == 'R':
        print('Turning right...')
        t = time.time()
        arduino.write(bytes('C-500 500', 'utf-8'))
        while time.time() - t < float(command[1:]) / 2:
            pass
        arduino.write(bytes('C0', 'utf-8'))
    elif command[0] == 'I0':
        print('Turning off protection...')
        arduino.write(bytes(command, 'utf-8'))
    elif command[0] == 'I1':
        print('Turning on protection...')
        arduino.write(bytes(command, 'utf-8'))
    else:
        arduino.write(bytes(command, 'utf-8'))

def process_commands(arduino):
    while True:
        show_instructions()
        commands = input("Enter commands (comma separated): ").split(',')
        import keyboard  # Make sure to install the keyboard module

        if commands[0].startswith('repeat '):
            repeat_commands = [commands[0][7:]] + commands[1:]
            print('Repeating commands:', repeat_commands)
            while True:
                if keyboard.is_pressed('s'):
                    break
                for command in repeat_commands:
                    time.sleep(1)
                    process_command(arduino, command)
                if keyboard.is_pressed('s'):
                    break
        else:
            for command in commands:
                if command == "exit":
                    break
                process_command(arduino, command)
            if command == "exit":
                break
    arduino.close()
 
# Show ports
def connect_and_process_commands():
    ports = get_serial_ports()
    print(ports)
    port = input('Enter port: ')
    # Connect to Arduino
    arduino = serial.Serial(ports[int(port)], 115200, timeout=0.1)
    arduino.write(bytes('A', 'utf-8'))
    # Process commands
    process_commands(arduino)


commands = {
    'CONNECT': 'A',
    'DISCONNECT': 'a',
    'GET_ENCODERS': 'N',
    'GET_MOTOR_VOLTAGES': 'T',
    'MOVE_FORWARD': 'F',
    'MOVE_BACKWARD': 'B',
    'TURN_LEFT': 'L',
    'TURN_RIGHT': 'R',
    'SET_PROTECTION': 'I1',
    'DISABLE_PROTECTION': 'I0',
    'SET_SPEED': 'C',
    'SET_SERVO': 'G',
    'REPEAT': 'repeat',
    'EXIT': 'exit',
    'OBSTACLE_DETECTED': 'e',
    'RESET_OBSTACLE_DETECTED': 'E'
}

def set_servo(arduino, angle):
    arduino.write(bytes(f'G{angle}', 'utf-8'))

def set_speed(arduino, left, right=None):
    if right is None:
        arduino.write(bytes(f'C{left}', 'utf-8'))
        print(bytes(f'C{left}', 'utf-8'))
    else:
        arduino.write(bytes(f'C{left} {right}', 'utf-8'))
        print(bytes(f'C{left} {right}', 'utf-8'))

def set_protection(arduino, enabled):
    if enabled:
        arduino.write(bytes(commands['SET_PROTECTION'], 'utf-8'))
    else:
        arduino.write(bytes(commands['DISABLE_PROTECTION'], 'utf-8'))

def obstacle_detected(arduino):
    arduino.write(bytes(commands['OBSTACLE_DETECTED'], 'utf-8'))
    print('Checking obstacle detected...')
    response = arduino.readline().decode('utf-8').rstrip()
    return response == 'OB'

def reset_obstacle_detected(arduino):
    arduino.write(bytes(commands['RESET_OBSTACLE_DETECTED'], 'utf-8'))

class Arduino:
    def __init__(self, port, baudrate=115200, timeout=0.1, attempt_connection=True):
        self.arduino = serial.Serial(port, baudrate, timeout=timeout)
        self.connected = False
        if attempt_connection:
            self.connect()
    
    def connect(self):
        self.connected = False
        self.write(commands['CONNECT'])
        while not self.connected:
            self.check_connection()
            wait(0.1)

    def check_connection(self):
        self.write(commands['GET_ENCODERS'])
        value = self.read()
        self.connected = value != ''

    def read(self):
        return self.arduino.readline().decode('utf-8').rstrip()
    
    def write(self, command):
        self.arduino.write(bytes(command, 'utf-8'))
    
    def close(self):
        self.arduino.close()
    
    def obstacle_detected(self):
        self.write(commands['OBSTACLE_DETECTED'])
        return self.read() == 'OB'
    
    def reset_obstacle_detected(self):
        self.write(commands['RESET_OBSTACLE_DETECTED'])
    
    def get_encoders(self):
        self.write(commands['GET_ENCODERS'])
        return self.read()

    def set_speed(self, left, right=None):
        if right is None:
            self.write(f'C{left}')
        else:
            self.write(f'C{left} {right}')
    
    def set_protection(self, enabled):
        if enabled:
            self.write(commands['SET_PROTECTION'])
        else:
            self.write(commands['DISABLE_PROTECTION'])
