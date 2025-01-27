import serial
import time
import sys
import glob
from utils import wait, sign

def connect_arduino(protection=True, port_selection=True):
    
    ports = get_serial_ports()
    print(ports)
    while True:
        port = input('Enter port: ')
        arduino = serial.Serial(ports[int(port)], 115200, timeout=0.1)
        arduino.write(bytes('A', 'utf-8'))
        value = ''
        arduino.write(bytes('N', 'utf-8')) # Send a command to check if connected
        value = arduino.readline().decode('utf-8').rstrip()
        if value != '':
            print('Connection successful')
            break
        print('Connection failed retry:')
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
        #print(bytes(f'C{left}', 'utf-8'))
    else:
        arduino.write(bytes(f'C{left} {right}', 'utf-8'))
        #print(bytes(f'C{left} {right}', 'utf-8'))

def set_prog_speed(arduino, left, right):
    arduino.write(bytes(f'D{left} {right}', 'utf-8'))

def set_protection(arduino, enabled):
    if enabled:
        arduino.write(bytes(commands['SET_PROTECTION'], 'utf-8'))
    else:
        arduino.write(bytes(commands['DISABLE_PROTECTION'], 'utf-8'))

def obstacle_detected(arduino):
    arduino.write(bytes(commands['OBSTACLE_DETECTED'], 'utf-8'))
    #print('Checking obstacle detected...')
    response = arduino.readline().decode('utf-8').rstrip()
    return response == 'OB'

def reset_obstacle_detected(arduino):
    arduino.write(bytes(commands['RESET_OBSTACLE_DETECTED'], 'utf-8'))

class Arduino:
    def __init__(self, port=None, baudrate=115200, timeout=0.1, attempt_connection=True, base_speed=500):
        self.base_speed = base_speed
        if port is None:
            while True: 
                print(get_serial_ports())
                ports = get_serial_ports()
                port = input('Enter port: ')
                self.arduino = serial.Serial(ports[int(port)], baudrate, timeout=timeout)
                self.write(commands['CONNECT'])
                if self.check_connection():
                    break
                print('Connection failed, retry...')
        self.connected = False
        self.last_enc = [0,0]
        if attempt_connection:
            self.connect()
    
    def connect(self):
        self.connected = False
        self.write(commands['CONNECT'])
        while not self.connected:
            self.check_connection()
            print('Not connected, retry...')
            wait(0.1)
        print('Connected')

    def check_connection(self):
        self.write(commands['GET_ENCODERS'])
        value = self.read()
        self.connected = value != ''
        return self.connected
    def read(self):
        return self.arduino.readline().decode('utf-8').rstrip()
    
    def smart_read(self):
        t = time.time()
        s = ''
        read = False
        while time.time() - t < 0.1 or not read:
            s += self.arduino.read().decode('utf-8')
            if s[-1] == '\n':
                read = True
                break
        return s

    
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
        #val = self.read()
        val = self.smart_read()
        try:
            self.last_enc = [int(x) for x in val.split(' ')]
            return self.last_enc
        except:
            return self.last_enc

    def get_motor_speed(self):
        self.write(commands['GET_MOTOR_VOLTAGES'])
        #val = self.read()
        val = self.smart_read()
        try:
            return [int(x) for x in val.split(' ')]
        except:
            return [0, 0]

    def set_speed(self, left=None, right=None):
        if left is None:
            left = self.base_speed
        if right is None:
            self.write(f'C{left}')
        else:
            self.write(f'C{left} {right}')
    
    def set_protection(self, enabled):
        if enabled:
            self.write(commands['SET_PROTECTION'])
        else:
            self.write(commands['DISABLE_PROTECTION'])

    def advance_for(self, val, left=None, right=None):
        if left is None:
            left = self.base_speed
        if right is None:
            right = self.base_speed
        self.set_speed(left, right)
        enc = self.get_encoders()
        while abs(enc[0] - self.get_encoders()[0]) < val and abs(enc[1] - self.get_encoders()[1]) < val:
            enc = self.get_encoders()
            wait(0.1)
        self.set_speed(0, 0)
    
    def turn_degrees(self, degrees, speed=None, right_angle_factor=250):
        if speed is None:
            speed = int(self.base_speed * 1.5)
        enc = self.get_encoders()
        #print('left:', speed * sign(degrees), 'right:', -speed * sign(degrees))
        self.set_speed(speed * sign(degrees), -speed * sign(degrees))
        val = right_angle_factor / 90 * abs(degrees)
        while abs(enc[0] - self.get_encoders()[0]) < val:
            #print(enc[0], self.get_encoders()[0])
            pass
        self.set_speed(0, 0)