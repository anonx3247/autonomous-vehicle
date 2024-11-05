import serial
import time
import sys
import glob

def show_instructions():
    print("Available commands:")
    print('exit - exit the program')
    print('A - Connect to Arduino')
    print('a - Disconnect from Arduino')
    print('G# - move servo to angle # (angle between 30 and 150)')
    print('C# - move motors at speed # (speed between -100 and 100)')
    print('c## - move motor # at speed # (speed between -100 and 100, motor between 1 and 2)')
    print('D# - move motors at speed # (speed between -100 and 100) (Progressive)')
    print('d## - move motor # at speed # (speed between -100 and 100, motor between 1 and 2) (Progressive)')
    print('N - get encoder values')
    print('T - get motor voltages')
    print('F# - move forward for # seconds')
    print('B# - move backward for # seconds')
    print('L# - turn left for # seconds')
    print('R# - turn right for # seconds')
    print('I# - set protection # (0 to disable, 1 to enable)')

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

def process_commands(arduino):
    while True:
        show_instructions()
        command = input("Enter command: ")
        print('command sent:', bytes(command, 'utf-8'))
        if command == "exit":
            break
        elif command == 'A':
            print('Connecting to Arduino...')
            arduino.write(bytes(command, 'utf-8'))
            value = arduino.readline().decode('utf-8').rstrip()
            if value != '':
                print('Connected to Arduino')
        elif command == 'a':
            print('Disconnecting from Arduino...')
            arduino.write(bytes(command, 'utf-8'))
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
            while time.time() - t < float(command[1:]):
                print('waiting...')
                pass
            arduino.write(bytes('C0', 'utf-8'))
        elif command[0] == 'B':
            print('Moving backward...')
            t = time.time()
            arduino.write(bytes('C-500', 'utf-8'))
            while time.time() - t < float(command[1:]):
                pass
            arduino.write(bytes('C0', 'utf-8'))
        elif command[0] == 'L':
            print('Turning left...')
            t = time.time()
            arduino.write(bytes('C500 -500', 'utf-8'))
            while time.time() - t < float(command[1:]):
                pass
            arduino.write(bytes('C0', 'utf-8'))
        elif command[0] == 'R':
            print('Turning right...')
            t = time.time()
            arduino.write(bytes('C-500 500', 'utf-8'))
            while time.time() - t < float(command[1:]):
                pass
            arduino.write(bytes('C0', 'utf-8'))
        else:
            arduino.write(bytes(command, 'utf-8'))
    arduino.close()

# Show ports
ports = get_serial_ports()
print(ports)
port = input('Enter port: ')
# Connect to Arduino
arduino = serial.Serial(ports[int(port)], 115200, timeout=0.1)
arduino.write(bytes('A', 'utf-8'))
# Process commands
process_commands(arduino)
