import serial as ser
import time
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

# Connect to Arduino
arduino = ser.Serial('/dev/ttyACM1', 115200, timeout=0.1)
arduino.write(bytes('A', 'utf-8'))
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

process_commands(arduino)