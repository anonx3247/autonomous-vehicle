import serial as ser

def show_instructions():
    print("Available commands:")
    print('exit')
    print('A - Connect to Arduino')
    print('a - Disconnect from Arduino')
    print('G# - move servo to angle # (angle between 30 and 150)')
    print('C# - move motors at speed # (speed between -100 and 100)')
    print('c## - move motor # at speed # (speed between -100 and 100, motor between 1 and 2)')
    print('D# - move motors at speed # (speed between -100 and 100) (Progressive)')
    print('d## - move motor # at speed # (speed between -100 and 100, motor between 1 and 2) (Progressive)')
    print('N - get encoder values')
    print('T - get motor voltages')

# Connect to Arduino
arduino = ser.Serial('/dev/ttyACM0', 115200, timeout=0.1)
arduino.write(bytes('A', 'utf-8'))
def process_commands(arduino):
    while True:
        show_instructions()
        command = input("Enter command: ")
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
        else:
            arduino.write(bytes(command, 'utf-8'))
    arduino.close()

process_commands(arduino)