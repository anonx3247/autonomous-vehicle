import serial
import time
import numpy as np
import struct

def read_i16(f):
    return struct.unpack('<h', bytearray(f.read(2)))[0]

def read_i32(f):
    return struct.unpack('<l', bytearray(f.read(4)))[0]

def write_i16(f, value):
    f.write(struct.pack('<h', value))

def write_i32(f, value):
    f.write(struct.pack('<l', value))

def dialogue_with_arduino():
    while True:
        print("\nDialogue direct avec l'arduino")
        command = input("Tapez votre commande arduino (Q pour finir) ")
        if command == "Q":
            break
        if command:
            arduino.write(command.encode('utf-8'))
            time.sleep(0.01)
            response = arduino.readline()  # on lit le message de réponse
            while response == b'':  # On attend d'avoir une vraie réponse
                response = arduino.readline()  # on lit le message de réponse
            print(response.decode())
            while arduino.inWaiting() > 0:  # tant qu'on a des messages dans le buffer de retour
                response = arduino.readline()  # on lit le message de réponse
                print(response.decode())

def wait_for_acknowledgement():
    response = b''
    while response == b'':  # attend l'acquitement du B2
        response = arduino.readline()
    # print(response.decode())

def main_program():
    global arduino
    arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.1)
    print("Connection à l'arduino")
    time.sleep(2)  # on attend 2s pour que la carte soit initialisée

    arduino.write(b'A20')  # demande de connection avec acquitement complet en ascii
    response = arduino.readline()
    if response.split()[0] == b'OK':
        print(response.decode())
        dialogue_with_arduino()

    arduino.write(b'a')  # deconnection de la carte
    arduino.close()  # fermeture de la liaison série
    print("Fin de programme")

if __name__ == "__main__":
    main_program()

