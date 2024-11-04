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

def send_cmd_i(cmd, arg1, arg2, arg3, arg4):
    arduino.write(cmd)
    write_i16(arduino, arg1)
    write_i16(arduino, arg2)
    write_i16(arduino, arg3)
    write_i16(arduino, arg4)
    wait_for_ack()

def send_cmd_l(cmd, arg1, arg2):
    arduino.write(cmd)
    write_i32(arduino, arg1)
    write_i32(arduino, arg2)
    wait_for_ack()

def receive_cmd_i(cmd):
    arduino.write(cmd)
    val1 = read_i16(arduino)
    val2 = read_i16(arduino)
    val3 = read_i16(arduino)
    val4 = read_i16(arduino)
    return val1, val2, val3, val4
    wait_for_ack()

def receive_cmd_l(cmd):
    arduino.write(cmd)
    val1 = read_i32(arduino)
    val2 = read_i32(arduino)
    return val1, val2
    wait_for_ack()

def wait_for_ack():
    response = b''
    while response == b'':  # wait for the acknowledgement from the B2
        response = arduino.readline()
    print(response.decode())

def reset_encoders():
    send_cmd_i(b'B', 0, 0, 0, 0)

def stop_car():
    send_cmd_i(b'C', 0, 0, 0, 0)

def stop_car_smooth():
    send_cmd_i(b'D', 0, 0, 20, 0)

def advance_car(v1, v2):
    send_cmd_i(b'C', v1, v2, 0, 0)

def advance_car_smooth(v1, v2, v3):
    send_cmd_i(b'D', v1, v2, v3, 0)

def back_car(v1, v2):
    send_cmd_i(b'C', -v1, -v2, 0, 0)

def back_car_smooth(v1, v2, v3):
    send_cmd_i(b'D', -v1, -v2, v3, 0)

def turn_left(v1, v2):
    send_cmd_i(b'C', v1, -v2, 0, 0)

def turn_right(v1, v2):
    send_cmd_i(b'C', -v1, v2, 0, 0)

def test_motor():
    step_time = 2  # duration of each step

    if True:
        print("The vehicle advances")
        advance_car(150, 150)
        time.sleep(1 * step_time)
        stop_car()
        time.sleep(1)

    if True:
        print("The vehicle backs up")
        back_car(150, 150)
        time.sleep(step_time)
        stop_car()
        time.sleep(1)

    if True:
        print("The vehicle turns left")
        turn_left(120, 120)
        time.sleep(step_time)
        stop_car()
        time.sleep(1)

        print("The vehicle turns right")
        turn_right(120, 120)
        time.sleep(step_time)
        stop_car()
        time.sleep(1)

    if True:
        print("The vehicle advances smoothly")
        advance_car_smooth(200, 200, 20)
        time.sleep(3 * step_time)
        stop_car()
        time.sleep(1)

    if True:
        print("The vehicle backs up smoothly")
        back_car_smooth(200, 200, 20)
        time.sleep(3 * step_time)
        stop_car()
        time.sleep(1)

    if True:
        print("Resetting position encoders")
        reset_encoders()
        print("Testing IR sensor")
        arduino.write(b'I1')
        wait_for_ack()
        print("The vehicle starts")
        advance_car(180, 180)

        vit1 = 1
        vit2 = 1
        while (vit1 != 0) or (vit2 != 0):
            time.sleep(0.5)
            tim, tim2, ir, dum1 = receive_cmd_i(b'R')
            enc1, enc2 = receive_cmd_l(b'N')
            print(enc1, enc2, ir)
            vit1, vit2, dum1, dum2 = receive_cmd_i(b'T')
        print("An obstacle has been detected")

