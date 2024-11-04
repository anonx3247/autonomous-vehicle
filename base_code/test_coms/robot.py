#########################################################################
# CENTRALESUPELEC : ST5 Integration teaching
#
# Communication client for robots
#   This program sends messages to the server regularly
#   to be informed of commands given at the control side.
#
#########################################################################
# Authors : Philippe Benabes & Koen de Turck
# Modifications by Morgan Roger
# TBD : message content could explicitly include origin 
#########################################################################

import zmq
import sys
import time

SERVER_IP = "192.168.0.192"
if len(sys.argv) > 1:
    SERVER_IP = sys.argv[1]
MY_ID = 'bot001'
if len(sys.argv) > 2:
    MY_ID = sys.argv[2]


def main():
    server_socket = connect_to(SERVER_IP)
    print("initial hello msg ...")
    register_msg = {"cmd": "log"}       # add header indicating origin ?
    send_message(server_socket, register_msg)
    
    while True:
        msg = {"cmd": "key"}
        reply = send_message(server_socket, msg)
        if reply["key"] != '':
            print("received:{}".format(reply))
        time.sleep(1)


def connect_to(ip):
    ctx = zmq.Context()
    reqsock = ctx.socket(zmq.REQ)
    reqaddr = "tcp://{}:5005".format(SERVER_IP)
    reqsock.connect(reqaddr)
    
    return reqsock

def send_message(sock, content):
    msg = {"from": MY_ID}       # header indicating origin
    msg.update(content)
    sock.send_pyobj(msg)
    reply = sock.recv_pyobj()
    
    return reply


if __name__ == "__main__":
    main()
