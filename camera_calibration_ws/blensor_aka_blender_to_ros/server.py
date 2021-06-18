#!/usr/bin/env python3

import socket
import pickle

HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 65432        # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    i = 0
    # with conn:
    print('Connected by', addr)
    data = []
    valid = False
    packet = []
    while True:
        packet = conn.recv(2**32)
        if not packet and valid:
            print(data.__len__())
            point_cloud = pickle.loads(b"".join(data))
            print("Point cloud ready to go, with shape: ",point_cloud.shape)
            valid = False
        elif not packet :
            conn, addr = s.accept()
        else:
            print("Appending...")
            data.append(packet)
            valid = True

            

    # conn.sendall(data)
