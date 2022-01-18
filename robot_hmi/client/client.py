#! /usr/bin/env python3
import socket
import pickle

HOST, PORT = '10.42.0.23', 9999
routine = {"nb_steps": 1, "step_distance": 0.2, "frequency": 5, "power_level": 9}
data = pickle.dumps(routine, protocol=0)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
s.sendall(data)
data = s.recv(2048)
s.close()

print('Received', repr(data))
