#!/usr/bin/python

import bluetooth
import atexit


def exit_handler():
	s.close()


output_file = '/home/pizero1/btpump.log'

address = "C2:2C:10:05:8A:4F"
passwrd = "1234"
port = 1


s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
s.connect((address, port))

while True:
	try:
		data = s.recv(100)
		
		fo = open(output_file, 'a')
		fo.write(data)
		fo.close()
	except:
		pass
