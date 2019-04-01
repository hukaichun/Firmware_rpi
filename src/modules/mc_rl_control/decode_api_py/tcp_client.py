import socket
import time
import numpy as np
import array
import struct


sock = socket.socket()        
# sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
for _ in range(10):
	try:
		sock.connect(("192.168.1.159", 8787))
		break
	except:
		print("retring")
		time.sleep(1)

# raw_data = np.zeros(shape=50000, dtype=float)
# raw_data = ""
data = []
received_data = []
for _ in range(100000):
	while True:
		raw_data = sock.recv(92000)
		# sock.recv_into(data)

		num_floats = int(len(raw_data) / 4)
		format_str = '<' + 'f' * num_floats
		data += struct.unpack(format_str, raw_data[:num_floats * 4])
		if data[-1] == 666:
			received_data = data
			data = []
			break;
	# for i in range(8):
	# print("loop")
	# print(len(data))
	print(received_data[0:23])
	sock.send("get message")

sock.close()