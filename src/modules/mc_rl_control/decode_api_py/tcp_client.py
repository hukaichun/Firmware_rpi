import socket
import time
import numpy as np
import array
import struct
import threading
import Queue

class drone(object):
	"""docstring for drone"""
	def __init__(self, ip="192.168.1.159", port=8787):
		
		self.sock = socket.socket()
		# self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		for _ in range(10):
			try:
				# self.sock.connect(("192.168.1.159", 8787))
				self.sock.connect((ip, port))
				break
			except:
				print("retring")
				time.sleep(1)

		self.queue = Queue.Queue()

	def __del__(self):

		self.sock.close()		

	def put_data(self):

		data = []
		received_data = []

		while True:			
			while True:
				raw_data = self.sock.recv(92000)

				num_floats = int(len(raw_data) / 4)
				format_str = '<' + 'f' * num_floats
				data += struct.unpack(format_str, raw_data[:num_floats * 4])
				if data[-1] == 666:
					received_data = data
					data = []
					break;
			i = 0
			splited_data = []
			data_in_queue = []

			while received_data[i] != 666:
				if received_data[i] == 9487:
					splited_data.append(received_data[i+1:i+23])
				i += 1

			if self.queue.empty() is False:
				data_in_queue = self.queue.get()
			data_in_queue += splited_data
			self.queue.put(data_in_queue)



	def get_data(self):
		if self.queue.empty() is False:
			return self.queue.get()
		else:
			return []


if __name__ == '__main__':
	pi_drone = drone(ip="140.113.154.156")
	receive_thread = threading.Thread(target = pi_drone.put_data)
	receive_thread.start()
	# receive_thread.join()
	# time.sleep(10)
	while True:
		qq = pi_drone.get_data()
		print(len(qq))
		time.sleep(4)
	receive_thread.join()

