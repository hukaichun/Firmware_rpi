# import struct



# with open("loga", "rb") as f:

# 	data = f.read()
# 	print(len(data))
# 	print(struct.unpack('H'*3, f.read(6)))
# 	# print(struct.unpack('H', f.read(2)))
# 	# print(struct.unpack('H', f.read(2)))
# 	# byte = f.read()
# 	# print(byte)
# 	# while byte:
# 	# 	# Do stuff with byte.
# 	# 	byte = f.read(2)
# 	# 	print(byte)

import array
import numpy as np

with open("log_file", "rb") as f:

	# data = f.read()
	U = array.array("f")
	U.fromstring(f.read())
	data_raw = U.tolist()
	data = []

	for i in range(len(data_raw)):
		if data_raw[i] == 9487:
			data.append(data_raw[i+1:i+23])
	print len(data[0])
	print data[5000]