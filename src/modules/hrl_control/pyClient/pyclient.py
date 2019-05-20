import socket
import struct

IP = "140.113.25.231"
PORT = 7777

sock_fd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

for i in range(3):
    import time
    try:
        print("wait server respond")
        sock_fd.connect((IP, PORT))
    except:
        time.sleep(1)
    else:
        while True:
            head = sock_fd.recv(10)
            if len(head)>0:
                head = struct.unpack("<BBBBBBBBBB", head)
                
                payload = sock_fd.recv(head[1])
                fmt = "<" + "".join(["B" for _ in range(head[1])])
                payload = struct.unpack(fmt, payload)
                
                checksum = sock_fd.recv(2)
                fmt = "<" + "BB"
                checksum = struct.unpack(fmt, checksum)


                print("[head]:{head}\n"
                      "[payload]:{payload}\n"
                      "[checksum]:{checksum}\n".format(
                            head = head,
                            payload = payload,
                            checksum = checksum
                        ))
            else:
                print("closed")
                break
        break
    print("server no respond")



