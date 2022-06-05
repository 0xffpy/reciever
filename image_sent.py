from re import T
import zmq
import numpy as np
import cv2
import time
context = zmq.Context()

#  Socket to talk to server
print("Connecting to hello world server…")
socket = context.socket(zmq.REQ)
socket.connect("tcp://192.168.0.101:5566") # We change IP address here as per Jetson IP address 

counter = 0
#  Do requests, waiting each time for a response
while True:
    try:

        #print("Sending request ")
        socket.send(b"Message from Ground Station")
    except Exception as r:
        print(r)
    try:
        #  Get the reply.
        frame = socket.recv()
        if frame == b'None':
            pass
        else:
            t = time.time()
            image = np.asarray(bytearray(frame), dtype="uint8")
            image = cv2.imdecode(image, cv2.IMREAD_COLOR)
            cv2.imwrite('{}.jpeg'.format(counter),image)
            counter = counter + 1
            t2 = time.time() - t 
            print(f"\n\n\n\n Takes {t2}ms ")
            #time.sleep(1)
    except Exception as error:
        print(error)