#
#   Hello World client in Python
#   Connects REQ socket to tcp://localhost:5555
#   Sends "Hello" to server, expects "World" back
#

from __future__ import print_function

import zmq
import time
import numpy as np
import msgpack

import msgpack_numpy as m
m.patch()


# serialization = "JSON"
# serialization = "MSGPACK"
serialization = "MSGPACK-PYTHON"
# serialization = "ARRAY"



context = zmq.Context()
#  Socket to talk to server
print("Connecting to Hello World server")
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")

# socket2 = context.socket(zmq.REQ)
# socket2.connect("tcp://localhost:5556")

#  Do 10 requests, waiting each time for a response
for request in range(100):
    print("Sending request %s" % request)


    img = np.random.rand(640, 480, 3).astype(np.int16)
    img2 = np.random.rand(640, 480, 3).astype(np.int16)


    start_time = time.time()
    if serialization == "MSGPACK":
        msg = {'img': zmq_utils.pack_array(img),
               'img2': zmq_utils.pack_array(img2)}
        msg_packed = msgpack.packb(msg)
        socket.send(msg_packed)
    elif serialization == "MSGPACK-PYTHON":
        msg = {'img': img,
               'img2': img2}
        msg_packed = msgpack.packb(msg)
        socket.send(msg_packed)
    elif serialization == "JSON":
        socket.send_json(msg)
    elif serialization == "ARRAY":
        send_array(socket, img)

    # wait for response
    socket.recv()
    end_time = time.time() - start_time
    print("round trip took:", end_time)

    time.sleep(0.2)