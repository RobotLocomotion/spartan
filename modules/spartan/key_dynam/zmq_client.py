#
#   Hello World client in Python
#   Connects REQ socket to tcp://localhost:5555
#   Sends "Hello" to server, expects "World" back
#

from __future__ import print_function

import zmq
import msgpack
import time
import numpy as np


def send_array(socket, A, flags=0, copy=True, track=False):
    """send a numpy array with metadata"""
    md = dict(
        dtype = str(A.dtype),
        shape = A.shape,
    )
    socket.send_json(md, flags|zmq.SNDMORE)
    return socket.send(A, flags, copy=copy, track=track)



# serialization = "JSON"
# serialization = "MSGPACK"
serialization = "ARRAY"


context = zmq.Context()
#  Socket to talk to server
print("Connecting to Hello World server")
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")

#  Do 10 requests, waiting each time for a response
for request in range(100):
    print("Sending request %s" % request)

    img = np.random.rand(640, 480, 3).astype(np.uint8)
    msg = img
    # msg["image"] = [1,2,3]

    start_time = time.time()
    if serialization == "MSGPACK":
        msg_packed = msgpack.packb(msg)
        socket.send(msg_packed)
    elif serialization == "JSON":
        socket.send_json(msg)
    elif serialization == "ARRAY":
        send_array(socket, img)

    print("time to send", time.time() - start_time)

    #  Get the reply.
    message = socket.recv()
    print("elapsed until reply received", time.time() - start_time)

    time.sleep(1.0)
    print("Received reply %s [ %s ]" % (request, message))