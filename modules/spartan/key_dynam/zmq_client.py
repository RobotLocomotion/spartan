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

from spartan.key_dynam.zmq_utils import ZMQClient

import msgpack_numpy as m
m.patch()


zmq_client = ZMQClient()

while True:

    print("sending message")
    msg = {'type': 'PLAN'}
    msg['data'] = {'pos': [0,1,2]}
    zmq_client.send_data(msg)
    zmq_client.recv_data()
    time.sleep(0.2)