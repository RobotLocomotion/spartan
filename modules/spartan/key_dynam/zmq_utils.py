import zmq
import pickle
import numpy
import msgpack
import msgpack_numpy as m
m.patch()

def send_pyobj(socket, data):
    pickle_data = pickle.dumps(data)
    socket.send(pickle_data)
    # socket.send_pyobj(data)

def recv_pyobj(socket):
    pickle_data = socket.recv()
    data = pickle.loads(pickle_data, encoding='latin1')
    return data

def send_array(socket, A, flags=0, copy=True, track=False):
    """send a numpy array with metadata"""
    md = dict(
        dtype = str(A.dtype),
        shape = A.shape,
    )
    socket.send_json(md, flags|zmq.SNDMORE)
    return socket.send(A, flags, copy=copy, track=track)

def recv_array(socket, flags=0, copy=True, track=False):
    """recv a numpy array"""
    md = socket.recv_json(flags=flags)
    msg = socket.recv(flags=flags, copy=copy, track=track)
    buf = memoryview(msg)
    A = numpy.frombuffer(buf, dtype=md['dtype'])
    return A.reshape(md['shape'])


class ZMQClient(object):

    def __init__(self, port=5555):
        """
        Setup the sockets and connect
        :param port:
        """
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.connect("tcp://localhost:%d" %(port))

    def send_data(self,
                  data, # dict, only non-primitive type is numpy.array
                  ):
        """
        Send data to server over ZMQServer

        Strings are always a problem https://stackoverflow.com/questions/36735773/interoperability-problems-python2-python3
        :param data:
        :return:
        """
        self._socket.send(msgpack.packb(data))

    def recv_data(self):
        """
        Receives the response from the server over socket
        Returns the data
        :return:
        """

        data_raw = self._socket.recv()
        return msgpack.unpackb(data_raw)

