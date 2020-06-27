import zmq
import pickle
import numpy


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


class ZMQControllerHelperREQ(object):
    """
    Handles REP duties
    """

    def __init__(self):
        self._context = zmq.Context()
        context = self._context

        # setup the sockets
        self._sockets = dict()

        # for streaming rgb images
        self._sockets['rgb'] = context.socket(zmq.REQ)
        self._sockets['rgb'].bind("tcp://localhost:5555")

        # for streaming depth images over
        self._sockets['depth'] = context.socket(zmq.REQ)
        self._sockets['depth'].bind("tcp://localhost:5556")

        # for streaming robot data over
        self._sockets['robot_data'] = context.socket(zmq.REQ)
        self._sockets['robot_data'].bind("tcp://localhost:5557")

        # add another socket for sending the plan across . . .

    def send_rgb(self, array):
        """Reads the rgb image"""
        return send_array(self._sockets['rgb'], array)

    def send_depth(self, array):
        """Reads the rgb image"""
        return send_array(self._sockets['depth'], array)

    def send_robot_data(self, data):
        """Reads the robot data"""
        return send_pyobj(self._sockets['robot_data'], data)

    def get_response(self):
        """
        Respond to rgb, depth sockets
        Send the controller data back over the 'robot_data' socket
        """

        # respond to the rgb, depth sockets
        # this is just to satisfy the REQ/REP protocol
        self._sockets['rgb'].recv()
        self._sockets['depth'].recv()
        data = recv_pyobj(self._sockets['robot_data'])
        return data