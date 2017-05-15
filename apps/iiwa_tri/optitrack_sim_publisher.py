#!/usr/bin/env python

# TODO(eric.cousineau): Replace with Naveen's optitrack simulator connection.
import sys
import os
import time

import lcm

# from director.tasks import basictasks

sys.path.append(os.path.join(os.environ['SPARTAN_SOURCE_DIR'], 'src/optitrack'))
from optitrack import optitrack_data_descriptions_t
from optitrack import optitrack_frame_t
from optitrack import optitrack_rigid_body_t
from optitrack import optitrack_rigid_body_description_t

def setattrs(obj, **kwargs):
    for (key, value) in kwargs.iteritems():
        setattr(obj, key, value)
    return obj

class SimpleBody:
    def __init__(self, name, xyz, quat):
        self.name = name
        self.xyz = xyz
        self.quat = quat

    def gen_msg(self, id, parent_id=0):
        body = setattrs(optitrack_rigid_body_t(),
            id = id,
            xyz = self.xyz,
            quat = self.quat,
            params = 0x01, # Tracking Value: True
            )
        desc = setattrs(optitrack_rigid_body_description_t(),
            id = id,
            parent_id = parent_id,
            name = self.name,
            )
        return (body, desc)


class OptitrackSimPublisher:
    def __init__(self):
        self.lc = lcm.LCM()

    def publishMocapData(self, t):
        frame = optitrack_frame_t()
        desc = optitrack_data_descriptions_t()

        bodies = [
            SimpleBody("base", [0., 0., 0.], [1., 0, 0, 0]),
            SimpleBody("test", [0.7, 0.2, 0.2], [1., 0, 0, 0]),
            ]
        
        n = len(bodies)
        frame.num_marker_sets = 0
        frame.num_rigid_bodies = n
        desc.num_rigid_bodies = n
        for (i, body) in enumerate(bodies):
            (body_msg, desc_msg) = body.gen_msg(i)
            frame.rigid_bodies.append(body_msg)
            desc.rigid_bodies.append(desc_msg)

        self.lc.publish('OPTITRACK_FRAMES', frame.encode())
        self.lc.publish('OPTITRACK_DATA_DESCRIPTIONS', desc.encode())

    def run(self):
        try:
            rate = Rate(0.1)
            while True:
                self.publishMocapData(rate.elapsed())
                rate.sleep()
        except KeyboardInterrupt:
            pass

class Rate:
    # Is there something akin to ros.Rate?
    def __init__(self, period):
        self.start = time.time()
        self.last_hit = self.start
        self.period = period

    def sleep(self):
        while time.time() - self.last_hit < self.period:
            time.sleep(self.period / 10.)
        self.last_hit += self.period

    def elapsed(self):
        return time.time() - self.start

if __name__ == "__main__":
    client = OptitrackSimPublisher()
    client.run()
