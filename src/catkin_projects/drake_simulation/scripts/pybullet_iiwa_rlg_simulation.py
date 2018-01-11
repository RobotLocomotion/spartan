#!/usr/bin/env python

# Given a configuration file (see this package's config folder for examples),
# simulates it.
# 
# Arguments:
#   config: Configuration file
#   rate: Desired real-time rate (will not run faster, might run slower)
# Press q to quit and r to restart the simulation.

import argparse
from copy import deepcopy
import math
import os
import pybullet
import threading
import time
import yaml

import lcm
from drake import lcmt_iiwa_command, lcmt_iiwa_status ,lcmt_robot_state

kIiwaUrdf = "${SPARTAN_SOURCE_DIR}/models/iiwa/iiwa_description/iiwa14_simplified_collision.urdf"
kSchunkUrdf = "${SPARTAN_SOURCE_DIR}/drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf"

IIWA_CONTROLLED_JOINTS = [
    "iiwa_joint_1",
    "iiwa_joint_2",
    "iiwa_joint_3",
    "iiwa_joint_4",
    "iiwa_joint_5",
    "iiwa_joint_6",
    "iiwa_joint_7",
]

def load_from_urdf_or_sdf(inp_path, position = [0, 0, 0], quaternion = [0, 0, 0, 1], fixed = True):
    full_path = os.path.expandvars(inp_path)
    ext = full_path.split(".")[-1]
    if ext == "urdf":
        print "Loading ", full_path, " as URDF in pos ", position
        return pybullet.loadURDF(full_path, basePosition=position, baseOrientation=quaternion, useFixedBase=fixed)
    elif ext == "sdf":
        print "Loading ", full_path, " as SDF in pos ", position
        x =pybullet.loadSDF(full_path)
        print x
        return x
    else:
        print "Unknown extension in path ", full_path, ": ", ext
        exit(-1)


class IiwaRlgSimulator():
    def __init__(self, config, timestep, rate):
        self.config = config
        self.timestep = timestep
        self.rate = rate

        self.iiwa_command_lock = threading.Lock()
        self.last_iiwa_position_command = [0.] * len(IIWA_CONTROLLED_JOINTS)

        # Set up command subscriber
        self.lc = lcm.LCM()
        self.iiwa_command_sub = self.lc.subscribe("IIWA_COMMAND", self.HandleIiwaCommand)        


    def ResetSimulation(self):
        pybullet.resetSimulation()

        pybullet.setGravity(0,0,-9.81)
        pybullet.setTimeStep(self.timestep)

        # Read in configuration file
        config = yaml.load(open(self.config))

        if config["with_ground"] == True:
            self.ground_id = load_from_urdf_or_sdf(os.environ["SPARTAN_SOURCE_DIR"] + "/build/bullet3/data/plane.urdf")
        else:
            self.ground_id = None

        # Load in the Kuka
        if config["robot"]:
            q0 = config["robot"]["base_pose"]
            position = q0[0:3]
            quaternion = pybullet.getQuaternionFromEuler(q0[3:6])
            self.kuka_id = load_from_urdf_or_sdf(kIiwaUrdf, position, quaternion, True)

        self.BuildJointNameToIdDict()
        self.BuildMotorIdList()

        # Models entry is a dictionary of model URDF strings
        model_dict = config["models"]
        self.object_ids = []

        # Add each model as requested
        for instance in config["instances"]:
            q0 = instance["q0"]
            position = q0[0:3]
            quaternion = pybullet.getQuaternionFromEuler(q0[3:8])
            fixed = instance["fixed"]
            self.object_ids.append(load_from_urdf_or_sdf(model_dict[instance["model"]], position, quaternion, fixed))

    def BuildJointNameToIdDict(self):
        num_joints = pybullet.getNumJoints(self.kuka_id)
        self.iiwa_joint_name_to_id = {}
        for i in range(num_joints):
          joint_info = pybullet.getJointInfo(self.kuka_id, i)
          self.iiwa_joint_name_to_id[joint_info[1].decode("UTF-8")] = joint_info[0]

    def BuildMotorIdList(self):
        self.iiwa_motor_id_list = [
            self.iiwa_joint_name_to_id[motor_name] for motor_name in IIWA_CONTROLLED_JOINTS
        ]

    def HandleIiwaCommand(self, channel, data):
        self.iiwa_command_lock.acquire()
        try:
            msg = lcmt_iiwa_command.decode(data)
            for i in range(msg.num_joints):
                self.last_iiwa_position_command[i] = msg.joint_position[i]
        except Exception as e:
            print "Exception ", e, " in lcm command handler"
        self.iiwa_command_lock.release()

    def GetIiwaTorqueCommand(self):
        self.iiwa_command_lock.acquire()
        command = deepcopy(self.last_iiwa_position_command)
        self.iiwa_command_lock.release()
        return command

    def PublishIiwaStatus(self):
        status_msg = lcmt_iiwa_status()
        status_msg.utime = time.time() * 1E6
        status_msg.num_joints = len(IIWA_CONTROLLED_JOINTS)
        # Get joint state info
        states = pybullet.getJointStates(self.kuka_id, self.iiwa_motor_id_list)
        positions = [states[i][0] for i in range(status_msg.num_joints)]
        velocities = [states[i][1] for i in range(status_msg.num_joints)]
        torques = [states[i][3] for i in range(status_msg.num_joints)]
        status_msg.joint_position_measured = positions
        status_msg.joint_velocity_estimated = velocities
        status_msg.joint_position_ipo = [0]*status_msg.num_joints
        status_msg.joint_torque_measured = torques
        status_msg.joint_torque_commanded = [0.]*status_msg.num_joints
        status_msg.joint_torque_external = [0.]*status_msg.num_joints

        self.iiwa_command_lock.acquire()
        status_msg.joint_position_commanded = deepcopy(self.last_iiwa_position_command)
        self.iiwa_command_lock.release()

        self.lc.publish("IIWA_STATUS", status_msg.encode())

    def RunSim(self):
        # Run simulation with time control
        start_time = time.time()
        sim_time = 0.0
        avg_sim_rate = -1.
        sim_rate_RC = 0.1 # RC time constant for estimating sim rate
        sim_rate_alpha = self.timestep / (sim_rate_RC + self.timestep) 
        last_print_time = time.time() - 100

        keep_going = True
        while 1:
            self.lc.handle_timeout(1)
            pybullet.setJointMotorControlArray(
                self.kuka_id,
                self.iiwa_motor_id_list,
                pybullet.POSITION_CONTROL,
                self.GetIiwaTorqueCommand()
                )

            # Do a sim step
            start_step_time = time.time()
            pybullet.stepSimulation()
            end_step_time = time.time()
            sim_time += self.timestep

            # Publish state
            self.PublishIiwaStatus()

            # Estimate sim rate
            this_sim_step_rate = self.timestep / (end_step_time - start_step_time)
            if avg_sim_rate < 0:
                # Initialization case
                avg_sim_rate = this_sim_step_rate
            else:
                avg_sim_rate = this_sim_step_rate * sim_rate_alpha + (1. - sim_rate_alpha) * avg_sim_rate

            # Sleep if we're running too fast
            elapsed = end_step_time - start_time
            target_sim_time = elapsed * self.rate
            if sim_time > target_sim_time:
                time.sleep(sim_time - target_sim_time)

            if time.time() - last_print_time > 0.1:
                last_print_time = time.time()
                print("Overall sim rate: ", sim_time / target_sim_time, ", current sim rate: ", avg_sim_rate, " at time ", sim_time)

            events = pybullet.getKeyboardEvents()
            for key in events.keys():
                if events[key] & pybullet.KEY_WAS_TRIGGERED:
                    if key == ord('q'):
                        return False
                    elif key == ord('r'):
                        print("Restarting")
                        return True

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("config", help="Configuration file to simulate.", type=str)
    parser.add_argument("-r", "--rate", help="Desired simulation rate (fraction of realtime)", type=float, default=1.0)
    parser.add_argument("-t", "--timestep", help="Simulation timestep", type=float, default=0.001)
    args = parser.parse_args()

    # Set up a simulation with a ground plane and desired timestep
    physicsClient = pybullet.connect(pybullet.GUI)#or pybullet.DIRECT for non-graphical version
    
    sim = IiwaRlgSimulator(args.config, args.timestep, args.rate)

    keep_simulating = True
    while keep_simulating:
        sim.ResetSimulation()
        keep_simulating = sim.RunSim()

