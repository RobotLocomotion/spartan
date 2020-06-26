import sys
import numpy as np
import time

# ROS
import rospy
import geometry_msgs.msg
import std_srvs.srv
import tf2_ros
import tf


# spartan
from spartan.utils.ros_utils import JointStateSubscriber
import spartan.utils.ros_utils as ros_utils
import spartan.utils.utils as spartan_utils
import spartan.utils.transformations as transformations
import spartan.utils.constants as constants
from spartan.manipulation.schunk_driver import SchunkDriver
# spartan ROS
import robot_msgs.msg
import geometry_msgs.msg

from teleop_mouse_manager import TeleopMouseManager
from imitation_tools.srv import *

from imitation_agent.deploy.software_safety import SoftwareSafety

DEBUG = False
VISUALIZE_FRAME = True
MOVE_HOME = True

def make_cartesian_gains_msg(kp_rot, kp_trans):
    msg = robot_msgs.msg.CartesianGain()

    msg.rotation.x = kp_rot
    msg.rotation.y = kp_rot
    msg.rotation.z = kp_rot

    msg.translation.x = kp_trans
    msg.translation.y = kp_trans
    msg.translation.z = kp_trans

    return msg

def make_force_guard_msg(scale):
    msg = robot_msgs.msg.ForceGuard()
    external_force = robot_msgs.msg.ExternalForceGuard()

    body_frame = "iiwa_link_ee"
    expressed_in_frame = "iiwa_link_ee"
    force_vec = scale*np.array([-1,0,0])

    external_force.force.header.frame_id = expressed_in_frame
    external_force.body_frame = body_frame
    external_force.force.vector.x = force_vec[0]
    external_force.force.vector.y = force_vec[1]
    external_force.force.vector.z = force_vec[2]

    msg.external_force_guards.append(external_force)

    return msg

def tf_matrix_from_pose(pose):
    trans, quat = pose
    mat = transformations.quaternion_matrix(quat)
    mat[:3, 3] = trans
    return mat

def do_main():
    rospy.init_node('simple_teleop', anonymous=True)

    # setup listener for tf2s (used for ee and controller poses)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    robotSubscriber = JointStateSubscriber("/joint_states")
    tf2_broadcaster = tf2_ros.TransformBroadcaster()
    tf_stamped = geometry_msgs.msg.TransformStamped()
    
    # wait until robot state found
    robotSubscriber = ros_utils.JointStateSubscriber("/joint_states")
    print("Waiting for full kuka state...")
    while len(robotSubscriber.joint_positions.keys()) < 3:
        rospy.sleep(0.1)
    print("got full state")

    # init gripper
    handDriver = SchunkDriver()
    

    # Start by moving to an above-table pregrasp pose that we know
    # EE control will work well from (i.e. far from singularity)

    stored_poses_dict = spartan_utils.getDictFromYamlFilename("../station_config/RLG_iiwa_1/stored_poses.yaml")
    above_table_pre_grasp = stored_poses_dict["Grasping"]["above_table_pre_grasp"]
    
    robotService = ros_utils.RobotService.makeKukaRobotService()

    if MOVE_HOME:
        success = robotService.moveToJointPosition(above_table_pre_grasp, maxJointDegreesPerSecond=30, timeout=5) # in sim, can do 60
        print("Moved to position")

        gripper_goal_pos = 0.0
        handDriver.sendGripperCommand(gripper_goal_pos, speed=0.1, timeout=0.01)
        print("sent close goal to gripper")
        time.sleep(2) # in sim, this can just be 0.1
        gripper_goal_pos = 0.1
        handDriver.sendGripperCommand(gripper_goal_pos, speed=0.1, timeout=0.01)
        print("sent open goal to gripper")
        time.sleep(0.5)

    gripper_goal_pos = 0.1
    frame_name = "iiwa_link_ee" # end effector frame name

    ee_tf_above_table = None
    for i in range(10):
        if i == 9:
            print "Couldn't find robot pose"
            sys.exit(0)
        try:
            ee_pose_above_table = ros_utils.poseFromROSTransformMsg(
                tfBuffer.lookup_transform("base", frame_name, rospy.Time()).transform)
            ee_tf_above_table = tf_matrix_from_pose(ee_pose_above_table)
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Troubling looking up robot pose...")
            time.sleep(0.1)

    # Then kick off task space streaming
    sp = rospy.ServiceProxy('plan_runner/init_task_space_streaming',
        robot_msgs.srv.StartStreamingPlan)
    init = robot_msgs.srv.StartStreamingPlanRequest()
    init.force_guard.append(make_force_guard_msg(20.))
    res = sp(init)
    
    print("Started task space streaming")
    pub = rospy.Publisher('plan_runner/task_space_streaming_setpoint',
        robot_msgs.msg.CartesianGoalPoint, queue_size=1)


    def cleanup():
        rospy.wait_for_service("plan_runner/stop_plan")
        sp = rospy.ServiceProxy('plan_runner/stop_plan',
            std_srvs.srv.Trigger)
        init = std_srvs.srv.TriggerRequest()
        print sp(init)
        print("Done cleaning up and stopping streaming plan")

    rospy.on_shutdown(cleanup)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(100) # max rate at which control should happen


    ee_tf_last_commanded = np.zeros((4,4))
    def get_initial_pose():
        while not rospy.is_shutdown():
            # get current tf from ros world to ee
            try:
                ee_pose_current = ros_utils.poseFromROSTransformMsg(
                    tfBuffer.lookup_transform("base", frame_name, rospy.Time()).transform)
                ee_tf_last_commanded = tf_matrix_from_pose(ee_pose_current)
                return ee_tf_last_commanded
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Troubling looking up robot pose...")
                rate.sleep()
                continue

    ee_tf_last_commanded = get_initial_pose()

    sys.path.append("../imitation_tools/scripts")
    from capture_imitation_data_client import start_bagging_imitation_data_client, stop_bagging_imitation_data_client
    
    if len(sys.argv) > 1 and sys.argv[1] == "--bag":
        bagging_started = start_bagging_imitation_data_client()
        if not bagging_started:
            raise ValueError("bagging failed to start")

        #time.sleep(0.5)
        #rospy.wait_for_service('save_scene_point_cloud', timeout=1.0)
        #save_scene_point_cloud = rospy.ServiceProxy('save_scene_point_cloud', SaveScenePointCloud)
        #resp1 = save_scene_point_cloud()
    
    pose_save_counter = 0
    saved_pose_dict = dict()
    saved_pose_counter = 0


    roll_goal = 0.0
    yaw_goal = 0.0
    pitch_goal = 0.0

    # software_safety = SoftwareSafety()
    # xyz = ee_tf_last_commanded[:3, 3]
    # rpy = np.asarray([roll_goal, pitch_goal, yaw_goal])
    #
    # software_safety.set_initial_goal(xyz, rpy)


    # init mouse manager
    mouse_manager = TeleopMouseManager()


    
    ee_tf_last_commanded = get_initial_pose()
    initial_pose = get_initial_pose() # T_W_E0
    T_W_E0 = initial_pose
    T_W_cmd0 = np.matmul(T_W_E0, constants.T_E_cmd)
    T_W_cmd = T_W_cmd0 # current command
    T_W_E = T_W_E0
    initial_quat = transformations.quaternion_from_matrix(initial_pose[:3,:3])
    try:

        # control loop
        while not rospy.is_shutdown():
          
            # # get teleop mouse
            events = mouse_manager.get_events()

            if events["r"]:
                success = robotService.moveToJointPosition(above_table_pre_grasp, timeout=3)
                roll_goal = 0.0
                yaw_goal = 0.0
                pitch_goal = 0.0
                ee_tf_last_commanded = get_initial_pose()
                res = sp(init)

            # if events["h"]:
            #     rospy.wait_for_service('save_hand_point_cloud', timeout=1.0)
            #     save_hand_point_cloud = rospy.ServiceProxy('save_hand_point_cloud', SaveHandPointCloud)
            #     for i in np.arange(0.0, 0.105, 0.005)[::-1]:
            #         handDriver.sendGripperCommand(i, speed=0.2, stream=True)
            #         time.sleep(0.4)
            #         resp1 = save_hand_point_cloud()
            #     time.sleep(2)
            #     gripper_goal_pos = 0.1
            #     handDriver.sendGripperCommand(gripper_goal_pos, speed=0.1, timeout=0.01)

            # if events["n"]:
            #     rospy.wait_for_service('save_scene_point_cloud', timeout=1.0)
            #     save_scene_point_cloud = rospy.ServiceProxy('save_scene_point_cloud', SaveScenePointCloud)
            #     resp1 = save_scene_point_cloud()

            pose_save_counter += 1
            if events["o"] and pose_save_counter >= 100: # this make it not happen to much
                joint_name_list = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']
                joint_position_vector = robotSubscriber.get_position_vector_from_joint_names(joint_name_list)
                print joint_position_vector
                print "joint positions saved"
                new_pose_name = "pose_"+str(saved_pose_counter).zfill(3)
                saved_pose_counter += 1
                saved_pose_dict[new_pose_name] = joint_position_vector.tolist()
                pose_save_counter = 0
                spartan_utils.saveToYaml(saved_pose_dict, "saved_poses.yaml")

            if events["escape"]:
                stop_bagging_imitation_data_client()
                if len(saved_pose_dict) > 0:
                    print("saving poses to disk")
                    spartan_utils.saveToYaml(saved_pose_dict, "saved_poses.yaml")
                sys.exit(0)
            
            scale_down = 0.0001
            delta_x = events["delta_x"]*scale_down
            delta_y = events["delta_y"]*-scale_down

            delta_forward = 0.0
            forward_scale = 0.001
            if events["w"]:
                delta_forward -= forward_scale
            if events["s"]:
                delta_forward += forward_scale

            # extract and normalize quat from tf
            delta_roll = 0.0
            delta_pitch = 0.0
            delta_yaw = 0.0

            if events["rotate_left"]:
                delta_pitch += 0.01
            if events["rotate_right"]:
                delta_pitch -= 0.01
            
            if events["side_button_back"]:
                yaw_goal += 0.01
                delta_yaw += 0.01
                print("side button back")
            if events["side_button_forward"]:
                yaw_goal -= 0.01
                delta_yaw -= 0.01
                print("side side_button_forward")

            yaw_goal = np.clip(yaw_goal, a_min = -1.314, a_max = 1.314)

            if events["d"]:
                delta_roll += 0.01
            if events["a"]:
                delta_roll -= 0.01

            pitch_goal = np.clip(pitch_goal, a_min = -1.314, a_max = 1.314)


            R_cmd_cmd_nxt = transformations.euler_matrix(delta_pitch, delta_roll, delta_yaw, 'syxz')[:3, :3]
            R_W_cmd_nxt = np.matmul(T_W_cmd[:3, :3], R_cmd_cmd_nxt)


            target_translation = np.asarray([delta_forward, delta_x, delta_y])
            T_W_cmd_nxt = np.eye(4)
            T_W_cmd_nxt[:3, 3] = T_W_cmd[:3, 3] + target_translation
            T_W_cmd_nxt[:3,:3] = R_W_cmd_nxt

            T_W_E_nxt = np.matmul(T_W_cmd_nxt, constants.T_cmd_E)


            # # above_table_quat_ee = transformations.quaternion_from_matrix(R.dot(ee_tf_above_table))
            # above_table_quat_ee = transformations.quaternion_from_matrix(ee_tf_above_table.dot(R))
            # above_table_quat_ee = np.array(above_table_quat_ee) / np.linalg.norm(above_table_quat_ee)
            #
            # # calculate controller position delta and add to start position to get target ee position
            # target_translation = np.asarray([delta_forward, delta_x, delta_y])
            # empty_matrx = np.zeros_like(ee_tf_last_commanded)
            # empty_matrx[:3, 3] = target_translation
            # ee_tf_last_commanded += empty_matrx
            # target_trans_ee = ee_tf_last_commanded[:3, 3]
            

            target_ee_pos = T_W_E_nxt[:3, 3]
            target_ee_quat = transformations.quaternion_from_matrix(T_W_E_nxt)
            if DEBUG:
                target_trans_ee = initial_pose[:3, 3]
                above_table_quat_ee = initial_quat

            # publish target pose as cartesian goal point
            new_msg = robot_msgs.msg.CartesianGoalPoint()
            new_msg.use_end_effector_velocity_mode = False
            new_msg.xyz_point.header.stamp = rospy.Time.now()
            new_msg.xyz_point.header.frame_id = "world" # should be a DRAKE frame
            new_msg.xyz_point.point.x = target_ee_pos[0]
            new_msg.xyz_point.point.y = target_ee_pos[1]
            new_msg.xyz_point.point.z = target_ee_pos[2]
            new_msg.xyz_d_point.x = 0.0
            new_msg.xyz_d_point.y = 0.0
            new_msg.xyz_d_point.z = 0.0
            new_msg.quaternion.w = target_ee_quat[0]
            new_msg.quaternion.x = target_ee_quat[1]
            new_msg.quaternion.y = target_ee_quat[2]
            new_msg.quaternion.z = target_ee_quat[3]


            # not sure what these should be exactly
            # new_msg.roll = roll_goal
            # new_msg.pitch = pitch_goal
            # new_msg.yaw = yaw_goal
            new_msg.gain = make_cartesian_gains_msg(5., 10.)
            new_msg.ee_frame_id = frame_name

            # software_safety.sys_exit_if_not_safe(new_msg)
            pub.publish(new_msg)

            T_W_cmd = T_W_cmd_nxt
            T_W_E = T_W_E_nxt


            if VISUALIZE_FRAME:
                tf_stamped.header.stamp = rospy.Time.now()
                tf_stamped.header.frame_id = "base"
                tf_stamped.child_frame_id = "simple_teleop_frame"
                tf_stamped.transform.translation.x = new_msg.xyz_point.point.x
                tf_stamped.transform.translation.y = new_msg.xyz_point.point.y
                tf_stamped.transform.translation.z = new_msg.xyz_point.point.z
                
                tf_stamped.transform.rotation.x = new_msg.quaternion.x
                tf_stamped.transform.rotation.y = new_msg.quaternion.y
                tf_stamped.transform.rotation.z = new_msg.quaternion.z
                tf_stamped.transform.rotation.w = new_msg.quaternion.w

                tf2_broadcaster.sendTransform(tf_stamped)


                # fingertip frame
                tf_stamped.header.stamp = rospy.Time.now()
                tf_stamped.header.frame_id = "base"
                tf_stamped.child_frame_id = "simple_teleop_fingertip_frame"
                tf_stamped.transform.translation.x = T_W_cmd[:3, 3][0]
                tf_stamped.transform.translation.y = T_W_cmd[:3, 3][1]
                tf_stamped.transform.translation.z = T_W_cmd[:3, 3][2]


                quat_tmp = transformations.quaternion_from_matrix(T_W_cmd)
                tf_stamped.transform.rotation.w = quat_tmp[0]
                tf_stamped.transform.rotation.x = quat_tmp[1]
                tf_stamped.transform.rotation.y = quat_tmp[2]
                tf_stamped.transform.rotation.z = quat_tmp[3]


                tf2_broadcaster.sendTransform(tf_stamped)



            #gripper
            if events["mouse_wheel_up"]:
                gripper_goal_pos += 0.006
            if events["mouse_wheel_down"]:
                gripper_goal_pos -= 0.006
            if gripper_goal_pos < 0:
                gripper_goal_pos = 0.0
            if gripper_goal_pos > 0.1:
                gripper_goal_pos = 0.1
            
            handDriver.sendGripperCommand(gripper_goal_pos, speed=0.2, stream=True)

            rate.sleep()



    except Exception as e:
        print "Suffered exception ", e

if __name__ == "__main__":
    do_main()