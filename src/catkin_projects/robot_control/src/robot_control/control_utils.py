import rospy, time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robot_msgs.msg import *
from robot_msgs.srv import *
import rospy

convergence_epsilon = rospy.get_param('convergence_epsilon')
convergence_timout = rospy.get_param('convergence_timout')

def wait_for_convergence(self, timeout=None):
    rospy.sleep(0.05)
    if timeout is None:
        timeout = time.time() + convergence_timout
    else:
        timeout = time.time() + timeout
    while True:
        if time.time() > timeout:
            return False

        if self.robot_conf_state == 'converged':
            return True
        rospy.sleep(0.05)

def update_conf_state(robot_state, robot_conf, cmd_conf, isDict=True, fixedBase=True):
    if robot_state != 'converged':
        if not robot_conf: return robot_state
        errors = []
        print_error = ''
        iterator = cmd_conf
        if isDict and fixedBase: robot_conf.pop('robotBase', None)
        if not isDict: iterator = xrange(0, len(robot_conf))

        for j in iterator:
            e = abs(robot_conf[j] - cmd_conf[j])
            errors.append(e)
            print_error = print_error + '%.3g' % e + ' '
        # print 'Max joint error:', max(errors)
        print print_error

        if all(i < convergence_epsilon for i in errors):
            print 'Converged to commanded conf!'
            robot_state = 'converged'
    return robot_state

def conf_to_traj(conf, current, names): #Both are confDict
    traj = JointTrajectory()
    traj.joint_names = names.robotRightArm

    p = JointTrajectoryPoint()
    p.positions = current['robotRightArm']
    p.velocities, p.accelerations, p.effort = [0]*len(traj.joint_names), [0]*len(traj.joint_names), [0]*len(traj.joint_names)
    traj.points.append(p)


    p2 = JointTrajectoryPoint()
    p2.positions = conf['robotRightArm']
    p2.velocities, p2.accelerations, p2.effort = [0]*len(traj.joint_names), [0]*len(traj.joint_names), [0]*len(traj.joint_names)
    traj.points.append(p2)
    return traj
