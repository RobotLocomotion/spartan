# system
import os
import numpy as np

# ROS
import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import tf2_ros
import rosbag
import actionlib


# spartan ROS
import spartan_grasp_msgs.msg
import spartan_grasp_msgs.srv

# spartan
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as rosUtils
from spartan.manipulation.schunk_driver import SchunkDriver

# director
from director import transformUtils
from director import visualization as vis


USING_DIRECTOR = True
if USING_DIRECTOR:
    from spartan.utils.taskrunner import TaskRunner


class GraspSupervisor(object):

    def __init__(self, storedPosesFile=None, cameraSerialNumber=1112170110, tfBuffer=None):
        self.storedPoses = spartanUtils.getDictFromYamlFilename(storedPosesFile)
        self.cameraSerialNumber = cameraSerialNumber

        self.cameraName = 'camera_' + str(cameraSerialNumber)
        self.pointCloudTopic = '/' + str(self.cameraName) + '/depth/points'
        self.graspFrameName = 'base'
        self.depthOpticalFrameName = self.cameraName + "_depth_optical_frame"

        self.robotService = rosUtils.RobotService(self.storedPoses['header']['joint_names'])

        self.usingDirector = True
        self.tfBuffer = tfBuffer # don't create a new one if it is passed in
        self.setupConfig()

        if USING_DIRECTOR:
            self.taskRunner = TaskRunner()
            self.taskRunner.callOnThread(self.setup)
        else:
            self.setup()

    def setup(self):
        self.setupSubscribers()
        self.setupTF()
        self.setupROSActions()
        self.gripperDriver = SchunkDriver()
        self.homePose = self.storedPoses[self.config['home_pose_name']]

    def setupDirector(self):
        self.taskRunner.callOnThread(self.setup)


    def setupConfig(self):
        self.config = dict()
        self.config['base_frame_id'] = "base"
        self.config['end_effector_frame_id'] = "iiwa_link_ee"
        self.config['pick_up_distance'] = 0.15 # distance to move above the table after grabbing the object
        self.config['scan'] = dict()
        self.config['scan']['pose_list'] = ['scan_left', 'scan_right']
        self.config['scan']['joint_speed'] = 60
        self.config['grasp_speed'] = 20

        normal_speed = 30
        self.config['speed'] = dict()
        self.config['speed']['stow'] = normal_speed
        self.config['speed']['pre_grasp'] = normal_speed
        self.config['speed']['grasp'] = 10
        
        self.config['home_pose_name'] = 'above_table_pre_grasp'
        self.config['grasp_nominal_direction'] = np.array([1,0,0]) # x forwards
        self.config['grasp_to_ee'] = dict()

        self.config['grasp_to_ee']['translation'] = dict()
        # self.config['grasp_to_ee']['translation']['x'] = 9.32362425e-02
        self.config['grasp_to_ee']['translation']['x'] = 0.085
        self.config['grasp_to_ee']['translation']['y'] = 0
        self.config['grasp_to_ee']['translation']['z'] = 0

        self.config['grasp_to_ee']['orientation'] = dict()
        self.config['grasp_to_ee']['orientation']['w'] = 0.97921432
        self.config['grasp_to_ee']['orientation']['x'] = -0.20277454
        self.config['grasp_to_ee']['orientation']['y'] = 0.00454233
        self.config['grasp_to_ee']['orientation']['z'] = -0.00107904

        self.graspToIiwaLinkEE = spartanUtils.transformFromPose(self.config['grasp_to_ee'])
        self.iiwaLinkEEToGraspFrame = self.graspToIiwaLinkEE.GetLinearInverse()

        pos = [-0.15, 0, 0]
        quat = [1,0,0,0]
        self.preGraspToGraspTransform = transformUtils.transformFromPose(pos, quat)

    def setupSubscribers(self):
        self.pointCloudSubscriber = rosUtils.SimpleSubscriber(self.pointCloudTopic, sensor_msgs.msg.PointCloud2)

        self.pointCloudSubscriber.start()

    def setupROSActions(self):

        actionName = '/spartan_grasp/GenerateGraspsFromPointCloudList'
        self.generate_grasps_client = actionlib.SimpleActionClient(actionName, spartan_grasp_msgs.msg.GenerateGraspsFromPointCloudListAction)
        # self.generate_grasps_client.wait_for_server()

        # goal = spartan_grasp_msgs.msg.GenerateGraspsFromPointCloudListGoal(pointCloudListMsg)
        # client.send_goal(goal)
        # client.wait_for_result()

        # result = client.get_result()
        # print "num scored_grasps = ", len(result.scored_grasps)

        # print "result ", result


    def setupTF(self):
        if self.tfBuffer is None:
            self.tfBuffer = tf2_ros.Buffer()
            self.tfListener = tf2_ros.TransformListener(self.tfBuffer)


    def getDepthOpticalFrameToGraspFrameTransform(self):
        depthOpticalFrameToGraspFrame = self.tfBuffer.lookup_transform(self.graspFrameName, self.depthOpticalFrameName, rospy.Time(0))

        print depthOpticalFrameToGraspFrame
        return depthOpticalFrameToGraspFrame


    """
    Captures the current PointCloud2 from the sensor. Also records the pose of camera frame.
    """
    def capturePointCloudAndCameraTransform(self, cameraOrigin = [0,0,0]):
    	# sleep to transforms can update
    	rospy.sleep(0.5)
        msg = spartan_grasp_msgs.msg.PointCloudWithTransform()
        msg.header.stamp = rospy.Time.now()

        msg.camera_origin.x = cameraOrigin[0]
        msg.camera_origin.y = cameraOrigin[1]
        msg.camera_origin.z = cameraOrigin[2]

        msg.point_cloud_to_base_transform = self.getDepthOpticalFrameToGraspFrameTransform()

        msg.point_cloud = self.pointCloudSubscriber.waitForNextMessage()

        self.testData = msg # for debugging
        return msg

    def moveHome(self):
    	rospy.loginfo("moving home")
    	self.robotService.moveToJointPosition(self.homePose, maxJointDegreesPerSecond=self.config['scan']['joint_speed'])

    # scans to several positions
    def collectSensorData(self, saveToBagFile=False):

    	rospy.loginfo("collecting sensor data")

        pointCloudListMsg = spartan_grasp_msgs.msg.PointCloudList()
        pointCloudListMsg.header.stamp = rospy.Time.now()

        data = dict()

        for poseName in self.config['scan']['pose_list']:
            joint_positions = self.storedPoses[poseName]
            self.robotService.moveToJointPosition(joint_positions, maxJointDegreesPerSecond=self.config['scan']['joint_speed'])

            pointCloudWithTransformMsg = self.capturePointCloudAndCameraTransform()

            pointCloudListMsg.point_cloud_list.append(pointCloudWithTransformMsg)
            data[poseName] = pointCloudWithTransformMsg

        self.sensorData = data
        self.pointCloudListMsg = pointCloudListMsg
        return pointCloudListMsg

    """
    Returns true if a grasp was found
    """
    def processGenerateGraspsResult(self, result):
        print "num scored_grasps = ", len(result.scored_grasps)
        if len(result.scored_grasps) == 0:
            rospy.loginfo("no valid grasps found")
            return False

        self.topGrasp = result.scored_grasps[0]
        rospy.loginfo("-------- top grasp score = %.3f", self.topGrasp.score)
        self.graspFrame = spartanUtils.transformFromROSPoseMsg(self.topGrasp.pose.pose)
        self.rotateGraspFrameToAlignWithNominal(self.graspFrame)
        return True

    # def requestGrasp(self, pointCloudListMsg):

    # 	rospy.loginfo("requesting grasp from spartan_grasp")

    #     serviceName = 'spartan_grasp/GenerateGraspsFromPointCloudList'
    #     rospy.wait_for_service(serviceName)
    #     s = rospy.ServiceProxy(serviceName, spartan_grasp_msgs.srv.GenerateGraspsFromPointCloudList)
    #     response = s(pointCloudListMsg)

    #     print "num scored_grasps = ", len(response.scored_grasps)
    #     if len(response.scored_grasps) == 0:
    #     	rospy.loginfo("no valid grasps found")
    #     	return False

    #     self.topGrasp = response.scored_grasps[0]
    #     rospy.loginfo("-------- top grasp score = %.3f", self.topGrasp.score)
    #     self.graspFrame = spartanUtils.transformFromROSPoseMsg(self.topGrasp.pose.pose)
    #     self.rotateGraspFrameToAlignWithNominal(self.graspFrame)

    def getIiwaLinkEEFrameFromGraspFrame(self, graspFrame):
    	return transformUtils.concatenateTransforms([self.iiwaLinkEEToGraspFrame, graspFrame])

        # print "response ", response

    def moveToFrame(self, graspFrame, speed=None):
        if speed is None:
            speed = self.config['grasp_speed']
    	poseStamped = self.makePoseStampedFromGraspFrame(graspFrame)
    	return self.robotService.moveToCartesianPosition(poseStamped, speed)

    """
    Make PoseStamped message from a given grasp frame
    """
    def makePoseStampedFromGraspFrame(self, graspFrame):
        iiwaLinkEEFrame = self.getIiwaLinkEEFrameFromGraspFrame(graspFrame)
        poseDict = spartanUtils.poseFromTransform(iiwaLinkEEFrame)
        poseMsg = rosUtils.ROSPoseMsgFromPose(poseDict)
        poseStamped = geometry_msgs.msg.PoseStamped()
        poseStamped.pose = poseMsg
        poseStamped.header.frame_id = "base"

        return poseStamped


    """
    Attempt a grasp
    return: boolean if it was successful or not
    """
    def attemptGrasp(self, graspFrame):

    	preGraspFrame = transformUtils.concatenateTransforms([self.preGraspToGraspTransform, self.graspFrame])


        preGraspFramePoseStamped = self.makePoseStampedFromGraspFrame(preGraspFrame)
        preGrasp_ik_response = self.robotService.runIK(preGraspFramePoseStamped, seedPose=self.homePose, nominalPose=self.homePose)

        if not preGrasp_ik_response.success:
            rospy.loginfo("pre grasp pose ik failed, returning")
            return False

        graspFramePoseStamped = self.makePoseStampedFromGraspFrame(graspFrame)
        preGraspPose = preGrasp_ik_response.joint_state.position
        grasp_ik_response = self.robotService.runIK(graspFramePoseStamped, seedPose=preGraspPose, nominalPose=preGraspPose)

        if not  grasp_ik_response.success:
            rospy.loginfo("grasp pose not reachable, returning")
            return False

        # store for future use
        self.preGraspFrame = preGraspFrame
        self.graspFrame = graspFrame

        self.gripperDriver.sendOpenGripperCommand()
        self.robotService.moveToJointPosition(preGraspPose, maxJointDegreesPerSecond=self.config['speed']['pre_grasp'])
        self.robotService.moveToJointPosition(preGraspPose, maxJointDegreesPerSecond=self.config['speed']['grasp'])
    	
        objectInGripper = self.gripperDriver.closeGripper()
        return objectInGripper


    def vtkFrameToPoseMsg(self, vtkFrame):
        poseDict = spartanUtils.poseFromTransform(vtkFrame)
        poseMsg = rosUtils.ROSPoseMsgFromPose(poseDict)
        poseStamped = geometry_msgs.msg.PoseStamped()
        poseStamped.pose = poseMsg
        poseStamped.header.frame_id = "base"

        return poseStamped

    """
    Moves the gripper up 15cm then moves home
    """
    def pickupObject(self):
        endEffectorFrame = self.tfBuffer.lookup_transform(self.config['base_frame_id'], self.config['end_effector_frame_id'], rospy.Time(0))

        eeFrameVtk = spartanUtils.transformFromROSTransformMsg(endEffectorFrame.transform)
        eeFrameVtk.PostMultiply()
        eeFrameVtk.Translate(0,0,self.config['pick_up_distance'])

        vis.updateFrame( eeFrameVtk, 'pickup frame')

        
        poseStamped = self.vtkFrameToPoseMsg(eeFrameVtk)
        speed = 10 # joint degrees per second
        self.robotService.moveToCartesianPosition(poseStamped, speed)
        self.moveHome()
        self.robotService.moveToJointPosition(self.storedPoses['stow_in_bin'], maxJointDegreesPerSecond=self.config['speed']['stow'])
        self.gripperDriver.sendOpenGripperCommand()
        rospy.sleep(0.5)
        self.moveHome()
        

    def planGraspAndPickupObject(self):
        graspFound = self.testInThread()
        if not graspFound:
            rospy.loginfo("no grasp found, returning")
            return False

        graspSuccessful = self.attemptGrasp(self.graspFrame)
        if not graspSuccessful:
            rospy.loginfo("grasp not successful returning")
            return False


        self.pickupObject()

    def testMoveToFrame(self):
    	pos = [ 0.51148583,  0.0152224 ,  0.50182436]
    	quat = [ 0.68751512,  0.15384615,  0.69882778, -0.12366916]
    	targetFrame = transformUtils.transformFromPose(pos, quat)
    	poseDict = spartanUtils.poseFromTransform(targetFrame)
    	poseMsg = rosUtils.ROSPoseMsgFromPose(poseDict)


    	poseStamped = geometry_msgs.msg.PoseStamped()
    	poseStamped.pose = poseMsg
    	poseStamped.header.frame_id = "base"
    	self.poseStamped = poseStamped

    	self.robotService.moveToCartesianPosition(poseStamped, 30)


    def showGraspFrame(self):
    	vis.updateFrame(self.graspFrame, 'grasp frame', scale=0.15)
    	vis.updateFrame(self.getIiwaLinkEEFrameFromGraspFrame(self.graspFrame), 'iiwa_link_ee_grasp_frame', scale=0.15)

    def showGripperFrame(self):
        iiwaLinkEE = self.robotSystem.robotStateModel.getLinkFrame('iiwa_link_ee')
        gripperFrame = transformUtils.concatenateTransforms([self.graspToIiwaLinkEE, iiwaLinkEE])
        vis.updateFrame(gripperFrame, 'Gripper Frame', scale=0.15)

    """
	Rotate the grasp frame to align with the nominal direction. In this case we want the ZAxis of the 
	grasp to be aligned with (1,0,0) in world frame. If it's not aligned rotate it by 180 degrees about
	the x-Axis of the grasp
    """
    def rotateGraspFrameToAlignWithNominal(self, graspFrame):
    	graspFrameZAxis = graspFrame.TransformVector(0,0,1)
    	if (np.dot(graspFrameZAxis, self.config['grasp_nominal_direction']) < 0):
    		graspFrame.PreMultiply()
    		graspFrame.RotateX(180)


    def saveSensorDataToBagFile(self, pointCloudListMsg=None, filename=None, overwrite=True):
        if pointCloudListMsg is None:
            pointCloudListMsg = self.pointCloudListMsg

        if filename is None:
            filename = os.path.join(spartanUtils.getSpartanSourceDir(), 'logs', 'grasp_sensor_data.bag')

        if overwrite and os.path.isfile(filename):
            os.remove(filename)

        bag = rosbag.Bag(filename, 'w')
        bag.write('data', pointCloudListMsg)
        bag.close()

    def testInThread(self):
        self.collectSensorData()

        # request the grasp via a ROS Action
        rospy.loginfo("waiting for spartan grasp server")
        self.generate_grasps_client.wait_for_server()
        rospy.loginfo("requsting grasps spartan grasp server")
        goal = spartan_grasp_msgs.msg.GenerateGraspsFromPointCloudListGoal(self.pointCloudListMsg)
        self.generate_grasps_client.send_goal(goal)

        # while the grasp is processing moveHome

        self.moveHome()

        rospy.loginfo("waiting for result")
        self.generate_grasps_client.wait_for_result()
        result = self.generate_grasps_client.get_result()
        self.generate_grasps_result = result
        rospy.loginfo("received result")

        return self.processGenerateGraspsResult(result)
        

    def testMoveHome(self):
   		self.taskRunner.callOnThread(self.moveHome)

    def test(self):
        self.taskRunner.callOnThread(self.testInThread)

    def testAttemptGrasp(self):
    	self.taskRunner.callOnThread(self.attemptGrasp, self.graspFrame)

    def testPickupObject(self):
        self.taskRunner.callOnThread(self.pickupObject)

    def testPipeline(self):
        self.taskRunner.callOnThread(self.planGraspAndPickupObject)
   

    @staticmethod
    def makeDefault(**kwargs):
        storedPosesFile = os.path.join(spartanUtils.getSpartanSourceDir(), 'src', 'catkin_projects', 'station_config','RLG_iiwa_1','stored_poses.yaml')

        return GraspSupervisor(storedPosesFile=storedPosesFile, **kwargs)
