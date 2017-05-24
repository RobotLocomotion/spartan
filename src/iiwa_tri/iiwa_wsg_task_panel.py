import functools

import director.objectmodel as om
from director import propertyset
from director import transformUtils
from director.tasks import robottasks
from director.tasks.taskuserpanel import TaskUserPanel

import iiwaplanning
import myplanner

class UpdateGraspTargetTask(robottasks.AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty(
            'Position', [0., 0., 0.], attributes=propertyset.PropertyAttributes(
                decimals=3, minimum=-1, maximum=1))
        properties.addProperty(
            'Orientation', [0.0, 0.0, 0.0],
            attributes=propertyset.PropertyAttributes(
                decimals=3, minimum=-360, maximum=360))
        properties.addProperty(
            'Dimensions', [0.1, 0.2, 0.3], attributes=propertyset.PropertyAttributes(
                decimals=3, minimum=0.001, maximum=1))


    def run(self):
        iiwaplanning.setBoxGraspTarget(self.properties.position,
                                       self.properties.orientation,
                                       self.properties.dimensions)
        iiwaplanning.addGraspFrames()


def getOffsets(dims):
    finger_length = 0.050
    x_offset = 0.0
    if (dims[0] / 2.0) > finger_length:
        x_offset = dims[0] / 4.0

    y_offset = 0.0
    if (dims[1] / 2.0) > finger_length:
        y_offset = dims[1] / 4.0

    z_offset = 0.0
    if (dims[2] / 2.0) > finger_length:
        z_offset = dims[2] / 4.0

    return (x_offset, y_offset, z_offset)


class IiwaWsgTaskPanel(TaskUserPanel):

    rigid_body_target_name = 'Target rigid body'
    rigid_body_base_name = 'Base rigid body'
    rigid_body_base_none = '(none)'
    place_target_name = 'Place target'

    def __init__(self, robotSystem, optitrack_vis):
        TaskUserPanel.__init__(self, windowTitle='Task Panel')

        robottasks.robotSystem = robotSystem
        iiwaplanning.init(robotSystem)
        self.planner = myplanner.MyPlanner(robotSystem, self.params)
        self.ui.imageFrame.hide()
        self.robotSystem = robotSystem
        # Robot toy dimensions
        self._default_target_dimensions = [0.06, 0.02, 0.09]
        # Water bottle dimensions
        #self._default_target_dimensions = [0.07, 0.07, 0.22]
        # Squishy ball dimensions
        #
        # TODO(sam.creasey): Why does the squishy ball generate such
        # weird / bound up plans at position 1?
        #self._default_target_dimensions = [0.06, 0.06, 0.06]


        self.addManualButton('add grasp frames', self.addGraspFrameFromList)
        self.addManualButton('plan pregrasp', self.planner.planPreGrasp)
        self.addManualButton('plan grasp', self.planner.planGrasp)

        self.params.addProperty(
            self.rigid_body_target_name, 0,
            attributes=propertyset.PropertyAttributes(enumNames=[""]))
        self.params.addProperty(
            self.rigid_body_base_name, 0,
            attributes=propertyset.PropertyAttributes(enumNames=[
                self.rigid_body_base_none]))
        optitrack_vis.connectRigidBodyListChanged(self.rigidBodyListChanged)
        self.optitrack_vis = optitrack_vis

        self.params.addProperty(
            'Frame 1', [0.8, 0.36, 0.27, 0., 0., 0.],
            attributes=propertyset.PropertyAttributes(singleStep=0.01))
        self.params.addProperty(
            'Frame 2', [0.8, -0.36, 0.27, 0., 0., 0.],
            attributes=propertyset.PropertyAttributes(singleStep=0.01))

        self.params.addProperty(
            self.place_target_name, 0,
            attributes=propertyset.PropertyAttributes(
                enumNames=['Frame 1', 'Frame 2']))
        self.addTasks()

    def rigidBodyListChanged(self, body_list):
        old_target_name = self.params.getPropertyEnumValue(
            self.rigid_body_target_name)
        self.params.setProperty(self.rigid_body_target_name, 0)
        self.params.setPropertyAttribute(self.rigid_body_target_name, 'enumNames',
                                         body_list)
        if old_target_name in body_list:
            self.params.setProperty(self.rigid_body_target_name, old_target_name)

        body_list = [self.rigid_body_base_none] + body_list
        old_base_name = self.params.getPropertyEnumValue(
            self.rigid_body_base_name)
        self.params.setProperty(self.rigid_body_base_name, 0)
        self.params.setPropertyAttribute(self.rigid_body_base_name, 'enumNames',
                                         body_list)
        # Because we will have started on '(none)'.
        assert old_base_name in body_list
        self.params.setProperty(self.rigid_body_base_name, old_base_name)

    def addGraspFrameFromList(self):
        target_name = self.params.getPropertyEnumValue(self.rigid_body_target_name)
        if len(target_name):
            obj = om.findObjectByName(target_name)
            dims = obj.getProperty('Dimensions')

            (x_offset, y_offset, z_offset) = getOffsets(dims)

            # In the frames we're dealing with here, the gripper is
            # facing along the X axis. Left/right/above/etc in the
            # notes below are for a box long dimension aligned with
            # world Y (so if it were directly in front of the arm it
            # would have to be gripped from the side/top).
            grasp_offsets = [
                # Approach the box from the right (when looking at the arm).
                ((x_offset, 0.0, 0.0), (-90, 180, 0)),
                # Approach from the left
                ((-x_offset, 0.0, 0.0), (-90, 0, 0)),
            ]

            target_lower = target_name.lower()
            allow_gripper_to_flip = False
            if target_lower.count('box') or target_lower.count('cube'):
                allow_gripper_to_flip = True

            if allow_gripper_to_flip:
                grasp_offsets += [
                    # Approach from right, gripper flipped
                    ((x_offset, 0.0, 0.0), (90, 180, 0)),
                    # Approach from left, gripper flipped
                    ((-x_offset, 0.0, 0.0), (90, 0, 0)),
                    ]

            allow_y_grasps = False
            if target_lower.count('box') or target_lower.count('cube'):
                allow_y_grasps = True
            if allow_y_grasps:
                grasp_offsets += [
                    # Attack from below, both gripper orientations.
                    ((0.0, y_offset, 0.0), (-90, 180, 90)),
                    ((0.0, y_offset, 0.0), (90, 180, 90)),
                    # Approach from above:
                    ((0.0, -y_offset, 0.0), (-90, 0, 90)),
                    ((0.0, -y_offset, 0.0), (90, 0, 90)),
                    ]

            allow_side_grasps = False
            if target_lower.count('cylinder') or target_lower.count('cube'):
                allow_side_grasps = True
            if allow_side_grasps:
                grasp_offsets =+ [
                    ((0.0, 0.0, z_offset), (-90, 90, 0)),
                    ((0.0, 0.0, -z_offset), (-90, -90, 0)),
                ]
                if allow_gripper_to_flip:
                    grasp_offsets =+ [
                        ((0.0, 0.0, z_offset), (90, 90, 0)),
                        ((0.0, 0.0, -z_offset), (90, -90, 0)),
                ]



            pregrasp_extra = 0.08
            pregrasp_offsets = [
                # Offset in the x direction (straight back)
                (-(dims[0] / 2.0 + pregrasp_extra), 0., 0.),
                # Offset along the z axis to slide along the box.
                (-(dims[0] / 2.0 + pregrasp_extra), 0., (dims[0] / 2.0 + pregrasp_extra)),
                ]

            if allow_y_grasps:
                pregrasp_offsets += [
                    (-dims[0] / 2.0, 0., -(dims[0] / 2.0 + pregrasp_extra)),
                    ]

            for i, grasp_offset in enumerate(grasp_offsets):
                for j, pregrasp_offset in enumerate(pregrasp_offsets):
                    iiwaplanning.makeGraspFrames(
                        obj, grasp_offset,
                        pregraspOffset=pregrasp_offset,
                        suffix=' %d' % ((i * 100) + j))
            self.planner.setAffordanceName(target_name)
            is_feasible = self.planner.selectGraspFrameSuffix()
            if not is_feasible:
                raise Exception('Infeasible task')

    def addGraspFrameFromProperty(self, name=None, use_grasped=False,
                                  offset=(0, 0, 0)):
        if name is None:
            name = self.params.getPropertyEnumValue(self.place_target_name)

        position = self.params.getProperty(name)
        xyz = [position[i] + offset[i] for i in xrange(0, 3)]
        rpy = position[3:6]
        dims = self._default_target_dimensions

        if use_grasped:
            obj = om.findObjectByName(self.planner.getAffordanceName())
            dims = obj.getProperty('Dimensions')
            # TODO(sam.creasey) this is wrong we should get the
            # rotation from somewhere.
            #dims = [dims[1], dims[2], dims[0]]
            dims = [dims[0], dims[2], dims[1]]
            object_to_world = obj.getChildFrame().transform
            grasp_to_world = om.findObjectByName(
                'grasp to world%s' % self.planner.getGraspFrameSuffix()).transform
            print "object to world", object_to_world.GetPosition(), object_to_world.GetOrientation()
            print "grasp to world", grasp_to_world.GetPosition(), grasp_to_world.GetOrientation()

        iiwaplanning.setBoxGraspTarget(xyz, rpy, dims)
        self.planner.setAffordanceName('box')
        (x_offset, _, _) = getOffsets(dims)
        print "dimensions", dims, "x_offset", x_offset
        self.planner.addBoxGraspFrames(
            graspOffset=([-x_offset, 0., dims[2]/2.], [0,0,0]))

    def addPostGrasp(self):
        obj = om.findObjectByName(self.planner.getAffordanceName())
        iiwaplanning.makePostGraspFrame(
            obj, 'grasp to world%s' % self.planner.getGraspFrameSuffix())


    def onPropertyChanged(self, propertySet, propertyName):
        print "property changed", propertyName, propertySet.getProperty(propertyName)

        if propertyName == self.rigid_body_base_name:
            rigid_body_base = self.params.getPropertyEnumValue(
                self.rigid_body_base_name)
            if rigid_body_base == self.rigid_body_base_none:
                base_transform = self.optitrack_vis.defaultOptitrackToWorld
            else:
                base_transform = om.findObjectByName(
                    rigid_body_base).getChildFrame().transform
            self.optitrack_vis.setRobotBaseTransform(base_transform)

    def addTasks(self):
        # some helpers
        self.folder = None
        def addTask(task, parent=None):
            parent = parent or self.folder
            self.taskTree.onAddTask(task, copy=False, parent=parent)

        def addFunc(name, func, parent=None):
            addTask(robottasks.CallbackTask(callback=func, name=name), parent=parent)

        def addFolder(name, parent=None):
            self.folder = self.taskTree.addGroup(name, parent=parent)
            return self.folder

        def addPlanAndExecute(name, planFunc):
            old_folder = self.folder
            addFolder(name, parent=self.folder)
            addFunc(name, planFunc)
            addTask(robottasks.DelayTask(name='wait', delayTime=0.25))
            addTask(robottasks.CheckPlanInfo())
            addFunc('execute', self.planner.commitManipPlan)
            addFunc('wait for execute', self.planner.waitForExecute)
            self.folder = old_folder


        addFolder('pick and place mocap->frame')
        addFunc('Target Rigid Body', self.addGraspFrameFromList)
        # TODO(sam.creasey) Figure out how best to back off here.
        addFunc('Target Backoff', self.addPostGrasp)
        addFunc('open gripper', self.planner.openGripper)
        addPlanAndExecute('plan pregrasp', self.planner.planPreGrasp)
        addPlanAndExecute('plan grasp', self.planner.planGrasp)
        addFunc('close gripper', self.planner.closeGripper)
        addTask(robottasks.DelayTask(name='wait', delayTime=1.0))
        addPlanAndExecute('plan backoff',
                          functools.partial(iiwaplanning.planReachGoal,
                                            goalFrameName='postgrasp to world',
                                            seedFromStart=True))
        addFunc('Target Frame',
                functools.partial(self.addGraspFrameFromProperty, name=None,
                                  use_grasped=True))
        addFunc('Target Backoff', self.addPostGrasp)
        addPlanAndExecute('plan prerelease', self.planner.planPreGrasp)
        addPlanAndExecute('plan release', self.planner.planGrasp)
        addFunc('open gripper', self.planner.openGripper)
        addPlanAndExecute('plan backoff',
                          functools.partial(iiwaplanning.planReachGoal,
                                            goalFrameName='postgrasp to world',
                                            seedFromStart=True))

        # addFolder('pick and place 2->1')
        # addFunc('Target Frame 2',
        #         functools.partial(self.addGraspFrameFromProperty, 'Frame 2'))
        # addPlanAndExecute('plan pregrasp', self.planner.planPreGrasp)
        # addPlanAndExecute('plan grasp', self.planner.planGrasp)
        # addFunc('close gripper', self.planner.closeGripper)
        # addTask(robottasks.DelayTask(name='wait', delayTime=1.0))
        # addPlanAndExecute('plan prerelease', self.planner.planPreGrasp)
        # addFunc('Target Frame 1',
        #         functools.partial(self.addGraspFrameFromProperty, 'Frame 1'))
        # addPlanAndExecute('plan prerelease', self.planner.planPreGrasp)
        # addPlanAndExecute('plan release', self.planner.planGrasp)
        # addFunc('open gripper', self.planner.openGripper)
