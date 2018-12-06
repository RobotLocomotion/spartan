# Grasping

## Coordinate Convention
Shows the gripper palm frame and the gripper fingertip frame. They are aligned but offset along the x-direction.
![alt text](gripper_frames.png "Gripper Frames")


## Spartan Grasp
We use `spartan_grasp` as our grasp planner. See the [README](https://github.com/manuelli/spartan_grasp) for instructions on launching the service.

## Grasp Supervisor

The code for interfacing with grasp planners from `spartan` lives in [`grasp_supervisor.py`](https://github.com/RobotLocomotion/spartan/blob/master/modules/spartan/manipulation/grasp_supervisor.py).

- **calling spartan grasp:** Look at the function `request_spartan_grasp`
- **executing a grasp:** Look at function `execute_grasp`.
- **picking up a grasped object:** Look at function `pickup_object`.


Note if you want to call these from the Director python terminal then due to weird ROS threading issues you need to use the
`test_<function_name>` versions of these which just calls the underlying function in a `TaskRunner` process. For example to
use `request_spartan_grasp` you would do, in the Director python terminal

```
graspSupervisor.test_request_spartan_grasp()
````

#### Params
There are many parameters governing grasping behavior. They live in [`params.yaml`](https://github.com/RobotLocomotion/spartan/blob/master/src/catkin_projects/station_config/RLG_iiwa_1/manipulation/params.yaml)




