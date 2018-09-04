# Schunk Driving

The Schunk power cable is a little barrel jack behind the robot. Plug it in when you start, unplug it at the end of the day.

The Schunk control page lives at http://192.170.10.20/ (or whatever the IP of your schunk is). Note the super-useful status panel at the right. If you find that you can't control it, pressing "STOP" and then "ACK" might help.

To control the Schunk from code, your best bet is to use `spartan/modules/spartan/manipulation/schunk_driver.py`. Import and use it in a Python ROS node, and you should be good to go.

## Some misc notes about wsg50-ros-pkg

The actual gripper driver is a submodule at `spartan/src/catkin_projects/wsg50-ros-pkg`. It has a complicated history (it's a [spartan-specific fork](https://github.com/gizatt/wsg50-ros-pkg) of the [Xamla fork](https://github.com/Xamla/wsg50-ros-pkg) of the [nalt WSG50 driver](https://github.com/nalt/wsg50-ros-pkg), which itself forks [a Robotnik WSG50 ROS driver](https://code.google.com/archive/p/wsg50-ros-pkg/). In reverse order:

- The nalt driver offers reasonable and reliable position control of the driver with continuous state reporting but does not allow on-the-fly max-force-setpoint control.
- The Xamla driver adds continuous max-force-setpoint control and formalizes the driver into two actionlib interfaces: `/wsg50_driver/wsg50/gripper_command` (GripperStandardActionServer, with commands of type `control_msgs::GripperCommandActionGoal`) and `/wsg50_driver/wsg50/gripper_control` (GripperActionServer, with commands of type `wsg_50_common::Command`).
- The RLG fork removes some of the Xamla-company-specific stuff, like a Xamla-specific heartbeat message, so that it builds in our ecosystem.

The Python Schunk control module *does not* use the actionlib interface, instead sending `wsg_50_common::CommandActionGoal` messages directly. This ought to be fixed up, but it generally works.

