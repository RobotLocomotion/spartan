# Vive Teleop
This is a guide to using the HTC vive headset and controllers to teleop the robot in simulation or using hardware.

# Prerequisites for VR computer
- Follow the [HTC Vive Installation Guide](https://support.steampowered.com/kb_article.php?ref=2001-UXCM-4439)
- Install [Steam](https://store.steampowered.com/about/) and SteamVR
- Clone spartan

# Prerequisites for robot computer (run on same computer if using simulation)
- Install spartan using docker

# Install required packages (VR computer)
Run the following commands to install required packages
```
sudo apt-get install libudev-dev libvulkan-dev libsdl2-dev libglfw3-dev libssl-dev zlib1g-dev python-pip ros-kinetic-tf ros-kinetic-tf2*

pip install openvr
```

Additionally, clone the [OpenVR repository](https://github.com/ValveSoftware/openvr) into a known location

# Install ROS Plugins using catkin (VR computer)
The Plugin is build using catkin. You have to create a catkin workspace first. Open up a terminal and:
```
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
```

Next clone the [RVIZ Plugin](https://github.com/jmcculloch2018/rviz_vive) and the [Controller Tracking Node](https://github.com/jmcculloch2018/htc_vive_teleop_stuff) into the src folder.

After cloning the RVIZ Plugin, open the *CMakeLists.txt* file in the *rviz_vive* folder and change the OpenVR path in line 30 to the location that you cloned OpenVR to.

Finally, build the plugins and source them
```
cd ..
catkin_make
source devel/setup.bash
```
# Run Vive Teleop Simulation
1. Start SteamVR and ensure that headset and controllers are tracking
2. In a new terminal, navigate to your spartan directory
  1. `./setup/docker/docker_run.py -nethost`
  2. In docker, start procman and run start simulation
  3. Through procman start the process *vive-teleop*
3. In a new terminal, navigate to your catkin workspace
  1. `source devel/setup.bash`
  2. `roslaunch htc_vive_teleop_stuff htc_vive_tf_and_joy.launch`
4. Close the instance of rviz opened by procman
5. In a new terminal, navigate to your catkin workspace
  1. `source devel/setup.bash`
  2. cd to your spartan folder and run `source build/setup_environment.sh`
  3. `rosrun rviz rviz -d $SPARTAN_SOURCE_DIR/src/catkin_projects/station_config/RLG_iiwa_1/rviz_vive.rviz`
6. Put on the vive headset and look around. Press and hold the trackpad to move the arm and hold the trigger to close the gripper.

# Run Vive Teleop on Robot
On robot computer:
1. Navigate to your spartan directory and run `./setup/docker/docker_run.py -nethost`
2. Run `export ROS_IP=128.30.27.155`
3. Start procman and set up robot normally
4. Through procman start the process *vive-teleop*

On VR computer:
1. Run `hostname -i` and look at the first entry to determine your ip address (should be 128.30.xx.xxx). Write this number anywhere you see YOUR_IP_HERE.
2. Start SteamVR and ensure that headset and controllers are tracking
3. In a new terminal, navigate to your catkin workspace
  1. `export ROS_MASTER_URI=http://128.30.27.155:11311`
  2. `export ROS_IP=YOUR_IP_HERE`
  3. `source devel/setup.bash`
  4. `roslaunch htc_vive_teleop_stuff htc_vive_tf_and_joy.launch`
4. Close the instance of rviz opened by procman
5. In a new terminal, navigate to your catkin workspace
  1. `export ROS_MASTER_URI=http://128.30.27.155:11311`
  2. `export ROS_IP=YOUR_IP_HERE`
  3. `source devel/setup.bash`
  4. cd to your spartan folder and run `source build/setup_environment.sh`
  5. `rosrun rviz rviz -d $SPARTAN_SOURCE_DIR/src/catkin_projects/station_config/RLG_iiwa_1/rviz_vive.rviz`
6. Put on the vive headset and look around. Press and hold the trackpad to move the arm and hold the trigger to close the gripper.
