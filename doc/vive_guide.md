# Vive Teleop
This is a guide to using the HTC vive headset and controllers to teleop the robot in simulation or using hardware.

# Prerequisites
- Follow the [HTC Vive Installation Guide](https://support.steampowered.com/kb_article.php?ref=2001-UXCM-4439)
- Install [Steam](https://store.steampowered.com/about/) and SteamVR
- Install Spartan using docker and checkout branch vive-teleop

# Install required packages
Run the following commands to install required packages
```
sudo apt-get install libudev-dev libvulkan-dev libsdl2-dev libglfw3-dev libssl-dev zlib1g-dev python-pip ros-kinetic-tf ros-kinetic-tf2*

pip install openvr
```

Additionally, clone the [OpenVR repository](https://github.com/ValveSoftware/openvr) into a known location

# Install ROS Plugins using catkin
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
# Run Vive Telop
1. Start SteamVR and ensure that headset and controllers are tracking
2. Start spartan and run simulation or robot setup through procman
3. Source your catkin workspace and run `roslaunch htc_vive_teleop_stuff htc_vive_tf_and_joy.launch`
4. Close the instance of rviz opened by procman and start it outside the docker container
  - Source your catkin workspace
  - cd to your spartan folder and run `source build/setup_environment.sh`
  - Run `rosrun rviz rviz -d $SPARTAN_SOURCE_DIR/src/catkin_projects/station_config/RLG_iiwa_1/rviz_vive.rviz";`
5. Through procman start the process *vive-teleop*
