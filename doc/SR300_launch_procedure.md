# SR300 Launch Procedure
You cannot launch the SR300 driver from inside the docker. The current solution is to launch from outside the docker. Note that for messages to make it into the docker you must run with ` -p "--network=host" ` option for `docker_run`.

## Setting up realsense camera driver on host machine
On the host machine you need to install
 1. `ros-kinetic-desktop`
 2. `ros-kinetic-realsense-camera`
 3. Follow instructions [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md) in the `Video4Linux backend preparation` section.
 4. You may need to reboot your system for changes to take effect.
 
 
 ## Launching driver
 ```
 roslaunch realsense_camera sr300_nodelet_rgbd.launch
 ```
 
