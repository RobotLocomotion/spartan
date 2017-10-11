Perception Sandbox
------------------

Example Catkin package using (at present) cv_bridge to view
camera images.

Remember to `use_ros` and THEN `use_spartan` (not other way around). If in doubt,
`use_spartan` again. And remember to `sudo chmod 666 /dev/video*` if you're in
doubt about video device use permissions. (Proper udev rules TODO(gizatt)).

You can launch just the image view node with `rosrun perception_sandbox opencv_view_node`
(assuming you have a roscore node running somewhere else, and images visible on
`/camera/rgb/image_raw`).

Or you can launch that node plus an R200 driver with
`roslaunch perception_sandbox realsense_camera_view`.