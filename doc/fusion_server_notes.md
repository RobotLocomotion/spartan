# Fusion Server Notes

**To make the fusion server work, you need to build the tsdf-fusion utility yourself by, after building spartan, running `cd ~/spartan/src/tsdf-fusion/ && ./compile.sh`.**

The fusion server (process 8.8 in the iiwa_hardware procman) offers a handful of services that you can call to trigger an automatic reconstruction of what's on the tabletop:

- /capture_scene
- /capture_scene_and_fuse
- /perform_elastic_fusion
- /start_bagging_fusion_data
- /stop_bagging_fusion_data

You could conceivably call `/start_bagging_fusion_data`, then navigate the robot around yourself, then call `/stop_bagging_fusion_data`, and then call `/perform_elastic_fusion`, but it's easier (and uses TSDF instead of elastic fusion, which is more reliable and better given our good camera calibration) to just use `/capture_scene_and_fuse`.

You can modify what `/capture_scene_and_fuse` does by tweaking `spartan/src/catkin_projects/fusion_server/src/fusion_server/fusion.py`. For example:

- You can change the poses it moves between by changing `config['scan']['pose_list']`, or by changing which pose list is used (e.g. swap from 'pose_list' to 'pose_list_quick') by tweaking the `move_robot_through_scan_poses` function.
- You can change the area that is reconstructed (to focus on e.g. the cutting board) by tweaking the `handle_capture_scene_and_fuse` function. There's a commented out changed call to `tsdf_fusion.run_tsdf_fusion_cuda` that does a higher-res reconstruction focused on just the cutting board in the center of the robot table.