## Camera calibration in Spartan

---

### Prepare to capture images

- Decide the device's serial number or name
- Create a new folder in `station_config/hand_eye_calibration` and create a new `.yaml` file.  Copy over the structure from another yaml file.
- Specify in `iiwamanipdev.py` what the name of this `.yaml` file is within `station_config`
- Create a new folder in `camera_config` and copy over the structure from other folders
- Rebuild catkin_project/fast
```
cd spartan/build
make catkin_projects/fast
```
- When you launch the sensor via `roslaunch camera_config openni2.launch`, pass the serial_number you chose

---

### Capture images

- Place the calibration target (place the pieces of tape with `*`s on them next to each other)

#### Capture RGB images

- Launch director
- `cal.run()` in the Python terminal (it's f8)
- (by default, the rgb images are captured)
- Open rviz, and look at the camera topic to make sure the target is in frame (at least in most images)
- Images will be saved to, for example:
```
spartan/calibration_data/20180201-233350_rgb/
```

#### Capture IR images

- WARNING: for unknown reasons saving mono16 ir images to .png does not work.  Make sure to use `.bmp` instead.  You can specify this in the `station_config/*/*.yaml` file
- Cover the projector (make sure to not tape the lens!!! A business card folded over, and taped so that no tape touches lenses)
- Set up the IR illuminators (see pic below)

<p align="center">
  <img src="./kuka_ir_illuminators.jpg" width="450"/>
</p>

- View the IR images in rviz and make sure the IR illumination is good enough (it can be sensitive to the height and angle of the IR illuminator)
- `cal.run(captureRGB=False, captureIR=True)`
- Open rviz, and look at the camera topic to make sure the target is in frame (at least in most images)
- Images will be saved to, for example:
```
spartan/calibration_data/20180201-2335959_ir/
```

Remember to remove the projector-covering device!!

### Run calibration optimization on images

- Find the folder names of the `_rgb` and `_ir` images
- copy those folders (`cp -r`) into a subdirectory of spartan_grasp (for examples `spartan_grasp/sandbox`)
- Edit `test_run_camera_calibration.py` to point to the folder paths, and also edit the name and serail number of the camera
- `./test_run_camera_calibration.py`
- Calibration results will be stored in `spartan_grasp/sandbox/calibration_results`
- Copy the calibration results over back into `spartan` directory world (under `camera_config` -- copy the other folder structures)
- Remember to rebuild catkin_projects with `make catkin_projects/fast`

### Test the calibration quality

Here are a few good simple tests:

1. Verify in rviz that the frames (rgb and depth) generally look in the right spot

2. Point the depth sensor down at the table and verify it looks generally flat

3. Can even point robot at itself and see if the point cloud matches the urdf


TODO
- add picture of IR illuminators setup
- later: clean up station_config structure (not just one cal.yaml)
- extrinsics should live in station_config
- adjust joint limits
