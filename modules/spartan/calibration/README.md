# Camera calibration in Spartan


The goal of all the following steps is to produce the following files. Suppose your camera is named `<camera_name>` then the following proceduce will result in 3 files being created in the folder 
`spartan/src/catkin_projects/camera_config/data/<camera_name>/master`

- `rgb_camera_info.yaml` contains intrinsics for RGB camera
- `depth_camera_info.yaml` contains intrinsics for depth camera.
- `camera_info.yaml` contains extrinsics for both rgb and ir cameras.


See `carmine_1` as an example of a camera that has already been calibrated.

---
## Setup
### Calibration Target
There should already be a calibration target setup which looks like
<p align="center">
  <img src="./doc/checkerboard.jpg" width="450"/>
</p>

The pdf of this target is in `spartan/src/catkin_projects/camera_config/data/calibration_targets/check_7x6_108mm.pdf`. When sized to print on letter size paper the edge length of an individual square was measured to be 25.6 mm.

### Prepare to capture images

- Decide the device's serial number or name
- Create a new folder in `src/catkin_projects/station_config/<robot_name>/hand_eye_calibration` and create a new `.yaml` file.  Copy over the structure from another yaml file. `<robot_name>` should be something like `RLG_iiwa_1` or `RLG_iiwa_2`.
- Specify in `iiwamanipdev.py` what the name of this `.yaml` file is within `station_config`
- Create a new folder in `src/catkin_projects/camera_config/data` and copy over the structure from other folders
- Rebuild catkin_project/fast
```
cd spartan/build
make catkin_projects/fast
```
- When you launch the sensor via `roslaunch camera_config openni2.launch`, pass the serial_number/camera_name you chose.
- **Note**: You cannot stream RGB and IR at the same time due to usb 2.

## Intrinsics Calibration

### Intrinsics calibration with ROS Camera Calibration
First we do intrinsics.
- Open director, put the robot into the `Calibration - intrinsics calibtration pose`
- Open rviz, view the rgb topic, make sure you can walk around with the calibration plate in it

In order to calibration intrinsics for rgb camera, execute the following command, replacing `<camera_name>` with the name of your camera.
```
rosrun camera_calibration cameracalibrator.py --size 7x6 --square 0.0256 image:=/<camera_name>/rgb/image_raw camera:=/<camera_name>/rgb
```

To run the calibration for the ir camera execute
```
rosrun camera_calibration cameracalibrator.py --size 7x6 --square 0.0256 image:=/<camera_name>/ir/image camera:=/<camera_name>/ir
```
- A window will pop out, you should move the calibration plate around until the `CALIBRATE` button gets colored.
- Click the `CALIBRATE` button, the calibration process might take a few seconds, during which the window might gray out, but it is working
- After the calibration is done. Click the `SAVE` button, the calibrated results will be saved to `/tmp/calibrationdata.tar.gz`. Run the following command to move the camera info file to the right place
```
python intrinsics_calibration.py --rgb --camera_name <camera_name>
```

This will create the file `spartan/src/catkin_projects/camera_config/data/<camera_name>/master/rgb_camera_info.yaml` which
holds the RGB intrinsics.

And for IR do:
```
python intrinsics_calibration.py --ir --camera_name <camera_name>
```

This will create the file `spartan/src/catkin_projects/camera_config/data/<camera_name>/master/depth_camera_info.yaml` which 
stores the depth camera intrinsics.

Make sure to rebuild!

```
cd spartan/build
make catkin-projects/fast
```

You'll be sad if you don't rebuild.



---
## Extrinsics Calibration

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
  <img src="./doc/kuka_ir_illuminators.jpg" width="450"/>
</p>

- View the IR images in rviz and make sure the IR illumination is good enough (it can be sensitive to the height and angle of the IR illuminator)
- `cal.run(captureRGB=False, captureIR=True)`
- Open rviz, and look at the camera topic to make sure the target is in frame (at least in most images)
- Images will be saved to, for example:
```
spartan/calibration_data/20180201-2335959_ir/
```

Remember to remove the projector-covering device!!

### Optimize Extrinsics using Handical

Handical's main dependency gtsam, should be be correctly built during the docker build.

First add gtsam to your PYTHONPATH. To do this do the following, you can see these defined in `entrypoint.sh`.

```
use_spartan
use_handical
```

Now we need to actually build and install install handical.

```
cd ~/spartan/src/handical
mkdir build
cd build
cmake ..
make -j8
sudo make install -j8
```

Then navigate to handical's python interface:

```
cd ~/spartan/src/handical/python
```

In your favorite text editor, change the args at the top of the `run_handical_rlg.py` file to wherever you stored your data:

```
rgb_calibration_data_folder = os.path.join(spartan_source_dir, 'calibration_data', '20190329-001052_rgb')
depth_calibration_data_folder = os.path.join(spartan_source_dir, 'calibration_data', '20180223-205019_ir')
camera_info_filename_destination = "src/catkin_projects/camera_config/data/carmine_1/master/camera_info.yaml"
```

Then run!

```
python run_handical_rlg.py
```

Note there is a very hacky step where you need to change the location estimate to be a valid translation/quaternion pose format.

And then make catkin-projects:

```
cd ~/spartan/build
make catkin-projects/fast -j8
```

## Testing

### Test the calibration quality

DO THESE TESTS:

1. Verify in rviz that the frames (rgb and depth) generally look in the right spot

2. Point the depth sensor down at the table and verify it looks generally flat

3. Can even point robot at itself and see if the point cloud matches the urdf

4. In RVIZ open `image_rect_color` and verify that it doesn't look distorted. A good way to do that is to look at the checkerboard and edges of the table.


TODO
- later: clean up station_config structure (not just one cal.yaml)
- extrinsics should live in station_config
- adjust joint limits
