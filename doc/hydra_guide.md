# Hydra Teleop

Make sure the Razer Hydra base station is on the desk in a reasonable orientation. (The current "safe zone" for the controller is a 30cm box "ahead" (opposite where the cables come out) and above of the base station.)

Launch the robot as usual, ensuring that under the ROS processes, the `razer_hydra` driver is launched. Make sure you plug in the Hydra *before* launching the Docker container, to ensure the USB device gets cloned into the container. (You should have a device `/dev/hydra` inside the container if you launched the docker container correctly.)

Open a new terminal in the container with ROS and Spartan sourced (`use_ros && use_spartan`), `cd ~/spartan/scripts/bin && python hydra_teleop.py`.

Now proceed with driving the robot! In its current iteration, controls work like this:

Pressing and holding the non-analog trigger (i.e. the "shoulder" button above the main trigger) starts movement, and releasing it stops it. Controller motion relative to the controller pose when the trigger is first pressed is replicated at the end effector of the robot, with **controller forward at start time corresponding to robot forward**. Because of that rotation, I recommend using this system with your body aligned with robot forward (i.e. stand behind the control computer).

Because the Razer Hydra uses a magnetic tracking system, its motion tracking is often very warped when LOS to the base station is interrupted by something heavy, or when the controller is too far or too close to the base station. For that reason, I added a "safety box" that the controller must remain inside for motion to happen. If you press the trigger and no motion happens, you probably started outside the box, so release the trigger and try again. Similarly, if you leave the box during motion, motion will stop, so release the trigger and recenter. The console prints 'Safe space violation' notices when you're outside the box, but they're not terribly information, so just try to feel it out until we can work out a better system.