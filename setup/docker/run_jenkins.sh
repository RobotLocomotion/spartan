#! /bin/bash
cd /home/jenkins/spartan
rm -rf build
mkdir build
cd build

. /opt/ros/kinetic/setup.bash
cmake -DWITH_PERCEPTION:BOOL=ON -DWITH_TRIMESH:BOOL=ON -DWITH_SCHUNK_DRIVER:BOOL=ON -DWITH_IIWA_DRIVER_RLG:BOOL=ON -DWITH_ROS:BOOL=ON -DWITH_REACHABILITY_ANALYZER:BOOL=ON ..
exit_status=$?
if [ ! $exit_status -eq 0 ]; then
  echo "Error code in CMake: " $exit_status
  exit $exit_status
fi

make -j8
exit_status=$?
if [ ! $exit_status -eq 0 ]; then
  echo "Error code in make: " $exit_status
  exit $exit_status
fi

# Try building *again* to ensure that re-installing various pieces doesn't
# break (see e.g. issue #159)
make -j8
exit_status=$?
if [ ! $exit_status -eq 0 ]; then
  echo "Error code in make: " $exit_status
  exit $exit_status
fi
