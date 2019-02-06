#! /bin/bash
. ~/spartan/setup/docker/entrypoint.sh
use_ros

cd ~/spartan
rm -rf build
mkdir build
cd build

cmake -DWITH_PERCEPTION:BOOL=ON -DWITH_BULLET3:BOOL=ON -DWITH_TRIMESH:BOOL=OFF -DWITH_SCHUNK_DRIVER:BOOL=ON -DWITH_ROS:BOOL=ON -DWITH_REACHABILITY_ANALYZER:BOOL=OFF ..
exit_status=$?
if [ ! $exit_status -eq 0 ]; then
  echo "Error code in CMake: " $exit_status
  exit $exit_status
fi

# That CMake build generates this environment configuration
# script, which includes some definitions the compilation requires
# to work.
# . setup_environment.sh
use_spartan

make -j8 --output-sync=target
exit_status=$?
if [ ! $exit_status -eq 0 ]; then
  echo "Error code in make: " $exit_status
  exit $exit_status
fi

# Try building *again* to ensure that re-installing various pieces doesn't
# break (see e.g. issue #159)
make -j8 -output-sync=target
exit_status=$?
if [ ! $exit_status -eq 0 ]; then
  echo "Error code in make: " $exit_status
  exit $exit_status
fi

# See if we can source everything OK.
# . setup_environment.sh
use_spartan

# Launch a fake X-server in the background
Xvfb :100 -screen 0 1280x1024x24 -ac +extension GLX +render -noreset &

# Run Spartan modules test.
# These tests *must* be run forked (as in, each test
# in its own process)
DISPLAY=:100 pytest --forked --junitxml results.xml ~/spartan/modules/spartan/test/
exit_status=$?
if [ ! $exit_status -eq 0 ]; then
  echo "Error code when running tests: " $exit_status
  exit $exit_status
fi