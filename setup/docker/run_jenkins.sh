#! /bin/bash
cd /home/jenkins/spartan
rm -rf build
mkdir build
cd build

cmake ..
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
