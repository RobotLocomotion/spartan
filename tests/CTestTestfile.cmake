# CMake generated Testfile for 
# Source directory: /home/drc/spartan/tests
# Build directory: /home/drc/spartan/tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_kuka_iiwa_app "/home/drc/spartan/scripts/bin/kuka_iiwa_app" "--testing")
add_test(test_pr2_app "/home/drc/spartan/apps/pr2/runapp.sh" "--testing")
