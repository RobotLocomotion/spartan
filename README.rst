Spartan
=======

This is a project repository for students of the Robot Locomotion Group at MIT.
It contains work for robotics research and applications leveraging Drake and
Director.


Build instructions
==================

Configure and build with cmake.  For example:

    mkdir build
    cd build
    cmake ../
    make

There is no requirement on the location of the build directory, you don't
have to place it inside the source directory as shown in the above example.


Environment setup
=================

After you configure the build you will find a file named `setup_environment.sh`
inside the build folder.  You can source this file in your ~/.bashrc file to
setup your environment for development.  However, it is highly recommended that
you do not automatically source the file, as it may conflict with other projects.
Instead, you can add code like this to your ~/.bashrc file:

    use_spartan()
    {
      source /path/to/spartan/build/setup_environment.sh
    }

With this method, the environment file will be sourced when you execute the
command `use_spartan` in a terminal, but by default new terminals will be clean.

You should read the contents of `setup_environment.sh` to see what it does.
In addition to modifying your PATH and other variables, it also defines some
useful aliases for developers.


Testing
=======

The environment file defines some commands to run tests inside sub-projects:

    run_tests_drake
    run_tests_director

The above commands move into the build directory of the sub-project and run
its tests.  You can pass additional commands which will be given to the test
driver (ctest), for example, to print a list of available tests:

    run_tests_director -N

To run tests in verbose mode:

    run_tests_director -V

To run a specific test matching a name or regex:

    run_tests_director -R testPyDrakeIk

To run tests in parallel:

    run_tests_drake -j12


Drake and Director submodules
=============================

This project intends to track the master branches of these submodules, but the
submodules are updated manually and only on demand, so they may not be completely
up to date all the time.

It is ok to set the submodule reference to a personal branch hosted on a
personal fork, as long as the changes in the branch are on track to be merged
upstream in the near term, and as long as you are willing to rebase
your branch onto upstream master on a frequent basis.


Scripts
=======

You can add executable scripts to the scripts/bin folder.  These scripts will
appear in your PATH via the sourced environment file.
