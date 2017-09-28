Spartan
=======

This is a project repository for robotics research and applications using
Drake_ and Director_.

.. _Drake: https://www.github.com/RobotLocomotion/drake
.. _Director: https://www.github.com/RobotLocomotion/director


Build instructions
==================

First, you should install the required dependencies to compile Drake and other
submodules. Follow the platform setup instructions in the Drake documentation::

    http://drake.mit.edu/from_source.html#mandatory-platform-specific-instructions

You will also need the appropriate dependencies for Director. Refer to the
Director `README`::

    https://github.com/RobotLocomotion/director/#dependencies

For Ubuntu 14.04, you may install a non-conservative set of dependencies for
Director by running the following script::

    sudo ./setup/ubuntu/14.04/install_prereqs.sh

For Ubuntu 16.04, you may install a non-conservative set of dependencies for
Director by running the following script::

    sudo ./setup/ubuntu/16.04/install_prereqs.sh


Make sure your submodules are up to date. From the top-level directory run::
    scripts/bin/setup_submodules.sh

You should avoid adding the ``--recursive`` flag to the git submodule command,
since Drake will automatically manage its recursive submodules at build time.

Next, create a new build directory and configure with cmake. For example::

    mkdir build
    cd build
    cmake ../

There is no requirement on the location of the build directory, you don't
have to place it inside the source directory as shown in the above example.

Finally, run the build::

    make

By default, cmake generates a Makefile, but it's possible to use other
build tools like ninja.

Building With Drivers
---------------------

Spartan has CMake options to include various proprietary drivers in the build.
The following CMake options and their corresponding drivers are supported:

-  ``WITH_IIWA_DRIVER``: drake-iiwa-driver_
-  ``WITH_SCHUNK_DRIVER``: drake-schunk-driver
-  ``WITH_OPTITRACK_DRIVER``: optitrack-driver_

.. _drake-iiwa-driver: https://github.com/RobotLocomotion/drake-iiwa-driver
.. _optitrack-driver: https://github.com/sammy-tri/optitrack-driver

Unless you are a member of the RobotLocomotion team, you will likely not have
the repository access required to download all the above libraries and should
leave these options disabled.

There is a workaround for building ``drake-iiwa-driver`` using a local version
of the ``kuka-fri`` proprietary driver. By default, ``drake-iiwa-driver`` pulls
in ``kuka-fri`` as a submodule from a private RobotLocomotion repo. To build
against a different version, follow these steps:

1. Clone ``drake-iiwa-driver`` to your local machine:

   ::

       git clone https://github.com/RobotLocomotion/drake-iiwa-driver

2. Delete the kuka-fri submodule.

   ::

       cd drake-iiwa-driver
       git rm kuka-fri

3. Extract your copy of the kuka-fri drivers, and apply patches according to the
   instructions in `drake-iiwa-driver/README.md`_.

4. Commit the changes and note the commit hash.

5. In the Spartan build directory, enable ``WITH_IIWA_DRIVER`` and reconfigure
   CMake. Two additional options will appear:

   -  ``IIWA_DRIVER_GIT_REPOSITORY``: Set to the clone of address for your local
      ``drake-iiwa-driver``.

   -  ``IIWA_DRIVER_GIT_TAG``: The (short) commit hash from above.

   An example config might be

   ::

       IIWA_DRIVER_GIT_REPOSITORY="file:///home/example/drake-iiwa-driver/"
       IIWA_DRIVER_GIT_TAG="a1b2c34"

6. Reconfigure CMake once more, and build.

   ::

       cd spartan/build
       cmake ..
       make

.. _drake-iiwa-driver/README.md: https://github.com/RobotLocomotion/drake-iiwa-driver/blob/master/README.md

Common Build Errors
-------------------

If you encounter an error such as::

    Target "RemoteTreeViewer" links to target "Eigen3::Eigen" but the
    target was not found.  Perhaps a find_package() call is missing for an
    IMPORTED target, or an ALIAS target is missing?

then reconfigure CMake with the flag ``-DWITH_ISSUE_5456_WORKAROUND=ON``.

If you encounter an error related to not being able to find ``eigen3`` as part of an apriltags build then the problem is that you don't have ``eigen3`` system intalled. Either ``apt-get install libeigen3-dev`` or set ``DUSE_APRILTAGS:BOOL=OFF`` in the top level ``CMakeLists.txt``.

Environment setup
=================

After you configure the build you will find a file named ``setup_environment.sh``
inside the build folder.  You can source this file in your ~/.bashrc file to
setup your environment for development.  However, it is highly recommended that
you do not automatically source the file, as it may conflict with other projects.
Instead, you can add code like this to your ~/.bashrc file::

    use_spartan()
    {
      source /path/to/spartan/build/setup_environment.sh
    }

With this method, the environment file will be sourced when you execute the
command ``use_spartan`` in a terminal, but by default new terminals will be clean.

You should read the contents of ``setup_environment.sh`` to see what it does.
In addition to modifying your PATH and other variables, it also defines some
useful aliases for developers.


LCM Multicast Setup
===================
Director relies on LCM for message passing. Since LCM uses UDP multicast a valid multicast route must always be defined. Follow the instructions `here
<http://lcm-proj.github.io/multicast_setup.html>`_ under the section "Using LCM on a Single Host." Basically you just need to run::

    sudo ifconfig lo multicast
    sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo


Testing
=======

You can run ``ctest`` in the build directory to run tests. Additionally, the
environment file adds some commands to run tests for sub-projects::

    run_tests_drake
    run_tests_director

The above commands move into the build directory of the sub-project and run
its tests.  You can pass additional arguments to the test driver (ctest). For
example, to print a list of available tests::

    run_tests_director -N

To run tests in verbose mode::

    run_tests_director -V

To run a specific test matching a name or regex::

    run_tests_director -R testPyDrakeIk

To run tests in parallel::

    run_tests_drake -j12

Test Failures
-------------
If you get an error during director tests related to an ``LCM Self Test`` then it is likely your network is not allowing LCM packets to return via loopback. See ``https://lcm-proj.github.io/multicast_setup.html``.


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
