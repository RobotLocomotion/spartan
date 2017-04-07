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

For Ubuntu 16.04, you may install a non-conservative set of dependencies for
Director by running the following script::

    sudo ./setup/ubuntu/16.04/install_prereqs.sh

Make sure your submodules are up to date.  The recommended command is::

    git submodule update --init

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
