# Testing

Documentation on the different testing options available in `spartan`.

You can run `ctest` in the build directory to run tests. Additionally, the
environment file adds some commands to run tests for sub-projects

```
run_tests_drake
run_tests_director
```
   
The above commands move into the build directory of the sub-project and run
its tests.  You can pass additional arguments to the test driver (ctest). For
example, to print a list of available tests
```
run_tests_director -N
```

To run tests in verbose mode

```
run_tests_director -V
```
   
To run a specific test matching a name or regex

```
run_tests_director -R testPyDrakeIk
```
  
To run tests in parallel
```
run_tests_drake -j12
```
    

## Whole Stack Tests
The following tests launch the whole simulation stack. The tests involve moving the arm to different positions using
the various ROS services that are available. To run the tests

```
pyunit --forked modules/spartan/test/
```

Note the `--forked` keyword is critical to make sure that `procman` doesn't leave around any hanging processes.
