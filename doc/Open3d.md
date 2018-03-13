# Building Open3D

Follow the instructions [here](http://www.open3d.org/docs/getting_started.html). We need to ensure that cmake finds the appropriate Python version. See [this](http://www.open3d.org/docs/compilation.html#python-binding) link. You need to run

```
cmake -DPYTHON_EXECUTABLE:FILEPATH=/usr/bin/python ../src
```