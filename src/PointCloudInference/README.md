Doing Inference over Point Clouds
=======

This is a collection of experiments and utilities for performing inference
over point cloud information from RGB-D sensors.

Local Region Analysis
=======

I'm thinking of learning a probabilistic model that, given a representation
of point cloud returns and free space information for some local region of space,
generates the probability that the center of that space is occupied or free.

This style of model would be useful for denoising and outlier rejection, and might
work for generating novel point clouds (which would be very cool)! It'd provide
a strong prior for perception algorithms operating over point clouds, so long as
those algorithms could figure out how to use this information. Similar styles of inference
could be used predict the likelihood of object placement in the environment (e.g. it's
unlikely to find ANY object in a wall, or in the middle of the air, but very likely
on a table surface).


Old Ideas
=======

Note: I don't think this is a complete enough format, but here in case I want to expand it:

The unit of data that this will operate over is a `PointCloudLocalRegion`, stored as a YAML
file, containing fields:
- about: Raw-text notes about this file
- width: Full-edge width of the region, in meters
- N: Number of voxels in each dimension.
- occupancy_grid: The actual occupancy grid, compressed with zlib.