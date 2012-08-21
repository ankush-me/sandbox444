# Defines CUTS and HOLES and the reference point-cloud

# Holes are a bunch of 3D points
Hole[] holes

# An array of cut is cuts. A cut is an array of 3D points.
Cut[] cuts

# The cloud on which the holes and cuts are defined
sensor_msgs/PointCloud2 cloud