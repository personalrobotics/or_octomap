or_octomap
==========

or_octomap is a collision checker and sensor system plugin for OpenRAVE, intended to allow OpenRAVE meshes to be collision checked against octrees. 

Two plugins are created:

`or_octomap` : A sensor system. When the "Enable" command is called, an OctomapServer is created which takes in new data from a point cloud. OpenRAVE objects can be filtered out of the octomap.

`or_octomap_checker`: A collision checker. `or_octomap` must be created and enabled first. Then, `or_octomap_checker` can be set as the collision checker. When this is done, all collision checks get tested against the octomap before they are tested against other OpenRAVE kinbodies.
