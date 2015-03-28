empty-room
==========

Research code related to the empty room project.

Folder layout:
--------------
* GAPS: The GAPS geometry library from Tom Funkhouser, Princeton University.
Contains various mesh and 3D geometry routines and data structures
This version is based on the version used in COS526 
http://www.cs.princeton.edu/courses/archive/fall12/cos526/homework.php
* data_grabber: HDR capture tool. Will capture a video stream of depth
and color images while cycling the exposure/gain to increase dynamic
range. Note that this requires a custom version of PCL to build.
* geometry_tests: Debugging tools to determine accuracy of sensors.
Contains a planarity tester and a plane perpendicularity tester.
* globalreg: A custom geometry-only pairwise and global registration
algorithm that enforces coplanarity and mutual perpendicularity on
detected planes in each point cloud.
* inverse_render: Primary module that takes in a mesh, a set of images
and the associated camera positions, and performs inverse rendering
and architectural analysis to extract the parameters of a room model.
* scripts: Matlab and python scripts useful in preparing synthetic
scenes for inverse rendering and for postprocessing inverse rendering
results. Also includes the user guide for preparing synthetic data for
inverse rendering.
* synthetic_preprocess: Tool to add noise and perturbations to synthetic
data.
* wallfinder [Obsolete]: Debugging visualizations for wallfinder used
in inverse rendering
