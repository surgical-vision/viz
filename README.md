viz
===

About 
-----

This project provides a robotic surgical visualization environment. It supports loading 3D meshes
and pose data (such as provided by a da Vinci robot), rendering the meshes in 2D and 3D viewports.
It also supports live editing of the estimated pose of the 3D models to allow calibration offsets from 
the tracking method to be calibrated out. 

Dependencies
------------

In all cases these are not strict minimum versions but merely the oldest version on which they have been tested to work.

* [OpenCV v2.3.1](http://opencv.org/downloads.html) 
* [Boost v1.48.0](http://www.boost.org/users/download/)
* [Cinder v0.8.5](https://github.com/cinder/Cinder)

Install guide
-------------

Cmake it.