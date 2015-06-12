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

Usage
-----

The examples directory contains an example model configuration file (in .json format) and an application configuration file.
Run the application as `> viz /path/to/config.cfg` or drag and drop the file into the window. You can run the video frames loading the 
pose data from the objects using the run button and also save the contents of each window using the appropriate button.
The example model configuration file contains the configuration for a da Vinci instrument. Unfortunately we cannot provide the CAD model
for this example but it gives a demonstration of how the components are specified and how each components DH parameters are specified.

Acknowledgements
----------------

If you use this software in your work, we would be grateful if you could acknowledge the authors

* [Max Allan](mailto:m.allan@cs.ucl.ac.uk), University College London
* [Philip Pratt](mailto:p.pratt@imperial.ac.uk), Imperial College London
