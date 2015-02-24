Novatel GPS Driver
==================
  
This project provides a cross-platform interface for the Novatel OEM4 and OEMV series of GPS receivers.  The Novatel SPAN system is also supported. 

The Novatel driver is written as a standlone library which depends on [Boost](http://http://www.boost.org) and a simple cross-platform serial port library [serial port library](https://github.com/wjwwood/serial).  It uses [Cmake](http://http://www.cmake.org) for the build system.  Example programs are provided that demonstrate the basic functionality of the library.

The driver can optionally be built using [Catkin](http://www.ros.org/wiki/catkin) and includes a [ROS](http://www.ros.org) node that reads from the sensor and publishes [NavSatFix](http://ros.org/doc/api/sensor_msgs/html/msg/NavSatFix.html) and [Odometry](http://ros.org/doc/api/nav_msgs/html/msg/Odometry.html) messages.    

# Installation 

## ROS Install

The serial and Novatel packages are both "wet" packages and require Catkin.  To build the libraries, first create a Catkin workspace (you can skip this step if you are adding the packages to an existing workspace.)

	mkdir -p ~/novatel_ws/src
	cd ~/novatel_ws/src
	catkin_init_workspace
	wstool init ./
	
Next, add the Serial, GPS Messages, and Novatel packages to the workspace:

	wstool set serial --git git@github.com:wjwwood/serial.git
	wstool set novatel --git git@github.com:GAVLab/novatel.git
	wstool set gps_msgs --git git@github.com:GAVLab/gps_msgs.git
	wstool update

Finally, build the packages:

	cd ../
	catkin_make


## Standalone Install

Although Catkin is the preferred build method, both packages can be installed without Catkin.  An older version of the serial library can be installed using the instructions below:

	git clone git://github.com/wjwwood/serial.git
	cd serial
	git checkout fuerte
	make
	sudo make install
	
Next, the Novatel library can be installed by:

	git clone git@github.com:GAVLab/novatel.git
	cd novatel
	make
	sudo make install
	
The above commands will build and install the Novatel static library.  To also build an example program for testing the interface:

	cd build
	cmake ../ -DNOVATEL_BUILD_EXAMPLES=ON
	make
	
To build the optional Novatel tests:

	cd build
	cmake ../ -DNOVATEL_BUILD_TESTS=ON
	make


# Operation




## Callback Definitions


## Supported Messages

# License

The BSD License

Copyright (c) 2013 David Hodo - Integrated Solutions for Systems / Auburn University

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# Authors

David Hodo <david.hodo@gmail.com>

Portions of this library are based on previous code by William Travis and Scott Martin.  Thanks to William Woodall for the serial library.
