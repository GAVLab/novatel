Novatel GPS Driver
==================

This project provides a cross-platform interface for the Novatel OEM4 and OEMV series of GPS receivers.  The Novatel SPAN system is also supported.  

# Installation

## Dependencies

The GAVLAB Novatel driver relies on two external libraries.  One provides a small set of common string manipulation methods.  The other provides a cross patform serial port interface.  To install the needed dependencies:

### String Utils

	git clone git@github.com:davidhodo/string_utils.git
	cd string_utils
	make
	sudo make install
	
### Serial 

The serial library can be installed using two methods.  If you are using ROS with the Novatel driver, the serial library should be installed using Catkin following the instructions available at [https://github.com/wjwwood/serial](https://github.com/wjwwood/serial).  

An older version of the serial library can be installed without Catkin using the instructions below:

	git clone git://github.com/wjwwood/serial.git
	cd serial
	git checkout fuerte
	make
	sudo make install
	
## Novatel Library

Once the dependencies are met the Novatel library can be installed by:

	git clone git@github.com:GAVLab/novatel.git
	cd novatel
	make
	sudo make install
	
The above commands will build and install the Novatel static library.  To also build an example program for testing the interface:

	cd build
	cmake ../ -DNOVATEL_BUILD_EXAMPLES=ON
	make



# API

## Callback Definitions


## Supported Messages


