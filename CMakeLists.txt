cmake_minimum_required(VERSION 2.8.3)
project(novatel)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
## libraries found here are added to catkin_LIBRARIES and linked automatically
find_package(catkin COMPONENTS serial roslib roscpp rosconsole tf gps_msgs)


#######################################
## Check for ROS ##
#######################################

option(BUILD_WITH_ROS "Build ROS node and use catkin." ON)

if (NOT catkin_FOUND)
	SET(BUILD_WITH_ROS OFF)
endif(NOT catkin_FOUND)

if (BUILD_WITH_ROS)
	## LIBRARIES: libraries you create in this project that dependent projects also need
	## CATKIN_DEPENDS: catkin_packages dependent projects also need
	## DEPENDS: system dependencies of this project that dependent projects also need
	catkin_package(
	  INCLUDE_DIRS include
	  LIBRARIES novatel
	  CATKIN_DEPENDS serial roslib roscpp rosconsole gps_msgs
	  DEPENDS Boost
	)
else()
	SET(CATKIN_PACKAGE_LIB_DESTINATION "${CMAKE_INSTALL_PREFIX}/lib")
	SET(CATKIN_PACKAGE_BIN_DESTINATION "${CMAKE_INSTALL_PREFIX}/bin")
	# TODO: need to find serial library here
	find_package(serial REQUIRED)
	set(catkin_INCLUDE_DIRS ${serial_INCLUDE_DIRS})
	set(catkin_LIBRARIES ${serial_LIBRARIES})
endif (BUILD_WITH_ROS)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Declare a cpp library
add_library(novatel
  src/novatel.cpp
)

target_link_libraries(novatel
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

option(NOVATEL_BUILD_EXAMPLES "Build all of the Novatel examples." ON)


if (NOVATEL_BUILD_EXAMPLES)
	# Declare a cpp executable
	add_executable(novatel_example examples/novatel_example.cpp)
	target_link_libraries(novatel_example novatel ${catkin_LIBRARIES} ${Boost_LIBRARIES})

	add_executable(novatel_read_bestutm examples/novatel_read_bestutm.cpp)
	target_link_libraries(novatel_read_bestutm novatel ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endif (NOVATEL_BUILD_EXAMPLES)

# Build ROS node
if (BUILD_WITH_ROS)
	# Declare a cpp executable
	add_executable(novatel_node src/novatel_node.cpp)
	target_link_libraries(novatel_node novatel ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endif (BUILD_WITH_ROS)


#############
## Install ##
#############


# Mark executables and/or libraries for installation
install(TARGETS novatel novatel_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

option(NOVATEL_BUILD_TESTS "Build all of the Novatel tests." OFF)


if (NOVATEL_BUILD_TESTS)
    # Find Google Test
    enable_testing()
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})

    # Compile the atrv Test program
    add_executable(novatel_tests tests/novatel_tests.cpp)
    # Link the Test program to the atrv library
    target_link_libraries(novatel_tests ${GTEST_BOTH_LIBRARIES}
                          novatel)

    add_test(AllTestsIntest_novatel novatel_tests)
endif (NOVATEL_BUILD_TESTS)
