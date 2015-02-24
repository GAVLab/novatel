cmake_minimum_required(VERSION 2.8.3)
project(novatel)


#######################################
## Check for ROS ##
#######################################

option(BUILD_WITH_ROS "Build ROS node and use catkin." ON)

if (NOT catkin_FOUND)
   SET(BUILD_WITH_ROS OFF)
endif(NOT catkin_FOUND)

if (BUILD_WITH_ROS)
  ## Find catkin macros and libraries
	## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
	## is used, also find other catkin packages
	## libraries found here are added to catkin_LIBRARIES and linked automatically
	find_package(catkin COMPONENTS serial roslib roscpp rosconsole tf gps_msgs nav_msgs sensor_msgs)
	
  ## LIBRARIES: libraries you create in this project that dependent projects also need
	## CATKIN_DEPENDS: catkin_packages dependent projects also need
	## DEPENDS: system dependencies of this project that dependent projects also need
	catkin_package(
	  INCLUDE_DIRS include
	  LIBRARIES novatel
	  CATKIN_DEPENDS serial roslib roscpp rosconsole tf gps_msgs nav_msgs sensor_msgs
	  DEPENDS Boost
	)
else()
	SET(CATKIN_PACKAGE_LIB_DESTINATION "${CMAKE_INSTALL_PREFIX}/lib")
	SET(CATKIN_PACKAGE_BIN_DESTINATION "${CMAKE_INSTALL_PREFIX}/bin")
	SET(CATKIN_PACKAGE_INCLUDE_DESTINATION "${CMAKE_INSTALL_PREFIX}/include")
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


if (CMAKE_BUILD_TYPE MATCHES Debug)
  set(LIB_NAME novatel_d)
  MESSAGE("Building DEBUG library ${LIB_NAME}")
else ()
  set(LIB_NAME novatel)
  MESSAGE("Building RELEASE library ${LIB_NAME}")
endif ()

# Declare a cpp library
add_library(${LIB_NAME}
  src/novatel.cpp
)

target_link_libraries(${LIB_NAME}
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES})

##############
## Examples ##
##############

option(NOVATEL_BUILD_EXAMPLES "Build all of the Novatel examples." OFF)


if(serial_FOUND)
  include_directories(${serial_INCLUDE_DIRS})
  list(APPEND NOVATEL_LINK_LIBS ${serial_LIBRARIES} pthread)
  if(UNIX AND NOT APPLE)
    list(APPEND NOVATEL_LINK_LIBS rt)
  endif()
else(serial_FOUND)
  message(FATAL_ERROR "Serial library was not found.")
endif(serial_FOUND)

if (NOVATEL_BUILD_EXAMPLES)
	# Declare a cpp executable
	add_executable(novatel_example examples/novatel_example.cpp)
	target_link_libraries(novatel_example 
                        ${LIB_NAME}
                        ${catkin_LIBRARIES} 
                        ${Boost_LIBRARIES})

  add_executable(novatel_read_from_file examples/novatel_read_from_file.cpp)
  target_link_libraries(novatel_read_from_file 
                        ${LIB_NAME}
                        ${catkin_LIBRARIES} 
                        ${Boost_LIBRARIES})

	add_executable(novatel_read_bestutm examples/novatel_read_bestutm.cpp)
	target_link_libraries(novatel_read_bestutm 
                        ${LIB_NAME}
                        ${catkin_LIBRARIES} 
                        ${Boost_LIBRARIES})
endif (NOVATEL_BUILD_EXAMPLES)

# Build ROS node
if (BUILD_WITH_ROS)
	# Declare a cpp executable
	add_executable(novatel_node src/novatel_node.cpp)
  add_dependencies(novatel_node ${catkin_EXPORTED_TARGETS})
	target_link_libraries(novatel_node novatel ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endif (BUILD_WITH_ROS)


#############
## Install ##
#############


# Mark executables and/or libraries for installation
if(BUILD_WITH_ROS)
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
else(BUILD_WITH_ROS) # do a classical install
  install(TARGETS ${LIB_NAME}
    ARCHIVE DESTINATION /usr/local/lib
    LIBRARY DESTINATION /usr/local/lib
    RUNTIME DESTINATION /usr/local/bin/${PROJECT_NAME}
  )
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION /usr/local/include/${PROJECT_NAME}
  )
  set(CMAKE_MODULE_PATH ${CMAKE_ROOT}/Modules)
  message(STATUS "module path: " ${CMAKE_MODULE_PATH})
  install(FILES Findnovatel.cmake DESTINATION ${CMAKE_MODULE_PATH})
endif (BUILD_WITH_ROS)


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
