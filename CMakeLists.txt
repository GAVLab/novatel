## Project Setup
cmake_minimum_required(VERSION 2.4.6)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(novatel)

## Configurations

# Use clang if available
IF(EXISTS /usr/bin/clang)
  set(CMAKE_CXX_COMPILER /usr/bin/clang++)
  set(CMAKE_OSX_DEPLOYMENT_TARGET "")
  set(CMAKE_CXX_FLAGS "-ferror-limit=5")
  set(CMAKE_BUILD_TYPE Debug)
ENDIF(EXISTS /usr/bin/clang)

option(NOVATEL_BUILD_TESTS "Build all of the Novatel tests." OFF)
option(NOVATEL_BUILD_EXAMPLES "Build all of the Novatel examples." OFF)

# Allow for building shared libs override
IF(NOT BUILD_SHARED_LIBS)
    set(BUILD_SHARED_LIBS OFF)
ENDIF(NOT BUILD_SHARED_LIBS)

# Set the default path for built executables to the "bin" directory
IF(NOT DEFINED(EXECUTABLE_OUTPUT_PATH))
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
ENDIF(NOT DEFINED(EXECUTABLE_OUTPUT_PATH))
# set the default path for built libraries to the "lib" directory
IF(NOT DEFINED(LIBRARY_OUTPUT_PATH))
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ENDIF(NOT DEFINED(LIBRARY_OUTPUT_PATH))

## Configure the build system

# Add the include folder to the include path
include_directories(${PROJECT_SOURCE_DIR}/include /usr/include/c++/4.6)

# Add default source files
set(NOVATEL_SRCS src/novatel.cpp)
# Add default header files
set(NOVATEL_HEADERS include/novatel/novatel.h 
					include/novatel/novatel_enums.h
                    include/novatel/novatel_structures.h)

# Find Boost, if it hasn't already been found
IF(NOT Boost_FOUND OR NOT Boost_SYSTEM_FOUND OR NOT Boost_FILESYSTEM_FOUND OR NOT Boost_THREAD_FOUND)
    find_package(Boost COMPONENTS system filesystem thread REQUIRED)
ENDIF(NOT Boost_FOUND OR NOT Boost_SYSTEM_FOUND OR NOT Boost_FILESYSTEM_FOUND OR NOT Boost_THREAD_FOUND)

link_directories(${Boost_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

set(NOVATEL_LINK_LIBS ${Boost_SYSTEM_LIBRARY}
                      ${Boost_FILESYSTEM_LIBRARY}
                      ${Boost_THREAD_LIBRARY})

# Find serial, if it hasn't already been found
IF(NOT serial_FOUND)
    find_package(serial REQUIRED)
ENDIF(NOT serial_FOUND)

if(serial_FOUND)
  include_directories(${serial_INCLUDE_DIRS})
  list(APPEND NOVATEL_LINK_LIBS ${serial_LIBRARIES})
else(serial_FOUND)
  message(FATAL_ERROR "Serial library was not found.")
endif(serial_FOUND)

## Build the xbow Library

# Compile the Library
add_library(novatel ${NOVATEL_SRCS} ${NOVATEL_HEADERS})
target_link_libraries(novatel ${NOVATEL_LINK_LIBS})

## Build Examples

# If asked to
IF(NOVATEL_BUILD_EXAMPLES)
    # Compile the novatel Test program
    add_executable(novatel_example examples/novatel_example.cpp)
    # Link the Test program to the novatel library
    target_link_libraries(novatel_example novatel)
ENDIF(NOVATEL_BUILD_EXAMPLES)

## Build tests

# If asked to
IF(NOVATEL_BUILD_TESTS)
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
ENDIF(NOVATEL_BUILD_TESTS)

## Setup install and uninstall

# Unless asked not to...
IF(NOT NOVATEL_DONT_CONFIGURE_INSTALL)
    # Configure make install
    IF(NOT CMAKE_INSTALL_PREFIX)
        SET(CMAKE_INSTALL_PREFIX /usr/local)
    ENDIF(NOT CMAKE_INSTALL_PREFIX)
    
    INSTALL(TARGETS novatel
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
    )
    
    INSTALL(FILES include/novatel/novatel.h include/novatel/novatel_structures.h
            DESTINATION include/novatel)
    
    IF(NOT CMAKE_FIND_INSTALL_PATH)
        set(CMAKE_FIND_INSTALL_PATH ${CMAKE_ROOT})
    ENDIF(NOT CMAKE_FIND_INSTALL_PATH)
    
    # INSTALL(FILES Findnovatel.cmake DESTINATION ${CMAKE_FIND_INSTALL_PATH}/Modules/)
    
    ADD_CUSTOM_TARGET(uninstall @echo uninstall package)
    
    IF (UNIX)
      ADD_CUSTOM_COMMAND(
        COMMENT "uninstall package"
        COMMAND xargs ARGS rm < install_manifest.txt
        
        TARGET  uninstall
      )
    ELSE(UNIX)
      ADD_CUSTOM_COMMAND(
        COMMENT "uninstall only implemented in unix"
        TARGET  uninstall
      )
    ENDIF(UNIX)
ENDIF(NOT NOVATEL_DONT_CONFIGURE_INSTALL)