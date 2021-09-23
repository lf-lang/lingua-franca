# This is a cmake build script providing a solution for compiling
# the RTI in this directory..
#
# Usage:
#
# To compile with cmake, run the following commands:
#
# $> mkdir build && cd build
# $> cmake ../
# $> make
# $> sudo make install
#
# This create a binary RTI in the current working directory. Please put this in
# a directory that is on the path.
#
# To enable DEBUG messages, use the following build commands instead:
# 
# $> mkdir build && cd build
# $> cmake -DCMAKE_BUILD_TYPE=DEBUG ../
# $> make
# $> sudo make install
#
# If you would like to go back to non-DEBUG mode, you would have to remove all
# contents of the `build` folder.

cmake_minimum_required(VERSION 3.12)
project(RTI VERSION 1.0.0 LANGUAGES C)

set(CoreLib ../../../core)

# Check which system we are running on to select the correct platform support
# file and assign the file's path to LF_PLATFORM_FILE
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  set(LF_PLATFORM_FILE ${CoreLib}/platform/lf_linux_support.c)
elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
  set(LF_PLATFORM_FILE ${CoreLib}/platform/lf_macos_support.c)
else()
  message(FATAL_ERROR "Your platform is not supported! RTI supports Linux and MacOS.")
endif()

include_directories(${CoreLib})
include_directories(${CoreLib}/platform)
include_directories(${CoreLib}/federated)

# Declare a new executable target and list all its sources
add_executable(RTI rti.c ${LF_PLATFORM_FILE})

IF(CMAKE_BUILD_TYPE MATCHES DEBUG)
    # Set the LOG_LEVEL to 4 to get DEBUG messages
    target_compile_definitions(RTI PUBLIC LOG_LEVEL=4)
ENDIF(CMAKE_BUILD_TYPE MATCHES DEBUG)

# Set the number of workers to enable threading
target_compile_definitions(RTI PUBLIC NUMBER_OF_WORKERS)

# Find threads and link to it
find_package(Threads REQUIRED)
target_link_libraries(RTI Threads::Threads)

install(
    TARGETS RTI
    DESTINATION bin
)
