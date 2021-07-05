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
#
# This create a binary RTI in the current working directory. Please put this in
# a directory that is on the path.

cmake_minimum_required(VERSION 3.12)
project(RTI VERSION 1.0.0 LANGUAGES C)

set(CMAKE_BUILD_TYPE Debug)
set(CoreLib ../../../core)

# check which system we are running on to select the correct platform support
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

# declare a new executable target and list all its sources
add_executable(RTI rti.c ${LF_PLATFORM_FILE})

# set the number of workers to enable threading
target_compile_definitions(RTI PUBLIC NUMBER_OF_WORKERS)

# find pthreads and link to it
find_package(Threads REQUIRED)
target_link_libraries(RTI Threads::Threads)

install(
    TARGETS RTI
    DESTINATION bin
)
