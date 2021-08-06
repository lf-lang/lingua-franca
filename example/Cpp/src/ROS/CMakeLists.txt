cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# require C++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# LF
# =======================
include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)
include(GNUInstallDirs)

set(DEFAULT_BUILD_TYPE "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE STRING "Choose the type of build." FORCE)
endif()

if(NOT REACTOR_CPP_BUILD_DIR)
  set(REACTOR_CPP_BUILD_DIR "" CACHE STRING "Choose the directory to build reactor-cpp in." FORCE)
endif()

ExternalProject_Add(
dep-reactor-cpp
  PREFIX "${REACTOR_CPP_BUILD_DIR}"
  GIT_REPOSITORY "https://github.com/tud-ccc/reactor-cpp.git"
  GIT_TAG "26e6e641916924eae2e83bbf40cbc9b933414310"
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_INSTALL_BINDIR:PATH=${CMAKE_INSTALL_BINDIR}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DREACTOR_CPP_VALIDATE=ON
    -DREACTOR_CPP_TRACE=OFF
    -DREACTOR_CPP_LOG_LEVEL=4
)

set(REACTOR_CPP_LIB_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
set(REACTOR_CPP_BIN_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}")
set(REACTOR_CPP_LIB_NAME "${CMAKE_SHARED_LIBRARY_PREFIX}reactor-cpp${CMAKE_SHARED_LIBRARY_SUFFIX}")
set(REACTOR_CPP_IMPLIB_NAME "${CMAKE_STATIC_LIBRARY_PREFIX}reactor-cpp${CMAKE_STATIC_LIBRARY_SUFFIX}")

add_library(reactor-cpp SHARED IMPORTED)
add_dependencies(reactor-cpp dep-reactor-cpp)
if(WIN32)
    set_target_properties(reactor-cpp PROPERTIES IMPORTED_IMPLIB "${REACTOR_CPP_LIB_DIR}/${REACTOR_CPP_IMPLIB_NAME}")
    set_target_properties(reactor-cpp PROPERTIES IMPORTED_LOCATION "${REACTOR_CPP_BIN_DIR}/${REACTOR_CPP_LIB_NAME}")
else()
    set_target_properties(reactor-cpp PROPERTIES IMPORTED_LOCATION "${REACTOR_CPP_LIB_DIR}/${REACTOR_CPP_LIB_NAME}")
endif()

if (APPLE)
  file(RELATIVE_PATH REL_LIB_PATH "${REACTOR_CPP_BIN_DIR}" "${REACTOR_CPP_LIB_DIR}")
  set(CMAKE_INSTALL_RPATH "@executable_path/${REL_LIB_PATH}")
else ()
  set(CMAKE_INSTALL_RPATH "${REACTOR_CPP_LIB_DIR}")
endif ()

set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)

set(LF_MAIN_TARGET BasicROS)
# =======================

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker
  src-gen/BasicROS/main.cc
  src-gen/BasicROS/BasicROS/BasicROS.cc
  src-gen/BasicROS/BasicROS/MessageGenerator.cc
  src-gen/BasicROS/BasicROS/MessageReceiver.cc
  src-gen/BasicROS/BasicROS/_lf_preamble.cc
)

target_include_directories(talker PUBLIC
  "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}"
  "${PROJECT_SOURCE_DIR}/src-gen/BasicROS"
  "${PROJECT_SOURCE_DIR}/src-gen/BasicROS/__include__"
  "${PROJECT_SOURCE_DIR}/install/cpp_sub/include/reactor-cpp"
)

target_link_libraries(talker reactor-cpp)

ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
