#!/bin/bash
lfc -o . -r CarlaIntersection.lf
cp -r src-gen/Intersection/Carla/CarlaIntersection/* lf_carla/src/
mv lf_carla/src/CarlaIntersection.c lf_carla/src/CarlaIntersection.cpp
mv lf_carla/src/core/platform/lf_linux_support.c lf_carla/src/core/platform/lf_linux_support.cpp
pushd lf_carla
colcon build
popd
