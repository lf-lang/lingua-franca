#!/bin/bash
lfc -o . -r CarlaIntersection.lf
src-gen/Intersection/Carla/CarlaIntersection/CarlaIntersection.c lf_carla/src/CarlaIntersection.cpp
pushd lf_carla
colcon build
popd
