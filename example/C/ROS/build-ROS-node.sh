#!/bin/bash
# usage: ./build-ROS-node name-of-lf-file-without-.lf name-of-ROS-package
lfc -r $1.lf
cp -R src-gen/* $2/src/
mv $2/src/$1.c $2/src/$1.cpp
pushd $2
colcon build --packages-select $2
popd
