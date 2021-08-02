#!/bin/bash
# usage: ./build-ROS-node name-of-lf-file-without-.lf name-of-ROS-package
lfc -r $1.lf
cp -R src-gen/$1/* $2/src/
mv $2/src/$1.c $2/src/$1.cpp
mv $2/src/core/platform/lf_linux_support.c $2/src/core/platform/lf_linux_support.cpp
pushd $2
colcon build --packages-select $2
popd
