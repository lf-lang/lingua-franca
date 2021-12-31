#!/bin/bash
pwd
cd $1
git fetch --all
git checkout $2
