#!/bin/bash
# Script to build all components from scratch, using the maximum available CPU power
#
# Given parameters are passed over to CMake.
# Examples:
#    * ./build_all.sh -DCMAKE_BUILD_TYPE=Release
#    * ./build_all.sh VERBOSE=1
#
# Written by Tiffany Huang, 12/14/2016
# Updated by Artem Artemev, 25/06/2017


# Go into the directory where this bash script is contained.
cd `dirname $0`

numproc=$([[ $(uname) == 'Darwin' ]] && sysctl -n hw.physicalcpu_max || nproc)

# Compile code.
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j ${numproc} $*
