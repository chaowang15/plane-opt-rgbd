#!/bin/sh

# Mesh partition
cd mesh_partition
if [ ! -d "build" ]; then
	mkdir build
fi
cd build
cmake ..
make
cd ../..
