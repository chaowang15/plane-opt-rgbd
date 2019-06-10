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

# Mesh visibility
cd mesh_visibility
if [ ! -d "build" ]; then
	mkdir build
fi
cd build
cmake ..
make
cp ../*.vert .
cp ../*.frag .
cd ../..
