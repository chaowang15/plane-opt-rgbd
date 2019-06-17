#!/bin/sh

if [ ! -d "bin" ]; then
	mkdir bin
fi

# Mesh partition
cd mesh_partition
if [ ! -d "build" ]; then
	mkdir build
fi
cd build
cmake ..
make
cp mesh_partition ../../bin
cd ../..

# Mesh visibility
cd mesh_visibility
if [ ! -d "build" ]; then
	mkdir build
fi
cd build
cmake ..
make
cp mesh_visibility ../../bin
cd ../..

# Image Blur Estimation
cd blur_estimation
if [ ! -d "build" ]; then
	mkdir build
fi
cd build
cmake ..
make
cp blur_estimation ../../bin
cd ../../

# Texture and geometry optimization
cd mesh_texture_opt
if [ ! -d "build" ]; then
	mkdir build
fi
cd build
cmake ..
make
cp mesh_texture_opt ../../bin
cd ../..
