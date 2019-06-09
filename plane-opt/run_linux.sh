#!/bin/sh

# This is the original RGB-D data including color images, depth images and camera pose files
# RGBD="/home/chao/dev/data/bundlefusion/copyroom/copyroom/"
RGBD="/home/chao/dev/data/bundlefusion/office3/office3/"

# Working directory contains the input PLY model. And all output files will be put inside this directory.
# WORKINGDIR="/home/chao/dev/data/bundlefusion/copyroom/test/"
WORKINGDIR="/home/chao/dev/data/bundlefusion/office3/"

# PLY filename
# PLYNAME="test"
PLYNAME="office3"

# 0 for BundleFusion/3DLite data, 1 for ICL-NUIM data (image type, camera pose details are different)
DATATYPE=0

# Current path
CODEPATH="$(pwd)"

# Target cluster/plane number in mesh partition
CLUSTERNUM=2000

## Mesh partition and simplification
cd $WORKINGDIR
$CODEPATH/mesh_partition/build/mesh_partition $PLYNAME".ply" $CLUSTERNUM $PLYNAME"_c"$CLUSTERNUM".ply" $PLYNAME"_c"$CLUSTERNUM".txt"
# For debug only
# $CODEPATH/mesh_partition/build/mesh_partition --run_post_processing=false cluster2000_simp.ply cluster2000_simp.txt

echo "*******************************************"
echo "*******************************************"
# Run again on the last output to better simplify the mesh by approximately half number of edges left
$CODEPATH/mesh_partition/build/mesh_partition --run_post_processing=false --simplification_border_edge_ratio=0.5 $PLYNAME"_c"$CLUSTERNUM".ply" $PLYNAME"_c"$CLUSTERNUM".txt" $PLYNAME"_c"$CLUSTERNUM".ply" $PLYNAME"_c"$CLUSTERNUM".txt"

echo "*******************************************"
echo "*******************************************"

## Texture and Geometry Optimization
