#!/bin/sh

# This is the original RGB-D data including color images, depth images and camera pose files
# RGBD="/home/chao/dev/data/bundlefusion/copyroom/copyroom/"
RGBD="/home/chao/dev/data/bundlefusion/office1/office1/"

# Working directory contains the input PLY model. And all output files will be put inside this directory.
# WORKINGDIR="/home/chao/dev/data/bundlefusion/copyroom/test/"
WORKINGDIR="/home/chao/dev/data/bundlefusion/office1/"

# PLY filename
# PLYNAME="test"
PLYNAME="office1"

# 0 for BundleFusion/3DLite data, 1 for ICL-NUIM data (image type, camera pose details are different)
DATATYPE=0

# Current path
CODEPATH="$(pwd)"

# Target cluster/plane number in mesh partition
CLUSTERNUM=2000

cd $WORKINGDIR

## Mesh partition and simplification
# echo ---------------------------------------
# $CODEPATH/mesh_partition/build/mesh_partition $PLYNAME".ply" $CLUSTERNUM $PLYNAME"_c"$CLUSTERNUM".ply" $PLYNAME"_c"$CLUSTERNUM".txt"
# # For debug only
# # $CODEPATH/mesh_partition/build/mesh_partition --run_post_processing=false cluster2000_simp.ply cluster2000_simp.txt
#
# # Run again on the last output to better simplify the mesh by approximately half number of edges left
# $CODEPATH/mesh_partition/build/mesh_partition --run_post_processing=false --simplification_border_edge_ratio=0.5 $PLYNAME"_c"$CLUSTERNUM".ply" $PLYNAME"_c"$CLUSTERNUM".txt" $PLYNAME"_c"$CLUSTERNUM".ply" $PLYNAME"_c"$CLUSTERNUM".txt"

## Mesh visibility
echo ---------------------------------------
$CODEPATH/mesh_visibility/build/mesh_visibility 

## Reset filenames of ICL-NUIM data to fit the input
if [ $DATATYPE == 1 ]; then
	for i in  $(seq $start $end)
	do
		echo "Copying frame $i"
		printf -v j "%06d" $i
		mv ${RGBD}"rgb/"$i.png ${RGBD}frame-$j.color.png
		mv ${RGBD}"rgb/"$i.pose.txt ${RGBD}frame-$j.pose.txt
		mv ${RGBD}"depth/"$i.png ${RGBD}frame-$j.depth.png
	done
    # NO info.txt file in ICL-NUIM data so we provide one
	cp $CODEPATH"ICL-NUIM-info.txt" $RGBD"info.txt"
fi

## Texture and Geometry Optimization
