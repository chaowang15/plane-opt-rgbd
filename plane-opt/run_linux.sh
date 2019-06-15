#!/bin/sh

# This is the original RGB-D data including color images, depth images and camera pose files
RGBD=/home/chao/dev/data/bundlefusion/copyroom/copyroom
# RGBD=/home/chao/dev/data/ICL-NUIM/of_kt2/rgbd
# RGBD=/home/chao/dev/data/bundlefusion/office2/office2

# Working directory contains the input PLY model. All output files will be put inside this directory.
WORKINGDIR=/home/chao/dev/data/bundlefusion/copyroom
# WORKINGDIR=/home/chao/dev/data/bundlefusion/office2
# WORKINGDIR=/home/chao/dev/data/ICL-NUIM/of_kt2

# PLY filename
PLYNAME=copyroom
# PLYNAME=office2
# PLYNAME=of_kt2

# 0 for BundleFusion/3DLite data, 1 for ICL-NUIM data (image type, camera pose details are different)
DATATYPE=0
# DATATYPE=1

# Target cluster/plane number in mesh partition
CLUSTERNUM=2000

# Start and end frame index
START=0
# START=1 # lr_kt2 or of_kt2 starts from frame 1 instead of 0
END=4479 # copyroom
# END=100 # for test
# END=6158
# END=880 # lr_kt2 or of_kt2
# END=3499 # office2
# END=3819 # office3


ROOTPATH="$(pwd)"
CODEPATH=$ROOTPATH/bin
cd $WORKINGDIR

# ## Mesh partition and simplification
# echo ---------------------------------------
# $CODEPATH/mesh_partition $PLYNAME".ply" $CLUSTERNUM $PLYNAME"_c"$CLUSTERNUM"_ini.ply" $PLYNAME"_c"$CLUSTERNUM"_ini.txt"
# # For debug only
# # $CODEPATH/mesh_partition --run_post_processing=false cluster2000_simp.ply cluster2000_simp.txt
# #
# # Run again on the last output to better simplify the mesh. This will be fast.
# # NOTE: simplification_border_edge_ratio is the target percentage of border edges left after simplification.
# echo ---------------------------------------
# $CODEPATH/mesh_partition --run_post_processing=false --simplification_border_edge_ratio=0.1 $PLYNAME"_c"$CLUSTERNUM"_ini.ply" $PLYNAME"_c"$CLUSTERNUM"_ini.txt" $PLYNAME"_c"$CLUSTERNUM".ply" $PLYNAME"_c"$CLUSTERNUM".txt"
#
# ## Reset filenames of ICL-NUIM data to fit the input
# if [ $DATATYPE = 1 ]; then
#     echo ---------------------------------------
#     echo Resetting filenames for ICL-NUIM dataset
# 	for i in  $(seq $START $END)
# 	do
#         printf -v j "%06d" $i # NOTE to run by 'bash ./run_linux.sh' instead of 'sh' command, otherwise there would be error about the -v option
# 		mv $RGBD/rgb/$i.png $RGBD/frame-$j.color.png
# 		mv $RGBD/depth/$i.png $RGBD/frame-$j.depth.png
#         # NOTE: you need to pre-put all poses files (filename like 1.pose.txt) into the RGBD folder
#         mv $RGBD/$i.pose.txt $RGBD/frame-$j.pose.txt
# 	done
#     rm -rf $RGBD/rgb
#     rm -rf $RGBD/depth
#     # NO info.txt file in ICL-NUIM data so we provide one
# 	cp $ROOTPATH/ICL-NUIM-info.txt $RGBD/info.txt
# fi
#
# ## Mesh visibility
# echo ---------------------------------------
# # Create a new folder 'visibility' to store visibility files
# if [ ! -d "visibility" ]; then
# 	mkdir visibility
# fi
# cp $ROOTPATH/mesh_visibility/*.vert .
# cp $ROOTPATH/mesh_visibility/*.frag .
# $CODEPATH/mesh_visibility -v $PLYNAME"_c"$CLUSTERNUM".ply" $RGBD visibility $START $END
# rm *.vert
# rm *.frag
#
# ## Image blur estimation
# echo ---------------------------------------
# if [ $DATATYPE = 0 ]; then
#     $CODEPATH/blur_estimation $RGBD $START $END blur.txt
# elif [ $DATATYPE = 1 ]; then
#     $CODEPATH/blur_estimation $RGBD $START $END blur.txt frame- .color.png 6
# fi

## Texture and Geometry Optimization
echo ---------------------------------------
# $CODEPATH/mesh_texture_opt $PLYNAME"_c"$CLUSTERNUM".ply" $PLYNAME"_c"$CLUSTERNUM".txt" blur.txt $RGBD visibility $DATATYPE $START $END $PLYNAME"_c"$CLUSTERNUM".obj"
# $CODEPATH/mesh_texture_opt --use_opt_pose=false --closest_pose_translation=0.03 --closest_pose_rotation_angle=0.02 $PLYNAME"_c"$CLUSTERNUM".ply" $PLYNAME"_c"$CLUSTERNUM".txt" blur.txt $RGBD visibility $DATATYPE $START $END $PLYNAME"_c"$CLUSTERNUM"_ini.obj"
$CODEPATH/mesh_texture_opt --plane_opt_loop_number=0 --closest_pose_translation=0.03 --closest_pose_rotation_angle=0.02 $PLYNAME"_c"$CLUSTERNUM".ply" $PLYNAME"_c"$CLUSTERNUM".txt" blur.txt $RGBD visibility $DATATYPE $START $END $PLYNAME"_c"$CLUSTERNUM"_ini.obj"
