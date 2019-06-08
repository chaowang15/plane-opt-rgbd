#!/bin/sh

# This is the original RGB-D data including color images, depth images and camera pose files
RGBD="/home/chao/dev/data/bundlefusion/copyroom/copyroom/"

# 0 for BundleFusion data, 1 for ICL-NUIM data
DATATYPE=0

# Working Directory contains the input PLY model. All output files will be put inside this directory.
WORKINGDIR="/home/chao/dev/data/bundlefusion/copyroom/copyroom_all/"
