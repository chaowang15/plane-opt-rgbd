# mesh_visibility

Given as input a mesh (PLY model with or without vertex color, OBJ model with or without face texture) and camera poses across frames, `mesh_visibility` can render the model under the given poses, and generate vertex/face color image, depth image and vertex visibility data per frame.

<img src="./examples/demo_mesh_visibility.gif" width="75%">


## Dependencies
- OpenCV 2.4.9 or later (image handler)
- GLEW (OpenGL support)
- GLFW (window and interface)
- GLM (OpenGL math, code already included)

## Build
```
mkdir build && cd build
cmake ..
make
```

## Usage
```
mesh_visibility -option input_mesh pose_path output_path start_frame end_frame
```

Example: go inside `examples` and unzip `copyroom_gt_poses.zip` first, then go inside your build folder and run
```
mesh_visibility -r ../../models/copyroom/copyroom_c2000_opt.obj ../examples/copyroom_gt_poses ~/dev/output/ 0 4479
```
You will get a rendering window like the gif image shown above.

Options:
- `-option`:
  - *-r*: rendering mode. Only show rendering window and won't save any output file. For keyboard control, press left and right arrow button to move forward and backward frames, C to render vertex color, D to render depth, T to render face texture;
  - *-d*: save rendered depth images;
  - *-c*: save rendered vertex color images (for PLY model with vertex colors);
  - *-t*: save rendered face texture images (for textured OBJ model);
  - *-v*: save vertex visibility file in binary;
  - *-a*: save all files (color images, depth images and visibility files);
- `input_mesh`: PLY or OBJ model;
- `pose_path`: contains input camera pose files with filename as `frame-XXXXXX.pose.txt`, where `XXXXXX` is 6-digit frame index, such as `frame-000001.pose.txt`. Each pose is a 4x4 transformation matrix. A typical data input is poses from [BundleFusion](http://graphics.stanford.edu/projects/bundlefusion/) dataset.
- `output_path`: contains output files. Images will be like `frame-XXXXXX.color.png` or `frame-XXXXXX.depth.png`. Visibility files will be like
`frame-XXXXXX.visibility.txt`.
- `start_frame, end_frame`: starting and ending frame index.

## Data
- Color and depth images are saved in PNG format. The depth image is 16UC1 image, and each depth value in millimeter is scaled by 5.0 for better rendering. For instance, a depth value 1000mm is saved as 5000 in the image.
- Vertex visibility data is saved **in binary** in a `frame-XXXXXX.visibility.txt` file. Data in each visibility file contains N+1 32-bit integers, where the first integer is the number of visible vertices, while the following N integers are indices of visible vertices. See `MeshVisibility::saveVisibleVertices2Binary()` in `mesh_visibility.cpp` for details.

## Note
- A typical input data for this code can be found from [BundleFusion](http://graphics.stanford.edu/projects/bundlefusion/) or [3DLite](http://graphics.stanford.edu/projects/3dlite/) data, which contains reconstructed PLY model and camera pose files for each RGB-D sequence.
- Default image size is 640x480 and set as constants. You can change it in `mesh_visibility.h`.
- If you find the code cannot render face textures (from OBJ model) into images, one possible reason is that graphics memory is not enough to save large texture images.
