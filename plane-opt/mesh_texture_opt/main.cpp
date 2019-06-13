#include <iostream>
#include "../common/tools.h"
#include <chrono>
#include <gflags/gflags.h>

DEFINE_string(frame_filename_prefix, std::string("frame-"), "");
DEFINE_string(frame_filename_prefix, std::string("frame-"), "");

int main(int argc, char** argv)
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    if (argc != 6)
    {
        PRINT_RED(
            "Usage: mesh_texture_opt input_ply input_cluster_file RGBD_path visibility_path output_obj");
        cout << "RGBD_path: contains color images, depth images and camera pose files." << endl;
        cout << "visibility_path: contains visibility files." << endl;
        return -1;
    }
    string ply_fname(argv[1]);

    return 0;
}
