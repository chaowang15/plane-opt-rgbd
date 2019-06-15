#include <iostream>
#include "../common/tools.h"
#include <chrono>
#include "mesh_opt.h"

int main(int argc, char** argv)
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    if (argc != 10)
    {
        PRINT_RED(
            "Usage: mesh_texture_opt input_ply input_cluster_file input_blurriness_file RGBD_path visibility_path RGBD_type "
            "start_frame "
            "end_frame output_obj");
        cout << "RGBD_path: contains color images, depth images and camera pose files." << endl;
        cout << "visibility_path: contains visibility files." << endl;
        cout << "RGBD_type: 0 for BundleFusion data; 1 for ICL-NUIM data." << endl;
        return -1;
    }
    string ply_fname(argv[1]), cluster_fname(argv[2]), blurriness_fname(argv[3]), rgbd_folder(argv[4]),
        visibility_folder(argv[5]), out_obj_fname(argv[9]);
    int data_type = atoi(argv[6]), start_fidx = atoi(argv[7]), end_fidx = atoi(argv[8]);
    if (rgbd_folder.back() != '\\' && rgbd_folder.back() != '/')
        rgbd_folder += "/";
    if (visibility_folder.back() != '\\' && visibility_folder.back() != '/')
        visibility_folder += "/";
    RGBDMeshOpt mesh_opt;

    /// Read a lot of things.
    PRINT_GREEN("Read PLY file %s", ply_fname.c_str());
    if (!mesh_opt.readPLY(ply_fname))
        return -1;

    PRINT_GREEN("Read cluster file %s", cluster_fname.c_str());
    if (!mesh_opt.readClusterFile(cluster_fname))
        return -1;
    mesh_opt.printMeshInfo();

    string camera_info_fname = rgbd_folder + "info.txt";
    PRINT_GREEN("Read camera parameter file %s", camera_info_fname.c_str());
    if (!mesh_opt.readCameraParamFile(camera_info_fname))
        return -1;

    PRINT_GREEN("Read image blurriness file %s", blurriness_fname.c_str());
    if (!mesh_opt.readImageBlurrinessFile(blurriness_fname))
        return -1;

    PRINT_GREEN("Read RGBD frames in folder %s", rgbd_folder.c_str());
    if (!mesh_opt.readRGBDFrames(rgbd_folder, visibility_folder, data_type, start_fidx, end_fidx))
        return -1;

    PRINT_GREEN("Run optimization pipeline now.");
    auto start = std::chrono::steady_clock::now();
    mesh_opt.runOptimizationPipeline();
    auto end = std::chrono::steady_clock::now();
    double delta = std::chrono::duration_cast<chrono::milliseconds>(end - start).count();

    PRINT_GREEN("Save textured mesh in %s", out_obj_fname.c_str());
    mesh_opt.saveTexturedMesh(out_obj_fname);

    string out_ply_fname = out_obj_fname.substr(0, out_obj_fname.length() - 3) + "ply";
    PRINT_GREEN("Save PLY mesh in %s", out_ply_fname.c_str());
    mesh_opt.savePLY(out_ply_fname);

    PRINT_GREEN("ALL DONE!");
    PRINT_RED("Time: %f ms", delta);

    return 0;
}
