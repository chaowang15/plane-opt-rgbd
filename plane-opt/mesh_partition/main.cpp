#include <iostream>
#include "partition.h"
#include "../common/tools.h"
#include <chrono>
#include <gflags/gflags.h>

DECLARE_bool(run_post_processing);
DECLARE_bool(run_mesh_simplification);

int main(int argc, char** argv)
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    if (argc != 3 && argc != 5)
    {
        PRINT_RED(
            "Usage: mesh_partition input_ply [target_cluster_num / input_cluster_file] [output_ply] [output_cluster_file]");
        cout << "Example:" << endl
             << "\tmesh_partition in.ply 2000" << endl
             << "\tmesh_partition in.ply in_cluster.txt [out.ply out_cluster.txt]" << endl;
        return -1;
    }
    string ply_fname(argv[1]);

    Partition partition;
    PRINT_GREEN("Read ply file: %s", ply_fname.c_str());
    if (!partition.readPLY(ply_fname))
    {
        PRINT_RED("ERROR in reading ply file %s", ply_fname.c_str());
        return -1;
    }
    partition.printModelInfo();

    string cluster_fname(argv[2]);
    bool flag_read_cluster_file = false;
    int target_cluster_num = -1;
    if (cluster_fname.length() > 4 && cluster_fname.substr(cluster_fname.length() - 4) == ".txt")
    {
        PRINT_GREEN("Read cluster file %s", cluster_fname.c_str());
        partition.readClusterFile(cluster_fname);
        flag_read_cluster_file = true;
        target_cluster_num = partition.getCurrentClusterNum();
    }
    else
        target_cluster_num = atoi(argv[2]);
    PRINT_GREEN("Initial cluster number: %d", target_cluster_num);

    string out_fname = ply_fname.substr(0, ply_fname.length() - 4);
    string out_ply_fname = out_fname + "-cluster" + to_string(target_cluster_num) + ".ply";
    string out_cluster_fname = out_fname + "-cluster" + to_string(target_cluster_num) + ".txt";
    if (argc == 5)
    {
        out_ply_fname = string(argv[3]);
        out_cluster_fname = string(argv[4]);
    }
    auto start = std::chrono::steady_clock::now();
    bool flag_success = true;
    if (flag_read_cluster_file)
    {
        if (FLAGS_run_post_processing)
        {
            PRINT_GREEN("Run post processing ...");
            partition.runPostProcessing();
            partition.doubleCheckClusters();
        }
    }
    else
    {
        partition.setTargetClusterNum(target_cluster_num);
        PRINT_GREEN("Run mesh partition ...");
        flag_success = partition.runPartitionPipeline();
        partition.doubleCheckClusters();
    }
    if (FLAGS_run_mesh_simplification)
    {
        PRINT_GREEN("Run mesh simplification...");
        partition.runSimplification();
        // partition.doubleCheckClusters();
    }
    PRINT_GREEN("Final cluster number: %d", partition.getCurrentClusterNum());
    auto end = std::chrono::steady_clock::now();
    double delta = std::chrono::duration_cast<chrono::milliseconds>(end - start).count();
    PRINT_RED("Time: %f ms", delta);
    if (flag_success)
    {
        PRINT_GREEN("Write ply file %s", out_ply_fname.c_str());
        partition.writePLY(out_ply_fname);

        PRINT_GREEN("Write cluster file %s", out_cluster_fname.c_str());
        partition.writeClusterFile(out_cluster_fname);
        PRINT_GREEN("ALL DONE.");
    }
    return 0;
}
