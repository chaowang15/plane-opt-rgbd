#include <iostream>
#include "partition.h"
#include "../common/tools.h"
#include <chrono>
#include <gflags/gflags.h>

int main(int argc, char** argv)
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    if (argc != 3 && argc != 5)
    {
        printInRed(
            "Usage: mesh_partition input_ply [target_cluster_num / input_cluster_file] [output_ply] [output_cluster_file]");
        printInRed("Example:");
        printInRed("\tmesh_partition in.ply 2000");
        printInRed("\tmesh_partition in.ply in_cluster.txt [out.ply out_cluster.txt]");
        return -1;
    }
    string ply_fname(argv[1]);

    Partition partition;
    printInGreen("Read ply file: " + ply_fname);
    if (!partition.readPLY(ply_fname))
    {
        printInRed("ERROR in reading ply file " + ply_fname);
        return -1;
    }
    partition.printModelInfo();

    string cluster_fname(argv[2]);
    bool flag_read_cluster_file = false;
    int target_cluster_num = -1;
    if (cluster_fname.length() > 4 && cluster_fname.substr(cluster_fname.length() - 4) == ".txt")
    {
        printInGreen("Read cluster file: " + cluster_fname);
        partition.readClusterFile(cluster_fname);
        flag_read_cluster_file = true;
        target_cluster_num = partition.getCurrentClusterNum();
    }
    else
        target_cluster_num = atoi(argv[2]);
    printInGreen("Cluster number: " + std::to_string(target_cluster_num));

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
        printInGreen("Run post processing step ...");
        partition.runPostProcessing();
    }
    else
    {
        partition.setTargetClusterNum(target_cluster_num);
        flag_success = partition.runPartitionPipeline();
    }
    printInGreen("Final cluster number: " + std::to_string(partition.getCurrentClusterNum()));

    auto end = std::chrono::steady_clock::now();
    double delta = std::chrono::duration_cast<chrono::milliseconds>(end - start).count();
    printInRed("Time: " + std::to_string(delta));



    if (flag_success)
    {
        cout << "Write ply file: " << out_ply_fname << endl;
        partition.writePLY(out_ply_fname);

        cout << "Write cluster file: " << out_cluster_fname << endl;
        partition.writeClusterFile(out_cluster_fname);
        cout << "ALL DONE." << endl;
    }
    return 0;
}
