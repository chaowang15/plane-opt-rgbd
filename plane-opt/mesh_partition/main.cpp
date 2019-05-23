#include <iostream>
#include "partition.h"
#include "../common/tools.h"
#include <chrono>

int main(int argc, char** argv)
{
    if (argc != 3 && argc != 5)
    {
        printInRed("Usage: mesh_partition input_ply target_cluster_num [output_ply] [output_cluster_file]");
        return -1;
    }
    string ply_fname(argv[1]);
    int target_cluster_num = atoi(argv[2]);
    string out_fname = ply_fname.substr(0, ply_fname.length() - 4);
    string out_ply_fname = out_fname + "-cluster" + to_string(target_cluster_num) + ".ply";
    string out_cluster_fname = out_fname + "-cluster" + to_string(target_cluster_num) + ".txt";
    if (argc == 5)
    {
        out_ply_fname = string(argv[3]);
        out_cluster_fname = string(argv[4]);
    }
    Partition partition;
    cout << "Read ply file: " << ply_fname << endl;
    if (!partition.readPLY(ply_fname))
    {
        printInRed("ERROR in reading ply file " + ply_fname);
        return -1;
    }
    partition.printModelInfo();
    partition.setTargetClusterNum(target_cluster_num);

    auto start = std::chrono::steady_clock::now();
    partition.runPartitionPipeline();
    auto end = std::chrono::steady_clock::now();
    double delta = std::chrono::duration_cast<chrono::seconds>(end - start).count();
    printInRed("Time: " + std::to_string(delta));

    cout << "Write ply file: " << out_ply_fname << endl;
    partition.writePLY(out_ply_fname);

    cout << "ALL DONE." << endl;
    return 0;
}
