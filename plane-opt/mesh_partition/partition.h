#ifndef PARTITION_H
#define PARTITION_H

#include <iostream>
#include <set>
#include <string>
#include <vector>
#include <unordered_set>
#include <Eigen/Eigen>
#include "covariance.h"
#include <memory>

using namespace std;
using namespace Eigen;

class Partition
{
public:
    struct Vertex
    {
        bool is_valid;
        int cluster_id;
        Vector3d pt;
        Vector3f color;
        unordered_set<int> nbr_vertices, nbr_faces; // neighbors
        Vertex() : is_valid(true), cluster_id(-1) {}
    };

    struct Face
    {
        int cluster_id;
        bool is_visited;
        bool is_valid;
        int indices[3];
        unordered_set<int> nbr_faces;
        Face() : cluster_id(-1), is_visited(false), is_valid(true) {}
    };

    struct Edge
    {
        int v1, v2;
        double energy;
        Edge(int a, int b): v1(a), v2(b), energy(0){}
        Edge(int a, int b, double c) : v1(a), v2(b), energy(c) {}
        bool operator<(const Edge& rhs) const { return energy < rhs.energy; }  // Sorted by energy
    };

    //! For swapping faces on cluster borders
    struct SwapFace
    {
        int face_id, from, to;
        SwapFace(int face, int f, int t) : face_id(face), from(f), to(t) {}
    };

    struct Cluster
    {
        double energy;
        unordered_set<int> faces; // faces each cluster contains
        unordered_set<int> nbr_clusters;
        vector<SwapFace> faces_to_swap;
        vector<std::shared_ptr<Edge>> edges;
        Vector3f color;
        CovObj cov, ini_cov; // current and initial covariance object
        Cluster() : energy(0) {}
        bool isValid() { return !faces.empty(); }
    };

public:
    Partition() {}
    ~Partition() {}

    bool readPLY(const std::string& filename);
    bool writePLY(const std::string& filename);
    void printModelInfo() { cout << "#Vertices: " << vertices_.size() << ", #Faces: " << faces_.size() << endl; }
    void runPartitionPipeline();
    void setTargetClusterNum(int num) { target_cluster_num_ = num; }

private:
    /* Merging */
    void runMerging();
    void initMerging();

    void runSwapping();

private:
    int vertex_num_, face_num_;
    int curr_cluster_num_, target_cluster_num_;
    Vector3d center_, maxcoord_, mincoord_;  // bounding box
    vector<Vertex> vertices_;
    vector<Face> faces_;
    vector<Cluster> clusters_;
    set<Edge> heap_;
};

#endif  // !PARTITION_H
