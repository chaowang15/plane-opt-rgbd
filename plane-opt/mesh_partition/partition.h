#ifndef PARTITION_H
#define PARTITION_H

#include <iostream>
#include <set>
#include <string>
#include <vector>
#include <unordered_set>
#include <Eigen/Eigen>
#include "covariance.h"
#include "myheap.h"

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

    struct Edge : public MxHeapable
    {
        int v1, v2;
        Edge(int a, int b): v1(a), v2(b){}
    };

    //! For swapping faces on cluster borders
    struct SwapFace
    {
        int face_id, from, to;
        SwapFace(int face, int f, int t) : face_id(face), from(f), to(t) {}
    };

    struct Cluster
    {
        double energy, ini_energy; // only to save some computation time of calling CovObj::energy() too frequently
        unordered_set<int> faces; // faces each cluster contains
        unordered_set<int> nbr_clusters;
        vector<SwapFace> faces_to_swap;
        vector<Edge*> edges;
        Vector3f color;
        CovObj cov, ini_cov; // current and initial covariance object
        Cluster() : energy(0), ini_energy(0){}
    };

public:
    Partition();
    ~Partition();

    bool readPLY(const std::string& filename);
    bool writePLY(const std::string& filename);
    void printModelInfo() { cout << "#Vertices: " << vertices_.size() << ", #Faces: " << faces_.size() << endl; }
    void runPartitionPipeline();
    void setTargetClusterNum(int num) { target_cluster_num_ = num; }

private:
    /* Merging */
    bool runMerging();
    void initMerging();
    void computeEdgeEnergy(Edge* edge);
    bool removeEdgeFromCluster(int cidx, Edge* edge);
    bool isClusterValid(int cidx) { return !clusters_[cidx].faces.empty(); }
    bool mergeOnce();
    void applyFaceEdgeContraction(Edge* edge);
    void mergeClusters(int c1, int c2);
    int findClusterNeighbors(int cidx);
	int findClusterNeighbors(int cidx, unordered_set<int>& cluster_faces, unordered_set<int>& neighbor_clusters);
    double getTotalEnergy();
    void createClusterColors();


    /* Swap */
    void runSwapping();
    void swapOnce();


private:
    int vertex_num_, face_num_;
    int curr_cluster_num_, target_cluster_num_;
    Vector3d center_, maxcoord_, mincoord_;  // bounding box
    vector<Vertex> vertices_;
    vector<Face> faces_;
    vector<Cluster> clusters_;
    MxHeap heap_;
    double total_energy_;
    // set<Edge> heap_;
    // set<Edge*, EdgeComp> heap_;
};

#endif  // !PARTITION_H
