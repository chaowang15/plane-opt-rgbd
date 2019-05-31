#ifndef PARTITION_H
#define PARTITION_H

#include <iostream>
#include <set>
#include <string>
#include <vector>
#include <unordered_set>
#include <Eigen/Eigen>
#include "covariance.h"
// #include "myheap.h"
#include "MxHeap.h"

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
        bool is_visited; // used in Breath-first search to get connected components in clusters
        bool is_border;
        int indices[3];
        CovObj cov;
        unordered_set<int> nbr_faces;
        Face() : cluster_id(-1), is_visited(false), is_border(false) {}
    };

    struct Edge : public MxHeapable
    {
        int v1, v2;
        Edge(int a, int b): v1(a), v2(b){}
    };

    struct Cluster
    {
        double energy; // to save some computation time of calling CovObj::energy() too frequently
        unordered_set<int> faces; // faces each cluster contains
        unordered_set<int> nbr_clusters;
        vector<pair<int, int>> faces_to_swap; // first is face-id, second is cluter to be swapped to
        vector<Edge*> edges;
        Vector3f color;
        CovObj cov;
        Cluster() : energy(0){}
    };

public:
    Partition();
    ~Partition();

    bool readPLY(const std::string& filename);
    bool writePLY(const std::string& filename);
    void printModelInfo() { cout << "#Vertices: " << vertices_.size() << ", #Faces: " << faces_.size() << endl; }
    bool runPartitionPipeline();
    void setTargetClusterNum(int num) { target_cluster_num_ = num; }
    void writeClusterFile(const std::string& filename);

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
    void updateCurrentClusterNum();

    /* Swap */
    void runSwapping();
    int swapOnce();
    double computeSwapDeltaEnergy(int fidx, int from, int to);

    /* Post processing */
    void processIslandClusters();
    int splitCluster(int cidx, vector<unordered_set<int>>& connected_components);
    int traverseFaceBFS(int start_fidx, int start_cidx, unordered_set<int>& component);
    void mergeIslandComponentsInCluster(int original_cidx, vector<unordered_set<int>>& connected_components);
    void mergeAdjacentPlanes();
    double computeMaxDisBetweenTwoPlanes(int c1, int c2, bool flag_use_projection = false);

private:
    int vertex_num_, face_num_;
    int init_cluster_num_, curr_cluster_num_, target_cluster_num_;
    Vector3d center_, maxcoord_, mincoord_;  // bounding box
    vector<Vertex> vertices_;
    vector<Face> faces_;
    vector<Cluster> clusters_;
    MxHeap heap_;
    double total_energy_;
    unordered_set<int> swap_clusters_; // candidate clusters with swapped faces

};

#endif  // !PARTITION_H
