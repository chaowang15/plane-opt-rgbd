#ifndef PARTITION_H
#define PARTITION_H

#include <iostream>
#include <set>
#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <Eigen/Eigen>
#include "covariance.h"
#include "MxHeap.h"

using namespace std;
using namespace Eigen;

class Partition
{
public:
    struct Vertex
    {
        bool is_valid; // false if it is removed (all its adjacent faces are removed)
        int cluster_id;
        Vector3d pt;
        Vector3f color;
        unordered_set<int> nbr_vertices, nbr_faces;  // neighbors
        Vertex() : is_valid(false), cluster_id(-1) {}
    };

    struct Face
    {
        int cluster_id;
        bool is_visited;  // used in Breath-first search to get connected components in clusters
        bool is_valid;    // false if this face is removed.
        int indices[3];
        CovObj cov;
        unordered_set<int> nbr_faces;
        Face() : cluster_id(-1), is_visited(false), is_valid(true) {}
    };

    struct Edge : public MxHeapable
    {
        int v1, v2;
        Edge(int a, int b) : v1(a), v2(b) {}
    };

    struct SwapFace
    {
        int face_id;
        int from;
        int to;
        SwapFace(int v, int f, int t)
        {
            face_id = v;
            from = f;
            to = t;
        }
    };

    struct Cluster
    {
        double energy;             // to save some computation time of calling CovObj::energy() too frequently
        unordered_set<int> faces;  // faces each cluster contains
        unordered_set<int> nbr_clusters;
        vector<SwapFace> faces_to_swap;
        Vector3f color;
        CovObj cov;
        Cluster() : energy(0) {}
    };

public:
    Partition();
    ~Partition();

    bool readPLY(const std::string& filename);
    bool writePLY(const std::string& filename);
    bool runPartitionPipeline();
    void writeClusterFile(const std::string& filename);
    bool readClusterFile(const std::string& filename);
    void setTargetClusterNum(int num) { target_cluster_num_ = num; }
    int getCurrentClusterNum() { return curr_cluster_num_; }
    void printModelInfo() { cout << "#Vertices: " << vertices_.size() << ", #Faces: " << faces_.size() << endl; }
    void runPostProcessing();

private:
    /* Merging */
    bool runMerging();
    void initMerging();
    void initVerticesAndFaces();
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
    void releaseEdges();

    /* Swap */
    void runSwapping();
    int swapOnce();
    double computeSwapDeltaEnergy(int fidx, int from, int to);

    /* Post processing */
    void processIslandClusters();
    int splitCluster(int cidx, vector<unordered_set<int>>& connected_components);
    int traverseFaceBFS(int start_fidx, int start_cidx, unordered_set<int>& component);
    void mergeIslandComponentsInCluster(int original_cidx, vector<unordered_set<int>>& connected_components);
    double computeMaxDisBetweenTwoPlanes(int c1, int c2, bool flag_use_projection = false);
    double computeAvgDisBtwTwoPlanes(int c1, int c2);
    void removeSmallClusters();
    void updateNewMeshIndices();
    void mergeAdjacentPlanes();
    void removeIslandClusters();

private:
    int vertex_num_, face_num_;
    int init_cluster_num_, curr_cluster_num_, target_cluster_num_;
    bool flag_read_cluster_file_;
    Vector3d center_, maxcoord_, mincoord_;  // bounding box
    vector<Vertex> vertices_;
    vector<Face> faces_;
    vector<Cluster> clusters_;
    vector<vector<Edge*>> global_edges_;
    MxHeap heap_;
    double total_energy_;
    unordered_set<int> clusters_in_swap_, last_clusters_in_swap_;
    unordered_map<long long, vector<int>> edge_to_face_;  // edge (two int32 endpoints) -> face id

    /* Used for the new mesh after removing some faces/vertices/clusters */
    unordered_map<int, int> vidx_old2new_, fidx_old2new_; // original vertex/face indices -> indices after removing some faces/vertices
    int new_vertex_num_, new_face_num_;
    bool flag_new_mesh_; // true if removing some faces/vertices/clusters; false by default
};

#endif  // !PARTITION_H
