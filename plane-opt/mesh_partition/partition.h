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
#include "qemquadrics.h"

using namespace std;
using namespace Eigen;

class Partition
{
public:
    struct Edge : public MxHeapable
    {
        int v1, v2;
        Edge(int a, int b) : v1(a), v2(b) {}
    };

    struct SwapFace
    {
        int face_id, from, to;
        SwapFace(int v, int f, int t)
        {
            face_id = v;
            from = f;
            to = t;
        }
    };

    struct Vertex
    {
        bool is_valid;  // false if it is removed (all its adjacent faces are removed)
        int cluster_id;
        Vector3d pt;
        Vector3f color;
        unordered_set<int> nbr_vertices, nbr_faces;
        vector<Edge*> nbr_edges;
        QEMQuadrics Q;
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

    struct Cluster
    {
        double energy;             // to save some computation time of calling CovObj::energy() too frequently
        bool is_visited;           // used in Breath-first search to remove small floating clusters
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
    void runSimplification();

private:
    /* Merging */
    bool runMerging();
    void initMerging();
    void initMeshConnectivity();
    void computeEdgeEnergy(Edge* edge);
    bool removeEdgeFromList(Edge* edge, vector<Edge*>& edgelist);
    bool isClusterValid(int cidx) { return !clusters_[cidx].faces.empty(); }
    bool mergeOnce();
    void applyFaceEdgeContraction(Edge* edge);
    void mergeClusters(int c1, int c2);
    int findClusterNeighbors(int cidx);
    int findClusterNeighbors(int cidx, unordered_set<int>& cluster_faces, unordered_set<int>& neighbor_clusters);
    double getTotalEnergy();
    void createClusterColors();
    void updateCurrentClusterNum();
    void releaseHeap();

    /* Swap */
    void runSwapping();
    int swapOnce();
    double computeSwapDeltaEnergy(int fidx, int from, int to);
    void processIslandClusters();
    int splitCluster(int cidx, vector<unordered_set<int>>& connected_components);
    int traverseFaceBFS(int start_fidx, int start_cidx, unordered_set<int>& component);
    void mergeIslandComponentsInCluster(int original_cidx, vector<unordered_set<int>>& connected_components);

    /* Post processing */
    double computeMaxDisBetweenTwoPlanes(int c1, int c2, bool flag_use_projection = false);
    double computeAvgDisBtwTwoPlanes(int c1, int c2);
    void removeSmallClusters();
    void updateNewMeshIndices();
    void mergeAdjacentPlanes();
    void mergeIslandClusters();

    /* Simplification */
    void initSimplification();
    void findInnerAndBorderEdges();
    void initInnerEdgeQuadrics();
    void initBorderEdges();
    void simplifyInnerEdges();
    void simplifyBorderEdges();
    bool checkEdgeContraction(Edge* edge);
    int getCommonNeighborNum(int v1, int v2);
    bool checkFlippedFaces(Edge* edge, int endpoint, const Vector3d& contracted_vtx);
    void applyVtxEdgeContraction(Edge* edge, int cluster_idx);

    /* Small functions */
    //! Check if a face contains two vertices
    inline bool checkFaceContainsVertices(int fidx, int v1, int v2)
    {
        return checkFaceContainsVertices(fidx, v1) && checkFaceContainsVertices(fidx, v2);
    }
    //! Check if a face contains one vertex
    inline bool checkFaceContainsVertices(int fidx, int v1)
    {
        return faces_[fidx].indices[0] == v1 || faces_[fidx].indices[1] == v1 || faces_[fidx].indices[2] == v1;
    }
    //! Convert an long long edge type to two endpoints
    inline void getEdge(const long long& key, int& v1, int& v2)
    {
        v2 = int(key & 0xffffffffLL);  // v2 is lower 32 bits of the 64-bit edge integer
        v1 = int(key >> 32);           // v1 is higher 32 bits
    }

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
    unordered_map<long long, vector<int>> edge_to_face_;         // edge (represented by two int32 endpoints) -> face id
    unordered_map<int, vector<long long>> cluster_inner_edges_;  // edges inside each cluster
    unordered_set<long long> border_edges_;                      // mesh border and cluster border edges
    unordered_map<int, int> vidx_old2new_;  // original vertex indices -> new mesh indices (after removing some faces)
    unordered_map<int, int> fidx_old2new_;  // original vertex indices -> new mesh indices (after removing some faces)
    int new_vertex_num_, new_face_num_;
    bool flag_new_mesh_;  // true if removing some faces/vertices/clusters; false by default

    // These are used to balance the importance of point and triangle quadrics, respectively.
    // However, actually equal values are good in experiments.
    const double kFaceCoefficient = 1.0, kPointCoefficient = 1.0;
    const int kMinInnerEdgeNum = 10;  // minimum inner edge number in a cluster after simplification
    int curr_edge_num_;
};

#endif  // !PARTITION_H
