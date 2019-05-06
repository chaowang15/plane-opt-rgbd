#ifndef PARTITION_H
#define PARTITION_H

#include <iostream>
#include <set>
#include <string>
#include <vector>
#include <unordered_set>
#include <Eigen/Eigen>
#include "covariance.h"

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
        unordered_set<int> neighbors, belonging_faces;
        Vertex() : is_valid(true), cluster_id(-1) {}
    };

    struct Face
    {
        int indices[3];
        int cluster_id;
        bool is_visited, is_valid;
        unordered_set<int> neighbors;
        Face() : cluster_id(-1), is_visited(false), is_valid(true) {}
    };

	struct Edge
	{
	    int v1, v2;
	    double energy;
	    Edge(int a, int b, double c) : v1(a), v2(b), energy(c) {}
	    bool operator<(const Edge &rhs) const { return energy < rhs.energy; } // Sorted by energy
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
        unordered_set<int> elements, neighbors;
        vector<SwapFace> faces_to_swap;
        Vector3f color;
        CovObj cov, ini_cov;
        Cluster() : energy(0) {}
        bool isValid() { return !elements.empty(); }
    };

public:
    Partition() {}
    ~Partition() {}

    bool readPLY(const string& filename);

private:


private:
	int vertex_num_, face_num_, cluster_num_;
	Vector3d center_, maxcoord_, mincoord_; // bounding box
	vector<Vertex> vertices_;
	vector<Face> faces_;
	vector<Cluster> clusters_;

	set<Edge> heap_;
};

#endif  // !PARTITION_H
