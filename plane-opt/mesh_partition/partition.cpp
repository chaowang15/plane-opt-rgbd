#include "partition.h"
#include <stdlib.h>
#include "../common/tools.h"
#include <unordered_map>
#include <random>
#include <chrono>
#include <queue>
#include <gflags/gflags.h>

DEFINE_double(point_plane_dis_threshold, 0.08, "");
DEFINE_double(normal_angle_threshold, 10.0, "");
DEFINE_double(max_normal_angle_threshold, 45.0, "");
DEFINE_double(center_normal_angle_threshold, 70.0, "");
DEFINE_double(energy_increase_threshold, 0.1, "");

Partition::Partition() {}

Partition::~Partition()
{
    std::cout << "Releasing edges ... " << std::endl;
    // This will release memory of all pointers, so no need to release 'edges' in each cluster.
    heap_.destroy();
}

//! Read PLY model
/*!
    NOTE about this function:
    - Supports both binary and ASCII PLY model;
    -
*/
bool Partition::readPLY(const std::string& filename)
{
    FILE* fin;
    if (!(fin = fopen(filename.c_str(), "rb")))
    {
        cerr << "ERROR: Unable to open file" << filename << endl;
        return false;
    }

    /************************************************************************/
    /* Read headers */

    // Mode for vertex and face type
    // 1 for vertex (no faces), 2 for vertex and faces,
    // 3 for vertex, vertex colors (no faces), 4 for vertex, vertex colors and faces
    int vertex_mode = 1;
    int ply_mode = 0;  // 0 for ascii, 1 for little-endian
    int color_channel_num = 0;
    int vertex_quality_dim = 0;  // vertex quality, such as vertex normal or other data per vertex defined by user
    int vertex_normal_dim = 0;   // vertex normal dimension
    char seps[] = " ,\t\n\r ";   // separators
    seps[5] = 10;
    int property_num = 0;
    char line[1024];
    maxcoord_[0] = maxcoord_[1] = maxcoord_[2] = std::numeric_limits<double>::min();
    mincoord_[0] = mincoord_[1] = mincoord_[2] = std::numeric_limits<double>::max();
    center_[0] = center_[1] = center_[2] = 0;

    while (true)
    {
        if (fgets(line, 1024, fin) == NULL)
            return false;
        char* token = strtok(line, seps);
        if (!strcmp(token, "end_header"))
            break;
        else if (!strcmp(token, "format"))
        {
            token = strtok(NULL, seps);
            if (!strcmp(token, "ascii"))
                ply_mode = 0;
            else if (!strcmp(token, "binary_little_endian"))
                ply_mode = 1;
            else
            {
                cout << "WARNING: can not read this type of PLY model: " << std::string(token) << endl;
                return false;
            }
        }
        else if (!strcmp(token, "element"))
        {
            token = strtok(NULL, seps);
            if (!strcmp(token, "vertex"))
            {
                // vertex count
                token = strtok(NULL, seps);
                sscanf(token, "%d", &vertex_num_);
                vertex_mode = 1;
            }
            else if (!strcmp(token, "face"))
            {
                // Face count
                token = strtok(NULL, seps);
                sscanf(token, "%d", &face_num_);
                vertex_mode++;  // mode with faces is 1 larger than mode without faces
            }
        }
        else if (!strcmp(token, "property"))
        {
            if (vertex_mode % 2 == 1)
            {
                if (property_num >= 3)  // skip property x,y,z
                {
                    token = strtok(NULL, seps);
                    if (!strcmp(token, "uchar"))  // color
                    {
                        token = strtok(NULL, seps);
                        color_channel_num++;
                        if (color_channel_num >= 3)  // color channel number must be 3 (RGB) or 4 (RGBA)
                            vertex_mode = 3;
                    }
                    else if (!strcmp(token, "float"))  // vertex quality data
                    {
                        // Currently just count it and skip
                        token = strtok(NULL, seps);
                        if (!strcmp(token, "nx") || !strcmp(token, "ny") || !strcmp(token, "nz"))
                            vertex_normal_dim++;
                        else
                            vertex_quality_dim++;
                    }
                }
                property_num++;
            }
            else if (vertex_mode % 2 == 0)
            {
                token = strtok(NULL, seps);
                bool face_flag = false;
                if (!strcmp(token, "list"))
                {
                    token = strtok(NULL, seps);
                    if (!strcmp(token, "uint8") || !strcmp(token, "uchar"))
                    {
                        token = strtok(NULL, seps);
                        if (!strcmp(token, "int") || !strcmp(token, "int32"))
                            face_flag = true;
                    }
                    if (!face_flag)
                    {
                        cout << "ERROR in Reading PLY model: the type of 'number of face indices' is not 'unsigned char', or "
                                "the type of 'vertex_index' is not 'int'."
                             << endl;
                        return false;
                    }
                }
            }
        }
    }
    if (color_channel_num != 0 && color_channel_num != 3 && color_channel_num != 4)
    {
        cout << "ERROR: Color channel number is " << color_channel_num << " but it has to be 0, 3, or 4." << endl;
        return false;
    }
    if (vertex_normal_dim != 0 && vertex_normal_dim != 3)
    {
        cout << "ERROR: Vertex normal dimension is " << vertex_normal_dim << " but it has to be 0 or 3." << endl;
        return false;
    }

    /************************************************************************/
    /* Read vertices and faces */
    vertices_.reserve(vertex_num_);
    faces_.reserve(face_num_);
    if (ply_mode == 1)  // binary mode
    {
        for (int i = 0; i < vertex_num_; i++)
        {
            // The vertex data order is: coordinates -> normal -> color -> others (qualities, radius, curvatures, etc)
            size_t haveread = 0;
            Vertex vtx;
            float vert[3];
            if ((haveread = fread(vert, sizeof(float), 3, fin)) != 3)
            {
                cout << "ERROR in reading PLY vertices in position " << ftell(fin) << endl;
                return false;
            }
            if (vertex_normal_dim)
            {
                float nor[3];
                if ((haveread = fread(nor, sizeof(float), vertex_normal_dim, fin)) != vertex_normal_dim)
                {
                    cout << "ERROR in reading PLY vertex normals in position " << ftell(fin) << endl;
                    return false;
                }
                // Currently we just abandon the vertex normal
                // vtx.normal = Vector3d(nor[0], nor[1], nor[2]);
            }
            if (color_channel_num)
            {
                unsigned char color[4];
                if ((haveread = fread(color, sizeof(unsigned char), color_channel_num, fin)) != color_channel_num)
                {
                    cout << "ERROR in reading PLY vertex colors in position " << ftell(fin) << endl;
                    return false;
                }
                vtx.color = Vector3f(color[0], color[1], color[2]) / 255;
            }
            if (vertex_quality_dim)
            {
                float qual[3];  // Currently we just abandon the vertex quality data
                if ((haveread = fread(qual, sizeof(float), vertex_quality_dim, fin)) != vertex_quality_dim)
                {
                    cout << "ERROR in reading PLY vertex qualities in position " << ftell(fin) << endl;
                    return false;
                }
                // Currently we just abandon the vertex normal
            }
            vtx.pt = Vector3d(vert[0], vert[1], vert[2]);
            vertices_.push_back(vtx);
            for (int j = 0; j < 3; ++j)
            {
                mincoord_[j] = min(mincoord_[j], double(vert[j]));
                maxcoord_[j] = max(maxcoord_[j], double(vert[j]));
                center_[j] += vert[j];
            }
        }

        // Face data
        for (int i = 0; i < face_num_; ++i)
        {
            unsigned char channel_num;
            size_t haveread = fread(&channel_num, 1, 1, fin);  // channel number for each face
            Face fa;
            if ((haveread = fread(fa.indices, sizeof(int), 3, fin)) != 3)  // currently only support triangular face
            {
                cout << "ERROR in reading PLY face indices in position " << ftell(fin) << endl;
                return false;
            }
            faces_.push_back(fa);
        }
    }
    else  // ASCII mode (face reader is still unfinished)
    {
        // Read vertices
        for (int i = 0; i < vertex_num_; i++)
        {
            // The vertex data order is: coordinates -> normal -> color -> others (qualities, radius, curvatures, etc)
            if (fgets(line, 1024, fin) == NULL)
                return false;
            char* token = strtok(line, seps);
            // Read 3D point
            Vertex vtx;
            float vert[3];
            for (int j = 0; j < 3; ++j)
            {
                token = strtok(NULL, seps);
                sscanf(token, "%f", &(vert[j]));
            }
            // Read vertex normal
            if (vertex_normal_dim)
            {
                float nor[3];
                for (int j = 0; j < vertex_normal_dim; ++j)
                {
                    token = strtok(NULL, seps);
                    sscanf(token, "%f", &(nor[j]));
                }
                // Currently we just abandon the vertex normal data
                // vtx.normal = Vector3d(nor[0], nor[1], nor[2]);
            }
            if (color_channel_num)
            {
                unsigned char color[4];
                for (int j = 0; j < vertex_quality_dim; ++j)
                {
                    token = strtok(NULL, seps);
                    sscanf(token, "%c", &(color[j]));
                }
                vtx.color = Vector3f(color[0], color[1], color[2]) / 255;
            }
            if (vertex_quality_dim)
            {
                float qual;
                for (int j = 0; j < vertex_quality_dim; ++j)
                {
                    token = strtok(NULL, seps);
                    sscanf(token, "%f", &qual);
                }
                // Currently we just abandon the vertex quality data
            }
            vtx.pt = Vector3d(vert[0], vert[1], vert[2]);
            vertices_.push_back(vtx);
            for (size_t j = 0; j < 3; ++j)
            {
                mincoord_[j] = min(mincoord_[j], double(vert[j]));
                maxcoord_[j] = max(maxcoord_[j], double(vert[j]));
                center_[j] += vert[j];
            }
        }
        // Read Faces
        for (int i = 0; i < face_num_; i++)
        {
            if (fgets(line, 1024, fin) == NULL)
                return false;
            char* token = strtok(line, seps);
            token = strtok(NULL, seps);
            for (int j = 0; j < 3; ++j)
            {
                token = strtok(NULL, seps);
                sscanf(token, "%d", &(faces_[i].indices[j]));
            }
        }
    }
    /* Note to compute center after reading all data */
    for (int j = 0; j < 3; ++j)
    {
        center_[j] /= vertex_num_;
    }

    // Just in case some vertices or faces are not read correctly
    face_num_ = static_cast<int>(faces_.size());
    vertex_num_ = static_cast<int>(vertices_.size());
    return true;
}

//! Write PLY file
bool Partition::writePLY(const std::string& filename)
{
    FILE* fout = NULL;
    fout = fopen(filename.c_str(), "wb");  // write in binary mode
    if (fout == NULL)
    {
        cout << "Unable to create file " << filename << endl;
        return false;
    }
    // Write headers
    fprintf(fout, "ply\n");
    fprintf(fout, "format binary_little_endian 1.0\n");
    fprintf(fout, "element vertex %d\n", vertex_num_);
    fprintf(fout, "property float x\n");
    fprintf(fout, "property float y\n");
    fprintf(fout, "property float z\n");
    fprintf(fout, "element face %d\n", face_num_);
    fprintf(fout, "property list uchar int vertex_indices\n");
    fprintf(fout, "property uchar red\n");  // face color
    fprintf(fout, "property uchar green\n");
    fprintf(fout, "property uchar blue\n");
    fprintf(fout, "property uchar alpha\n");
    fprintf(fout, "end_header\n");
    float pt3[3];
    unsigned char kFaceVtxNum = 3;
    unsigned char rgba[4] = {255};
    for (int i = 0; i != vertex_num_; ++i)
    {
        for (int j = 0; j < 3; ++j)
            pt3[j] = float(vertices_[i].pt[j]);
        fwrite(pt3, sizeof(float), 3, fout);
    }
    for (int i = 0; i != face_num_; ++i)
    {
        fwrite(&kFaceVtxNum, sizeof(unsigned char), 1, fout);
        fwrite(faces_[i].indices, sizeof(int), 3, fout);
        int cidx = faces_[i].cluster_id;
        if (cidx == -1)
        {
            cout << "ERROR: face " << i << " doesn't belong to any cluster!" << endl;
        }
        else
        {
            for (int j = 0; j < 3; ++j)
                rgba[j] = static_cast<unsigned char>(clusters_[cidx].color[j] * 255);
        }
        fwrite(rgba, sizeof(unsigned char), 4, fout);
    }
    fclose(fout);
    return true;
}

bool Partition::runPartitionPipeline()
{
    init_cluster_num_ = face_num_;
    curr_cluster_num_ = face_num_;
    assert(target_cluster_num_ < curr_cluster_num_ && target_cluster_num_ > 0);

    printInGreen("Mesh partition by merging neighbor faces:");
    if (!runMerging())
        return false;

    printInGreen("(Optional) Optimization by swapping neighbor faces between clusters:");
    runSwapping();

    printInGreen("Post process: merge neighbor clusters:");
    mergeAdjacentPlanes();

    printInCyan("#Final Clusters: " + std::to_string(curr_cluster_num_));
    createClusterColors();

    return true;
}

bool Partition::runMerging()
{
    initMerging();

    cout << "Merging ..." << endl;
    float progress = 0.0;  // for printing a progress bar
    int cluster_diff = curr_cluster_num_ - target_cluster_num_;
    const int kStep = (cluster_diff < 100) ? 1 : (cluster_diff / 100);
    int count = 0;
    while (curr_cluster_num_ > target_cluster_num_)
    {
        if (count % kStep == 0 || count == cluster_diff - 1)
        {
            progress = (count == cluster_diff - 1) ? 1.0 : static_cast<float>(count) / cluster_diff;
            printProgressBar(progress);
        }
        if (!mergeOnce())
            return false;
        // Special case: sometimes all existing clusters have no neighbors (like floating faces)
        if (heap_.size() == 0)
        {
            printInMagenta("WARNING: Now heap is empty, but still not reaching the target cluster number. ");
            break;
        }
        count++;
    }
    cout << "Result Cluster Number: " << curr_cluster_num_ << ", Energy: " << getTotalEnergy() << endl;
    return true;
}

void Partition::initMerging()
{
    clusters_.resize(curr_cluster_num_);

    // One edge -> multiple faces, since the mesh may be non-manifold
    typedef std::pair<int, int> EdgePair;
    // unordered_map<EdgePair, std::vector<int>, boost::hash<EdgePair>> edge_to_face;
    unordered_map<long long, vector<int>> edge_to_face;  // each long long int presents an edge with two int endpoints
    cout << "Initialize neighbors ... " << endl;
    vector<int> fa(3);
    float progress = 0.0;  // used to print a progress bar
    const int kStep = (face_num_ < 100) ? 1 : (face_num_ / 100);
    for (int fidx = 0; fidx < face_num_; fidx++)
    {
        // Print a progress bar
        if (fidx % kStep == 0 || fidx == face_num_ - 1)
        {
            progress = (fidx == face_num_ - 1) ? 1.0 : static_cast<float>(fidx) / face_num_;
            printProgressBar(progress);
        }

        Face& face = faces_[fidx];
        face.cluster_id = fidx;  // initially each face is a single cluster

        // Initialize neighbors of vertices and faces
        for (int i = 0; i < 3; ++i)
            fa[i] = face.indices[i];
        // One directed edge may be shared by more than one face in a non-manifold edges. So we
        // sort vertices here to use undirected edge to determine face neighbors.
        std::sort(fa.begin(), fa.end());
        for (int i = 0; i < 3; ++i)
        {
            vertices_[fa[i]].nbr_vertices.insert(fa[(i + 1) % 3]);
            vertices_[fa[i]].nbr_vertices.insert(fa[(i + 2) % 3]);
            vertices_[fa[i]].nbr_faces.insert(fidx);
            long long a = static_cast<long long>((i == 2) ? fa[0] : fa[i]);
            long long b = static_cast<long long>((i == 2) ? fa[i] : fa[i + 1]);
            long long edge = (a << 32) | b;  // fast bit operation
            for (int f : edge_to_face[edge])
            {
                face.nbr_faces.insert(f);
                faces_[f].nbr_faces.insert(fidx);
                clusters_[fidx].nbr_clusters.insert(f);
                clusters_[f].nbr_clusters.insert(fidx);
            }
            edge_to_face[edge].push_back(fidx);
        }

        // Initialize covariance quadratic objects
        clusters_[fidx].faces.insert(fidx);  // initially each face is a cluster
        CovObj Q(vertices_[face.indices[0]].pt, vertices_[face.indices[1]].pt, vertices_[face.indices[2]].pt);
        clusters_[fidx].cov = faces_[fidx].cov = Q;
    }

    // Initialize edges
    cout << "Initialize edges ... " << endl;
    progress = 0;
    for (int cidx = 0; cidx < init_cluster_num_; cidx++)
    {
        if (cidx % kStep == 0 || cidx == init_cluster_num_ - 1)
        {
            progress = (cidx == init_cluster_num_ - 1) ? 1.0 : static_cast<float>(cidx) / init_cluster_num_;
            printProgressBar(progress);
        }
        Cluster& cluster = clusters_[cidx];
        for (int nbr : cluster.nbr_clusters)
        {
            if (cidx < nbr)
            {
                Edge* edge = new Edge(cidx, nbr);
                computeEdgeEnergy(edge);
                heap_.insert(edge);
                cluster.edges.push_back(edge);
                clusters_[nbr].edges.push_back(edge);
            }
        }
    }
}

//! Compute energy of the edge.
/*!
    This function assumes the energy data in each cluster is already the latest, like calling
    `clusters_[cidx].energy = clusters_[cidx].cov.energy()`. This can save some time.
*/
void Partition::computeEdgeEnergy(Edge* edge)
{
    CovObj cov = clusters_[edge->v1].cov;
    cov += clusters_[edge->v2].cov;
    double energy = cov.energy() - clusters_[edge->v1].energy - clusters_[edge->v2].energy;
    edge->heap_key(-energy);  // it's a max heap by default but we need a min heap
}

//! Remove one edge pointer from a cluster. Note that it doesn't delete/release the pointer.
bool Partition::removeEdgeFromCluster(int cidx, Edge* edge)
{
    if (edge == nullptr)
        return false;
    bool flag_found_edge = false;
    auto iter = clusters_[cidx].edges.begin();
    while (iter != clusters_[cidx].edges.end())
    {
        Edge* e = *iter;
        if (e == nullptr || e == edge)
        {
            iter = clusters_[cidx].edges.erase(iter);
            flag_found_edge = true;  // Not return here since there may be duplicate edges
        }
        else
        {
            iter++;
        }
    }
    return flag_found_edge;
}

bool Partition::mergeOnce()
{
    Edge* edge = (Edge*)heap_.extract();
    if (!edge)
    {
        cout << endl << "ERROR: No edge exists in the heap!" << endl;
        return false;
    }
    if (isClusterValid(edge->v1) && isClusterValid(edge->v2))
    {
        applyFaceEdgeContraction(edge);
        curr_cluster_num_--;
    }
    else
    {
        cout << "ERROR: This edge does not exist in clusters. Quiting..." << endl;
        return false;
    }
    return true;
}

//! Edge contraction
void Partition::applyFaceEdgeContraction(Edge* edge)
{
    int c1 = edge->v1, c2 = edge->v2;  // two vertices present cluster ids
    mergeClusters(c1, c2);             // merge cluster c2 to c1
    clusters_[c1].cov += clusters_[c2].cov;
    clusters_[c1].energy = clusters_[c1].cov.energy();  // remember to update energy as well

    // NOTE: this function also works but slow, so already computed neighbor clusters in mergeClusters()
    // findClusterNeighbors(c1);

    // Remove all old edges next to c1 and c2 from the heap and corresponding edge lists
    for (Edge* e : clusters_[c1].edges)
    {
        // For each edge (c1, u) or (u, c1), delete this edge and remove it from cluster u's edge list,
        // since cluster c1 and c2's edge list will be cleared later. This is because, each edge pointer is
        // included twice in two clusters, but can only be released once.
        int u = (e->v1 == c1) ? e->v2 : e->v1;
        heap_.remove(e);
        removeEdgeFromCluster(u, e);
        delete e;
    }
    for (Edge* e : clusters_[c2].edges)
    {
        int u = (e->v1 == c2) ? e->v2 : e->v1;
        heap_.remove(e);
        removeEdgeFromCluster(u, e);
        delete e;
    }
    clusters_[c1].edges.clear();
    clusters_[c2].edges.clear();

    // Add new edges between v1 and all its new neighbors into edge list
    for (int cidx : clusters_[c1].nbr_clusters)
    {
        if (c1 > cidx)
            std::swap(c1, cidx);
        Edge* e = new Edge(c1, cidx);
        computeEdgeEnergy(e);
        heap_.insert(e);
        clusters_[c1].edges.push_back(e);
        clusters_[cidx].edges.push_back(e);
    }
}

//! Merge cluster c2 into cluster c1
void Partition::mergeClusters(int c1, int c2)
{
    // Merge all faces from c2 to c1, and update the corresponding cluster index
    for (int fidx : clusters_[c2].faces)
    {
        clusters_[c1].faces.insert(fidx);
        faces_[fidx].cluster_id = c1;
    }
    clusters_[c2].faces.clear();
    // Update relevant cluster neighbors
    for (int cidx : clusters_[c2].nbr_clusters)
    {
        if (cidx != c1)
        {
            clusters_[cidx].nbr_clusters.erase(c2);
            clusters_[cidx].nbr_clusters.insert(c1);
        }
    }
    clusters_[c1].nbr_clusters.insert(clusters_[c2].nbr_clusters.begin(), clusters_[c2].nbr_clusters.end());
    clusters_[c1].nbr_clusters.erase(c1);
    clusters_[c1].nbr_clusters.erase(c2);
    clusters_[c2].nbr_clusters.clear();
}

//! Find neighbor clusters of an input cluster with faces
int Partition::findClusterNeighbors(int cidx, unordered_set<int>& cluster_faces, unordered_set<int>& neighbor_clusters)
{
    neighbor_clusters.clear();
    for (int fidx : cluster_faces)  // brute-force: check all faces in the cluster
    {
        for (int nbr : faces_[fidx].nbr_faces)
        {
            int ncidx = faces_[nbr].cluster_id;
            if (ncidx != cidx)
                neighbor_clusters.insert(ncidx);
        }
    }
    return int(neighbor_clusters.size());
}

//! Overload: find neighbor clusters of an cluster
int Partition::findClusterNeighbors(int cidx)
{
    return findClusterNeighbors(cidx, clusters_[cidx].faces, clusters_[cidx].nbr_clusters);
}

double Partition::getTotalEnergy()
{
    double energy = 0;
    for (int i = 0; i < init_cluster_num_; ++i)
        if (isClusterValid(i))
            energy += clusters_[i].energy;
    return energy;
}

void Partition::createClusterColors()
{
    // srand(time(NULL)); // randomize seed
    for (int i = 0; i < init_cluster_num_; ++i)
        if (isClusterValid(i))  // create a random color
            clusters_[i].color = Vector3f(float(rand()) / RAND_MAX, float(rand()) / RAND_MAX, float(rand()) / RAND_MAX);
}

void Partition::runSwapping()
{
    for (int i = 0; i < init_cluster_num_; ++i)
    {
        if (isClusterValid(i))
            swap_clusters_.insert(i);
    }
    double last_energy = getTotalEnergy();
    double scale = 1e5;  // scale the energy since it is usually too small
    cout << "Energy 0 (x " << scale << "): " << last_energy * scale << endl;
    double curr_energy = 0;
    const int kMaxIter = 300;  // change it as you like
    int iter = 0;
    while (iter++ < kMaxIter)
    {
        int count_swap_faces = swapOnce();
        curr_energy = getTotalEnergy();
        cout << "Energy " << iter << ": " << curr_energy * scale << ", #Swapped faces: " << count_swap_faces << endl;
        if ((last_energy - curr_energy) / last_energy < 1e-7 || count_swap_faces == 0)
            break;
        last_energy = curr_energy;
    }

    processIslandClusters();
}

//! Run swapping step for one time, which means swapping some border face to its neighbor cluster once.
//! Return the number of faces got swapped.
int Partition::swapOnce()
{
    // Find all faces to swap
    vector<int> vec_clusters(swap_clusters_.begin(), swap_clusters_.end());  // cannot use set in OpenMP for loop
#pragma omp parallel for
    for (size_t i = 0; i < vec_clusters.size(); ++i)
    {
        int cidx = vec_clusters[i];
        clusters_[cidx].faces_to_swap.clear();
        for (int fidx : clusters_[cidx].faces)
        {
            unordered_set<int> visited_clusters;
            double max_delta_energy = 0;
            int opt_cidx = -1;
            for (int nbr : faces_[fidx].nbr_faces)
            {
                int ncidx = faces_[nbr].cluster_id;
                if (ncidx != cidx && !visited_clusters.count(ncidx))
                {  // Swapping faces must be in cluster border
                    visited_clusters.insert(ncidx);
                    double delta_energy = computeSwapDeltaEnergy(fidx, cidx, ncidx);
                    // Find the neighbor cluster with max energy decrease
                    if (delta_energy > max_delta_energy)
                    {
                        opt_cidx = ncidx;
                        max_delta_energy = delta_energy;
                    }
                }
            }
            if (opt_cidx != -1)
                clusters_[cidx].faces_to_swap.push_back(std::make_pair(fidx, opt_cidx));
        }
    }

    // Swap faces now
    int count_swap_faces = 0;
    swap_clusters_.clear();
    for (size_t i = 0; i < vec_clusters.size(); ++i)
    {
        int cidx = vec_clusters[i];
        if (clusters_[cidx].faces_to_swap.empty())
            continue;
        int from = cidx;
        for (auto it : clusters_[cidx].faces_to_swap)
        {
            int fidx = it.first;
            int to = it.second;
            faces_[fidx].cluster_id = to;
            clusters_[to].cov += faces_[fidx].cov;
            clusters_[from].cov -= faces_[fidx].cov;
            clusters_[from].faces.erase(fidx);
            clusters_[to].faces.insert(fidx);
            swap_clusters_.insert(from);
            swap_clusters_.insert(to);
        }
        count_swap_faces += clusters_[cidx].faces_to_swap.size();
    }
    // Remember to update the energy each time after updating the covariance object
    for (int cidx : swap_clusters_)
        clusters_[cidx].energy = clusters_[cidx].cov.energy();
    return count_swap_faces;
}

// Compute delta energy (energy changes) by swapping face 'fidx' from cluster 'from' to a neighbor cluster 'to'
double Partition::computeSwapDeltaEnergy(int fidx, int from, int to)
{
    double energy0 = clusters_[from].energy + clusters_[to].energy;
    CovObj cov_from = clusters_[from].cov, cov_to = clusters_[to].cov;
    cov_from -= faces_[fidx].cov;
    cov_to += faces_[fidx].cov;
    double energy1 = cov_from.energy() + cov_to.energy();
    return energy0 - energy1;
}

/*! After swapping step, some clusters may be split into unconnected 'component islands' (like
    one component totally located inside another cluster). This is not good. Here we split each
    of these 'island' clusters into different connected components and merge the island components
    to their neighbor clusters.
*/
void Partition::processIslandClusters()
{
    for (int i = 0; i < face_num_; ++i)
        faces_[i].is_visited = false;
    int count_split_clusters = 0;
	int last_valid_cidx = 0;
    for (int cidx = 0; cidx < init_cluster_num_; ++cidx)
    {
        if (!isClusterValid(cidx))
            continue;
        vector<unordered_set<int>> connected_components;
		auto cmp = [](unordered_set<int>& a, unordered_set<int>& b) { return a.size() > b.size(); };
        if (splitCluster(cidx, connected_components) > 1)
        {
			std::sort(connected_components.begin(), connected_components.end(), cmp);
            mergeIslandComponentsInCluster(cidx, connected_components);
            count_split_clusters++;
			if (connected_components.size() > 1)
			{
				// Create a new cluster for each unmerged component
				for (size_t i = 1; i < connected_components.size(); ++i) // leave the largest component in origial cluster position
				{
					// Find a valid cluster index to 'hold' the component as a new cluster
					int pos = last_valid_cidx;
					while (pos < init_cluster_num_ && isClusterValid(pos))
						pos++; // get next available position
					assert(pos < init_cluster_num_);
					last_valid_cidx = pos;
					clusters_[pos].faces.clear();
					clusters_[pos].cov.clearCov();
					for (int fidx : connected_components[i])
					{
						faces_[fidx].cluster_id = pos;
						clusters_[pos].cov += faces_[fidx].cov;
						clusters_[pos].faces.insert(fidx);
					}
					clusters_[pos].energy = clusters_[pos].cov.energy();
				}
			}
        }
    }
    cout << "#Split clusters: " << count_split_clusters << endl;
}

//! Split faces in one cluster into connected components by breath-first search. Return the component number.
int Partition::splitCluster(int cidx, vector<unordered_set<int>>& connected_components)
{
    int count_left_faces = int(clusters_[cidx].faces.size());
    connected_components.push_back(unordered_set<int>());
    for (int fidx : clusters_[cidx].faces)
    {
        int count = traverseFaceBFS(fidx, cidx, connected_components.back());
        if (count == 0)  // visited face
            continue;
        count_left_faces -= count;
        if (count_left_faces == 0)
            break;  // quit until all faces in the cluster are visited
        connected_components.push_back(unordered_set<int>());
    }
    return int(connected_components.size());
}

//! Traverse (unvisited) faces with BFS. Return the number of visited faces.
int Partition::traverseFaceBFS(int start_fidx, int start_cidx, unordered_set<int>& component)
{
    if (faces_[start_fidx].is_visited)
        return 0;
    faces_[start_fidx].is_visited = true;
    queue<int> qe;  // typical BFS with a queue
    qe.push(start_fidx);
    while (!qe.empty())
    {
        int fidx = qe.front();
        qe.pop();
        component.insert(fidx);
        for (int nbr : faces_[fidx].nbr_faces)
        {
            if (faces_[nbr].is_visited || faces_[nbr].cluster_id != start_cidx)
                continue;
            faces_[nbr].is_visited = true;
            qe.push(nbr);
        }
    }
    return int(component.size());
}

//! Merge each island component from one cluster to its neighbor cluster.
/*!
    NOTE: 'island' component is a connected component with only 1 neighbor cluster.
*/
void Partition::mergeIslandComponentsInCluster(int original_cidx, vector<unordered_set<int>>& connected_components)
{
    if (connected_components.size() <= 1)
        return;
    for (auto iter = connected_components.begin(); iter != connected_components.end();)
    {
        unordered_set<int> neighbors;
        unordered_set<int>& component = *iter;
        int neighbor_num = findClusterNeighbors(original_cidx, component, neighbors);
        if (neighbor_num != 1)
		{
			++iter;
			continue;
		}
        // Merge this component into its neighbor cluster
        int target_cidx = *neighbors.begin();
        for (int fidx : component)
        {
            clusters_[target_cidx].cov += faces_[fidx].cov;
            clusters_[original_cidx].cov -= faces_[fidx].cov;
            clusters_[target_cidx].faces.insert(fidx);
            clusters_[original_cidx].faces.erase(fidx);
            faces_[fidx].cluster_id = target_cidx;
            faces_[fidx].is_visited = false;  // do NOT forget this
        }
        clusters_[original_cidx].energy = clusters_[original_cidx].cov.energy();
        clusters_[target_cidx].energy = clusters_[target_cidx].cov.energy();
		iter = connected_components.erase(iter);
    }
}

//! Merge adjacent planes together if satisfying merging criteria. This is usually some post process step.
void Partition::mergeAdjacentPlanes()
{
    const double kPointPlaneDisThrsd = FLAGS_point_plane_dis_threshold;
    const double kNormalAngleThrsd = cos(M_PI * FLAGS_normal_angle_threshold / 180);
    const double kCenterNormalAngleThrsd = cos(M_PI * FLAGS_center_normal_angle_threshold / 180);
    heap_.destroy();
    for (int cidx = 0; cidx < init_cluster_num_; ++cidx)
    {
        if (!isClusterValid(cidx))
            continue;
        clusters_[cidx].cov.computePlaneNormal();
        findClusterNeighbors(cidx);
    }
    for (int cidx = 0; cidx < init_cluster_num_; ++cidx)
    {
		if (!isClusterValid(cidx))
			continue;
        const Vector3d& n = clusters_[cidx].cov.normal_;
    }
}
