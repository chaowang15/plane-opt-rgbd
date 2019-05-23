#include "partition.h"
#include <stdlib.h>
#include "../common/tools.h"
#include <unordered_map>
#include <random>

Partition::Partition()
{

}


Partition::~Partition()
{
	std::cout << "Releasing edges ... " << std::endl;
	// This will release memory of all pointers, so no need to release 'edges' in each cluster.
	heap_.destroy();
}

//! Read PLY model
/*!
    It supports both binary and ASCII PLY model.
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

void Partition::runPartitionPipeline()
{
	curr_cluster_num_ = face_num_;
	assert(target_cluster_num_ < curr_cluster_num_ && target_cluster_num_ > 0);

    printInGreen("Mesh partition by merging neighbor faces.");
    if (!runMerging())
		return;

    printInGreen("(Optional) Optimization by swapping neighbor faces between clusters.");
	runSwapping();

    printInGreen("Merge neighbor clusters.");

    printInCyan("#Final Clusters: " + std::to_string(curr_cluster_num_));

	createClusterColors();

}

bool Partition::runMerging()
{
    initMerging();

	cout << "Merging ..." << endl;
	float progress = 0.0; // for printing a progress bar
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
		count++;
	}
	assert(curr_cluster_num_ == target_cluster_num_);

	cout << "Energy: " << getTotalEnergy() << endl;
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
    float progress = 0.0; // used to print a progress bar
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
        clusters_[fidx].cov = clusters_[fidx].ini_cov = Q;
    }

    // Initialize edges
    cout << "Initialize edges ... " << endl;
	progress = 0;
    for (int cidx = 0; cidx < curr_cluster_num_; cidx++)
    {
		if (cidx % kStep == 0 || cidx == face_num_ - 1)
		{
			progress = (cidx == face_num_ - 1) ? 1.0 : static_cast<float>(cidx) / face_num_;
			printProgressBar(progress);
		}
		Cluster& cluster = clusters_[cidx];
        for (int nbr : cluster.nbr_clusters)
        {
            if (cidx < nbr)
            {
				Edge *edge = new Edge(cidx, nbr);
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
	edge->heap_key(-energy); // it's a max heap by default but we need a min heap
}

//! Remove one edge pointer from a cluster. Note that it doesn't delete/release the pointer.
bool Partition::removeEdgeFromCluster(int cidx, Edge* edge)
{
	if (edge == nullptr) return false;
	bool flag_found_edge = false;
	auto iter = clusters_[cidx].edges.begin();
	while (iter != clusters_[cidx].edges.end())
	{
		Edge* e = *iter;
		if (e == nullptr || e != edge)
		{
			iter++;
		}
		else
		{
			iter = clusters_[cidx].edges.erase(iter);
			flag_found_edge = true; // Not return here since there may be duplicate edges
		}
	}
	return flag_found_edge;
}

bool Partition::mergeOnce()
{
	Edge* edge = (Edge*)heap_.extract();
	if (!edge)
	{
		cout << "ERROR: No edge exists in the heap. Quitting..." << endl;
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
	int c1 = edge->v1, c2 = edge->v2; // two vertices present cluster ids
	mergeClusters(c1, c2); // merge cluster c2 to c1
	clusters_[c1].cov += clusters_[c2].cov;
	clusters_[c1].energy = clusters_[c1].cov.energy(); // remember to update energy as well

	// Get all neighbors of the new merged cluster v1
	findClusterNeighbors(c1);

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
		Edge *e = new Edge(c1, cidx);
		clusters_[c1].edges.push_back(e);
		clusters_[cidx].edges.push_back(e);
	}

	// Insert all new edges
	for (Edge* e : clusters_[c1].edges)
	{
		computeEdgeEnergy(e);
		heap_.insert(e);
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
}

//! Find neighbor clusters of an input cluster with faces
int Partition::findClusterNeighbors(int cidx, unordered_set<int>& cluster_faces, unordered_set<int>& neighbor_clusters)
{
	neighbor_clusters.clear();
	for (int fidx : cluster_faces) // brute-force: check all faces in the cluster
	{
		for (int nbr : faces_[fidx].nbr_faces)
		{
			int target_id = faces_[nbr].cluster_id;
			if (target_id != cidx)
				neighbor_clusters.insert(target_id);
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
	for (int i = 0; i < face_num_; ++i)
		if (isClusterValid(i))
			energy += clusters_[i].energy;
	return energy;
}

void Partition::createClusterColors()
{
	//srand(time(NULL)); // randomize seed
	for (int i = 0; i < face_num_; ++i)
		if (isClusterValid(i)) // create a random color
			clusters_[i].color = Vector3f(float(rand()) / RAND_MAX, float(rand()) / RAND_MAX, float(rand()) / RAND_MAX);
}


void Partition::runSwapping()
{

}
