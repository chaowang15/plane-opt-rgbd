#include "mesh_opt.h"
#include <Eigen/SparseCholesky>
#include <queue>
#include <fstream>
#include "../common/tools.h"

// DEFINE_string(frame_filename_suffix, std::string(".color.png"), "");
DEFINE_int32(rgbd_frame_gap, 5, "select 1 keyframe from every 'rgbd_frame_gap' frames");
DEFINE_int32(texture_image_resolution, 4096, "4096 x 4096 texture image by default");
DEFINE_int32(image_border_width, 5, "");
DEFINE_int32(global_opt_loop_number, 10, "0 to skip the entire optimization");
DEFINE_int32(pose_opt_loop_number, 5, "0 to completely skip pose optimization");
DEFINE_int32(plane_opt_loop_number, 2, "0 to completely skip plane optimization");

DEFINE_double(closest_pose_translation, 0.05, "in meter");
DEFINE_double(closest_pose_rotation_angle, 0.09, "in radians. About root of squared sum of 3 degree per axis");
DEFINE_double(patch_boundingbox_borderwidth, 0.02, "in meter. Pad each patch rectangle with a small width on the border.");
DEFINE_double(
    unit_meter_resolution, 300, "number of pixels in 1cm in the output texture image. Must >=300 according to experiments.");
DEFINE_bool(use_noisy_poses, false, "for debug");
DEFINE_bool(use_opt_geometry, true, "false: use original mesh; true: optimized mesh");
DEFINE_bool(run_opt_geometry, true, "false to skip the geometry optimization");

RGBDMeshOpt::RGBDMeshOpt() {}

RGBDMeshOpt::~RGBDMeshOpt() {}

/************************************************************************/
/* Data I/O
 */
/************************************************************************/

//! Read PLY model
/*!
    This function supports PLY model with:
    - both binary and ASCII format;
    - 3 RGB channel vertex color and face color;
    - 3-dim vertex normal (even though doesn't save it);
    - 1-dim vertex quality (even though doesn't save it);
*/
bool RGBDMeshOpt::readPLY(const std::string& filename)
{
    FILE* fin;
    if (!(fin = fopen(filename.c_str(), "rb")))
    {
        cout << "ERROR: Unable to open file" << filename << endl;
        return false;
    };

    /************************************************************************/
    /* Read headers */

    // Mode for vertex and face type
    // 1 for vertex (no faces), 2 for vertex and faces,
    // 3 for vertex, vertex colors (no faces), 4 for vertex, vertex colors and faces
    int vertex_mode = 1;
    int ply_mode = 0;  // 0 for ascii, 1 for little-endian (binary)
    size_t vertex_color_channel = 0, face_color_channel = 0;
    size_t vertex_quality_dim = 0;  // vertex quality, any kind of float value per vertex defined by user
    size_t vertex_normal_dim = 0;   // vertex normal dimension
    char seps[] = " ,\t\n\r ";      // separators
    seps[5] = 10;
    int property_num = 0;
    char line[1024];
    while (true)
    {
        if (fgets(line, 1024, fin) == NULL)
            continue;
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
                cout << "ERROR in Reading PLY model: can not read this type of PLY model: " << string(token) << endl;
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
                        if (!strcmp(token, "red") || !strcmp(token, "green") || !strcmp(token, "blue") ||
                            !strcmp(token, "alpha"))
                            vertex_color_channel++;
                        else
                        {
                            cout << "ERROR in Reading PLY model: cannot read this vertex color type -- " << string(token)
                                 << endl;
                            return false;
                        }
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
                if (!strcmp(token, "list"))  // face component
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
                else if (!strcmp(token, "uchar"))  // face color
                {
                    token = strtok(NULL, seps);
                    if (!strcmp(token, "red") || !strcmp(token, "green") || !strcmp(token, "blue") || !strcmp(token, "alpha"))
                        face_color_channel++;
                    else
                    {
                        cout << "ERROR in Reading PLY model: cannot read this face color type -- " << string(token) << endl;
                        return false;
                    }
                }
            }
        }
    }
    if (vertex_color_channel != 0 && vertex_color_channel != 3 && vertex_color_channel != 4)
    {
        cout << "ERROR: Vertex color channel is " << vertex_color_channel << " but it has to be 0, 3, or 4." << endl;
        return false;
    }
    if (face_color_channel != 0 && face_color_channel != 3 && face_color_channel != 4)
    {
        cout << "ERROR: Face color channel is " << face_color_channel << " but it has to be 0, 3, or 4." << endl;
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
            // Vertex data order must be:
            // coordinates -> normal -> color -> others (qualities, radius, curvatures, etc)
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
                // NOTE: currently we just abandon the vertex normal
            }
            if (vertex_color_channel)
            {
                unsigned char color[4];
                if ((haveread = fread(color, sizeof(unsigned char), vertex_color_channel, fin)) != vertex_color_channel)
                {
                    cout << "ERROR in reading PLY vertex colors in position " << ftell(fin) << endl;
                    return false;
                }
                // NOTE: currently abandon the vertex color data
            }
            if (vertex_quality_dim)
            {
                float qual[3];  // Currently we just abandon the vertex quality data
                if ((haveread = fread(qual, sizeof(float), vertex_quality_dim, fin)) != vertex_quality_dim)
                {
                    cout << "ERROR in reading PLY vertex qualities in position " << ftell(fin) << endl;
                    return false;
                }
                // NOTE: currently abandon the vertex quality data
            }
            // You can still read other types of data here

            // Save vertex
            vtx.pt3 = Vector3d(vert[0], vert[1], vert[2]);
            vtx.opt_pt3 = vtx.pt3;
            vertices_.push_back(vtx);
        }

        // Face data
        for (int i = 0; i < face_num_; ++i)
        {
            unsigned char channel_num;
            size_t haveread = fread(&channel_num, 1, 1, fin);  // channel number for each face
            Face fa;
            // Face data order must be: face indices -> face color -> others
            if ((haveread = fread(fa.indices, sizeof(int), 3, fin)) != 3)  // currently only support triangular face
            {
                cout << "ERROR in reading PLY face indices: reader position " << ftell(fin) << endl;
                return false;
            }
            if (face_color_channel)
            {
                unsigned char color[4];
                if ((haveread = fread(color, sizeof(unsigned char), face_color_channel, fin)) != face_color_channel)
                {
                    cout << "ERROR in reading PLY face colors: reader position " << ftell(fin) << endl;
                    return false;
                }
                // NOTE: currently we only read face color data but don't save them, since in this program, face color
                // is the same as the corresponding cluster color which is stored and read in the cluster file.
            }
            // Read other types of face data here.

            // Save face
            faces_.push_back(fa);
        }
    }
    else  // ASCII mode (face reader is still unfinished)
    {
        // Read vertices (with C functions)
        for (int i = 0; i < vertex_num_; i++)
        {
            // Vertex data order must be:
            // coordinates -> normal -> color -> others (qualities, radius, curvatures, etc)
            if (fgets(line, 1024, fin) == NULL)
                continue;
            char* token = strtok(line, seps);
            // Read 3D point
            Vertex vtx;
            float vert[3];
            for (size_t j = 0; j < 3; ++j)
            {
                token = strtok(NULL, seps);
                sscanf(token, "%f", &(vert[j]));
            }
            // Read vertex normal
            if (vertex_normal_dim)
            {
                float nor[3];
                for (size_t j = 0; j < vertex_normal_dim; ++j)
                {
                    token = strtok(NULL, seps);
                    sscanf(token, "%f", &(nor[j]));
                }
                // Currently we just abandon the vertex normal data
            }
            if (vertex_color_channel)
            {
                unsigned char color[4];
                for (size_t j = 0; j < vertex_quality_dim; ++j)
                {
                    token = strtok(NULL, seps);
                    sscanf(token, "%c", &(color[j]));
                }
                // Currently we just abandon vertex color
            }
            if (vertex_quality_dim)
            {
                float qual;
                for (size_t j = 0; j < vertex_quality_dim; ++j)
                {
                    token = strtok(NULL, seps);
                    sscanf(token, "%f", &qual);
                }
                // Currently abandon the vertex quality data
            }
            // Read other types of vertex data here.

            // Save vertex
            vtx.pt3 = Vector3d(vert[0], vert[1], vert[2]);
            vtx.opt_pt3 = vtx.pt3;
            vertices_.push_back(vtx);
        }
        // Read Faces
        for (int i = 0; i < face_num_; i++)
        {
            if (fgets(line, 1024, fin) == NULL)
                continue;
            char* token = strtok(line, seps);
            token = strtok(NULL, seps);
            Face fa;
            for (int j = 0; j < 3; ++j)
            {
                token = strtok(NULL, seps);
                sscanf(token, "%d", &(fa.indices[j]));
            }
            if (face_color_channel)
            {
                unsigned char color[4];
                for (int j = 0; j < 4; ++j)
                {
                    token = strtok(NULL, seps);
                    sscanf(token, "%c", &(color[j]));
                }
                // Currently abandon the face color data
            }
            faces_.push_back(fa);
        }
    }

    // Just in case some vertices or faces are not read correctly
    face_num_ = static_cast<int>(faces_.size());
    vertex_num_ = static_cast<int>(vertices_.size());
    return true;
}

//! Read cluster file. Note it can only be used when there are no existing clusters, like call this function
//! just after reading PLY file and haven't run any other partition functions.
bool RGBDMeshOpt::readClusterFile(const std::string& filename)
{
    if (vertex_num_ == 0 || face_num_ == 0)
    {
        PRINT_RED("ERROR: must read the mesh at first! ");
        return false;
    }
    FILE* fin = fopen(filename.c_str(), "rb");
    if (fin == NULL)
    {
        PRINT_RED("ERROR: cannot find cluster file %s", filename.c_str());
        return false;
    }
    if (fread(&cluster_num_, sizeof(int), 1, fin) != 1)
    {
        PRINT_RED("ERROR in reading cluster number in cluster file %s", filename.c_str());
        return false;
    }
    if (cluster_num_ < 1)
    {
        PRINT_RED("ERROR: cluster number is %d", cluster_num_);
        return false;
    }
    clusters_.resize(cluster_num_);
    float color[3];
    for (int i = 0; i < cluster_num_; ++i)
    {
        int cidx = -1, cluster_size = -1;
        if (fread(&cidx, sizeof(int), 1, fin) != 1)
            return false;
        if (fread(&cluster_size, sizeof(int), 1, fin) != 1)
            return false;
        assert(cidx >= 0 && cidx < face_num_ && cluster_size >= 0 && cluster_size <= face_num_);
        vector<int> cluster_elems(cluster_size);
        if (fread(&cluster_elems[0], sizeof(int), cluster_size, fin) != size_t(cluster_size))
        {
            PRINT_RED("ERROR in reading indices in cluster file %s", filename.c_str());
            return false;
        }
        clusters_[i].faces.insert(cluster_elems.begin(), cluster_elems.end());
        if (fread(&color[0], sizeof(float), 3, fin) != 3)
        {
            PRINT_RED("ERROR in reading colors in cluster file %s", filename.c_str());
            return false;
        }
        for (int j = 0; j < 3; ++j)
            clusters_[i].color[j] = color[j];
    }
    fclose(fin);
    return true;
}

bool RGBDMeshOpt::readCameraParamFile(const string& filename)
{
    ifstream readin(filename, ios::in);
    if (readin.fail() || readin.eof())
    {
        PRINT_RED("ERROR: Cannot read camera parameter file %s", filename.c_str());
        return false;
    }
    string str_line, str_dummy, str;
    float dummy;
    while (!readin.eof() && !readin.fail())
    {
        getline(readin, str_line);
        if (readin.eof())
            break;
        istringstream iss(str_line);
        iss >> str;
        if (str == "m_colorWidth")
            iss >> str_dummy >> color_width_;
        else if (str == "m_colorHeight")
            iss >> str_dummy >> color_height_;
        else if (str == "m_depthWidth")
            iss >> str_dummy >> depth_width_;
        else if (str == "m_depthHeight")
            iss >> str_dummy >> depth_height_;
        else if (str == "m_calibrationColorIntrinsic")
        {
            iss >> str_dummy >> color_calib_.fx >> dummy >> color_calib_.cx >> dummy >> dummy >> color_calib_.fy >>
                color_calib_.cy;
            color_calib_.setCalibMatrix();
        }
        else if (str == "m_calibrationDepthIntrinsic")
        {
            iss >> str_dummy >> depth_calib_.fx >> dummy >> depth_calib_.cx >> dummy >> dummy >> depth_calib_.fy >>
                depth_calib_.cy;
            depth_calib_.setCalibMatrix();
        }
    }
    readin.close();
    color_calib_.width = color_width_;
    color_calib_.height = color_height_;
    depth_calib_.width = depth_width_;
    depth_calib_.height = depth_height_;
    if (!color_calib_.isValid() || !depth_calib_.isValid())
    {
        PRINT_RED("ERROR: camera parameter information is missing in file %s", filename.c_str());
        return false;
    }
    return true;
}

bool RGBDMeshOpt::readImageBlurrinessFile(const string& filename)
{
    ifstream readin(filename, ios::in);
    if (readin.fail() || readin.eof())
    {
        PRINT_RED("ERROR: Cannot read image blurriness file %s", filename.c_str());
        return false;
    }
    while (!readin.eof() && !readin.fail())
    {
        int frame_idx = -1;
        double blurriness = 0;
        readin >> frame_idx >> blurriness;
        if (readin.eof())
            break;
        if (frame_idx != -1)
            image_blurriness_[frame_idx] = blurriness;
    }
    return true;
}

bool RGBDMeshOpt::readCameraPoseFile(const string& filename, Matrix4d& T)
{
    ifstream readin(filename, ios::in);
    if (readin.fail())
    {
        PRINT_RED("ERROR: cannot open the pose file %s", filename.c_str());
        return false;
    }
    // NOTE: each pose must be a 4x4 matrix and here we don't check the validity.
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            readin >> T(i, j);
    readin.close();
    return true;
}

//! NOTE: this function must be called after 'readImageBlurrinessFile()', since image
//! blurriness data will be used here to get keyframes.
bool RGBDMeshOpt::readRGBDFrames(
    const string& rgbd_path, const string& visibility_path, int data_type, int start_fidx, int end_fidx)
{
    string color_image_format(".jpg");  // BundleFusion data by default
    depth_scale_factor_ = 1000;
    if (data_type == 1)  // ICL-NUIM data
    {
        color_image_format = ".png";
        depth_scale_factor_ = 5000;
    }
    int curr_fidx = start_fidx;
    while (curr_fidx <= end_fidx)
    {
        int end = std::min(curr_fidx + FLAGS_rgbd_frame_gap, end_fidx + 1);
        double min_blurriness = 1e5;
        int keyframe_idx = -1;
        // Find a keyframe with smallest image blurriness value
        for (int start = curr_fidx; start < end; ++start)
        {
            if (image_blurriness_.find(start) == image_blurriness_.end())
            {
                PRINT_RED("ERROR: No blurriness for frame %d. Quit now.", start);
                return false;
            }
            if (image_blurriness_[start] < min_blurriness)
            {
                keyframe_idx = start;
                min_blurriness = image_blurriness_[start];
            }
        }
        assert(keyframe_idx != -1);

        string str_frame_idx = to_string(keyframe_idx);
        // NOTE: default frame filename is like 'frame-000001.suffix'. Fix its format here.
        string frame_fname = "frame-" + string(6 - str_frame_idx.length(), '0') + str_frame_idx;
        Matrix4d T;
        if (!readCameraPoseFile(rgbd_path + frame_fname + ".pose.txt", T))
            return false;
        if (frames_.empty() || !isTwoPosesClose(frames_.back().T, T))
        {  // Only add a new keyframe if it is NOT too close with previous keyframe

            cv::Mat color_img, depth_img;
            vector<int> visible_vertices;
            if (!readColorImg(rgbd_path + frame_fname + ".color" + color_image_format, color_img) ||
                !readDepthImg(rgbd_path + frame_fname + ".depth.png", depth_img) ||
                !readVisibilityFile(visibility_path + frame_fname + ".visibility.txt", visible_vertices))
            {  // NOTE: all depth images are in png in different data types, while color image is in jpg or png.
                return false;
            }
            Frame frame;
            frame.color_img = std::move(color_img);
            frame.depth_img = std::move(depth_img);
            frame.visible_vertices = std::move(visible_vertices);
            frame.T = std::move(T);
            frames_.push_back(frame);

            // cout << "Adding keyframe " << keyframe_idx << endl;
            // for (int vidx : frame.visible_vertices)
            //     cout << vidx << " ";
            // cout << endl;
        }
        curr_fidx += FLAGS_rgbd_frame_gap;
    }
    cout << "#Keyframes: " << frames_.size() << endl;
    return true;
}

bool RGBDMeshOpt::readColorImg(const string& filename, cv::Mat& img)
{
    img = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
    if (img.empty() || img.depth() != CV_8U)
    {
        PRINT_RED("ERROR: cannot read color image %s", filename.c_str());
        return false;
    }
    return true;
}

bool RGBDMeshOpt::readDepthImg(const string& filename, cv::Mat& img)
{
    img = cv::imread(filename, CV_LOAD_IMAGE_ANYDEPTH);
    if (img.empty() || img.depth() != CV_16U)
    {
        PRINT_RED("ERROR: cannot read depth image %s", filename.c_str());
        return false;
    }
    return true;
}

bool RGBDMeshOpt::readVisibilityFile(const string& filename, vector<int>& visible_vertices)
{
    FILE* fin = fopen(filename.c_str(), "rb");
    if (fin == NULL)
    {
        PRINT_RED("ERROR: cannot read visibility file %s", filename.c_str());
        return false;
    }
    int num = -1;
    if (fread(&num, sizeof(int), 1, fin) != 1)
        return false;
    if (num <= 0)
    {
        PRINT_YELLOW("WARNING: number of visible vertices in file %s is <= 0", filename.c_str());
        return true;
    }
    visible_vertices.clear();
    visible_vertices.resize(num);
    if (fread(&visible_vertices[0], sizeof(int), num, fin) != size_t(num))
    {
        PRINT_RED("ERROR in reading visibility indices in file %s", filename.c_str());
        return false;
    }
    fclose(fin);
    return true;
}

bool RGBDMeshOpt::savePLY(const string& filename)
{
    FILE* fout = NULL;
    fout = fopen(filename.c_str(), "wb");  // write in binary mode
    if (fout == NULL)
    {
        PRINT_RED("Unable to create file %s", filename.c_str());
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
        {
            if (FLAGS_use_opt_geometry == 0)
                pt3[j] = float(vertices_[i].opt_pt3[j]);
            else
                pt3[j] = float(vertices_[i].pt3[j]);
        }
        fwrite(pt3, sizeof(float), 3, fout);
    }
    for (int i = 0; i != face_num_; ++i)
    {
        fwrite(&kFaceVtxNum, sizeof(unsigned char), 1, fout);
        fwrite(faces_[i].indices, sizeof(int), 3, fout);
        int cidx = faces_[i].cluster_id;
        if (cidx == -1)
        {
            // Will use default color for this
            PRINT_YELLOW("WARNING: face %d doesn't belong to any cluster!", i);
        }
        else
        {
            for (int j = 0; j < 3; ++j)
                rgba[j] = (unsigned char)(clusters_[cidx].color[j] * 255);
        }
        fwrite(rgba, sizeof(unsigned char), 4, fout);
    }
    fclose(fout);
    return true;
}

void RGBDMeshOpt::saveTexturedMesh(const string& obj_fname)
{
    string model_fname = obj_fname.substr(0, obj_fname.length() - 4);

    // Save texture images
    vector<string> texture_image_fnames(texture_images_.size());
    for (size_t i = 0; i < texture_images_.size(); ++i)
    {
        texture_image_fnames[i] = model_fname + "_" + to_string(i) + ".png";
        cout << "Saving texture image " << texture_image_fnames[i] << endl;
        cv::imwrite(texture_image_fnames[i], texture_images_[i]);
    }

    // Save MTL file
    string mtl_fname = model_fname + ".mtl";
    FILE* fout = fopen(mtl_fname.c_str(), "w");
    if (fout == NULL)
    {
        PRINT_RED("Unable to create file %s", mtl_fname.c_str());
        return;
    }
    for (int i = 0; i < int(texture_image_fnames.size()); ++i)
    {
        fprintf(fout, "newmtl material%d\n", i);
        fprintf(fout,
            "Ka 1.000000 1.000000 1.000000\n"
            "Kd 1.000000 1.000000 1.000000\n"
            "Ks 0.000000 0.000000 0.000000\n"
            "Tr 1.000000\n"
            "illum 1\n"
            "Ns 0.000000\n"
            "map_Kd %s\n\n",
            texture_image_fnames[i].c_str());
    }
    fclose(fout);

    // Save obj file
    fout = fopen(obj_fname.c_str(), "w");  // write in binary mode
    if (fout == NULL)
    {
        PRINT_RED("Unable to create obj file %s", obj_fname.c_str());
        return;
    }
    fprintf(fout, "mtllib %s\n", mtl_fname.c_str());
    for (int i = 0; i < vertex_num_; ++i)
    {
        if (FLAGS_use_opt_geometry)
            fprintf(fout, "v %f %f %f\n", vertices_[i].opt_pt3[0], vertices_[i].opt_pt3[1], vertices_[i].opt_pt3[2]);
        else
            fprintf(fout, "v %f %f %f\n", vertices_[i].pt3[0], vertices_[i].pt3[1], vertices_[i].pt3[2]);
    }
    int vidx = 1;  // vertex texture index starts from 1 instead of 0 in obj file
    for (TexturePatch& patch : patches_)
    {
        patch.base_vtx_index = vidx;  // start vertex index for the first vertex of the patch
        for (size_t i = 0; i < patch.uv_textures.size(); ++i)
            fprintf(fout, "vt %f %f\n", patch.uv_textures[i][0], patch.uv_textures[i][1]);
        vidx += patch.uv_textures.size();
    }
    for (TexturePatch& patch : patches_)
    {
        fprintf(fout, "usemtl material%d\n", patch.texture_img_idx);
        int cidx = patch.cluster_id;
        for (int fidx : clusters_[cidx].faces)
        {
            int v0 = faces_[fidx].indices[0], v1 = faces_[fidx].indices[1], v2 = faces_[fidx].indices[2];
            fprintf(fout, "f %d/%d %d/%d %d/%d\n", v0 + 1, patch.base_vtx_index + patch.vertex_to_patch[v0], v1 + 1,
                patch.base_vtx_index + patch.vertex_to_patch[v1], v2 + 1, patch.base_vtx_index + patch.vertex_to_patch[v2]);
        }
    }
    fclose(fout);
}

/************************************************************************/
/* Pipeline
 */
/************************************************************************/

void RGBDMeshOpt::initMeshConnectivity()
{
    cout << "Initialize vertices and faces ... " << endl;
    unordered_map<long long, vector<int>> edge_to_face;
    vector<int> fa(3);
    float progress = 0.0;  // used to print a progress bar
    const int kStep = (face_num_ < 100) ? 1 : (face_num_ / 100);
    for (int fidx = 0; fidx < face_num_; fidx++)
    {
        // Print a progress bar
        if (fidx % kStep == 0 || fidx == face_num_ - 1)
        {
            progress = (fidx == face_num_ - 1) ? 1.0f : static_cast<float>(fidx) / face_num_;
            printProgressBar(progress);
        }
        Face& face = faces_[fidx];
        for (int i = 0; i < 3; ++i)
            fa[i] = face.indices[i];
        // We use UNDIRECTED edges, so sort the 3 vertices to avoid edge duplication.
        // So each edge now may be included in 1, 2 or even 3 or more (non-manifold mesh) faces.
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
            }
            edge_to_face[edge].push_back(fidx);
        }
    }
}

//! Overload: find neighbor clusters of an cluster
int RGBDMeshOpt::findClusterNeighbors(int cidx)
{
    clusters_[cidx].nbr_clusters.clear();
    for (int fidx : clusters_[cidx].faces)  // brute-force: check all faces in the cluster
    {
        for (int nbr : faces_[fidx].nbr_faces)
        {
            int ncidx = faces_[nbr].cluster_id;
            if (ncidx != cidx)
                clusters_[cidx].nbr_clusters.insert(ncidx);
        }
    }
    return int(clusters_[cidx].nbr_clusters.size());
}

void RGBDMeshOpt::runOptimizationPipeline()
{
    PRINT_CYAN("Initialization.");
    initAll();

    PRINT_CYAN("Plane and camera pose optimization.");
    runPlaneAndCameraPoseOpt();

    PRINT_CYAN("Geometry optimization.");
    if (FLAGS_run_opt_geometry)
        runMeshGeometryOpt();

    PRINT_CYAN("Compute texture image colors.");
    generateFinalTexelColors();
}

void RGBDMeshOpt::runPlaneAndCameraPoseOpt()
{
    cout << "Running plane and camera pose optimization ..." << endl;
    last_global_energy_ = 1e10;  // some large number
    lambda1_ = 0;
    for (int loop = 0; loop < FLAGS_global_opt_loop_number; ++loop)
    {
        cout << "------------------------------------------" << endl;
        cout << "Loop " << loop << ":" << endl;
        cout << "Pose optimization: " << endl;
        optimizePoses();
        cout << "Plane optimization: " << endl;
        optimizePlanes();
        cout << "Color optimization ..." << endl;
        computeAllTexelColors();
    }
    cout << "DONE." << endl;
}

//! Optimize camera poses in all frames
void RGBDMeshOpt::optimizePoses()
{
    VectorXd jrow(6);
    for (int iter = 0; iter < FLAGS_pose_opt_loop_number; ++iter)
    {
        double energy1 = 0, energy2 = 0;  // energy 1 is for color difference, energy2 is for point-plane distance
        for (int fidx = 0; fidx < frame_num_; ++fidx)
        {
            frames_[fidx].JTJ.setZero();
            frames_[fidx].JTr.setZero();
        }
        for (TexturePatch& patch : patches_)
        {
            int cidx = patch.cluster_id;
            int tidx = patch.texture_img_idx;
            const Vector3d& kNormal = clusters_[cidx].opt_normal;
            const double& kW = clusters_[cidx].opt_w;
            for (auto it : patch.texel_positions)
            {
                int x = it.first, y = it.second;
                Texel& texel = texels_[tidx][y][x];
                int fa = texel.face_id;
                double opt_graycolor = texel.opt_graycolor;
                bool flag_run_color_opt = false;
                for (int fidx : faces_[fa].visible_frames)
                {
                    Vector3d pt3_local = globalToCameraSpace(texel.pt3_proj, frames_[fidx].opt_inv_T);
                    Vector2d pt2_color;
                    if (!isCameraPointVisibleInFrame(pt3_local, fidx, pt2_color))
                        continue;
                    Vector2d grad = compute2DPointGraycolorGradientBilinear(pt2_color, fidx);
                    // Compute Jacobian of energy1 (color difference term) w.r.t. delta pose
                    // Refer to math derivation for more details.
                    double x = pt3_local[0], y = pt3_local[1], z = pt3_local[2];
                    double a = grad[0] * color_calib_.fx / z;
                    double b = grad[1] * color_calib_.fy / z;
                    double c = -(a * x + b * y) / z;
                    jrow[0] = -b * z + c * y;
                    jrow[1] = a * z - c * x;
                    jrow[2] = -a * y + b * x;
                    jrow[3] = a;
                    jrow[4] = b;
                    jrow[5] = c;
                    double r = compute2DPointGraycolorBilinear(pt2_color, fidx) - opt_graycolor;
                    for (int i = 0; i < 6; ++i)
                    {
                        frames_[fidx].JTr(i, 0) += jrow[i] * r;
                        frames_[fidx].JTJ(i, i) += jrow[i] * jrow[i];
                        for (int j = i + 1; j < 6; ++j)
                        {
                            double val = jrow[i] * jrow[j];
                            frames_[fidx].JTJ(i, j) += val;
                            frames_[fidx].JTJ(j, i) += val;
                        }
                    }
                    energy1 += r * r;
                    flag_run_color_opt = true;
                }
                if (flag_run_color_opt)
                {
                    double dis_pt2plane = texel.pt3_global.dot(kNormal) + kW;
                    energy2 += dis_pt2plane * dis_pt2plane;
                }
            }
        }
        if (lambda1_ == 0 && energy2 != 0)
            lambda1_ = energy1 / energy2;
        energy2 *= lambda1_;
        curr_global_energy_ = energy1 + energy2;
        cout << "Energy (iter " << iter << "): " << curr_global_energy_ << " (" << energy1 << " + " << energy2 << ")" << endl;
        if (last_global_energy_ < curr_global_energy_)
        {
            // Recover result in last iteration which is better.
            for (int fidx = 0; fidx < frame_num_; ++fidx)
                frames_[fidx].opt_inv_T = frames_[fidx].lastT;
            curr_global_energy_ = last_global_energy_;
            break;
        }
        last_color_energy_ = energy1;
        last_global_energy_ = curr_global_energy_;
        for (int fidx = 0; fidx < frame_num_; ++fidx)
        {
            MatrixXd Xi = -frames_[fidx].JTJ.llt().solve(frames_[fidx].JTr);
            bool flag_is_Xi_finite = true;
            for (int i = 0; i < 6; ++i)
            {
                if (!isfinite(Xi(i, 0)))
                {
                    PRINT_YELLOW("WARNING: camera pose in frame %d cannot be optimized more.", fidx);
                    flag_is_Xi_finite = false;
                    break;
                }
            }
            if (!flag_is_Xi_finite)
                continue;
            // Recover rotation matrix via. Rodrigues' formula
            Eigen::Affine3d aff_mat;
            aff_mat.linear() = (Eigen::Matrix3d)Eigen::AngleAxisd(Xi(2), Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(Xi(1), Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(Xi(0), Eigen::Vector3d::UnitX());
            aff_mat.translation() = Eigen::Vector3d(Xi(3), Xi(4), Xi(5));
            Matrix4d delta = aff_mat.matrix();
            frames_[fidx].lastT = frames_[fidx].opt_inv_T;
            frames_[fidx].opt_inv_T = delta * frames_[fidx].opt_inv_T;
        }
    }
    // Set all relevant variables w.r.t. optimized poses
    for (int fidx = 0; fidx < frame_num_; ++fidx)
    {
        frames_[fidx].opt_inv_R = frames_[fidx].opt_inv_T.block(0, 0, 3, 3);
        frames_[fidx].opt_inv_t = frames_[fidx].opt_inv_T.block(0, 3, 3, 1);
        frames_[fidx].opt_T = frames_[fidx].opt_inv_T.inverse();
        frames_[fidx].opt_R = frames_[fidx].opt_T.block(0, 0, 3, 3);
        frames_[fidx].opt_t = frames_[fidx].opt_T.block(0, 3, 3, 1);
    }
}

void RGBDMeshOpt::optimizePlanes()
{
    Vector3d res_p2pixel;
    MatrixXd m13(1, 3), m34(3, 4), m14(1, 4), Xi(4, 1);
    const double kSqrtLambda1 = sqrt(lambda1_);
    for (int iter = 0; iter < FLAGS_plane_opt_loop_number; ++iter)
    {
        for (int cidx = 0; cidx < cluster_num_; ++cidx)
        {
            clusters_[cidx].JTJ.setZero();
            clusters_[cidx].JTr.setZero();
            clusters_[cidx].is_optimized = false;
        }
        double energy1 = 0, energy2 = 0;
        for (TexturePatch& patch : patches_)
        {
            int cidx = patch.cluster_id;
            int tidx = patch.texture_img_idx;
            const Vector3d& kNormal = clusters_[cidx].opt_normal;
            const double& kW = clusters_[cidx].opt_w;
            for (auto it : patch.texel_positions)
            {
                int x = it.first, y = it.second;
                Texel& texel = texels_[tidx][y][x];
                int fa = texel.face_id;
                double opt_graycolor = texel.opt_graycolor;
                double dis_pt2plane = texel.pt3_global.dot(kNormal) + kW;
                texel.pt3_proj = texel.pt3_global - dis_pt2plane * kNormal;
                bool flag_run_color_opt = false;
                for (int fidx : faces_[fa].visible_frames)
                {
                    Vector3d pt3_local = globalToCameraSpace(texel.pt3_proj, frames_[fidx].opt_inv_T);
                    Vector2d pt2_color;
                    if (!isCameraPointVisibleInFrame(pt3_local, fidx, pt2_color))
                        continue;
                    Vector2d grad = compute2DPointGraycolorGradientBilinear(pt2_color, fidx);
                    // Compute Jacobian of energy1 (color difference term) w.r.t. plane normal and w
                    // Refer to math derivation for more details.
                    double x = pt3_local[0], y = pt3_local[1], z = pt3_local[2];
                    m13(0, 0) = grad[0] * color_calib_.fx / z;
                    m13(0, 1) = grad[1] * color_calib_.fy / z;
                    m13(0, 2) = -(m13(0, 0) * x + m13(0, 1) * y) / z;
                    Vector3d Rjni = frames_[fidx].opt_inv_R * kNormal;
                    for (int i = 0; i < 3; ++i)
                    {
                        m34(0, i) = -Rjni[i];
                        for (int j = 0; j < 3; ++j)
                            m34(i, j) = -Rjni(i) * texel.pt3_global[j] - dis_pt2plane * frames_[fidx].opt_inv_R(i, j);
                    }
                    m14 = m13 * m34;
                    double r = compute2DPointGraycolorBilinear(pt2_color, fidx) - opt_graycolor;
                    for (int i = 0; i < 4; ++i)
                    {
                        clusters_[cidx].JTr(i, 0) += m14(0, i) * r;  // Note that JTr is 4x1 but m14 matrix is 1x4
                        clusters_[cidx].JTJ(i, i) += m14(0, i) * m14(0, i);
                        for (int j = i + 1; j < 4; ++j)
                        {
                            double val = m14(0, i) * m14(0, j);
                            clusters_[cidx].JTJ(i, j) += val;
                            clusters_[cidx].JTJ(j, i) += val;
                        }
                    }
                    energy1 += r * r;
                    flag_run_color_opt = true;
                }
                if (flag_run_color_opt)
                {
                    // Compute Jacobian of regularization term
                    double r = kSqrtLambda1 * dis_pt2plane;
                    energy2 += dis_pt2plane * dis_pt2plane;
                    Vector4d jrow(texel.pt3_global[0], texel.pt3_global[1], texel.pt3_global[2], 1);
                    jrow *= kSqrtLambda1;
                    for (int i = 0; i < 4; ++i)
                    {
                        clusters_[cidx].JTr(i, 0) += jrow[i] * r;
                        clusters_[cidx].JTJ(i, i) += jrow[i] * jrow[i];
                        for (int j = i + 1; j < 4; ++j)
                        {
                            double val = jrow[i] * jrow[j];
                            clusters_[cidx].JTJ(i, j) += val;
                            clusters_[cidx].JTJ(j, i) += val;
                        }
                    }
                    // Remember to update flag for the plane
                    if (!clusters_[cidx].is_optimized)
                        clusters_[cidx].is_optimized = true;
                }
            }
        }
        energy2 *= lambda1_;
        curr_global_energy_ = energy1 + energy2;  // the energy2 already considers lambda1
        cout << "   Energy (iter " << iter << "): " << curr_global_energy_ << " (" << energy1 << " + " << energy2 << ")"
             << endl;
        if (last_global_energy_ < curr_global_energy_ || last_color_energy_ < energy1)
        {
            // Recover plane parameters in the last iteration if it's better
            for (int cidx = 0; cidx < cluster_num_; ++cidx)
            {
                if (!clusters_[cidx].is_optimized)
                    continue;
                clusters_[cidx].opt_normal = clusters_[cidx].last_normal;
                clusters_[cidx].opt_w = clusters_[cidx].last_w;
            }
            curr_global_energy_ = last_global_energy_;
            break;
        }
        last_color_energy_ = energy1;
        last_global_energy_ = curr_global_energy_;
        for (int cidx = 0; cidx < cluster_num_; ++cidx)
        {
            if (!clusters_[cidx].is_optimized)
                continue;
            Xi = -clusters_[cidx].JTJ.llt().solve(clusters_[cidx].JTr);
            if (!isfinite(Xi(0, 0)) || !isfinite(Xi(1, 0)) || !isfinite(Xi(2, 0)) || !isfinite(Xi(3, 0)))
            {
                PRINT_YELLOW("WARNING: cluster %d cannot be optimized more.", cidx);
                continue;
            }
            clusters_[cidx].last_normal = clusters_[cidx].opt_normal;
            clusters_[cidx].last_w = clusters_[cidx].opt_w;
            clusters_[cidx].opt_normal += Xi.block(0, 0, 3, 1);
            clusters_[cidx].opt_w += Xi(3, 0);
            double len = clusters_[cidx].opt_normal.norm();
            clusters_[cidx].opt_normal.normalize();
            clusters_[cidx].opt_w /= len;
        }
    }
}

//! Get all connected components of the mesh by BFS. Result is in 'connected_components_'.
void RGBDMeshOpt::getConnectedComponents()
{
    for (int i = 0; i < vertex_num_; ++i)
        vertices_[i].is_visited = false;
    queue<int> qu;
    for (int i = 0; i < vertex_num_; ++i)
    {
        if (vertices_[i].is_visited)
            continue;
        qu.push(i);
        vertices_[i].is_visited = true;
        vector<int> vset;
        while (!qu.empty())
        {
            int v = qu.front();
            qu.pop();
            vset.push_back(v);
            for (int nv : vertices_[v].nbr_vertices)
            {
                if (!vertices_[nv].is_visited)
                {
                    vertices_[nv].is_visited = true;
                    qu.push(nv);
                }
            }
        }
        connected_components_.push_back(std::move(vset));
    }
}

void RGBDMeshOpt::runMeshGeometryOpt()
{
    // Get connected components and run optimization per component, since we need to
    // ensure all vertices to be optimized are connected.
    getConnectedComponents();

    // In order to ensure that Laplacian matrix is full-rank (non-sigular),
    // we fix the position of one single vertex for each connected component.
    // Here we use the first vertex in each connected component.
    const int kComponentNum = int(connected_components_.size());
    vector<int> component_vertices;
    for (int i = 0; i < kComponentNum; ++i)
    {
        int vidx = connected_components_[i][0];
        component_vertices.push_back(vidx);
        vertices_[vidx].component_id_x = i;
        connected_components_[i].erase(connected_components_[i].begin());
    }
    // Save each vertex's position in connected components
    for (int i = 0; i < kComponentNum; ++i)
    {
        for (int j = 0; j < int(connected_components_[i].size()); ++j)
        {
            int vidx = connected_components_[i][j];
            vertices_[vidx].component_id_x = i;
            vertices_[vidx].component_id_y = j;
        }
    }
    vector<MatrixXd> JTRs;  // for all components
    for (int i = 0; i < kComponentNum; ++i)
    {
        int n = int(connected_components_[i].size());
        MatrixXd JTR(n, 3);
        JTR.setZero();
        JTRs.push_back(JTR);
    }
    cout << "Computing Jacobian for all components ..." << endl;

    /// Create geometry term of Jacobian matrix for all components
    // Long long int is the 2D position in the large Jacobian matrix. We use sparse matrix here so
    // have to store for each 2D position its Jacobian value. So it's a 2D pair -> double mapping.
    vector<unordered_map<long long, double>> spmat_values(kComponentNum);
    int oldv[3], newv[3];  // original vertex index and new index in Jacobian matrix
    for (TexturePatch& patch : patches_)
    {
        for (auto it : patch.texel_positions)
        {
            int x = it.first, y = it.second;
            Texel& texel = texels_[patch.texture_img_idx][y][x];
            int fa = texel.face_id;
            if (fa == -1)
                continue;
            int cidx = vertices_[faces_[fa].indices[0]].component_id_x;  // component index
            int fixed_vidx = component_vertices[cidx];
            const Vector3d& q = texel.pt3_proj;

            // Check if the face contains the fixed vertex in the component
            int idx_in_face = -1;
            for (int i = 0; i < 3; ++i)
            {
                oldv[i] = faces_[fa].indices[i];
                newv[i] = vertices_[oldv[i]].component_id_y;
                if (oldv[i] == fixed_vidx)
                    idx_in_face = i;
            }
            if (idx_in_face != -1)
            {
                // This is the special case that one vertex of the face is the fixed vertex in
                // the connected component. We need to deal with this specifically.
                int idx1 = (idx_in_face + 1) % 3, idx2 = (idx_in_face + 2) % 3;
                int v1 = newv[idx1], v2 = newv[idx2];
                // accumulate values into corresponding positions
                spmat_values[cidx][getKey(v1, v1)] += texel.barycentrics[idx1] * texel.barycentrics[idx1];
                spmat_values[cidx][getKey(v2, v2)] += texel.barycentrics[idx2] * texel.barycentrics[idx2];
                double val = texel.barycentrics[idx1] * texel.barycentrics[idx2];
                spmat_values[cidx][getKey(v2, v1)] += val;
                spmat_values[cidx][getKey(v1, v2)] += val;
                Vector3d qv = q - texel.barycentrics[idx_in_face] * vertices_[fixed_vidx].opt_pt3;
                for (int i = 0; i < 3; ++i)
                {
                    JTRs[cidx](v1, i) += texel.barycentrics[idx1] * qv[i];
                    JTRs[cidx](v2, i) += texel.barycentrics[idx2] * qv[i];
                }
            }
            else
            {
                // This is the common case that 3 vertices of the face do not contain the fixed vertex.
                for (int i = 0; i < 3; ++i)
                {
                    for (int j = i; j < 3; ++j)  // note that j >= i here
                    {
                        double val = texel.barycentrics[i] * texel.barycentrics[j];
                        spmat_values[cidx][getKey(newv[i], newv[j])] += val;
                        if (i != j)  // symmetric matrix
                            spmat_values[cidx][getKey(newv[j], newv[i])] += val;
                    }
                }
                for (int i = 0; i < 3; ++i)
                    for (int j = 0; j < 3; ++j)
                        JTRs[cidx](newv[i], j) += texel.barycentrics[i] * q[j];
            }
        }
    }
    // Create regularization/Laplacian term of Jacobian matrix
    for (int cidx = 0; cidx < kComponentNum; ++cidx)
    {
        for (int vidx : connected_components_[cidx])
        {
            int n = int(vertices_[vidx].nbr_vertices.size());
            if (n == 0)
            {
                PRINT_YELLOW("WARNING: vertex %d has no neighbors in the mesh. This is bad.", vidx);
                continue;
            }
            double c0 = 1, c1 = -1.0 / n, c2 = 1.0 / (n * n);
            vector<int> indices;
            indices.push_back(vertices_[vidx].component_id_y);
            bool flag_with_fixed_vertex = false;
            for (int nvidx : vertices_[vidx].nbr_vertices)
            {
                if (nvidx == component_vertices[cidx])
                    flag_with_fixed_vertex = true;
                else
                    indices.push_back(vertices_[nvidx].component_id_y);
            }
            for (size_t i = 0; i < indices.size(); ++i)
            {
                for (size_t j = 0; j < indices.size(); ++j)
                {
                    if (i == 0 && j == 0)
                        spmat_values[cidx][getKey(indices[i], indices[j])] += c0;
                    else if (i == 0 || j == 0)
                        spmat_values[cidx][getKey(indices[i], indices[j])] += c1;
                    else
                        spmat_values[cidx][getKey(indices[i], indices[j])] += c2;
                }
            }
            if (flag_with_fixed_vertex)
            {
                // For neighbors of the fixed vertex in the connected component,
                // the Laplacian term will be like ||LX - D|| with D matrix non-zero
                // for rows of the fixed vertex's neighbors.
                Vector3d q = vertices_[component_vertices[cidx]].opt_pt3 / n;
                for (size_t i = 0; i < indices.size(); ++i)
                {
                    for (size_t j = 0; j < 3; ++j)
                    {
                        if (i == 0)
                            JTRs[cidx](indices[i], j) += q[j];
                        else
                            JTRs[cidx](indices[i], j) -= q[j] / n;
                    }
                }
            }
        }
    }
    // Solve linear system
    cout << "Solving linear system for each component ..." << endl;
    for (int i = 0; i < kComponentNum; ++i)
    {
        int n = int(connected_components_[i].size());
        SparseMatrix<double> JTJ(n, n);
        vector<Triplet<double>> triples;
        triples.reserve(spmat_values[i].size());
        for (auto it : spmat_values[i])
        {
            int x, y;
            getPair(it.first, x, y);
            triples.push_back(Triplet<double>(x, y, it.second));
        }
        JTJ.setFromTriplets(triples.begin(), triples.end());
        SimplicialLLT<SparseMatrix<double>> solver;
        solver.compute(JTJ);
        if (solver.info() != Eigen::Success)
        {
            PRINT_YELLOW("WARNING: Failed to create solver for component %d", i);
            continue;
        }
        MatrixXd X(n, 3);
        X = solver.solve(JTRs[i]);
        for (int j = 0; j < n; ++j)
        {
            int vidx = connected_components_[i][j];
            for (int k = 0; k < 3; ++k)
                vertices_[vidx].opt_pt3[k] = X(j, k);
        }
    }
    cout << "DONE." << endl;
}

void RGBDMeshOpt::initAll()
{
    initMeshConnectivity();
    initClusters();
    initRGBDFrames();
    initTexturePatches();
}

void RGBDMeshOpt::initClusters()
{
    cout << "Initialize clusters/planes" << endl;
    for (int i = 0; i < cluster_num_; ++i)
    {
        for (int fidx : clusters_[i].faces)
        {
            Face& f = faces_[fidx];
            CovObj Q(vertices_[f.indices[0]].pt3, vertices_[f.indices[1]].pt3, vertices_[f.indices[2]].pt3);
            clusters_[i].cov += Q;
            for (int j = 0; j < 3; ++j)
            {
                int vidx = f.indices[j];
                if (vertices_[vidx].cluster_id == -2)  // vertex cluster-id is initialized as -2
                    vertices_[vidx].cluster_id = i;
                else if (vertices_[vidx].cluster_id != i)
                    vertices_[vidx].cluster_id = -1;  // cluster border vertex
            }
            faces_[fidx].cluster_id = i;
        }
        clusters_[i].cov.computePlaneNormal();
        clusters_[i].normal = clusters_[i].cov.normal_;

        // Check if the normal is inverse or not
        int fidx = *clusters_[i].faces.begin();  // any face from the cluster is fine here
        const Vector3d& v0 = vertices_[faces_[fidx].indices[0]].pt3;
        const Vector3d& v1 = vertices_[faces_[fidx].indices[1]].pt3;
        const Vector3d& v2 = vertices_[faces_[fidx].indices[2]].pt3;
        Vector3d nor = (v1 - v0).cross(v2 - v0);
        if (clusters_[i].normal.dot(nor) < 0)
            clusters_[i].normal = -clusters_[i].normal;

        clusters_[i].w = -clusters_[i].normal.dot(clusters_[i].cov.center_);
        clusters_[i].opt_normal = clusters_[i].normal;
        clusters_[i].opt_center = clusters_[i].center = clusters_[i].cov.center_;
        clusters_[i].opt_w = clusters_[i].w;

        clusters_[i].JTJ = MatrixXd(4, 4);
        clusters_[i].JTr = MatrixXd(4, 1);
        clusters_[i].JTJ.setZero();
        clusters_[i].JTr.setZero();
    }
}

void RGBDMeshOpt::initRGBDFrames()
{
    cout << "Initialize RGBD frames." << endl;
    assert(color_width_ == depth_width_ && color_height_ == depth_height_ && color_width_ == frames_[0].color_img.cols &&
           color_height_ == frames_[0].color_img.rows);

    frame_num_ = static_cast<int>(frames_.size());
    for (int fridx = 0; fridx < frame_num_; ++fridx)
    {
        Frame& frame = frames_[fridx];
        // Save visible info for vertices and faces
        for (int vidx : frame.visible_vertices)
        {
            vertices_[vidx].visible_frames.insert(fridx);
            vertices_[vidx].is_visible = true;
            // A face is visible to some frame as long as at least one vertex is visible.
            for (int nbr : vertices_[vidx].nbr_faces)
                faces_[nbr].visible_frames.insert(fridx);
        }

        // Compute color image pixel gradients
        cv::cvtColor(frame.color_img, frame.gray_img, CV_RGB2GRAY);
        frame.pixel_gradients.resize(color_height_, vector<Vector2d>(color_width_));
        for (int y = 1; y < color_height_ - 1; ++y)  // exclude outmost border
        {
            for (int x = 1; x < color_width_ - 1; ++x)
            {
                Vector2i u(x, y);
                frame.pixel_gradients[y][x][0] = computePixelGraycolorGradient(u, fridx, kScharrKernelX);
                frame.pixel_gradients[y][x][1] = computePixelGraycolorGradient(u, fridx, kScharrKernelY);
            }
        }
        frame.JTJ = MatrixXd(6, 6);
        frame.JTr = MatrixXd(6, 1);
        frame.JTJ.setZero();
        frame.JTr.setZero();
        frame.opt_T = frame.T;
        frame.opt_inv_T = frame.inv_T = frame.T.inverse();
        frame.opt_R = frame.R = frame.T.block(0, 0, 3, 3);
        frame.opt_inv_R = frame.inv_R = frame.inv_T.block(0, 0, 3, 3);
        frame.opt_t = frame.t = frame.T.block(0, 3, 3, 1);
        frame.opt_inv_t = frame.inv_t = frame.inv_T.block(0, 3, 3, 1);
        if (FLAGS_use_noisy_poses && fridx > 0)
        {  // Use poses from previous pose as 'noisy' poses
            frame.opt_T = frames_[fridx - 1].T;
            frame.opt_R = frames_[fridx - 1].R;
            frame.opt_t = frames_[fridx - 1].t;
            frame.opt_inv_T = frames_[fridx - 1].inv_T;
            frame.opt_inv_R = frames_[fridx - 1].inv_R;
            frame.opt_inv_t = frames_[fridx - 1].inv_t;
        }
    }
}

void RGBDMeshOpt::initTexturePatches()
{
    cout << "Initialize texture patches." << endl;

    createTexturePatches();

    packAllPatches();

    computeTexelsForAllPatches();

    // Compute initial color for each texel.
    computeAllTexelColors();
}

//! Create patch for each cluster/plane. Each patch is a rectangle bounding box covering this cluster/plane.
void RGBDMeshOpt::createTexturePatches()
{
    if (!patches_.empty())
        patches_.clear();
    patches_.resize(cluster_num_);
    const Vector3d kNegZ(0, 0, -1);
    for (int cidx = 0; cidx < cluster_num_; ++cidx)
    {
        TexturePatch& patch = patches_[cidx];
        patch.cluster_id = cidx;
        unordered_set<int> cluster_vertices;
        for (int fidx : clusters_[cidx].faces)
        {
            for (int i = 0; i < 3; ++i)
                cluster_vertices.insert(faces_[fidx].indices[i]);
        }

        // Project all mesh vertices of each cluster onto its plane and transform to x-y plane to
        // get the patch shape. After transformation, the original plane center will be the origin
        // point, and plane normal will be -z direction (while +x is to the right and +y to the bottom).
        int cluster_vertex_num = static_cast<int>(cluster_vertices.size());
        const Vector3d& kNormal = clusters_[cidx].normal;
        const Vector3d& kCenter = clusters_[cidx].center;
        const double& kW = clusters_[cidx].w;
        Vector3d axis = (kNormal.cross(kNegZ)).normalized();
        double angle = acos(kNormal.dot(kNegZ));
        Matrix3d Rot;
        Rot = Eigen::AngleAxisd(angle, axis);
        int new_vidx = 0;
        patch.uv_textures.reserve(cluster_vertices.size());
        Eigen::AlignedBox2d box;
        for (int vidx : cluster_vertices)
        {
            patch.vertex_to_patch[vidx] = new_vidx++;  // old mesh vertex index -> new index in patch
            const Vector3d& v = vertices_[vidx].pt3;
            Vector3d vproj = v - (kNormal.dot(v) + kW) * kNormal;  // project mesh vertex on the plane
            vproj -= kCenter;                                      // move plane center to origin point
            Vector3d vnew = Rot * vproj;                           // transform to x-y plane
            assert(vnew[2] < 1e-10);                               // ensure z-value is removed
            Vector2d uv(vnew[0], vnew[1]);                         // now is only the transformed X,Y coordinate
            box.extend(uv);
            patch.uv_textures.push_back(uv);
        }
        // We will move the min corner of the patch bounding box to the origin (0,0), and compute the
        // uv coordinates for each vertex.
        const double kExtendWidth = FLAGS_patch_boundingbox_borderwidth * 2.0;  // extend the bbox for a small width
        Vector2d max_corner = box.max(), min_corner = box.min();
        double height = max_corner[1] - min_corner[1] + kExtendWidth;
        for (size_t i = 0; i < patch.uv_textures.size(); ++i)
        {
            // Here we don't scale the uv coordinates to [0,1] now, since they are just the uv in current
            // patch instead of final texture image. Will do it later after packing all patches.
            patch.uv_textures[i][0] -= min_corner[0];  // now only the distance to the min corner
            patch.uv_textures[i][1] -= min_corner[1];
            patch.uv_textures[i][0] += FLAGS_patch_boundingbox_borderwidth;
            patch.uv_textures[i][1] += FLAGS_patch_boundingbox_borderwidth;
            patch.uv_textures[i][1] = height - patch.uv_textures[i][1];  // NOTE that v-coordinate is inverse
        }
        Vector2d new_min_corner(0, 0);
        Vector2d new_max_corner =
            (max_corner - min_corner + Vector2d(kExtendWidth, kExtendWidth)) * FLAGS_unit_meter_resolution;
        patch.width = static_cast<int>(ceil(new_max_corner[0]));
        patch.height = static_cast<int>(ceil(new_max_corner[1]));
        patch.area = patch.width * patch.height;
        // cout << "Cluster " << cidx << ": " << patch.width << " " << patch.height << endl;
    }
}

void RGBDMeshOpt::packAllPatches()
{
    // Used to sort the texture patch from largest area to smallest
    auto TexturePatchComparator = [](TexturePatch& t1, TexturePatch& t2) {
        return t1.area > t2.area || (t1.area == t2.area && t1.height > t2.height);
    };
    std::sort(patches_.begin(), patches_.end(), TexturePatchComparator);
    vector<std::unique_ptr<TreeNode>> roots;  // each root denotes one output texture image
    int img_width = FLAGS_texture_image_resolution, img_height = FLAGS_texture_image_resolution;
    for (TexturePatch& patch : patches_)
    {
        int img_idx = 0;
        if (patch.width > img_width || patch.height > img_height)
        {
            PRINT_YELLOW(
                "WARNING: patch size (%d, %d) is too large than default texture image width %d. Will enlarge the image.",
                patch.width, patch.height, img_width);
            img_width = img_height = std::max(patch.width, patch.height);
        }
        while (true)
        {
            if (img_idx == int(roots.size()))
            {  // create new texture image if current images cannot hold the patch
                std::unique_ptr<TreeNode> root(new TreeNode(0, 0, img_width, img_height));
                roots.push_back(std::move(root));
                cv::Mat texture_img(img_height, img_width, CV_8UC3, cv::Scalar(255, 255, 255));
                texture_images_.push_back(std::move(texture_img));
            }
            if (packPatchRecursive(roots[img_idx], patch))
            {  // Each patch is ensured to be packed in some texture image.
                patch.texture_img_idx = img_idx;
                break;
            }
            img_idx++;
        }
        // Compute final texture coordinates for all vertices in the patch
        for (size_t i = 0; i < patch.uv_textures.size(); ++i)
        {
            patch.uv_textures[i][0] = (patch.uv_textures[i][0] * FLAGS_unit_meter_resolution + patch.blx) / img_width;
            patch.uv_textures[i][1] = (patch.uv_textures[i][1] * FLAGS_unit_meter_resolution + patch.bly) / img_height;
        }
    }
    cout << "#Texture images: " << texture_images_.size() << endl;
}

//! Pack each patch into the final texture images using binary split partition algorithm. Return true
//! if the patch is successfully packed into the texture image, otherwise return false (such as texture
//! image has no space to hold the patch, etc). Note that here we pack each patch into texture image
//! from bottom-left in order to fit the texture coordinates, where bottom-left is (0,0) and top right is (1,1).
//! Ref: https://slizerboy.wordpress.com/tag/texture-atlas/
bool RGBDMeshOpt::packPatchRecursive(std::unique_ptr<TreeNode>& root, TexturePatch& patch)
{
    if (root == nullptr)
        return false;

    // Traverse until meeting a leaf node with enough space to keep the patch
    if (!root->is_leaf)
    {
        if (packPatchRecursive(root->left, patch))
            return true;
        return packPatchRecursive(root->right, patch);
    }
    // The node space is not big enough to hold the patch
    if (patch.width > root->width || patch.height > root->height)
        return false;

    // Split the node into two children nodes
    root->left.reset(new TreeNode());        // left node holds current patch
    root->left->left.reset(new TreeNode());  // left->left node fits current patch
    // Update corners of root->left->right node and root->right node (they are still blank/leaf nodes).
    // Note that root->right node's space is always larger than root->left->right node
    if (root->width >= root->height)
    {
        root->left->right.reset(new TreeNode(root->minx, root->miny + patch.height, patch.width, root->height - patch.height));
        root->right.reset(new TreeNode(root->minx + patch.width, root->miny, root->width - patch.width, root->height));
    }
    else
    {
        root->left->right.reset(new TreeNode(root->minx + patch.width, root->miny, root->width - patch.width, patch.height));
        root->right.reset(new TreeNode(root->minx, root->miny + patch.height, root->width, root->height - patch.height));
    }
    root->is_leaf = false;   // Set the node as non-leaf
    patch.blx = root->minx;  // Save patch corner point in texture image
    patch.bly = root->miny;
    return true;
}

void RGBDMeshOpt::computeTexelsForAllPatches()
{
    texels_.resize(texture_images_.size());
    for (size_t i = 0; i < texture_images_.size(); ++i)
        texels_[i].resize(texture_images_[i].rows, vector<Texel>(texture_images_[i].cols));
    int texel_num = 0;
    double c0 = 0, c1 = 0, c2 = 0;
    for (TexturePatch& patch : patches_)
    {
        int cidx = patch.cluster_id;
        int tidx = patch.texture_img_idx;
        int img_width = texture_images_[tidx].cols, img_height = texture_images_[tidx].rows;
        for (int fidx : clusters_[cidx].faces)
        {
            // Get bounding box for the face
            Face& face = faces_[fidx];
            Eigen::AlignedBox2d box;
            for (int i = 0; i < 3; ++i)
            {
                int vidx = face.indices[i];
                if (patch.vertex_to_patch.find(vidx) == patch.vertex_to_patch.end())
                {
                    PRINT_RED("ERROR: vertex %d is not saved in its patch. This shouldn't happen.", vidx);
                    continue;
                }
                // We already computed uv for each vertex in each cluster before.
                face.uv[i] = patch.uv_textures[patch.vertex_to_patch[vidx]];
                face.uv[i][0] *= img_width;
                face.uv[i][1] = img_height * (1.0 - face.uv[i][1]);
                box.extend(face.uv[i]);
            }
            // Create texel points inside the bounding box of the face
            Vector2d max_corner = box.max(), min_corner = box.min();
            int top = static_cast<int>(floor(min_corner[1]));  // Note +y is to the bottom
            int bottom = static_cast<int>(ceil(max_corner[1]));
            bottom = std::min(bottom, img_height - 1);
            int left = static_cast<int>(floor(min_corner[0]));
            int right = static_cast<int>(ceil(max_corner[0]));
            right = std::min(right, img_width - 1);
            for (int i = top; i <= bottom; ++i)
            {
                for (int j = left; j <= right; ++j)
                {
                    Texel& texel = texels_[tidx][i][j];
                    if (texel.is_valid)
                    {
                        // This means the texel point is already created in some other face, since we are using
                        // bounding box for each face, so there will be overlap between the boxes.
                        continue;
                    }
                    Vector2d u(j, i);
                    if (!computeBarycentricCoordinates(u, face.uv[0], face.uv[1], face.uv[2], c0, c1, c2))
                        continue;
                    texel.is_valid = true;
                    texel.face_id = fidx;
                    // texel's 3D point is computed by interpolation of barycentric coordinates
                    texel.pt3_global = c0 * vertices_[face.indices[0]].opt_pt3 + c1 * vertices_[face.indices[1]].opt_pt3 +
                                       c2 * vertices_[face.indices[2]].opt_pt3;
                    texel.barycentrics[0] = c0;
                    texel.barycentrics[1] = c1;
                    texel.barycentrics[2] = c2;
                    patch.texel_positions.push_back(make_pair(j, i));
                    texel_num++;
                }
            }
        }
    }
    cout << "#Texels: " << texel_num << endl;
}

void RGBDMeshOpt::computeAllTexelColors()
{
    for (TexturePatch& patch : patches_)
    {
        int tidx = patch.texture_img_idx;
        for (auto it : patch.texel_positions)
        {
            int x = it.first, y = it.second;
            Texel& texel = texels_[tidx][y][x];
            computeTexelColorByAverage(texel);
        }
    }
}

void RGBDMeshOpt::computeTexelColorByAverage(Texel& texel)
{
    int count = 0;
    int fa = texel.face_id;
    int cidx = faces_[fa].cluster_id;
    const Vector3d& kNormal = clusters_[cidx].opt_normal;
    const double& kW = clusters_[cidx].opt_w;
    texel.pt3_proj = texel.pt3_global - (texel.pt3_global.dot(kNormal) + kW) * kNormal;
    double graycolor = 0;
    Vector3f rgb(0, 0, 0);
    for (int fidx : faces_[fa].visible_frames)
    {
        Vector3d pt3 = globalToCameraSpace(texel.pt3_proj, frames_[fidx].opt_inv_T);
        Vector2d pt2_color;
        if (!isCameraPointVisibleInFrame(pt3, fidx, pt2_color))
            continue;
        graycolor += compute2DPointGraycolorBilinear(pt2_color, fidx);
        rgb += compute2DPointRGBcolorBilinear(pt2_color, fidx);
        count++;
    }
    if (count)
    {
        graycolor /= count;
        rgb /= count;
        texel.opt_graycolor = graycolor;
        texel.opt_rgb = rgb;
    }
}

//!
void RGBDMeshOpt::generateFinalTexelColors()
{
    for (int cidx = 0; cidx < cluster_num_; ++cidx)
    {
        TexturePatch& patch = patches_[cidx];

        // Expanding patch to neighbor pixels to remove seams between texture patches in final texture mappping
        expandTexturePatch(patch);
        int tidx = patch.texture_img_idx;
        cv::Mat& tex_img = texture_images_[tidx];
        for (auto it : patch.texel_positions)
        {
            int x = it.first, y = it.second;
            Texel& texel = texels_[tidx][y][x];
            computeTexelColorByAverage(texel);
            for (int k = 0; k < 3; ++k)
                tex_img.at<cv::Vec3b>(y, x)[k] = (unsigned char)(texel.opt_rgb[2 - k] * 255);
        }
    }
}

//! Expand each patch from border pixels to their neighbor pixels and add new pixels into the patch.
//! This is to remove the seams between patches in the final texture images.
void RGBDMeshOpt::expandTexturePatch(TexturePatch& patch)
{
    int tidx = patch.texture_img_idx;
    int img_height = texture_images_[tidx].rows;
    // Bounding box of the patch in its texture image
    int top = img_height - patch.bly - patch.height;
    int bottom = img_height - patch.bly - 1;
    int left = patch.blx;
    int right = patch.blx + patch.width - 1;
    int loop = 10;  // number of neighbor pixels to extend. 10 seems good enough.
    double c0 = 0, c1 = 0, c2 = 0;
    while (loop-- > 0)
    {
        int n = int(patch.texel_positions.size());
        for (int idx = 0; idx < n; ++idx)
        {
            int i = patch.texel_positions[idx].first, j = patch.texel_positions[idx].second;
            Texel& texel = texels_[tidx][j][i];
            if (!texel.is_valid)
                continue;
            int fidx = texel.face_id;
            if (fidx == -1)
                continue;
            for (int k = 0; k < 4; ++k)
            {
                // Find only patch neighbor texels and expand to its 4 neighbors
                int x = i + kPixel4NeighDirs[k][0], y = j + kPixel4NeighDirs[k][1];
                if (x < left || x > right || y < top || y > bottom)
                    continue;
                Texel& ntexel = texels_[tidx][y][x];
                if (ntexel.is_valid)
                    continue;

                Vector2d u(x, y);
                computeBarycentricCoordinates(u, faces_[fidx].uv[0], faces_[fidx].uv[1], faces_[fidx].uv[2], c0, c1, c2);
                if (fabs(c0 + c1 + c2 - 1) > 1e-5)  // ensure barycentric coordinates are valid
                    continue;
                ntexel.is_valid = true;
                ntexel.face_id = fidx;
                ntexel.pt3_global = c0 * vertices_[faces_[fidx].indices[0]].opt_pt3 +
                                    c1 * vertices_[faces_[fidx].indices[1]].opt_pt3 +
                                    c2 * vertices_[faces_[fidx].indices[2]].opt_pt3;
                int cidx = faces_[fidx].cluster_id;
                ntexel.pt3_proj =
                    ntexel.pt3_global -
                    (ntexel.pt3_global.dot(clusters_[cidx].opt_normal) + clusters_[cidx].opt_w) * clusters_[cidx].opt_normal;
                patch.texel_positions.push_back(make_pair(x, y));
            }
        }
    }
}

/************************************************************************/
/* Math
 */
/************************************************************************/

//! Check if two poses are close to each other.
bool RGBDMeshOpt::isTwoPosesClose(const Matrix4d& T1, const Matrix4d& T2)
{
    Vector3d t1 = T1.block(0, 3, 3, 1), t2 = T2.block(0, 3, 3, 1);
    if ((t1 - t2).norm() < FLAGS_closest_pose_translation)
        return true;
    Matrix3d R1 = T1.block(0, 0, 3, 3), R2 = T2.block(0, 0, 3, 3);
    Vector3d relative_angles = (R1.inverse() * R2).eulerAngles(0, 1, 2);  // angles of relative rotation per axis
    if (relative_angles.norm() < FLAGS_closest_pose_rotation_angle)
        return true;
    return false;
}

//! Compute the grayscale color gradient of a 2D pixel with a given kernel.
//! Note: You must ensure that the input pixel 'pt2' is NOT on the image border before calling this function.
double RGBDMeshOpt::computePixelGraycolorGradient(const Vector2i& pt2, int frame_idx, const double kernel[3][3])
{
    double grad = 0;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            grad += static_cast<double>(frames_[frame_idx].gray_img.at<uchar>(pt2[1] - 1 + i, pt2[0] - 1 + j)) * kernel[i][j];
    return grad / 255;
}

//! Compute the gradient of a 2D point using bilinear interpolation (with given image grayscale gradients)
//! Note: You must ensure that the input pixel is NOT on the image border before calling this function.
Vector2d RGBDMeshOpt::compute2DPointGraycolorGradientBilinear(const Vector2d& pt2, int frame_idx)
{
    int x = int(pt2[0]), y = int(pt2[1]);
    Vector2d grady1 = (static_cast<double>(x) + 1 - pt2[0]) * frames_[frame_idx].pixel_gradients[y][x] +
                      (pt2[0] - x) * frames_[frame_idx].pixel_gradients[y][x + 1];
    Vector2d grady2 = (static_cast<double>(x) + 1 - pt2[0]) * frames_[frame_idx].pixel_gradients[y + 1][x] +
                      (pt2[0] - x) * frames_[frame_idx].pixel_gradients[y + 1][x + 1];
    return (double(y) + 1 - pt2[1]) * grady1 + (pt2[1] - y) * grady2;
}

//! Compute the grayscale color of a 2D point using bilinear interpolation
double RGBDMeshOpt::compute2DPointGraycolorBilinear(const Vector2d& pt2, int frame_idx)
{
    int x = int(pt2[0]), y = int(pt2[1]);
    double grayy1 = (static_cast<double>(x) + 1 - pt2[0]) * static_cast<double>(frames_[frame_idx].gray_img.at<uchar>(y, x)) +
                    (pt2[0] - x) * double(frames_[frame_idx].gray_img.at<uchar>(y, x + 1));
    double grayy2 =
        (static_cast<double>(x) + 1 - pt2[0]) * static_cast<double>(frames_[frame_idx].gray_img.at<uchar>(y + 1, x)) +
        (pt2[0] - x) * double(frames_[frame_idx].gray_img.at<uchar>(y + 1, x + 1));
    return ((static_cast<double>(y) + 1 - pt2[1]) * grayy1 + (pt2[1] - y) * grayy2) / 255;
}

Vector3f RGBDMeshOpt::compute2DPointRGBcolorBilinear(const Vector2d& pt2, int frame_idx)
{
    Vector3f x0y0, x1y0, x0y1, x1y1;
    int x = int(pt2[0]), y = int(pt2[1]);
    for (int i = 0; i < 3; ++i)
    {
        x0y0[i] = float(frames_[frame_idx].color_img.at<cv::Vec3b>(y, x)[2 - i]);
        x1y0[i] = float(frames_[frame_idx].color_img.at<cv::Vec3b>(y, x + 1)[2 - i]);
        x0y1[i] = float(frames_[frame_idx].color_img.at<cv::Vec3b>(y + 1, x)[2 - i]);
        x1y1[i] = float(frames_[frame_idx].color_img.at<cv::Vec3b>(y + 1, x + 1)[2 - i]);
    }
    Vector3f y0color = (float(x) + 1 - float(pt2[0])) * x0y0 + (float(pt2[0]) - x) * x1y0;
    Vector3f y1color = (float(x) + 1 - float(pt2[0])) * x0y1 + (float(pt2[0]) - x) * x1y1;
    return ((float(y) + 1 - float(pt2[1])) * y0color + (float(pt2[1]) - y) * y1color) / 255;
}

//! Compute a 2D point's barycentric coordinates in triangle (v0, v1, v2). Return true if
//! it is inside the triangle (including on border) and false if outside.
//! Ref: http://mathworld.wolfram.com/TriangleInterior.html
bool RGBDMeshOpt::computeBarycentricCoordinates(
    const Vector2d& p, const Vector2d& v0, const Vector2d& v1, const Vector2d& v2, double& c0, double& c1, double& c2)
{
    Vector2d e1 = v1 - v0, e2 = v2 - v0, e0 = p - v0;
    double e12 = e1[0] * e2[1] - e1[1] * e2[0];  // e1 x e2
    if (fabs(e12) < 1e-8)
    {  // triangle is degenerate: two edges are almost colinear
        c0 = c1 = c2 = 0;
        return false;
    }
    c1 = (e0[0] * e2[1] - e0[1] * e2[0]) / e12;  // e0 x e2
    c2 = (e1[0] * e0[1] - e1[1] * e0[0]) / e12;  // e1 x e0
    c0 = 1 - c1 - c2;
    return c1 >= 0 && c2 >= 0 && c1 + c2 <= 1;
}

//! A point in camera space is visible only if:
//! 1) its 2D projection pixel on color image is inside color image range;
//! 2) its 2D projection pixel on depth image is inside depth image range;
//! 3) its depth is close to the depth of its projection pixel on depth image;
bool RGBDMeshOpt::isCameraPointVisibleInFrame(const Vector3d& pt3, int frame_idx, Vector2d& pt2_color)
{
    if (!projectCameraPointToFrame(pt3, color_calib_, pt2_color))
        return false;
    Vector2d pt2_depth;
    if (!projectCameraPointToFrame(pt3, depth_calib_, pt2_depth))
        return false;
    int x = static_cast<int>(pt2_depth[0] + 0.5), y = static_cast<int>(pt2_depth[1] + 0.5);
    double depth = double(frames_[frame_idx].depth_img.at<unsigned short>(y, x)) / depth_scale_factor_;
    if (fabs(depth - pt3[2]) > kDepthResidue)
        return false;
    return true;
}

bool RGBDMeshOpt::isDepthValid(double depth)
{
    return depth > kSmallestDepth && depth < kLargestDepth;
}

bool RGBDMeshOpt::is2DPointOnImageBorder(Vector2d& pt2, CalibrationParams& calib)
{
    if (pt2[0] < FLAGS_image_border_width || pt2[0] + FLAGS_image_border_width > calib.width ||
        pt2[1] < FLAGS_image_border_width || pt2[1] + FLAGS_image_border_width > calib.height)
        return true;
    return false;
}

bool RGBDMeshOpt::isPixelOnImageBorder(Vector2i& pixel, CalibrationParams& calib)
{
    if (pixel[0] < FLAGS_image_border_width || pixel[0] + FLAGS_image_border_width > calib.width ||
        pixel[1] < FLAGS_image_border_width || pixel[1] + FLAGS_image_border_width > calib.height)
        return true;
    return false;
}

/************************************************************************/
/* Conversion between spaces
 */
/************************************************************************/

bool RGBDMeshOpt::projectCameraPointToFrame(const Vector3d& pt3, CalibrationParams& calib, Vector2d& pt2)
{
    if (!isDepthValid(pt3[2]))
        return false;
    pt2 = cameraToImgSpace(pt3, calib);
    if (is2DPointOnImageBorder(pt2, calib))
        return false;
    return true;
}

bool RGBDMeshOpt::projectCameraPointToFrame(const Vector3d& pt3, CalibrationParams& calib)
{
    if (!isDepthValid(pt3[2]))
        return false;
    Vector2d pt2 = cameraToImgSpace(pt3, calib);
    if (is2DPointOnImageBorder(pt2, calib))
        return false;
    return true;
}

bool RGBDMeshOpt::projectGlobalPointToFrame(const Vector3d& pt3, CalibrationParams& calib, const Matrix4d& inv_T, Vector2d& pt2)
{
    Vector3d pt = globalToCameraSpace(pt3, inv_T);
    return projectCameraPointToFrame(pt, calib, pt2);
}

bool RGBDMeshOpt::projectGlobalPointToFrame(const Vector3d& pt3, CalibrationParams& calib, const Matrix4d& inv_T)
{
    Vector3d pt = globalToCameraSpace(pt3, inv_T);
    return projectCameraPointToFrame(pt, calib);
}

Vector3d RGBDMeshOpt::depthToCameraSpace(int ux, int uy, double depth)
{
    Vector4d pt = depth_calib_.intrinsic_inv * Vector4d(depth * ux, depth * uy, depth, 1.0);
    return pt.head<3>();
}

Vector3d RGBDMeshOpt::depthToGlobalSpace(int ux, int uy, double depth, const Matrix4d& T)
{
    Vector3d pt3 = depthToCameraSpace(ux, uy, depth);
    return cameraToGlobalSpace(pt3, T);
}

Vector2d RGBDMeshOpt::globalToImgSpace(const Vector3d& pt, CalibrationParams& calib, const Matrix4d& inv_T)
{
    Vector3d pt3 = globalToCameraSpace(pt, inv_T);
    return cameraToImgSpace(pt3, calib);
}

Vector3d RGBDMeshOpt::globalToCameraSpace(const Vector3d& pt, const Matrix4d& inv_T)
{
    return (inv_T * Vector4d(pt[0], pt[1], pt[2], 1.0)).head<3>();
}

Vector3d RGBDMeshOpt::cameraToGlobalSpace(const Vector3d& pt, const Matrix4d& T)
{
    return (T * Vector4d(pt[0], pt[1], pt[2], 1.0)).head<3>();
}

Vector2d RGBDMeshOpt::cameraToImgSpace(const Vector3d& pt, CalibrationParams& calib)
{
    return Vector2d(calib.fx * pt[0] / pt[2] + calib.cx, calib.fy * pt[1] / pt[2] + calib.cy);
}
