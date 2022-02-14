#include "mesh_visibility.h"
#include <fstream>
#include <sstream>
#include <iomanip>  // std::setprecision
#include <random>
#include <set>
#include <unordered_set>

MeshVisibility::MeshVisibility()
{
    vertex_num_ = face_num_ = 0;
}

MeshVisibility::~MeshVisibility()
{
    vertices_.clear();
    faces_.clear();
}

bool MeshVisibility::readPLY(const string filename)
{
    FILE *fin;
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
    int ply_mode = 0;  // 0 for ascii, 1 for little-endian
    size_t vertex_color_channel = 0, face_color_channel = 0;
    size_t vertex_quality_dim = 0;  // vertex quality, such as vertex normal or other data per vertex defined by user
    size_t vertex_normal_dim = 0;   // vertex normal dimension
    char seps[] = " ,\t\n\r ";      // separators
    seps[5] = 10;
    int property_num = 0;
    char line[1024];
    while (true)
    {
        if (fgets(line, 1024, fin) == NULL)
            continue;
        char *token = strtok(line, seps);
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
    vertices_.resize(vertex_num_);
    faces_.resize(face_num_ * 3);
    if (ply_mode == 1)  // binary mode
    {
        for (int i = 0; i < vertex_num_; i++)
        {
            // The vertex data order is: coordinates -> normal -> color -> others (qualities, radius, curvatures, etc)
            size_t haveread = 0;
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
            }
            if (vertex_color_channel)
            {
                unsigned char color[4];
                if ((haveread = fread(color, sizeof(unsigned char), vertex_color_channel, fin)) != vertex_color_channel)
                {
                    cout << "ERROR in reading PLY vertex colors in position " << ftell(fin) << endl;
                    return false;
                }
                vertices_[i].color = glm::vec3(float(color[0]) / 255, float(color[1]) / 255, float(color[2]) / 255);
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
            vertices_[i].pos = glm::vec3(vert[0], vert[1], vert[2]);
        }

        // Face data
        for (int i = 0; i < face_num_; ++i)
        {
            unsigned char channel_num;
            size_t haveread = fread(&channel_num, 1, 1, fin);  // channel number for each face
            int fa[3];
            if ((haveread = fread(fa, sizeof(int), 3, fin)) != 3)  // currently only support triangular face
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
                // Currently we only read face color data without saving them
            }
            for (int j = 0; j < 3; ++j)
                faces_[3 * i + j] = fa[j];
        }
    }
    else
    {
        cout << "ERROR: cannot read ASCII ply file. Currently only support binary format." << endl;
        return false;
    }
    return true;
}

bool MeshVisibility::readMTLandTextureImages(const string filename)
{
    ifstream readin(filename, ios::in);
    if (readin.fail() || readin.eof())
    {
        std::cout << "Cannot read MTL file " << filename << std::endl;
        return false;
    }
    string str_line, str_first, str_img, str_mtl;
    while (!readin.eof() && !readin.fail())
    {
        getline(readin, str_line);
        if (!readin.good() || readin.eof())
            break;
        istringstream iss(str_line);
        if (iss >> str_first)
        {
            if (str_first == "newmtl")
            {
                iss >> str_mtl;
                material_names_[str_mtl] = int(ori_texture_images_.size());
            }
            if (str_first == "map_Kd")
            {
                iss >> str_img;
                str_img = obj_folder_ + str_img;
                cv::Mat img = cv::imread(str_img, cv::IMREAD_UNCHANGED);
                if (img.empty() || img.depth() != CV_8U)
                {
                    cout << "ERROR: cannot read color image " << str_img << endl;
                    return false;
                }
                ori_texture_images_.push_back(std::move(img));
            }
        }
    }
    readin.close();

    // Stick multiple texture images into one final image for rendering.
    // The final image size will be (texture-image-columns, n * texture-image-height),
    // and both height and width are enlarged to values power of 2. Here n is the number
    // of input texture images (suppose all texture images have exactly the same resolution).
    texture_img_num_ = int(ori_texture_images_.size());
    int width = 0, height = 0;
    int y_base = 0;
    for (int i = 0; i < texture_img_num_; ++i)
    {
        width = max(width, ori_texture_images_[i].cols);
        image_y_bases_.push_back(y_base);
        y_base += ori_texture_images_[i].rows;
    }
    height = y_base;

    // Ths size of texture image loaded in OpenGL must be power of 2
    int n = 2;
    while (n < width)
        n *= 2;
    texture_image_width_ = n;
    n = 2;
    while (n < height)
        n *= 2;
    texture_image_height_ = n;

    // texture_image_ = cv::imread("test_red.png"); // debug
    texture_image_ = cv::Mat(texture_image_height_, texture_image_width_, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int i = 0; i < texture_img_num_; ++i)
    {
        ori_texture_images_[i].copyTo(
            texture_image_(cv::Rect(0, texture_image_height_ - image_y_bases_[i] - ori_texture_images_[i].rows,
                ori_texture_images_[i].cols, ori_texture_images_[i].rows)));
    }
    // cv::imwrite("test.png", texture_image_); // debug

    return true;
}

bool MeshVisibility::readOBJ(const string filename)
{
    ifstream readin(filename, ios::in);
    if (readin.fail() || readin.eof())
    {
        std::cout << "Cannot read OBJ file " << filename << std::endl;
        return false;
    }
    string str_line, str_first, str_mtl_name, mtl_fname;
    vector<glm::vec3> vertices, normals;
    vector<glm::vec2> uvs;
    float x, y, z;
    unsigned int f, vt, vn, cur_tex_idx;
    flag_vtx_normal_ = flag_vtx_texture_ = false;
    int face_vidx = 0;
    while (!readin.eof() && !readin.fail())
    {
        getline(readin, str_line);
        if (!readin.good() || readin.eof())
            break;
        if (str_line.size() <= 1)
            continue;
        istringstream iss(str_line);
        iss >> str_first;
        if (str_first[0] == '#')
            continue;
        else if (str_first == "mtllib")
        {  // mtl file
            iss >> mtl_fname;
            size_t pos = filename.find_last_of("\\/");
            if (pos != string::npos)
                obj_folder_ = filename.substr(0, pos + 1);
            mtl_fname = obj_folder_ + mtl_fname;
            if (!readMTLandTextureImages(mtl_fname))
                return false;
        }
        else if (str_first == "v")
        {
            iss >> x >> y >> z;
            vertices.push_back(glm::vec3(x, y, z));
        }
        else if (str_first == "vt")
        {
            iss >> x >> y;
            uvs.push_back(glm::vec2(x, y));
            if (!flag_vtx_texture_)
                flag_vtx_texture_ = true;
        }
        else if (str_first == "vn")
        {
            iss >> x >> y >> z;
            normals.push_back(glm::vec3(x, y, z));
            if (!flag_vtx_normal_)
                flag_vtx_normal_ = true;
        }
        else if (str_first == "usemtl")
        {
            iss >> str_mtl_name;
            if (material_names_.find(str_mtl_name) == material_names_.end())
            {
                cout << "ERROR: cannot find this material " << str_mtl_name << " in the mtl file " << mtl_fname << endl;
                return false;
            }
            cur_tex_idx = material_names_[str_mtl_name];
        }
        else if (str_first == "f")
        {
            int loop = 3;
            while (loop-- > 0)
            {
                iss >> f;
                Vertex vtx;
                f--;
                vtx.pos = vertices[f];
                if (flag_vtx_texture_)
                {
                    if (flag_vtx_normal_)
                    {  // 'f/vt/vn'
                        iss.get();
                        iss >> vt;
                        vt--;
                        iss.get();
                        iss >> vn;
                        vn--;
                    }
                    else
                    {  // 'f/vt'
                        iss.get();
                        iss >> vt;
                        vt--;
                    }
                    // Since we put/stick multiple input texture images into one large final image, so we need to
                    // modify corresponding texture uv coordinates.
                    vtx.uv[0] = uvs[vt][0] * ori_texture_images_[cur_tex_idx].cols / texture_image_width_;
                    vtx.uv[1] = (uvs[vt][1] * ori_texture_images_[cur_tex_idx].rows + image_y_bases_[cur_tex_idx]) /
                                (texture_image_height_);
                    if (flag_vtx_normal_)
                        vtx.normal = normals[vn];
                }
                else if (flag_vtx_normal_)
                {  // 'f//vn'
                    iss.get();
                    iss.get();
                    iss >> vn;
                    vn--;
                    vtx.normal = normals[vn];
                }
                vertices_.push_back(vtx);
                faces_.push_back(face_vidx++);
            }
        }
        // otherwise continue -- unrecognized line
    }
    readin.close();
    vertex_num_ = int(vertices.size());
    face_num_ = face_vidx / 3;
    return true;
}

void MeshVisibility::initModelDataBuffer()
{
    image_buffer_.initNew(kImageWidth, kImageHeight);

    glGenVertexArrays(1, &VAO_);
    glGenBuffers(1, &VBO_);
    glGenBuffers(1, &EBO_);

    glBindVertexArray(VAO_);
    // load data into vertex buffers
    glBindBuffer(GL_ARRAY_BUFFER, VBO_);
    glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(Vertex), &vertices_[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, faces_.size() * sizeof(unsigned int), &faces_[0], GL_STATIC_DRAW);

    // Set attributes for vertices
    // vertex positions
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)0);
    // vertex colors
    glEnableVertexAttribArray(1);
    // The first parameter is the offset, while the last one is the offset variable name and struct name which
    // must be exactly the same as that used in struct Vertex.
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
        (void *)offsetof(Vertex, color));  // name of 'color' variable in struct Vertex

    // texture uv coordinates
    if (flag_vtx_texture_)
    {
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
            (void *)offsetof(Vertex, uv));  // name of 'color' variable in struct Vertex
    }
    // normals
    if (flag_vtx_normal_)
    {
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
            (void *)offsetof(Vertex, normal));  // name of 'color' variable in struct Vertex
    }

    // glGenTextures(1, &texture0);
    // glBindTexture(GL_TEXTURE_2D, texture0);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	// Set texture wrapping to GL_REPEAT
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texture_image_width_, texture_image_height_, 0, GL_BGR, GL_UNSIGNED_BYTE,
    // texture_image_.data); glGenerateMipmap(GL_TEXTURE_2D);

    cv::flip(texture_image_, texture_image_, 0);
    glGenTextures(1, &texture0_);
    glBindTexture(GL_TEXTURE_2D, texture0_);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Set texture clamping method
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    // NOTE: this will generate texture on GPU. It may crash on a GPU card with insufficient memory if
    // using a super large texture image.
    glTexImage2D(GL_TEXTURE_2D,  // Type of texture
        0,                       // Pyramid level (for mip-mapping) - 0 is the top level
        GL_RGB,                  // Internal colour format to convert to
        texture_image_width_,    // Image width  i.e. 640 for Kinect in standard mode
        texture_image_height_,   // Image height i.e. 480 for Kinect in standard mode
        0,                       // Border width in pixels (can either be 1 or 0)
        GL_BGR,                  // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
        GL_UNSIGNED_BYTE,        // Image data type
        texture_image_.ptr());   // The actual image data itself

    glGenerateMipmap(GL_TEXTURE_2D);

    glBindVertexArray(0);
}

void MeshVisibility::draw()
{
    // draw mesh
    glViewport(0, 0, kImageWidth, kImageHeight);
    glBindVertexArray(VAO_);
    glDrawElements(GL_TRIANGLES, int(faces_.size()), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void MeshVisibility::deallocate()
{
    // optional: de-allocate all resources once they've outlived their purpose:
    glDeleteVertexArrays(1, &VAO_);
    glDeleteBuffers(1, &VBO_);
    glDeleteBuffers(1, &EBO_);
}

void MeshVisibility::prepareImageBuffer()
{
    image_buffer_.bindForWriting();
}

void MeshVisibility::extractImageBuffer()
{
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    image_buffer_.bindForReading();
    image_buffer_.setReadBuffer(GBuffer::GBUFFER_TEXTURE_TYPE_DEPTH);
    // In 'image_buffer_arr_', the channel 0 is depth value, while channel 2 is vertex index.
    // This is set in the fragment shader 'depth.frag'.
    glReadPixels(0, 0, kImageWidth, kImageHeight, GL_RGB, GL_FLOAT, image_buffer_arr_);
}

void MeshVisibility::saveColor2PNG(const string filename)
{
    image_buffer_.setReadBuffer(GBuffer::GBUFFER_TEXTURE_TYPE_COLOR);
    glReadPixels(0, 0, kImageWidth, kImageHeight, GL_RGB, GL_FLOAT, image_buffer_arr_);

    cv::Mat mat(kImageHeight, kImageWidth, CV_8UC3);
    for (int i = 0; i < kImageHeight; i++)
    {
        for (int j = 0; j < kImageWidth; j++)
        {
            // The extracted data from shader is upside down to the PNG image so we
            // need to reverse the vertical coordinate.
            cv::Vec3b &bgr = mat.at<cv::Vec3b>(kImageHeight - i - 1, j);  // opencv uses BGR instead of RGB
            bgr[2] = (unsigned char)(image_buffer_arr_[i][3 * j] * 255);  // rgb data is float value in [0,1]
            bgr[1] = (unsigned char)(image_buffer_arr_[i][3 * j + 1] * 255);
            bgr[0] = (unsigned char)(image_buffer_arr_[i][3 * j + 2] * 255);
        }
    }
    cv::imwrite(filename, mat);
}

void MeshVisibility::saveDepth2PNG(const string filename)
{
    image_buffer_.setReadBuffer(GBuffer::GBUFFER_TEXTURE_TYPE_DEPTH);
    glReadPixels(0, 0, kImageWidth, kImageHeight, GL_RGB, GL_FLOAT, image_buffer_arr_);

    cv::Mat mat(kImageHeight, kImageWidth, CV_16U);
    for (int i = 0; i < kImageHeight; i++)
    {
        for (int j = 0; j < kImageWidth; j++)
        {
            // The extracted data from shader is upside down to the PNG image so we
            // need to reverse the vertical coordinate.
            unsigned short &d = mat.at<unsigned short>(kImageHeight - i - 1, j);
            float depth = image_buffer_arr_[i][3 * j];
            d = (unsigned int)(depth * 5000);  // scale depth data to see it more clearly
        }
    }
    cv::imwrite(filename, mat);
}

//! Save the entire visibility image into binary file. The visibility image is actually a 32FC1
//! matrix with exactly the same resolution as the color or depth image. Here we save it in a
//! binary file instead of an image.
void MeshVisibility::saveVisibilityImage2Binary(const string filename)
{
    image_buffer_.setReadBuffer(GBuffer::GBUFFER_TEXTURE_TYPE_DEPTH);
    glReadPixels(0, 0, kImageWidth, kImageHeight, GL_RGB, GL_FLOAT, image_buffer_arr_);
    vector<int> visible_vlist;
    visible_vlist.reserve(kImageHeight * kImageWidth);
    for (int i = 0; i < kImageHeight; i++)
    {
        for (int j = 0; j < kImageWidth; j++)
        {
            // Channel 2 is vertex index, which is set in fragment shader.
            float vidx = image_buffer_arr_[kImageHeight - i - 1][3 * j + 2];
            // If a pixel position doesn't denote any visible vertex, its value is set as -1.
            // Sometimes a pixel's vertex index is a floating value between (0,1), set as as invisible.
            int idx = ((vidx > 0 && vidx < 1) || vidx < 0) ? -1 : int(vidx);
            visible_vlist.push_back(vidx);
        }
    }
    FILE *fout = fopen(filename.c_str(), "wb");
    int num = int(visible_vlist.size());  // save #visible vertices at first
    fwrite(&num, sizeof(int), 1, fout);
    fwrite(&visible_vlist[0], sizeof(int), num, fout);
    fclose(fout);
}

//! Save only indices of visible vertices into a binary file.
void MeshVisibility::saveVisibleVertices2Binary(const string filename)
{
    image_buffer_.setReadBuffer(GBuffer::GBUFFER_TEXTURE_TYPE_DEPTH);
    glReadPixels(0, 0, kImageWidth, kImageHeight, GL_RGB, GL_FLOAT, image_buffer_arr_);

    // Put indices of visible vertices into an array and save it in binary
    unordered_set<int> set_vlist;
    for (int i = 0; i < kImageHeight; i++)
    {
        for (int j = 0; j < kImageWidth; j++)
        {
            // Channel 2 is vertex index, which is set in fragment shader.
            float vidx = image_buffer_arr_[kImageHeight - i - 1][3 * j + 2];
            // int idx = (vidx > 0 && vidx < 1) ? -1 : int(vidx);
            if (vidx == 0 || vidx >= 1)
                set_vlist.insert(int(vidx));
        }
    }
    vector<int> vec_vlist(set_vlist.begin(), set_vlist.end());
    FILE *fout = fopen(filename.c_str(), "wb");
    int num = int(vec_vlist.size());
    fwrite(&num, sizeof(int), 1, fout);
    fwrite(&vec_vlist[0], sizeof(int), num, fout);
    fclose(fout);
}

//! Only for debug: compute some transformation matrix for test
glm::mat4 MeshVisibility::computeTransformation()
{
    // Projection matrix : 45 degree Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
    glm::mat4 Projection = glm::perspective(glm::radians(45.0f), 4.0f / 3.0f, 1.f, 100.f);
    // Or, for an ortho camera :
    // glm::mat4 Projection = glm::ortho(-10.0f,10.0f,-10.0f,10.0f,0.0f,100.0f); // In world coordinates
    // NOTE that GLM uses column-major matrix format just like OpenGL matrices, so here print its transpose.
    // std::cout << glm::to_string(glm::transpose(Projection)) << std::endl;

    // Camera matrix
    // glm::mat4 View = glm::lookAt(
    //	glm::vec3(4, 3, 3), // Camera is at (4,3,3), in World Space
    //	glm::vec3(0, 0, 0), // and looks at the origin
    //	glm::vec3(0, 1, 0)  // Head is up (set to 0,-1,0 to look upside-down)
    //	);

    glm::mat4 View = glm::lookAt(glm::vec3(0, 5, 1),  // Camera is at (4,3,3), in World Space
        glm::vec3(0, 5, 0),                           // the point that camera is looking at
        glm::vec3(0, 1, 0)                            // Head is up (set to 0,-1,0 to look upside-down)
    );
    // glm::mat4 View = glm::mat4(1.0f);

    // Model matrix
    glm::mat4 Model = glm::mat4(1.0f);

    // Our ModelViewProjection : multiplication of our 3 matrices
    // Remember, matrix multiplication is the other way around
    glm::mat4 transform = Projection * View * Model;
    return transform;
}

//! Compute transformation matrix for one frame.
glm::mat4 MeshVisibility::computeTransformationForFrame(int frame_idx)
{
    // In the 3D world space, the model is fixed while the camera is moving under different poses.
    // However, in OpenGL space, the camera position is fixed while the model is transformed inversely.
    glm::mat4 trans_model = glm::inverse(transforms_[frame_idx]);
    glm::mat4 trans_scale = glm::mat4(1.0);
    // trans_scale[0][0] = trans_scale[1][1] = trans_scale[2][2] = scale_factor_; // scale if you want

    // In 3D model (world) coordinate space, +x is to the right, +z is to the inside of the screen, so +y is to the bottom.
    glm::mat4 trans_camera = glm::lookAt(glm::vec3(0, 0, 0),  // In OpenGL camera position is fixed at (0,0,0)
        glm::vec3(0, 0, 1),                                   // +z, position where the camera is looking at
        glm::vec3(0, -1, 0)                                   // +y direction
    );
    // cout << endl << endl << glm::to_string(glm::transpose(trans_camera)) << endl;
    // Note for the order of multiplication of different matrices.
    return transform_perspective_ * trans_camera * trans_scale * trans_model;
}

void MeshVisibility::computePerspectiveMatrix()
{
    transform_perspective_ = glm::mat4(0);
    transform_perspective_[0][0] = fx_ / cx_;
    transform_perspective_[1][1] = fy_ / cy_;
    transform_perspective_[2][2] = (kNear + kFar) / (kNear - kFar);
    transform_perspective_[2][3] = -1;  // glm matrix is in column-major
    transform_perspective_[3][2] = 2 * kFar * kNear / (kNear - kFar);
    // cout << glm::to_string(glm::transpose(transform_perspective_))<< endl;
}

bool MeshVisibility::readCameraPoses(const string filepath, int start_fidx, int end_fidx)
{
    int frame_idx = start_fidx;
    while (frame_idx <= end_fidx)
    {
        string pose_fname = filepath + getFilename(frame_idx) + ".pose.txt";
        ifstream readin(pose_fname, ios::in);
        if (readin.fail() || readin.eof())
        {
            cout << "ERROR: Can not read pose file " << pose_fname << endl;
            return false;
        }
        glm::mat4 trans;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                readin >> trans[j][i];  // glm matrix is in column-major
        transforms_.push_back(trans);
        readin.close();
        frame_idx++;
    }
    return true;
}

bool MeshVisibility::readCameraIntrinsicsFile(const string filename)
{
    // Read intrinsics parameter file
    string target_str = "m_calibrationDepthIntrinsic";
    ifstream readin(filename, ios::in);
    if (readin.fail() || readin.eof())
    {
        cout << "ERROR: Cannot read intrinsics file " << filename << endl;
        return false;
    }
    string str_line, str_dummy;
    float dummy;
    while (!readin.eof() && !readin.fail())
    {
        getline(readin, str_line);
        if (readin.eof())
            break;
        if (str_line.substr(0, target_str.length()) == target_str)
        {
            istringstream iss(str_line);
            iss >> str_dummy >> str_dummy;
            iss >> fx_ >> dummy >> cx_ >> dummy >> dummy >> fy_ >> cy_;

            computePerspectiveMatrix();
            break;
        }
    }
    readin.close();
    return true;
}
