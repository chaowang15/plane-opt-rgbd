#ifndef MESH_OPT_H
#define MESH_OPT_H

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <unordered_set>
#include <unordered_map>
#include <chrono>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../common/covariance.h"
#include <gflags/gflags.h>

using namespace std;
using namespace Eigen;

class RGBDMeshOpt
{
public:
    RGBDMeshOpt();
    ~RGBDMeshOpt();
    bool readPLY(const string& filename);
    bool readClusterFile(const string& filename);
    bool readCameraParamFile(const string& filename);
    bool readImageBlurrinessFile(const string& filename);
    bool readRGBDFrames(const string& rgbd_path, const string& visibility_path, int data_type, int start_fidx, int end_fidx);
    void runOptimizationPipeline();
    void saveTexturedMesh(const string& obj_fname);
    bool savePLY(const string& filename);
    void printMeshInfo()
    {
        cout << "#Vertices: " << vertex_num_ << ", #Faces: " << face_num_ << ", #Clusters: " << cluster_num_ << endl;
    }

private:
    struct CalibrationParams
    {
        int width, height;
        double fx, fy, cx, cy;
        Matrix4d intrinsic;
        Matrix4d intrinsic_inv;
        CalibrationParams()
        {
            clearParams();
            width = height = 0;
        }
        CalibrationParams(int w, int h) : width(w), height(h) { clearParams(); }

        void clearParams()
        {
            fx = fy = cx = cy = 0;
            intrinsic = intrinsic_inv = Matrix4d::Zero();
        }

        bool isValid() { return fx > 0 && fy > 0 && cx > 0 && cy > 0 && width > 0 && height > 0; }
        void setCalibMatrix()
        {
            intrinsic = Matrix4d::Identity();
            intrinsic(0, 0) = fx;
            intrinsic(0, 2) = cx;
            intrinsic(1, 1) = fy;
            intrinsic(1, 2) = cy;
            intrinsic_inv = intrinsic.inverse();
        }
    };

    struct Vertex
    {
        int cluster_id;
        bool is_visible;
        bool is_visited;                     // visited or not during expanding visible vertices to neighbors by BFS
        int component_id_x, component_id_y;  // (x,y) index inside a connected component
        Vector3d pt3, opt_pt3;
        Vector2d pt2;
        unordered_set<int> nbr_vertices, nbr_faces, visible_frames;
        // Cluster border vertex's cluster-id will be set as -1 later
        Vertex() : cluster_id(-2), is_visible(false), is_visited(false), component_id_x(-1), component_id_y(-1) {}
    };

    struct Face
    {
        int cluster_id;
        bool is_visited, is_visible;
        int indices[3];
        Vector2d uv[3];  // uv texture coordinates for 3 vertices in a texture image
        unordered_set<int> nbr_faces, visible_frames;
        Face() : cluster_id(-1), is_visited(false), is_visible(false) {}
    };

    struct Cluster
    {
        bool is_valid;                           // is valid for projecting its faces onto images
        bool is_visible;                         // at least one of its faces is visible to a frame
        bool is_optimized;                       // to be optimized in plane opt
        unordered_set<int> faces, nbr_clusters;  // neighbor clusters
        Vector3f color;
        CovObj cov;
        Vector3d normal, center, opt_normal, opt_center, last_normal;
        double w, opt_w, last_w;
        MatrixXd JTJ, JTr;  // plane Jacobian
        Cluster() : is_valid(false), is_visible(false), is_optimized(false) {}
    };

    struct Frame
    {
        bool is_optimized;
        Matrix4d T, inv_T, opt_T, opt_inv_T, lastT;  // camera pose parameters
        Matrix3d R, inv_R, opt_R, opt_inv_R;
        Vector3d t, opt_t, inv_t, opt_inv_t;
        MatrixXd JTJ, JTr;  // Jacobians
        cv::Mat color_img, depth_img, gray_img;
        vector<int> visible_vertices;
        vector<vector<Vector2d>> pixel_gradients;  // grayscale color gradients, row-major with size color_height x color_width
        Frame() : is_optimized(false) {}
    };

    // A binary tree structure for packing patches in texture images
    struct TreeNode
    {
        bool is_leaf;                   // leaf node flag
        int minx, miny, width, height;  // bounding box parameters
        std::unique_ptr<TreeNode> left, right;
        // An empty node is a non-leaf node (with patch in it)
        TreeNode() : is_leaf(false), minx(0), miny(0), width(0), height(0), left(nullptr), right(nullptr) {}
        TreeNode(int x, int y, int w, int h)  // a node with data is a leaf by default
            : is_leaf(true), minx(x), miny(y), width(w), height(h), left(nullptr), right(nullptr)
        {
        }
    };

    struct ImgPixel
    {
        double graycolor;
        Vector2d pt2_color;
        ImgPixel() : graycolor(0) {}
    };

    // A texel is a pixel in some texture image, and is created from its corresponding patch.
    struct Texel
    {
        bool is_valid;
        int face_id;
        int opt_fidx;  // best frame, -1 denotes not visible to any frame
        double opt_graycolor;
        Vector3f opt_rgb;
        Vector3d pt3_global, pt3_proj;
        Vector3d barycentrics;
        unordered_map<int, ImgPixel> visible_frame_pixels;  // visible frame -> projection pixel on color image in the frame
        Texel() : is_valid(false), face_id(-1), opt_fidx(-1), opt_graycolor(0), opt_rgb(1.0, 1.0, 1.0) {}
    };

    // A texture patch is a 2D rectangle region for one cluster/plane. It contains texels and 2D vertices projected from
    // the 3D vertices in the cluster.
    struct TexturePatch
    {
        int width, height, area;                  // parameters of the patch rectangle
        int texture_img_idx;                      // index of texture image that this patch is in
        int cluster_id;                           // corresponding cluster index
        int base_vtx_index;                       // for writing texture OBJ mesh
        int blx, bly;                             // bottom-left corner point in texture image
        Vector2i texture_img_blpos;               // bottom left position in the final texture image
        unordered_map<int, int> vertex_to_patch;  // mesh vertex index -> new index in patch (start from 0)
        vector<Vector2d> uv_textures;             // texture uv-coords for each vertex, same size as 'vertex_to_patch'
        vector<pair<int, int>> texel_positions;   // texel positions (i, j) in texture image 'texture_img_idx'
        TexturePatch() : width(0), height(0), area(0), texture_img_idx(-1), cluster_id(-1), base_vtx_index(0) {}
    };

private:
    bool readCameraPoseFile(const string& filename, Matrix4d& T);
    bool readColorImg(const string& filename, cv::Mat& img);
    bool readDepthImg(const string& filename, cv::Mat& img);
    bool readVisibilityFile(const string& filename, vector<int>& visible_vertices);

    void initMeshConnectivity();
    int findClusterNeighbors(int cidx);
    void initAll();
    void initClusters();
    void initRGBDFrames();
    void initTexturePatches();
    void createTexturePatches();
    void packAllPatches();
    void computeTexelsForAllPatches();
    void computeAllTexelColors();
    bool packPatchRecursive(std::unique_ptr<TreeNode>& root, TexturePatch& patch);
    void computeTexelColorByAverage(Texel& texel);
    void generateFinalTexelColors();
    void expandTexturePatch(TexturePatch& patch);
    void runPlaneAndCameraPoseOpt();
    void optimizePoses();
    void optimizePlanes();
    void runMeshGeometryOpt();
    void getConnectedComponents();

    /* Math */
    bool isTwoPosesClose(const Matrix4d& T1, const Matrix4d& T2);
    double computePixelGraycolorGradient(const Vector2i& pt2, int frame_idx, const double kernel[3][3]);
    Vector2d compute2DPointGraycolorGradientBilinear(const Vector2d& pt2, int frame_idx);
    double computeImgPointGraycolorBilinear(const Vector2d& pt2, int frame_idx);
    Vector3f compute2DPointRGBcolorBilinear(const Vector2d& pt2, int frame_idx);

    bool isCameraPointVisibleInFrame(const Vector3d& pt3, int frame_idx, Vector2d& pt2_color);
    bool computeBarycentricCoordinates(
        const Vector2d& p, const Vector2d& v0, const Vector2d& v1, const Vector2d& v2, double& c0, double& c1, double& c2);
    bool isDepthValid(double depth);
    bool is2DPointOnImageBorder(Vector2d& pt2, CalibrationParams& calib);
    bool isPixelOnImageBorder(Vector2i& pixel, CalibrationParams& calib);

    /* Conversion between spaces */
    bool projectCameraPointToFrame(const Vector3d& pt3, CalibrationParams& calib, Vector2d& pt2);
    bool projectCameraPointToFrame(const Vector3d& pt3, CalibrationParams& calib);
    bool projectGlobalPointToFrame(const Vector3d& pt3, CalibrationParams& calib, const Matrix4d& inv_T, Vector2d& pt2);
    bool projectGlobalPointToFrame(const Vector3d& pt3, CalibrationParams& calib, const Matrix4d& inv_T);
    Vector3d depthToCameraSpace(int ux, int uy, double depth);
    Vector3d depthToGlobalSpace(int ux, int uy, double depth, const Matrix4d& T);
    Vector2d globalToImgSpace(const Vector3d& pt, CalibrationParams& calib, const Matrix4d& inv_T);
    Vector2d cameraToImgSpace(const Vector3d& pt, CalibrationParams& calib);
    Vector3d cameraToGlobalSpace(const Vector3d& pt, const Matrix4d& T);
    Vector3d globalToCameraSpace(const Vector3d& pt, const Matrix4d& inv_T);
    Vector3d projectPixelOntoLocalPlane(Vector2i& pt2, CalibrationParams& calib, const Matrix3d& R, const Vector3d& t,
        const Vector3d& plane_normal, const double& plane_w);

    // Use a long long int variable as a key in a map for a pair of ints
    inline long long getKey(long long a, long long b)
    {
        return (a << 32) | b;  // higher digits is a
    }
    // Split a long long int into original two integers
    inline void getPair(long long key, int& v1, int& v2)
    {
        v2 = int(key & 0xffffffffLL);  // lower digits is v2
        v1 = int(key >> 32);           // higher digits is v1
    }

private:
    /* 3D Mesh */
    int vertex_num_, face_num_, cluster_num_;
    vector<Vertex> vertices_;
    vector<Face> faces_;
    vector<Cluster> clusters_;

    /* RGBD data */
    CalibrationParams color_calib_, depth_calib_;
    int color_width_, color_height_, depth_width_, depth_height_;
    int frame_num_;
    vector<Frame> frames_;
    unordered_map<int, double> image_blurriness_;
    double depth_scale_factor_;

    /* Textures */
    vector<TexturePatch> patches_;          // for all clusters
    vector<cv::Mat> texture_images_;        // final packed texture images
    vector<vector<vector<Texel>>> texels_;  // texels in all texture image, its 3D since each texture image is 2D

    /* Optimization */
    double last_global_energy_, curr_global_energy_, last_color_energy_;
    vector<vector<int>> connected_components_;
    double lambda1_;

    /* constants */
    const double kPI = 3.1415926;
    const double kSmallestDepth = 0.5;  // in meter
    const double kLargestDepth = 6.0;
    const double kDepthResidue = 0.05;

    // Image gradient kernels
    const double kScharrKernelX[3][3] = {
        {-3.0 / 16, 0, 3.0 / 16},
        {-10.0 / 16, 0, 10.0 / 16},
        {-3.0 / 16, 0, 3.0 / 16},
    };
    const double kScharrKernelY[3][3] = {
        {-3.0 / 16, -10.0 / 16, -3.0 / 16},
        {0, 0, 0},
        {3.0 / 16, 10.0 / 16, 3.0 / 16},
    };
    // pixel neighbors
    const int kPixel8NeighDirs[8][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {1, -1}, {-1, 1}, {1, 1}};
    const int kPixel4NeighDirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
};

#endif
