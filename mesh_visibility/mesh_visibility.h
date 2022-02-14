#ifndef MESH_VISIBILITY_H
#define MESH_VISIBILITY_H

#include <iostream>
#include <string>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/ext.hpp>
#include "shader.h"
#include "gbuffer.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unordered_map>

using namespace std;

/* Constants */
const float kNear = 0.1f;
const float kFar = 10.0f;
const unsigned int kImageWidth = 1280;
const unsigned int kImageHeight = 960;
const unsigned int kDigitNumInFrameName = 6; // used to create valid output filename


// For Shader
struct Vertex
{
	glm::vec3 pos;// position
	glm::vec3 color; // color
	glm::vec2 uv;
	glm::vec3 normal;
	Vertex(){pos = color = glm::vec3(0);}
};

class MeshVisibility
{
public:
	/* Model parameters */
	//vector<Vertex> vertices_;
	//vector<unsigned int> faces_;
	int vertex_num_;
	int face_num_;
	string mesh_suffix_;

	/* Camera parameters */
	int frame_num_;
	float fx_, fy_, cx_, cy_; // intrinsic parameters
	vector<glm::mat4> transforms_; // extrinsic poses
	glm::mat4 transform_perspective_; // perspective transformation
	glm::vec3 camera_initial_center_;

	/* Buffers and shader-related */
	//unsigned int VAO_, VBO_, EBO_;
	GBuffer image_buffer_;
	float image_buffer_arr_[kImageHeight][kImageWidth * 3]; // rendered image buffer
	cv::Mat texture_image_;
	vector<cv::Mat> ori_texture_images_;
	unsigned int VAO_, VBO_, EBO_;
	vector<Vertex> vertices_;
	vector<unsigned int> faces_;
	string obj_folder_;
	bool flag_vtx_normal_, flag_vtx_texture_;
	unsigned int texture0_;
	vector<int> image_y_bases_;
	int texture_image_width_, texture_image_height_;
	int texture_img_num_;
	unordered_map<string, int> material_names_;

public:
	MeshVisibility();
	~MeshVisibility();

	/* Rendering functions */
	void initModelDataBuffer();
	void draw();
	void deallocate();

	/* Data I/O*/
	bool readPLY(const string filename);
	bool readOBJ(const string filename);
	bool readMTLandTextureImages(const string filename);
	bool readCameraPoses(const string filepath, int start_fidx, int end_fidx);
	bool readCameraIntrinsicsFile(const string filepath);
	void saveColor2PNG(const string filename);
	void saveDepth2PNG(const string filename);
	void saveVisibleVertices2Binary(const string filename);
	void saveVisibilityImage2Binary(const string filename);

	/* Transformation */
	glm::mat4 computeTransformation(); // for debug
	glm::mat4 computeTransformationForFrame(int frame_idx);
	void computePerspectiveMatrix();

	/* Frame buffer */
	void prepareImageBuffer();
	void extractImageBuffer();

	/* Small functions */
	// Filename
	inline string getFilename(int frame_idx){
		string str_idx = to_string(frame_idx);
		return "frame-" + string(kDigitNumInFrameName - str_idx.length(), '0') + str_idx;
	}
	// Compute 3D coordinates from 2D pixel (row, col) with depth value
	glm::vec4 compute3DpointFromDepth(int row, int col, float depth){
		float z = depth;
		float x = ((float)col - cx_) * z / fx_;
		float y = ((float)row - cy_) * z / fy_;
		return glm::vec4(x, y, z, 1);
	}
	// Given a pixel index on the image, get its corresponding point index in the original mesh
	int getOriginalVtxIdx(int pixel_idx){
		int row = pixel_idx / kImageWidth;
		int col = pixel_idx % kImageWidth;
		return getOriginalVtxIdx(row, col);
	}
	int getOriginalVtxIdx(int row, int col){
		// The buffer from shader is upside-down ( y-axis is inversed) from a common 2D image space.
		float vidx = image_buffer_arr_[kImageHeight - row - 1][3 * col + 2]; // channel 2 is vertex index
		// The original index of an invalid/invisible vertex is some float value between 0 and 1 (it is 0.4 in my
		// computer and maybe different in other devices).
		return (vidx > 0 && vidx < 1) ? -1 : int(vidx);
	}
	// Given a pixel index on the image, get its corresponding depth value from rendered image
	float getDepthValue(int row, int col){
		// The buffer from shader is upside-down ( y-axis is inversed) from a common 2D image space.
		return image_buffer_arr_[kImageHeight - row - 1][3 * col]; // channel 0 is depth
	}
};

#endif
