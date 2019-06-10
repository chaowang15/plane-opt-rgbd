// Include standard headers
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "shader.h"
#include "mesh_visibility.h"
#include <iostream>
#include "../common/tools.h"

GLFWwindow* window;

// For apt0-boxes.ply
// const int kStartFrameIdx = 1150;
// const int kEndFrameIdx = 1460;

//// For apt-test.ply
// const int kStartFrameIdx = 1;
// const int kEndFrameIdx = 199;

enum ProgramMode
{
    RENDER_MODEL = 0,
    SAVE_VISIBILITY,
    SAVE_DEPTH_IMAGE,
    SAVE_VERTEX_COLOR_IMAGE,
    SAVE_ALL_FILES,
    SAVE_TEXTURE_IMAGE
};
ProgramMode program_mode_ = RENDER_MODEL;

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

void printUsage()
{
    printInRed("Usage: mesh_visibility -option input_mesh RGBD_path output_path start_frame end_frame");
    cout << "-option:" << endl
         << "  -r: render model only (use left and right arrow to move forward and backward frames)" << endl
         << "  -d: save rendered depth images" << endl
         << "  -c: save rendered vertex color images" << endl
         << "  -v: save rendered vertex visibility files" << endl
         << "  -t: save rendered face texture images (only for textured OBJ model)" << endl
         << "  -a: save all files (color and depth images and visibility files)" << endl;
    cout << "input_mesh:" << endl << "   PLY or OBJ model" << endl;
    cout << "RGBD_path:" << endl << "   contains camera pose files (filename like 'frame-XXXXXX.pose.txt')" << endl;
    cout << "output_path: " << endl << "   path for output files (filename will be like 'frame-XXXXXX.suffix')" << endl;
    cout << "start_frame, end_frame:" << endl << "   start and end frame index (such as 0, 1000, respectively)" << endl;
}

bool initGLFWAndShader()
{
    if (!glfwInit())
    {
        std::cout << "Failed to initialize GLFW" << std::endl;
        return false;
    }
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // We don't want the old OpenGL
    if (program_mode_ != RENDER_MODEL)
        glfwWindowHint(GLFW_VISIBLE, false);  // hide window after creation
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);  // uncomment this statement to fix compilation on OS X
#endif

    // Open a window and create its OpenGL context
    window = glfwCreateWindow(kImageWidth, kImageHeight, "RenderingWindow", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version "
                     "of the tutorials."
                  << std::endl;
        glfwTerminate();
        return false;
    }
    // glfwHideWindow(window);
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // Initialize GLEW
    glewExperimental = true;  // Needed for core profile
    if (glewInit() != GLEW_OK)
    {
        fprintf(stderr, "Failed to initialize GLEW\n");
        getchar();
        glfwTerminate();
        return false;
    }

    // Ensure we can capture the escape key being pressed below
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

    // Dark blue background
    glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    return true;
}

void runRenderMode(MeshVisibility* mesh, int start_fidx, int end_fidx)
{
    Shader myshader;
    myshader.LoadShaders("rendermode.vert", "rendermode.frag");
    myshader.setInt("texture_sampler", 0);
    int frame_idx = 0, frame_num = end_fidx - start_fidx + 1;
    do
    {
        // Clear the screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        myshader.useProgram();
        myshader.setFloat("near", kNear);
        myshader.setFloat("far", kFar);
        if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
        {
            frame_idx++;
            if (frame_idx >= frame_num)
                frame_idx = 0;
            // cout << "Frame " << frame_idx + kStartFrameIdx << endl;
        }
        else if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
        {
            frame_idx--;
            if (frame_idx < 0)
                frame_idx = frame_num - 1;
            // cout << "Frame " << frame_idx + kStartFrameIdx << endl;
        }
        myshader.setMat4("transform", mesh->computeTransformationForFrame(frame_idx));
        // Press 'C' to show color and 'D' to show depth
        if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS)
        {
            myshader.setInt("flag_show_color", true);
            myshader.setBool("flag_show_texture", false);
        }
        else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        {
            myshader.setBool("flag_show_color", false);
            myshader.setBool("flag_show_texture", false);
        }
        else if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS && mesh->flag_vtx_texture_)
        {
            myshader.setBool("flag_show_color", false);
            myshader.setBool("flag_show_texture", true);
        }
        if (mesh->flag_vtx_texture_)
        {
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, mesh->texture0_);
        }
        mesh->draw();
        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        glfwSwapBuffers(window);
        glfwPollEvents();
    }  // Check if the ESC key was pressed or the window was closed
    while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS && glfwWindowShouldClose(window) == 0);
    myshader.deleteProgram();
}

void runSaveMode(MeshVisibility* mesh, int start_fidx, int end_fidx, const string& output_path)
{
    Shader myshader;
    myshader.LoadShaders("savemode.vert", "savemode.frag");
    myshader.setInt("texture_sampler", 0);

	cout << "Processing frames ... " << endl;
	float progress = 0.0;  // for printing a progress bar
	int frame_num = end_fidx - start_fidx + 1;
	const int kStep = (frame_num < 100) ? 1 : (frame_num / 100);
    for (int fidx = start_fidx; fidx <= end_fidx; ++fidx)
    {
        // Clear the screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        myshader.useProgram();
        myshader.setFloat("near", kNear);
        myshader.setFloat("far", kFar);
        if (program_mode_ == SAVE_VERTEX_COLOR_IMAGE || program_mode_ == SAVE_ALL_FILES)
        {
            myshader.setBool("flag_show_color", true);
            myshader.setBool("flag_show_texture", false);
        }
        else if (program_mode_ == SAVE_TEXTURE_IMAGE)
        {
            myshader.setBool("flag_show_color", false);
            myshader.setBool("flag_show_texture", true);
        }
        mesh->prepareImageBuffer();

        // NOTE: depth test and clear function MUST be put after binding frame buffer and
        // run for each frame, or the extracted depth and color data will not have depth test.
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        myshader.setMat4("transform", mesh->computeTransformationForFrame(fidx - start_fidx));
        if (mesh->flag_vtx_texture_)
        {
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, mesh->texture0_);
            myshader.setInt("texture_sampler", 0);
        }
        mesh->draw();
        mesh->extractImageBuffer();

        int current_frame = fidx - start_fidx;
		if (current_frame % kStep == 0 || fidx == end_fidx)
		{
			progress = (fidx == end_fidx) ? 1.0f : static_cast<float>(current_frame) / frame_num;
			printProgressBar(progress);
		}
        string output_fname = output_path + mesh->getFilename(current_frame);
        if (program_mode_ == SAVE_VERTEX_COLOR_IMAGE)
            mesh->saveColor2PNG(output_fname + ".rcolor.png");
        else if (program_mode_ == SAVE_DEPTH_IMAGE)
            mesh->saveDepth2PNG(output_fname + ".rdepth.png");
        else if (program_mode_ == SAVE_TEXTURE_IMAGE)
            mesh->saveColor2PNG(output_fname + ".rtexture.png");
        else if (program_mode_ == SAVE_VISIBILITY)
            mesh->saveVisibleVertices2Binary(output_fname + ".visibility.txt");
        else if (program_mode_ == SAVE_ALL_FILES)
        {
            mesh->saveColor2PNG(output_fname + ".rcolor.png");
            mesh->saveDepth2PNG(output_fname + ".rdepth.png");
            mesh->saveVisibleVertices2Binary(output_fname + ".visibility.txt");
            // This is to save the entire visibility image into binary.
            // mesh->saveVisibilityImage2Binary(output_fname + ".visimage.txt");
        }
        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    myshader.deleteProgram();
    cout << "All frames are processed." << endl;
}

int main(int argc, char** argv)
{
    if (argc != 7)
    {
        printUsage();
        return -1;
    }
    string mode(argv[1]);
    if (mode == "-r")
        program_mode_ = RENDER_MODEL;
    else if (mode == "-d")
        program_mode_ = SAVE_DEPTH_IMAGE;
    else if (mode == "-c")
        program_mode_ = SAVE_VERTEX_COLOR_IMAGE;
    else if (mode == "-v")
        program_mode_ = SAVE_VISIBILITY;
    else if (mode == "-t")
        program_mode_ = SAVE_TEXTURE_IMAGE;
    else if (mode == "-a")
        program_mode_ = SAVE_ALL_FILES;
    else
    {
        printUsage();
        return -1;
    }
    string mesh_fname(argv[2]), rgbd_path(argv[3]), output_path(argv[4]);
    if (rgbd_path.back() != '/' && rgbd_path.back() != '\\')
        rgbd_path += "/";
    if (output_path.back() != '/' && output_path.back() != '\\')
        output_path += "/";
    const int kStartFrameIdx = atoi(argv[5]), kEndFrameIdx = atoi(argv[6]);

    // Load mesh and initialize it
    MeshVisibility* mesh = new MeshVisibility();
    string mesh_suffix = mesh_fname.substr(mesh_fname.length() - 3, 3);
    printInGreen("Reading mesh file " + mesh_fname);
    if (mesh_suffix == "ply" || mesh_suffix == "PLY")
    {
        if (!mesh->readPLY(mesh_fname))
        {
            delete mesh;
            return -1;
        }
        cout << "#Vertex: " << mesh->vertex_num_ << ", #Faces: " << mesh->face_num_ << endl;
        mesh->mesh_suffix_ = "ply";
    }
    else if (mesh_suffix == "obj" || mesh_suffix == "OBJ")
    {
        if (!mesh->readOBJ(mesh_fname))
        {
            delete mesh;
            return -1;
        }
        cout << "#Vertex: " << mesh->vertex_num_ << ", #Faces: " << mesh->face_num_ << endl;
        mesh->mesh_suffix_ = "obj";
    }
    printInGreen("Reading all camera pose files in directory " + rgbd_path);
    if (!mesh->readCameraPoses(rgbd_path, kStartFrameIdx, kEndFrameIdx))
    {
        delete mesh;
        return -1;
    }
    printInGreen("Reading intrinsic file " + rgbd_path + "info.txt");
    if (!mesh->readCameraIntrinsicsFile(rgbd_path + "info.txt"))
    {
        delete mesh;
        return -1;
    }
    if (!initGLFWAndShader())
    {
        delete mesh;
        return -1;
    }
    mesh->initModelDataBuffer();
    if (program_mode_ == RENDER_MODEL)
    {
        printInGreen("Rendering mode ... ");
        printInBlue(
            "Usage: LEFT and RIGHT key to move backward and forward frames, C to render vertex color, D to render depth, T to "
            "render face texture.");
        runRenderMode(mesh, kStartFrameIdx, kEndFrameIdx);
    }
    else
    {
        printInGreen("Saving mode ... ");
        runSaveMode(mesh, kStartFrameIdx, kEndFrameIdx, output_path);
    }
    mesh->deallocate();
    delete mesh;

    // Close OpenGL window and terminate GLFW
    glfwTerminate();

    return 0;
}
