// Standard includes
#include <cstdio>
#include <cmath>

// OpenGL includes
#include <GL/glew.h>
#include <GL/freeglut.h>

// ZED includes
#include <sl/Camera.hpp>
#include <sl/defines.hpp>

// Sample includes
#include <GLObject.hpp>
#include "utils.hpp"
#include <cuda_gl_interop.h>

// QR code includes
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

// Definitions due to some bugs in VS
#define  GLUT_DOUBLE 0x0002
#define  GLUT_RGBA 0x0000

#define CV_CN_SHIFT 3
#define CV_DEPTH_MAX (1 << CV_CN_SHIFT)
#define CV_MAT_DEPTH_MASK (CV_DEPTH_MAX - 1)
#define CV_MAT_DEPTH(flags) ((flags) & CV_MAT_DEPTH_MASK)
#define CV_MAKETYPE(depth,cn) (CV_MAT_DEPTH(depth) + (((cn)-1) << CV_CN_SHIFT))
#define CV_8U 0
#define CV_8UC3 CV_MAKETYPE(CV_8U,3)
#define CV_8UC1 CV_MAKETYPE(CV_8U,1)
#define CV_8UC4 CV_MAKETYPE(CV_8U,4)

using namespace cv;
using namespace std;

const int CV_QR_NORTH = 0;
const int CV_QR_EAST = 1;
const int CV_QR_SOUTH = 2;
const int CV_QR_WEST = 3;

float cv_distance(Point2f P, Point2f Q);									// Get Distance between two points
float cv_lineEquation(Point2f L, Point2f M, Point2f J);						// Perpendicular Distance of a Point J from line formed by Points L and M; Solution to equation of the line Val = ax+by+c 
float cv_lineSlope(Point2f L, Point2f M, int& alignement);					// Slope of a line by two Points L and M on it; Slope of line, S = (x1 -x2) / (y1- y2)
void cv_getVertices(vector<vector<Point> > contours, int c_id, float slope, vector<Point2f>& X);
void cv_updateCorner(Point2f P, Point2f ref, float& baseline, Point2f& corner);
void cv_updateCornerOr(int orientation, vector<Point2f> IN, vector<Point2f> &OUT);
bool getIntersectionPoint(Point2f a1, Point2f a2, Point2f b1, Point2f b2, Point2f& intersection);
float cross(Point2f v1, Point2f v2);

// Define if you want to use the mesh as a set of chunks or as a global entity.
#define USE_CHUNKS 1

// ZED object (camera, mesh, pose)
sl::Camera zed;
sl::Mat left_image;															// sl::Mat to hold images
sl::Pose pose;																// sl::Pose to hold pose data
sl::Mesh mesh;																// sl::Mesh to hold the mesh generated during spatial mapping
sl::SpatialMappingParameters spatial_mapping_params;
sl::MeshFilterParameters filter_params;
sl::TRACKING_STATE tracking_state;

// For CUDA-OpenGL interoperability
cudaGraphicsResource* cuda_gl_ressource;									//cuda GL resource           

// OpenGL mesh container
std::vector<MeshObject> mesh_object;										// Opengl mesh container
sl::float3 vertices_color;													// Defines the color of the mesh

// OpenGL camera projection matrix
sl::Transform camera_projection;

// Opengl object
Shader* shader_mesh = nullptr;												//GLSL Shader for mesh
Shader* shader_image = nullptr;												//GLSL Shader for image
GLuint imageTex;															//OpenGL texture mapped with a cuda array (opengl gpu interop)
GLuint shMVPMatrixLoc;														//Shader variable loc
GLuint shColorLoc;															//Shader variable loc
GLuint texID;																//Shader variable loc (sampler/texture)
GLuint fbo = 0;																//FBO
GLuint renderedTexture = 0;													//Render Texture for FBO
GLuint quad_vb;																//buffer for vertices/coords for image

// OpenGL Viewport size
int wWnd = 1280;
int hWnd = 720;

// Spatial Mapping status
bool mapping_is_started = false;
std::chrono::high_resolution_clock::time_point t_last;

//// Sample functions
void close();
void run();
void startMapping();
void stopMapping();
void keyPressedCallback(unsigned char c, int x, int y);
int initGL();
void drawGL();
void qr_code_detection();

// QR code variables
sl::Mat image_sl;
cv::Mat image_cv;

Mat gray;
Mat edges;
Mat traces;																	// For Debug Visuals
Mat qr, qr_raw, qr_gray, qr_thres;

vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
vector<Point> pointsseq;													//used to save the approximated sides of each contour

int mark, A, B, C, top, my_right, bottom, median1, median2, outlier;
float AB, BC, CA, dist, slope, areat, arear, areab, large, padding;
int my_align, orientation;
int DBG = 1;																// Debug Flag
int key = 0;

int main(int argc, char** argv) {
    // Init GLUT window
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);

    // Setup configuration parameters for the ZED    
    sl::InitParameters parameters;
    if (argc > 1) parameters.svo_input_filename = argv[1];

    parameters.depth_mode = sl::DEPTH_MODE_QUALITY;							// Use QUALITY depth mode to improve mapping results
    parameters.coordinate_units = sl::UNIT_METER;
    parameters.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP; // OpenGL coordinates system

    // Open the ZED
    sl::ERROR_CODE err = zed.open(parameters);
    if (err != sl::ERROR_CODE::SUCCESS) {
        std::cout << sl::toString(err) << std::endl;
        zed.close();
        return -1;
    }

	sl::Resolution image_size = zed.getResolution();

    wWnd = (int) image_size.width;
    hWnd = (int) image_size.height;

    // Create GLUT window
    glutInitWindowSize(wWnd, hWnd);
    glutCreateWindow("ZED Spatial Mapping");

    // Configure Spatial Mapping and filtering parameters
    spatial_mapping_params.range_meter = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RANGE_NEAR);
    spatial_mapping_params.resolution_meter = sl::SpatialMappingParameters::get(sl::SpatialMappingParameters::MAPPING_RESOLUTION_HIGH);
    spatial_mapping_params.save_texture = true;
    spatial_mapping_params.max_memory_usage = 512;
    spatial_mapping_params.use_chunk_only = !USE_CHUNKS;					// If we use chunks we do not need to keep the mesh consistent

    filter_params.set(sl::MeshFilterParameters::MESH_FILTER_LOW);

    // Initialize OpenGL
    int res = initGL();
    if (res != 0) {
        std::cout << "Failed to initialize OpenGL" << std::endl;
        zed.close();
        return -1;
    }

    std::cout << "*************************************************************" << std::endl;
    std::cout << "**      Press the Space Bar key to start and stop          **" << std::endl;
    std::cout << "*************************************************************" << std::endl;

    // Set glut callback before start
    glutKeyboardFunc(keyPressedCallback);									// Callback that starts spatial mapping when space bar is pressed
    glutDisplayFunc(run);													// Callback that updates mesh data
    glutCloseFunc(close);													// Close callback

	// Capture Image from Image Input
	zed.retrieveImage(image_sl, sl::VIEW_LEFT, sl::MEM_CPU, wWnd, hWnd);

	// Copy the sl::Mat to a cv::Mat
	cv::Mat buffer_cv = cv::Mat(image_sl.getHeight(), image_sl.getWidth(), CV_8UC4, image_sl.getPtr<sl::uchar1>(sl::MEM_CPU));
	buffer_cv.copyTo(image_cv);

	gray = Mat(image_cv.size(), CV_MAKETYPE(image_cv.depth(), 1));
	edges = Mat(image_cv.size(), CV_MAKETYPE(image_cv.depth(), 1));
	traces = Mat(image_cv.size(), CV_8UC3);

    /* Start the glut main loop thread*/
    glutMainLoop();

    return 0;
}

/**
This function close the sample (when a close event is generated)
**/
void close() {
    left_image.free();

    if (shader_mesh) delete shader_mesh;
    if (shader_image) delete shader_image;

    mesh_object.clear();
    zed.close();
}

/**
Start the spatial mapping process
**/
void startMapping() {
    // clear previously used objects
    mesh.clear();
    mesh_object.clear();

#if !USE_CHUNKS
    // Create only one object that will contain the full mesh.
    // Otherwise, different MeshObject will be created for each chunk when needed
    mesh_object.emplace_back();
#endif

    // Enable positional tracking before starting spatial mapping
    zed.enableTracking();
    // Enable spatial mapping
    zed.enableSpatialMapping(spatial_mapping_params);

    // Start a timer, we retrieve the mesh every XXms.
    t_last = std::chrono::high_resolution_clock::now();

    mapping_is_started = true;
    std::cout << "** Spatial Mapping is started ... **" << std::endl;
}

/**
Stop the spatial mapping process
**/
void stopMapping() {
    // Stop the mesh request and extract the whole mesh to filter it and save it as an obj file
    mapping_is_started = false;
    std::cout << "** Stop Spatial Mapping ... **" << std::endl;

    // Extract the whole mesh
    sl::Mesh wholeMesh;
    zed.extractWholeMesh(wholeMesh);
    std::cout << ">> Mesh has been extracted..." << std::endl;

    // Filter the extracted mesh
    wholeMesh.filter(filter_params, USE_CHUNKS);
    std::cout << ">> Mesh has been filtered..." << std::endl;

    // If textures have been saved during spatial mapping, apply them to the mesh
    if (spatial_mapping_params.save_texture) {
        wholeMesh.applyTexture(sl::MESH_TEXTURE_RGB);
        std::cout << ">> Mesh has been textured..." << std::endl;
    }

    //Save as an OBJ file
    std::string saveName = getDir() + "mesh_gen.obj";
    bool t = wholeMesh.save(saveName.c_str());
    if (t) std::cout << ">> Mesh has been saved under " << saveName << std::endl;
    else std::cout << ">> Failed to save the mesh under  " << saveName << std::endl;

    // Update the displayed Mesh
#if USE_CHUNKS
    mesh_object.clear();
    mesh_object.resize(wholeMesh.chunks.size());
    for (int c = 0; c < wholeMesh.chunks.size(); c++)
        mesh_object[c].updateMesh(wholeMesh.chunks[c].vertices, wholeMesh.chunks[c].triangles);
#else
    mesh_object[0].updateMesh(wholeMesh.vertices, wholeMesh.triangles);
#endif
}

/**
Update the mesh and draw image and wireframe using OpenGL
**/
void run() {

    if (zed.grab() == sl::SUCCESS) {
        // Retrieve image in GPU memory
        zed.retrieveImage(left_image, sl::VIEW_LEFT, sl::MEM_GPU);

        // CUDA - OpenGL interop : copy the GPU buffer to a CUDA array mapped to the texture.
        cudaArray_t ArrIm;
        cudaGraphicsMapResources(1, &cuda_gl_ressource, 0);
        cudaGraphicsSubResourceGetMappedArray(&ArrIm, cuda_gl_ressource, 0, 0);
        cudaMemcpy2DToArray(ArrIm, 0, 0, left_image.getPtr<sl::uchar1>(sl::MEM_GPU), left_image.getStepBytes(sl::MEM_GPU), left_image.getPixelBytes()*left_image.getWidth(), left_image.getHeight(), cudaMemcpyDeviceToDevice);
        cudaGraphicsUnmapResources(1, &cuda_gl_ressource, 0);

        // Update pose data (used for projection of the mesh over the current image)
        tracking_state = zed.getPosition(pose);

		////////////////////////////////////////////////////////////////////////////////// QR code \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

		qr_code_detection();

		///////////////////////////////////////////////////////////////////////////////////// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

        if (mapping_is_started) {

            // Compute elapse time since the last call of sl::Camera::requestMeshAsync()
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t_last).count();
            // Ask for a mesh update if 500ms have spend since last request
            if (duration > 500) {
                zed.requestMeshAsync();
                t_last = std::chrono::high_resolution_clock::now();
            }

            if (zed.getMeshRequestStatusAsync() == sl::SUCCESS) {
                // Get the current mesh generated and send it to opengl
                if (zed.retrieveMeshAsync(mesh) == sl::SUCCESS) {
#if USE_CHUNKS
                    for (int c = 0; c < mesh.chunks.size(); c++) {
                        // If the chunk does not exist in the rendering process -> add it in the rendering list
                        if (mesh_object.size() < mesh.chunks.size()) mesh_object.emplace_back();
                        // If the chunck has been updated by the spatial mapping, update it for rendering
                        if (mesh.chunks[c].has_been_updated)
                            mesh_object[c].updateMesh(mesh.chunks[c].vertices, mesh.chunks[c].triangles);
                    }
#else
                    mesh_object[0].updateMesh(mesh.vertices, mesh.triangles);
#endif
                }
            }
        }

        // Display image and mesh using OpenGL 
        drawGL();
    }

    // If SVO input is enabled, close the window and stop mapping if video reached the end
    if (zed.getSVOPosition() > 0 && zed.getSVOPosition() == zed.getSVONumberOfFrames() - 1)
        glutLeaveMainLoop();

    // Prepare next update
    glutPostRedisplay();
}

/**
Initialize OpenGL window and objects
**/
int initGL() {

    // Init glew after window has been created
    glewInit();
    glClearColor(0.0, 0.0, 0.0, 0.0);

    // Create and Register OpenGL Texture for Image (RGBA -- 4channels)
    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, &imageTex);
    glBindTexture(GL_TEXTURE_2D, imageTex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, wWnd, hWnd, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, NULL);
    glBindTexture(GL_TEXTURE_2D, 0);
    cudaError_t err1 = cudaGraphicsGLRegisterImage(&cuda_gl_ressource, imageTex, GL_TEXTURE_2D, cudaGraphicsRegisterFlagsWriteDiscard);
    if (err1 != cudaError::cudaSuccess) return -1;

    // Create GLSL Shaders for Mesh and Image
    shader_mesh = new Shader((GLchar*) MESH_VERTEX_SHADER, (GLchar*) MESH_FRAGMENT_SHADER);
    shMVPMatrixLoc = glGetUniformLocation(shader_mesh->getProgramId(), "u_mvpMatrix");
    shColorLoc = glGetUniformLocation(shader_mesh->getProgramId(), "u_color");
    shader_image = new Shader((GLchar*) IMAGE_VERTEX_SHADER, (GLchar*) IMAGE_FRAGMENT_SHADER);
    texID = glGetUniformLocation(shader_image->getProgramId(), "texImage");

    // Create Frame Buffer for offline rendering
    // Here we render the composition of the image and the projection of the mesh on top of it in a texture (using FBO - Frame Buffer Object)
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    // Generate a render texture (which will contain the image and mesh in wireframe overlay)
    glGenTextures(1, &renderedTexture);
    glBindTexture(GL_TEXTURE_2D, renderedTexture);

    // Give an empty image to OpenGL ( the last "0" as pointer )
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, wWnd, hWnd, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    // Set "renderedTexture" as our color attachment #0
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, renderedTexture, 0);

    // Set the list of draw buffers.
    GLenum DrawBuffers[1] = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(1, DrawBuffers);

    // Always check that our framebuffer is ok
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cout << "invalid FrameBuffer" << std::endl;
        return -1;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Create Projection Matrix for OpenGL. We will use this matrix in combination with the Pose (on REFERENCE_FRAME_WORLD) to project the mesh on the 2D Image.
    sl::CameraParameters camLeft = zed.getCameraInformation().calibration_parameters.left_cam;
    camera_projection(0, 0) = 1.0f / tanf(camLeft.h_fov * M_PI / 180.f * 0.5f);
    camera_projection(1, 1) = 1.0f / tanf(camLeft.v_fov * M_PI / 180.f * 0.5f);
    float znear = 0.001f;
    float zfar = 100.f;
    camera_projection(2, 2) = -(zfar + znear) / (zfar - znear);
    camera_projection(2, 3) = -(2.f * zfar * znear) / (zfar - znear);
    camera_projection(3, 2) = -1.f;
    camera_projection(0, 2) = (camLeft.image_size.width - 2.f * camLeft.cx) / camLeft.image_size.width;
    camera_projection(1, 2) = (-1.f * camLeft.image_size.height + 2.f * camLeft.cy) / camLeft.image_size.height;
    camera_projection(3, 3) = 0.f;

    // Generate the Quad for showing the image in a full viewport
    static const GLfloat g_quad_vertex_buffer_data[] = {
        -1.0f, -1.0f, 0.0f,
        1.0f, -1.0f, 0.0f,
        -1.0f, 1.0f, 0.0f,
        -1.0f, 1.0f, 0.0f,
        1.0f, -1.0f, 0.0f,
        1.0f, 1.0f, 0.0f};

    // Color of wireframe (soft blue)
    vertices_color.r = 0.35f;
    vertices_color.g = 0.65f;
    vertices_color.b = 0.95f;

    // Generate a buffer to handle vertices for the GLSL shader
    glGenBuffers(1, &quad_vb);
    glBindBuffer(GL_ARRAY_BUFFER, quad_vb);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_quad_vertex_buffer_data), g_quad_vertex_buffer_data, GL_STATIC_DRAW);

    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);

    return 0;
}

/**
This function draws a text with OpenGL
**/
void printGL(float x, float y, const char *string) {
    glRasterPos2f(x, y);
    int len = (int) strlen(string);
    for (int i = 0; i < len; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, string[i]);
    }
}

/**
OpenGL draw function
Render Image and wireframe mesh into a texture using the FrameBuffer
**/
void drawGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_TEXTURE_2D);
    glActiveTexture(GL_TEXTURE0);

    glViewport(0, 0, wWnd, hWnd);

    // Render image and wireframe mesh into a texture using frame buffer
    // Bind the frame buffer and specify the viewport (full screen)
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    // Render the ZED view (Left) in the framebuffer
    glUseProgram(shader_image->getProgramId());
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, imageTex);
    glUniform1i(texID, 0);
    //invert y axis and color for this image (since its reverted from cuda array)
    glUniform1i(glGetUniformLocation(shader_image->getProgramId(), "revert"), 1);
    glUniform1i(glGetUniformLocation(shader_image->getProgramId(), "rgbflip"), 1);

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, quad_vb);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*) 0);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glDisableVertexAttribArray(0);
    glUseProgram(0);

    // If the Positional tracking is good, we can draw the mesh over the current image
    if (tracking_state == sl::TRACKING_STATE_OK && mesh_object.size()) {
        glDisable(GL_TEXTURE_2D);
        // Send the projection and the Pose to the GLSL shader to make the projection of the 2D image.
        sl::Transform vpMatrix = sl::Transform::transpose(camera_projection * sl::Transform::inverse(pose.pose_data));
        glUseProgram(shader_mesh->getProgramId());
        glUniformMatrix4fv(shMVPMatrixLoc, 1, GL_FALSE, vpMatrix.m);

        glUniform3fv(shColorLoc, 1, vertices_color.v);
        // Draw the mesh in GL_TRIANGLES with a polygon mode in line (wire)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

#if USE_CHUNKS
        for (int c = 0; c < mesh_object.size(); c++)
            mesh_object[c].draw(GL_TRIANGLES);
#else
        mesh_object[0].draw(GL_TRIANGLES);
#endif

        glUseProgram(0);
    }

    // Unbind the framebuffer since the texture is now updated
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Render the texture to the screen
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glUseProgram(shader_image->getProgramId());
    glBindTexture(GL_TEXTURE_2D, renderedTexture);
    glUniform1i(texID, 0);
    glUniform1i(glGetUniformLocation(shader_image->getProgramId(), "revert"), 0);
    glUniform1i(glGetUniformLocation(shader_image->getProgramId(), "rgbflip"), 0);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, quad_vb);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*) 0);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glDisableVertexAttribArray(0);
    glUseProgram(0);
    glDisable(GL_TEXTURE_2D);

    // Show actions
    glColor4f(0.25f, 0.99f, 0.25f, 1.f);
    if (!mapping_is_started)
        printGL(-0.99f, 0.9f, "Press Space Bar to activate Spatial Mapping.");
	else {
		printGL(-0.99f, 0.9f, "Press Space Bar to stop spatial mapping.");
		printGL(-0.99f, 0.7f, "Press C to grab the QR code.");
	}

    std::string positional_tracking_state_str("POSITIONAL TRACKING STATE : ");
    std::string spatial_mapping_state_str("SPATIAL MAPPING STATE : ");
    std::string state_str;
    // Show mapping state
    if ((tracking_state == sl::TRACKING_STATE_OK)) {
        sl::SPATIAL_MAPPING_STATE spatial_mapping_state = zed.getSpatialMappingState();
        if (spatial_mapping_state == sl::SPATIAL_MAPPING_STATE_OK || spatial_mapping_state == sl::SPATIAL_MAPPING_STATE_INITIALIZING)
            glColor4f(0.25f, 0.99f, 0.25f, 1.f);
        else if (spatial_mapping_state == sl::SPATIAL_MAPPING_STATE_NOT_ENABLED)
            glColor4f(0.55f, 0.65f, 0.55f, 1.f);
        else
            glColor4f(0.95f, 0.25f, 0.25f, 1.f);
        state_str = spatial_mapping_state_str + sl::toString(spatial_mapping_state).c_str();
    } else {
        if (mapping_is_started) {
            glColor4f(0.95f, 0.25f, 0.25f, 1.f);
            state_str = positional_tracking_state_str + sl::toString(tracking_state).c_str();
        } else {
            glColor4f(0.55f, 0.65f, 0.55f, 1.f);
            state_str = spatial_mapping_state_str + sl::toString(sl::SPATIAL_MAPPING_STATE_NOT_ENABLED).c_str();
        }
    }
    printGL(-0.99f, 0.83f, state_str.c_str());

    // Swap buffers
    glutSwapBuffers();
}

/**
This function handles keyboard events (especially space bar to start the mapping)
**/
void keyPressedCallback(unsigned char c, int x, int y) {
    switch (c) {
        case 32: // Space bar id	
        if (!mapping_is_started) // User press the space bar and spatial mapping is not started 
            startMapping();
        else // User press the space bar and spatial mapping is started 
            stopMapping();
        break;
        case 'q':
        glutLeaveMainLoop(); // End the process	
        break;
        default:
        break;
    }
}

// Routines used in Main loops

// Function: Routine to get Distance between two points
// Description: Given 2 points, the function returns the distance

float cv_distance(Point2f P, Point2f Q)
{
	return sqrt(pow(abs(P.x - Q.x), 2) + pow(abs(P.y - Q.y), 2));
}


// Function: Perpendicular Distance of a Point J from line formed by Points L and M; Equation of the line ax+by+c=0
// Description: Given 3 points, the function derives the line quation of the first two points,
//	  calculates and returns the perpendicular distance of the the 3rd point from this line.

float cv_lineEquation(Point2f L, Point2f M, Point2f J)
{
	float a, b, c, pdist;

	a = -((M.y - L.y) / (M.x - L.x));
	b = 1.0;
	c = (((M.y - L.y) / (M.x - L.x)) * L.x) - L.y;

	// Now that we have a, b, c from the equation ax + by + c, time to substitute (x,y) by values from the Point J

	pdist = (a * J.x + (b * J.y) + c) / sqrt((a * a) + (b * b));
	return pdist;
}

// Function: Slope of a line by two Points L and M on it; Slope of line, S = (x1 -x2) / (y1- y2)
// Description: Function returns the slope of the line formed by given 2 points, the alignement flag
//	  indicates the line is vertical and the slope is infinity.

float cv_lineSlope(Point2f L, Point2f M, int& alignement)
{
	float dx, dy;
	dx = M.x - L.x;
	dy = M.y - L.y;

	if (dy != 0)
	{
		alignement = 1;
		return (dy / dx);
	}
	else				// Make sure we are not dividing by zero; so use 'alignement' flag
	{
		alignement = 0;
		return 0.0;
	}
}



// Function: Routine to calculate 4 Corners of the Marker in Image Space using Region partitioning
// Theory: OpenCV Contours stores all points that describe it and these points lie the perimeter of the polygon.
//	The below function chooses the farthest points of the polygon since they form the vertices of that polygon,
//	exactly the points we are looking for. To choose the farthest point, the polygon is divided/partitioned into
//	4 regions equal regions using bounding box. Distance algorithm is applied between the centre of bounding box
//	every contour point in that region, the farthest point is deemed as the vertex of that region. Calculating
//	for all 4 regions we obtain the 4 corners of the polygon ( - quadrilateral).
void cv_getVertices(vector<vector<Point> > contours, int c_id, float slope, vector<Point2f>& quad)
{
	Rect box;
	box = boundingRect(contours[c_id]);

	Point2f M0, M1, M2, M3;
	Point2f A, B, C, D, W, X, Y, Z;

	A = box.tl();
	B.x = box.br().x;
	B.y = box.tl().y;
	C = box.br();
	D.x = box.tl().x;
	D.y = box.br().y;


	W.x = (A.x + B.x) / 2;
	W.y = A.y;

	X.x = B.x;
	X.y = (B.y + C.y) / 2;

	Y.x = (C.x + D.x) / 2;
	Y.y = C.y;

	Z.x = D.x;
	Z.y = (D.y + A.y) / 2;

	float dmax[4];
	dmax[0] = 0.0;
	dmax[1] = 0.0;
	dmax[2] = 0.0;
	dmax[3] = 0.0;

	float pd1 = 0.0;
	float pd2 = 0.0;

	if (slope > 5 || slope < -5)
	{

		for (int i = 0; i < contours[c_id].size(); i++)
		{
			pd1 = cv_lineEquation(C, A, contours[c_id][i]);	// Position of point w.r.t the diagonal AC 
			pd2 = cv_lineEquation(B, D, contours[c_id][i]);	// Position of point w.r.t the diagonal BD

			if ((pd1 >= 0.0) && (pd2 > 0.0))
			{
				cv_updateCorner(contours[c_id][i], W, dmax[1], M1);
			}
			else if ((pd1 > 0.0) && (pd2 <= 0.0))
			{
				cv_updateCorner(contours[c_id][i], X, dmax[2], M2);
			}
			else if ((pd1 <= 0.0) && (pd2 < 0.0))
			{
				cv_updateCorner(contours[c_id][i], Y, dmax[3], M3);
			}
			else if ((pd1 < 0.0) && (pd2 >= 0.0))
			{
				cv_updateCorner(contours[c_id][i], Z, dmax[0], M0);
			}
			else
				continue;
		}
	}
	else
	{
		int halfx = (A.x + B.x) / 2;
		int halfy = (A.y + D.y) / 2;

		for (int i = 0; i < contours[c_id].size(); i++)
		{
			if ((contours[c_id][i].x < halfx) && (contours[c_id][i].y <= halfy))
			{
				cv_updateCorner(contours[c_id][i], C, dmax[2], M0);
			}
			else if ((contours[c_id][i].x >= halfx) && (contours[c_id][i].y < halfy))
			{
				cv_updateCorner(contours[c_id][i], D, dmax[3], M1);
			}
			else if ((contours[c_id][i].x > halfx) && (contours[c_id][i].y >= halfy))
			{
				cv_updateCorner(contours[c_id][i], A, dmax[0], M2);
			}
			else if ((contours[c_id][i].x <= halfx) && (contours[c_id][i].y > halfy))
			{
				cv_updateCorner(contours[c_id][i], B, dmax[1], M3);
			}
		}
	}

	quad.push_back(M0);
	quad.push_back(M1);
	quad.push_back(M2);
	quad.push_back(M3);

}

// Function: Compare a point if it more far than previously recorded farthest distance
// Description: Farthest Point detection using reference point and baseline distance
void cv_updateCorner(Point2f P, Point2f ref, float& baseline, Point2f& corner)
{
	float temp_dist;
	temp_dist = cv_distance(P, ref);

	if (temp_dist > baseline)
	{
		baseline = temp_dist;			// The farthest distance is the new baseline
		corner = P;						// P is now the farthest point
	}

}

// Function: Sequence the Corners wrt to the orientation of the QR Code
void cv_updateCornerOr(int orientation, vector<Point2f> in, vector<Point2f>& out)
{
	Point2f M0, M1, M2, M3;	
	if (orientation == CV_QR_NORTH)
	{
		M0 = in[0];
		M1 = in[1];
		M2 = in[2];
		M3 = in[3];
	}
	else if (orientation == CV_QR_EAST)
	{
		M0 = in[1];
		M1 = in[2];
		M2 = in[3];
		M3 = in[0];
	}
	else if (orientation == CV_QR_SOUTH)
	{
		M0 = in[2];
		M1 = in[3];
		M2 = in[0];
		M3 = in[1];
	}
	else if (orientation == CV_QR_WEST)
	{
		M0 = in[3];
		M1 = in[0];
		M2 = in[1];
		M3 = in[2];
	}

	out.push_back(M0);
	out.push_back(M1);
	out.push_back(M2);
	out.push_back(M3);
}

// Function: Get the Intersection Point of the lines formed by sets of two points
bool getIntersectionPoint(Point2f a1, Point2f a2, Point2f b1, Point2f b2, Point2f& intersection)
{
	Point2f p = a1;
	Point2f q = b1;
	Point2f r(a2 - a1);
	Point2f s(b2 - b1);

	if (cross(r, s) == 0) { return false; }

	float t = cross(q - p, s) / cross(r, s);

	intersection = p + t*r;
	return true;
}

float cross(Point2f v1, Point2f v2)
{
	return v1.x*v2.y - v1.y*v2.x;
}

void qr_code_detection() {
	traces = Scalar(0, 0, 0);
	qr_raw = Mat::zeros(100, 100, CV_8UC3);
	qr = Mat::zeros(100, 100, CV_8UC3);
	qr_gray = Mat::zeros(100, 100, CV_8UC1);
	qr_thres = Mat::zeros(100, 100, CV_8UC1);

	// Capture Image from Image Input
	zed.retrieveImage(image_sl, sl::VIEW_LEFT, sl::MEM_CPU, wWnd, hWnd);

	// Copy the sl::Mat to a cv::Mat
	cv::Mat buffer_cv = cv::Mat(image_sl.getHeight(), image_sl.getWidth(), CV_8UC4, image_sl.getPtr<sl::uchar1>(sl::MEM_CPU));
	buffer_cv.copyTo(image_cv);

	cvtColor(image_cv, gray, CV_RGB2GRAY);	// Convert Image captured from Image Input to GrayScale	
	Canny(gray, edges, 100, 200, 3);		// Apply Canny edge detection on the gray image


	findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE); // Find contours with hierarchy

	mark = 0;								// Reset all detected marker count for this frame

											// Get Moments for all Contours and the mass centers
	vector<Moments> mu(contours.size());
	vector<Point2f> mc(contours.size());

	for (int i = 0; i < contours.size(); i++)
	{
		mu[i] = moments(contours[i], false);
		mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
	}


	// Start processing the contour data

	// Find Three repeatedly enclosed contours A,B,C
	// NOTE: 1. Contour enclosing other contours is assumed to be the three Alignment markings of the QR code.
	// 2. Alternately, the Ratio of areas of the "concentric" squares can also be used for identifying base Alignment markers.
	// The below demonstrates the first method

	for (int i = 0; i < contours.size(); i++)
	{
		//Find the approximated polygon of the contour we are examining
		approxPolyDP(contours[i], pointsseq, arcLength(contours[i], true)*0.02, true);
		if (pointsseq.size() == 4)      // only quadrilaterals contours are examined
		{
			int k = i;
			int c = 0;

			while (hierarchy[k][2] != -1)
			{
				k = hierarchy[k][2];
				c = c + 1;
			}
			if (hierarchy[k][2] != -1)
				c = c + 1;

			if (c >= 5)
			{
				if (mark == 0)		A = i;
				else if (mark == 1)	B = i;		// i.e., A is already found, assign current contour to B
				else if (mark == 2)	C = i;		// i.e., A and B are already found, assign current contour to C
				mark = mark + 1;
			}
		}
	}


	if (mark >= 3)		// Ensure we have (atleast 3; namely A,B,C) 'Alignment Markers' discovered
	{
		// We have found the 3 markers for the QR code; Now we need to determine which of them are 'top', 'right' and 'bottom' markers

		// Determining the 'top' marker
		// Vertex of the triangle NOT involved in the longest side is the 'outlier'

		AB = cv_distance(mc[A], mc[B]);
		BC = cv_distance(mc[B], mc[C]);
		CA = cv_distance(mc[C], mc[A]);

		if (AB > BC && AB > CA)
		{
			outlier = C; median1 = A; median2 = B;
		}
		else if (CA > AB && CA > BC)
		{
			outlier = B; median1 = A; median2 = C;
		}
		else if (BC > AB && BC > CA)
		{
			outlier = A;  median1 = B; median2 = C;
		}

		top = outlier;							// The obvious choice

		dist = cv_lineEquation(mc[median1], mc[median2], mc[outlier]);	// Get the Perpendicular distance of the outlier from the longest side			
		slope = cv_lineSlope(mc[median1], mc[median2], my_align);		// Also calculate the slope of the longest side

																		// Now that we have the orientation of the line formed median1 & median2 and we also have the position of the outlier w.r.t. the line
																		// Determine the 'right' and 'bottom' markers

		if (align == 0)
		{
			bottom = median1;
			my_right = median2;
		}
		else if (slope < 0 && dist < 0)		// Orientation - North
		{
			bottom = median1;
			my_right = median2;
			orientation = CV_QR_NORTH;
		}
		else if (slope > 0 && dist < 0)		// Orientation - East
		{
			my_right = median1;
			bottom = median2;
			orientation = CV_QR_EAST;
		}
		else if (slope < 0 && dist > 0)		// Orientation - South			
		{
			my_right = median1;
			bottom = median2;
			orientation = CV_QR_SOUTH;
		}

		else if (slope > 0 && dist > 0)		// Orientation - West
		{
			bottom = median1;
			my_right = median2;
			orientation = CV_QR_WEST;
		}


		// To ensure any unintended values do not sneak up when QR code is not present
		float area_top, area_right, area_bottom;

		if (top < contours.size() && my_right < contours.size() && bottom < contours.size() && contourArea(contours[top]) > 10 && contourArea(contours[my_right]) > 10 && contourArea(contours[bottom]) > 10)
		{

			vector<Point2f> L, M, O, tempL, tempM, tempO;
			Point2f N;

			vector<Point2f> src, dst;		// src - Source Points basically the 4 end co-ordinates of the overlay image
											// dst - Destination Points to transform overlay image	

			Mat warp_matrix;

			cv_getVertices(contours, top, slope, tempL);
			cv_getVertices(contours, my_right, slope, tempM);
			cv_getVertices(contours, bottom, slope, tempO);

			cv_updateCornerOr(orientation, tempL, L); 			// Re-arrange marker corners w.r.t orientation of the QR code
			cv_updateCornerOr(orientation, tempM, M); 			// Re-arrange marker corners w.r.t orientation of the QR code
			cv_updateCornerOr(orientation, tempO, O); 			// Re-arrange marker corners w.r.t orientation of the QR code

			int iflag = getIntersectionPoint(M[1], M[2], O[3], O[2], N);


			src.push_back(L[0]);
			src.push_back(M[1]);
			src.push_back(N);
			src.push_back(O[3]);

			dst.push_back(Point2f(0, 0));
			dst.push_back(Point2f(qr.cols, 0));
			dst.push_back(Point2f(qr.cols, qr.rows));
			dst.push_back(Point2f(0, qr.rows));

			if (src.size() == 4 && dst.size() == 4)			// Failsafe for WarpMatrix Calculation to have only 4 Points with src and dst
			{
				warp_matrix = getPerspectiveTransform(src, dst);
				warpPerspective(image_cv, qr_raw, warp_matrix, Size(qr.cols, qr.rows));
				copyMakeBorder(qr_raw, qr, 10, 10, 10, 10, BORDER_CONSTANT, Scalar(255, 255, 255));

				cvtColor(qr, qr_gray, CV_RGB2GRAY);
				threshold(qr_gray, qr_thres, 127, 255, CV_THRESH_BINARY);

				//threshold(qr_gray, qr_thres, 0, 255, CV_THRESH_OTSU);
				//for( int d=0 ; d < 4 ; d++){	src.pop_back(); dst.pop_back(); }
			}

			//Draw contours on the image
			drawContours(image_cv, contours, top, Scalar(255, 200, 0), 2, 8, hierarchy, 0);
			drawContours(image_cv, contours, my_right, Scalar(0, 0, 255), 2, 8, hierarchy, 0);
			drawContours(image_cv, contours, bottom, Scalar(255, 0, 100), 2, 8, hierarchy, 0);

			// Insert Debug instructions here
			if (DBG == 1)
			{
				// Debug Prints
				// Visualizations for ease of understanding
				if (slope > 5)
					circle(traces, Point(10, 20), 5, Scalar(0, 0, 255), -1, 8, 0);
				else if (slope < -5)
					circle(traces, Point(10, 20), 5, Scalar(255, 255, 255), -1, 8, 0);

				// Draw contours on Trace image for analysis	
				drawContours(traces, contours, top, Scalar(255, 0, 100), 1, 8, hierarchy, 0);
				drawContours(traces, contours, my_right, Scalar(255, 0, 100), 1, 8, hierarchy, 0);
				drawContours(traces, contours, bottom, Scalar(255, 0, 100), 1, 8, hierarchy, 0);

				// Draw points (4 corners) on Trace image for each Identification marker	
				circle(traces, L[0], 2, Scalar(255, 255, 0), -1, 8, 0);
				circle(traces, L[1], 2, Scalar(0, 255, 0), -1, 8, 0);
				circle(traces, L[2], 2, Scalar(0, 0, 255), -1, 8, 0);
				circle(traces, L[3], 2, Scalar(128, 128, 128), -1, 8, 0);

				circle(traces, M[0], 2, Scalar(255, 255, 0), -1, 8, 0);
				circle(traces, M[1], 2, Scalar(0, 255, 0), -1, 8, 0);
				circle(traces, M[2], 2, Scalar(0, 0, 255), -1, 8, 0);
				circle(traces, M[3], 2, Scalar(128, 128, 128), -1, 8, 0);

				circle(traces, O[0], 2, Scalar(255, 255, 0), -1, 8, 0);
				circle(traces, O[1], 2, Scalar(0, 255, 0), -1, 8, 0);
				circle(traces, O[2], 2, Scalar(0, 0, 255), -1, 8, 0);
				circle(traces, O[3], 2, Scalar(128, 128, 128), -1, 8, 0);

				// Draw point of the estimated 4th Corner of (entire) QR Code
				circle(traces, N, 2, Scalar(255, 255, 255), -1, 8, 0);

				// Draw the lines used for estimating the 4th Corner of QR Code
				line(traces, M[1], N, Scalar(0, 0, 255), 1, 8, 0);
				line(traces, O[3], N, Scalar(0, 0, 255), 1, 8, 0);


				// Show the Orientation of the QR Code wrt to 2D Image Space
				int fontFace = FONT_HERSHEY_PLAIN;

				if (orientation == CV_QR_NORTH)
				{
					putText(traces, "NORTH", Point(20, 30), fontFace, 1, Scalar(0, 255, 0), 1, 8);
				}
				else if (orientation == CV_QR_EAST)
				{
					putText(traces, "EAST", Point(20, 30), fontFace, 1, Scalar(0, 255, 0), 1, 8);
				}
				else if (orientation == CV_QR_SOUTH)
				{
					putText(traces, "SOUTH", Point(20, 30), fontFace, 1, Scalar(0, 255, 0), 1, 8);
				}
				else if (orientation == CV_QR_WEST)
				{
					putText(traces, "WEST", Point(20, 30), fontFace, 1, Scalar(0, 255, 0), 1, 8);
				}

				// Debug Prints
			}

		}
	}

	imshow("Image", image_cv);

	if (!traces.empty()) {
		imshow("Traces", traces);
	}

	imshow("QR code", qr_thres);

	key = waitKey(1);	// OPENCV: wait for 1ms before accessing next frame
}

//EOF