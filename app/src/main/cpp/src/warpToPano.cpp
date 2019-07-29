//
// Created by ale on 19-2-17.
//
#include "warpToPano.h"
#include "shader_m.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


#include <iostream>

using namespace i3d;
using namespace cv;
using namespace std;


#define WARP_TO_PANOS_DEBUG 1

#ifdef WARP_TO_PANOS_DEBUG
#endif

static void GenerateSceneVertices(const Mat &depth, const Intrinsics &intrinsics, vector<float> &vertices)
{
    #ifdef WARP_TO_PANOS_DEBUG
        cout<<"GenerateSceneVertices begin"<<endl;
    #endif

    const float MAX_DIFF = 0.05;

    const float rows = depth.rows;
    const float cols = depth.cols;

    float cx, cy, f;
    cx = cols/2; cy = rows/2;
    f = intrinsics.f*rows/2/intrinsics.cy;

    #ifdef WARP_TO_PANOS_DEBUG
        cout<<"GenerateSceneVertices f, cx, cy: "<<f<<", "<<cx<<", "<<cy<<endl;
    #endif

    vertices.clear();
    for (float h = 0; h < rows; ++h)
    {
        for (float w = 0; w < cols; ++w)
        {
            float d = depth.at<float>(h, w);
            float x, y, z;
            float u, v, e;

            if(!(d>0))
                continue;

            if (h > 0 && w > 0)
            {
                float dleft = depth.at<float>(h, (int) w - 1);
                float dtop = depth.at<float>(h - 1, (int) w);
                if (abs(dleft - d) < MAX_DIFF*d && abs(dtop - d) < MAX_DIFF*d)//dleft > 0 && dtop > 0 &&
                {
                    //v0
                    z = d;
                    x = z * (w - cx) / f;
                    y = z * (h - cy) / f;
                    u = w / (float) (cols - 1);
                    v = 1 - h / (float) (rows - 1);
                    vertices.push_back(x); vertices.push_back(y); vertices.push_back(z);
                    vertices.push_back(u); vertices.push_back(v); vertices.push_back(0);
                    //v1, top
                    z = dtop;
                    x = z * (w - cx) / f;
                    y = z * (h - 1 - cy) / f;
                    u = (w) / (float) (cols - 1);
                    v = 1 - (h - 1) / (float) (rows - 1);
                    vertices.push_back(x); vertices.push_back(y); vertices.push_back(z);
                    vertices.push_back(u); vertices.push_back(v); vertices.push_back(0);
                    //v2, left
                    z = dleft;
                    x = z * (w - 1 - cx) / f;
                    y = z * (h - cy) / f;
                    u = (w - 1) / (float) (cols - 1);
                    v = 1 - h / (float) (rows - 1);
                    vertices.push_back(x); vertices.push_back(y); vertices.push_back(z);
                    vertices.push_back(u); vertices.push_back(v); vertices.push_back(0);
                }
            }

            if (h + 1 < rows && w + 1 < cols)
            {
                float dright = depth.ptr<float>(h)[(int) w + 1];
                float dbottom = depth.ptr<float>(h + 1)[(int) w];
                if (abs(dright - d) < MAX_DIFF*d && abs(dbottom - d) < MAX_DIFF*d)
                {
                    //v0
                    z = d;
                    x = z * (w - cx) / f;
                    y = z * (h - cy) / f;
                    u = w / (float) (cols - 1);
                    v = 1 - h / (float) (rows - 1);
                    vertices.push_back(x); vertices.push_back(y); vertices.push_back(z);
                    vertices.push_back(u); vertices.push_back(v); vertices.push_back(0);
                    //v1, bottom
                    z = dbottom;
                    x = z * (w - cx) / f;
                    y = z * (h + 1 - cy) / f;
                    u = (w) / (float) (cols - 1);
                    v = 1 - (h + 1) / (float) (rows - 1);
                    vertices.push_back(x); vertices.push_back(y); vertices.push_back(z);
                    vertices.push_back(u); vertices.push_back(v); vertices.push_back(0);
                    //v2, right
                    z = dright;
                    x = z * (w + 1 - cx) / f;
                    y = z * (h - cy) / f;
                    u = (w + 1) / (float) (cols - 1);
                    v = 1 - h / (float) (rows - 1);
                    vertices.push_back(x); vertices.push_back(y); vertices.push_back(z);
                    vertices.push_back(u); vertices.push_back(v); vertices.push_back(0);
                }
            }
        }
    }
    #ifdef WARP_TO_PANOS_DEBUG
        cout<<"GenerateSceneVertices vertices.size(): "<<vertices.size()<<endl;
        cout<<"GenerateSceneVertices finish"<<endl;
    #endif
}
static int GenerateSkybox(const Frame &frame, const Intrinsics &intrinsics, vector<Mat>& images, vector<Mat>& depths )
{
    #ifdef WARP_TO_PANOS_DEBUG
        cout<<"GenerateSkybox begin"<<endl;
    #endif
    const unsigned int SCR_WIDTH = PANO_H;
    const unsigned int SCR_HEIGHT = PANO_H;

    // glfw: initialize and configure
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);

    // glfw window creation
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // glad: load all OpenGL function pointers
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    vector<float> vertices;
    GenerateSceneVertices(frame.depth, intrinsics, vertices);
    #ifdef WARP_TO_PANOS_DEBUG
        cout<<"GenerateSkybox vertices.size(): "<<vertices.size()<<endl;
    #endif
    GLuint VAO;
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);
    GLuint VBO;
    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(float), &vertices[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
////////////////////////////////////////////////////////////////////////////////////////////////////

// load and create a texture
    GLuint color_tex_id;
    cv::Mat color_tex_ = frame.image;
    cv::flip(color_tex_, color_tex_, 0);
    //opencv不自动4byte对齐，但是opengl默认传入cpu的数据为4byte对齐
    cv::Mat color_tex;
    cv::cvtColor(color_tex_, color_tex, COLOR_BGR2BGRA);

    glGenTextures(1, &color_tex_id);
    glBindTexture(GL_TEXTURE_2D, color_tex_id);
// set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
// set texture filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);//Linear for RGB color
    glTexImage2D(GL_TEXTURE_2D,     // Type of texture
                 0,                 // Pyramid level (for mip-mapping) - 0 is the top level
                 GL_RGB,            // Internal colour format to convert to
                 color_tex.cols,          // Image width  i.e. 640 for Kinect in standard mode
                 color_tex.rows,          // Image height i.e. 480 for Kinect in standard mode
                 0,                 // Border width in pixels (can either be 1 or 0)
                 GL_BGRA, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                 GL_UNSIGNED_BYTE,  // Image data type
                 color_tex.data);//ptr());        // The actual image data itself
    glGenerateMipmap(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);

    // load and create a texture
    GLuint depth_tex_id;
    //cv::Mat image_1 = imread("blackwhite_texture.jpg");
    //Mat depth = frame.depth;
    //cv::flip(depth, depth, 0);
    cv::Mat depth_tex = color_tex;
    //depth.convertTo(depth_tex, CV_32FC1);
    //depth_tex = depth_tex/100000.0f+0.1f;
    glGenTextures(1, &depth_tex_id);
    glBindTexture(GL_TEXTURE_2D, depth_tex_id);
// set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
// set texture filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);//Nearest for depth(require sharp edges)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D,     // Type of texture
                 0,                 // Pyramid level (for mip-mapping) - 0 is the top level
                 GL_RGB,            // Internal colour format to convert to
                 depth_tex.cols,          // Image width  i.e. 640 for Kinect in standard mode
                 depth_tex.rows,          // Image height i.e. 480 for Kinect in standard mode
                 0,                 // Border width in pixels (can either be 1 or 0)
                 GL_BGR, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                 GL_UNSIGNED_BYTE,  // Image data type
                 depth_tex.data);//ptr());        // The actual image data itself
    glGenerateMipmap(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);

// render loop
    glEnable(GL_DEPTH_TEST);
    Shader colorShader("cubemap.vertexshader", "cubemap.fragmentshader");

    //front, right, back, left, top, bottom/down
    vector<glm::mat4> views(6, glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f)));
    views[1] = glm::rotate(views[0], glm::radians(-90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    views[2] = glm::rotate(views[0], glm::radians(-180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    views[3] = glm::rotate(views[0], glm::radians(-270.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    views[4] = glm::rotate(views[0], glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    views[5] = glm::rotate(views[0], glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    //znear不能太小，否则将无法渲染稍远处的物体，因为znear与深度缓冲的精度有关
    /*You may have configured your zNear and zFar clipping planes in a way that severely limits your depth buffer precision.
     * Generally, this is caused by a zNear clipping plane value that's too close to 0.0. As the zNear clipping plane is set
     * increasingly closer to 0.0, the effective precision of the depth buffer decreases dramatically. Moving the zFar
     * clipping plane further away from the eye always has a negative impact on depth buffer precision, but it's not one as
     * dramatic as moving the zNearclipping plane.     */
    glm::mat4 projection = glm::perspective(glm::radians(90.0f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100000.0f);
    glm::mat4 model = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first
    model = glm::rotate(model, (float)frame.rx, glm::vec3(1.0f, 0.0f, 0.0f));
    model = glm::rotate(model, (float)frame.ry, glm::vec3(0.0f, 1.0f, 0.0f));
    model = glm::rotate(model, (float)frame.rz, glm::vec3(0.0f, 0.0f, 1.0f));
    model = glm::translate(model, glm::vec3(frame.tx, frame.ty, frame.tz));

    images.clear();
    depths.clear();
    for(int i=0; i<6; i++)
    {
        //////////////////////////////color//////////////////////////////
        glClearColor(1.0f, 1.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // bind textures on corresponding texture units
        glActiveTexture(GL_TEXTURE0);//default actived
        glBindTexture(GL_TEXTURE_2D, color_tex_id);
        //colorShader.setInt("texture1", 0);
        colorShader.use();
        colorShader.setMat4("projection", projection);
        colorShader.setMat4("view", views[i]);
        colorShader.setMat4("model", model);

        glBindVertexArray(VAO);
        glDrawArrays(GL_TRIANGLES, 0, vertices.size()/6);
        glBindVertexArray(0);
        glBindTexture(GL_TEXTURE_2D, 0);
        glfwSwapBuffers(window);

        cv::Mat pixel_color(SCR_HEIGHT, SCR_WIDTH, CV_8UC3, cv::Scalar(0,0,0));
        glPixelStorei(GL_PACK_ALIGNMENT, (pixel_color.step & 3) ? 1 : 4);
        glPixelStorei(GL_PACK_ROW_LENGTH, pixel_color.step/pixel_color.elemSize());
        glReadPixels(0, 0, pixel_color.cols, pixel_color.rows, GL_BGR, GL_UNSIGNED_BYTE, pixel_color.data);
        cv::flip(pixel_color, pixel_color, 0);
        images.push_back(pixel_color);

        //////////////////////////////depth//////////////////////////////
        glClearColor(1.0f, 1.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        // bind textures on corresponding texture units
        glActiveTexture(GL_TEXTURE0);//default actived
        glBindTexture(GL_TEXTURE_2D, depth_tex_id);
        //depthShader.setInt("texture1", 0);
        colorShader.setMat4("projection", projection);
        colorShader.setMat4("view", views[i]);
        colorShader.setMat4("model", model);

        glBindVertexArray(VAO);
        glDrawArrays(GL_TRIANGLES, 0, vertices.size()/6);
        glBindVertexArray(0);
        glBindTexture(GL_TEXTURE_2D, 0);
        glfwSwapBuffers(window);
        cv::Mat pixel_depth(SCR_HEIGHT, SCR_WIDTH, CV_32FC1, cv::Scalar(0));
        glReadPixels(0, 0, pixel_depth.cols, pixel_depth.rows, GL_ALPHA, GL_FLOAT, pixel_depth.data);
        cv::flip(pixel_depth, pixel_depth, 0);
        depths.push_back(pixel_depth);

        //////////////////////////////debug//////////////////////////////
        #ifdef WARP_TO_PANOS_DEBUG
        {
            cv::Mat img1, img2;
            resize(pixel_color, img1, Size(800, 800));
            resize(pixel_depth, img2, Size(800, 800));
            //imshow("pixel_color", img1);
            //imshow("pixel_depth", img2);

            //cout<<pixel_depth(Rect(0, 500, 100, 50))<<endl;
            //cv::waitKey();
        }
        #endif
    }//end of rendering
    glfwTerminate();

    #ifdef WARP_TO_PANOS_DEBUG
        cout<<"GenerateSkybox finish"<<endl;
    #endif
    return 0;
}

static void GenerateCubemapVertices(vector<float>& vertices, const Mat& image, const Mat& depth)
{
    float rows = depth.rows-1;
    float cols = depth.cols-1;

    //imshow("image", image);
    //imshow("depth", depth);
    //cv::waitKey();//for debug

    for(float row = 0; row<rows+1; ++row)
    {
        for(float col = 0; col<cols+1; ++col)
        {

            float d = depth.at<float>(row, col);
            if(d<1.0f)// && d<1.0f)
            {
                float x = (col-cols/2)/cols;
                float y = (row-rows/2)/rows;
                float z = 0.5;
                vertices.push_back( x); vertices.push_back( y); vertices.push_back( z);
                vertices.push_back( (float)image.at<Vec3b>(row, col)[2]/255.0);
                vertices.push_back( (float)image.at<Vec3b>(row, col)[1]/255.0);
                vertices.push_back( (float)image.at<Vec3b>(row, col)[0]/255.0);
                vertices.push_back( d);
            }

        }
    }

    #ifdef WARP_TO_PANOS_DEBUG
    {
        double minD, maxD;
        minMaxLoc(depth, &minD, &maxD);
        cout<<"cubemap depth, min, max:"<<minD<<", "<<maxD<<endl;
        cout<<"GenerateCubemapVertices vertices.size(): "<<vertices.size()<<endl;
    }
    #endif
}
static int GeneratePanorama(const vector<Mat>& images, const vector<Mat>& depths, Mat& pano_image, Mat& pano_depth)
{
    const unsigned int SCR_WIDTH = PANO_H*2;
    const unsigned int SCR_HEIGHT = PANO_H;
    //Initialization of glfw and glad
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    //glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    //glfwSetWindowSizeLimits(window, 640, 480, GLFW_DONT_CARE, GLFW_DONT_CARE);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    // glad: load all OpenGL function pointers
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////

    GLuint VAOs[6];
    glGenVertexArrays(6, VAOs);
    vector<float> vertices;
    vector<int> vertex_sizes;
    vertex_sizes.clear();
    for(int i=0; i<6; i++)
    {
        glBindVertexArray(VAOs[i]);
        Mat img = images[i];
        Mat dps = depths[i];
        vertices.clear();
        GenerateCubemapVertices(vertices, img, dps);
        vertex_sizes.push_back(vertices.size());
        //cout<<"vertices "<<i<<" sizes: "<<vertices.size()<<endl;
        //Vertex buffer object
        GLuint VBO;
        glGenBuffers(1, &VBO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(float), &vertices[0], GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7*sizeof(float), (void*)0);

        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7*sizeof(float), (void*)(3*sizeof(float)));
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        glBindVertexArray(0);
    }

    //set shader
    Shader colorShader("colorPano.vertexshader", "colorPano.fragmentshader");
    colorShader.use();
    //cout<<"shader set up finished"<<endl;
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //rendering
    glEnable(GL_DEPTH_TEST);
    //glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClearColor(1.0f, 0.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glm::mat4 view = glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    vector<glm::mat4> models(6, glm::mat4(1.0f));
    models[1] = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    models[2] = glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    models[3] = glm::rotate(glm::mat4(1.0f), glm::radians(270.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    models[4] = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    models[5] = glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    for(int i=0; i<6; i++)
    {
        colorShader.setMat4("view", view);
        colorShader.setMat4("model", models[i]);
        glBindVertexArray(VAOs[i]);
        glDrawArrays(GL_POINTS, 0, vertex_sizes[i]/7);
        glBindVertexArray(0);
    }
    glfwSwapBuffers(window);
    //cout<<"rendering finished"<<endl;
    //read buffers from default rendering buffer
    cv::Mat pixels(SCR_HEIGHT, SCR_WIDTH, CV_8UC3, cv::Scalar(0,0,0));
    glPixelStorei(GL_PACK_ALIGNMENT, (pixels.step & 3) ? 1 : 4);
    glPixelStorei(GL_PACK_ROW_LENGTH, pixels.step/pixels.elemSize());
    glReadPixels(0, 0, pixels.cols, pixels.rows, GL_BGR, GL_UNSIGNED_BYTE, pixels.data);
    cv::flip(pixels, pano_image, 0);
    //cv::Mat img_show;
    //resize(pixels, img_show, Size(1600, 800));
    //imshow("pixels", img_show);

    cv::Mat pixeld(SCR_HEIGHT, SCR_WIDTH, CV_32FC1, cv::Scalar(0.0));
    glReadPixels(0, 0, pixeld.cols, pixeld.rows, GL_ALPHA, GL_FLOAT, pixeld.data);
    cv::flip(pixeld, pano_depth, 0);
    //resize(pixeld, img_show, Size(1600, 800));
    //imshow("depth", img_show);

    glfwTerminate();//release all resources allocated by opengl and glfw

    //cv::waitKey();//for debug

    return 0;
}

static void GeneratePanoramaEdge(Frame& frame)
{
    uint minw,minh,maxw,maxh;
    minw = minh = PANO_W;
    maxw = maxh = 0;

    int rows = frame.pano_depth.rows;
    int cols = frame.pano_depth.cols;

    Mat pano_depth = frame.pano_depth;
    for(int h=0; h<rows; ++h)
    {
        for(int w=0; w<cols; ++w)
        {
            if(pano_depth.at<float>(h, w)<1.0f)
            {
                minw = minw < w ? minw : w;
                minh = minh < h ? minh : h;
                maxw = maxw > w ? maxw : w;
                maxh = maxh > h ? maxh : h;
            }
        }
    }

    frame.minw = minw;
    frame.minh = minh;
    frame.maxw = maxw;
    frame.maxh = maxh;

    /*
    Mat pano_i, pano_d;
    resize(frame.pano_image, pano_i, Size(1000, 500));
    resize(frame.pano_depth, pano_d, Size(1000, 500));
    imshow("pano_i", pano_i);
    imshow("pano_d", pano_d*10);
     */
    //waitKey();
    Mat pano_error = Mat(rows, cols, CV_32FC1, Scalar(0.0f));
    frame.pano_image = frame.pano_image(cv::Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();
    frame.pano_depth = frame.pano_depth(cv::Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();
    frame.pano_error = pano_error(cv::Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();
}


int warpToPano(Frame& kframe, Intrinsics& intrinsics)
{
    vector<Mat> images, depths;

    #ifdef WARP_TO_PANOS_DEBUG
        cout<<"warpToPano"<<endl;
    #endif

#ifdef WARP_TO_PANOS_DEBUG
    cout<<"***************GenerateSkybox(kframe, intrinsics, images, depths);"<<endl;
    double Time;
    Time = (double)cvGetTickCount();
#endif

    GenerateSkybox(kframe, intrinsics, images, depths);

#ifdef WARP_TO_PANOS_DEBUG
    Time = (double)cvGetTickCount() - Time;
    printf( "run time = %gs\n", Time /(cvGetTickFrequency()*1000000) );//秒
    cout<<"***************GenerateSkybox(kframe, intrinsics, images, depths);"<<endl;
#endif
    /*for(int i=0; i<6; ++i)
    {
        imshow("image", images[i]);
        imshow("depth", depths[i]);
        waitKey(0);
    }*/
#ifdef WARP_TO_PANOS_DEBUG
    cout<<"***************GeneratePanorama(images, depths, kframe.pano_image, kframe.pano_depth);"<<endl;
    Time = (double)cvGetTickCount();
#endif
    GeneratePanorama(images, depths, kframe.pano_image, kframe.pano_depth);
#ifdef WARP_TO_PANOS_DEBUG
    Time = (double)cvGetTickCount() - Time;
    printf( "run time = %gs\n", Time /(cvGetTickFrequency()*1000000) );//秒
    cout<<"***************GeneratePanorama(images, depths, kframe.pano_image, kframe.pano_depth);"<<endl;
#endif
    /*
    imshow("pano_img", frame.pano_image);
    imshow("pano_depth", frame.pano_depth);
    waitKey();*/
#ifdef WARP_TO_PANOS_DEBUG
    cout<<"***************GeneratePanoramaEdge(kframe);"<<endl;
    Time = (double)cvGetTickCount();
#endif

    GeneratePanoramaEdge(kframe);

#ifdef WARP_TO_PANOS_DEBUG
    Time = (double)cvGetTickCount() - Time;
    printf( "run time = %gs\n", Time /(cvGetTickFrequency()*1000000) );//秒
    cout<<"***************GeneratePanoramaEdge(kframe);"<<endl;
#endif
}

void warpToPanos(vector<Frame>& kframes, Intrinsics& intrinsics)
{
    for(int i=0; i<kframes.size(); ++i)
    {
        warpToPano(kframes[i], intrinsics);
        //imshow("pano_img", kframes[i].pano_image);
        //imshow("pano_depth", kframes[i].pano_depth);
        //waitKey();
    }

}