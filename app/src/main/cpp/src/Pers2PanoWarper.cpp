//
// Created by ale on 19-5-13.
//

#include "Pers2PanoWarper.h"

using namespace std;
using namespace cv;
using namespace i3d;

Pers2PanoWarper::Pers2PanoWarper(std::vector<i3d::Frame> &kframes,
        i3d::Intrinsics &intrinsics)
        : WIDTH0(PANO_H), HEIGHT0(PANO_H)
        , WIDTH1(PANO_W), HEIGHT1(PANO_H)
{
    //申请好opengl所需要的GPU资源，比如glfwwindow/context，
    //framebufferobjects(color texture attachment 和depth/stencil test render buffer object),
    //shaders
    InitGL();

    //在当前glfw建立的OpenGL context下，对每帧进行三角网格化天空盒与采样至全景图的操作
    for(int i=0; i<kframes.size(); ++i)
        warpToPano(kframes[i], intrinsics);

    //清除OpenGL申请的GPU资源与glfw 申请的window/context
    CleanUpGL();
}

Pers2PanoWarper::~Pers2PanoWarper()
{
    //不要再次释放OpenGL/glfw资源，否则会出错---double free or corruption
    //CleanUpGL();
}

int Pers2PanoWarper::InitGL()
{
    // glfw: initialize and configure
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    //glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    // glfw window creation
    GLFWwindow* window = glfwCreateWindow(100, 100, "LearnOpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    //glfwSetFramebufferSizeCallback(window, [](GLFWwindow* window, int width, int height){glViewport(0, 0, width, height);});
    //glfwSetCursorPosCallback(window, [](GLFWwindow* window, double xpos, double ypos){});
    //glfwSetScrollCallback(window, [](GLFWwindow* window, double xoffset, double yoffset){});

    // glad: load all OpenGL function pointers
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // framebuffer configuration
    glGenFramebuffers(1, &frameBufferObject0);
    glBindFramebuffer(GL_FRAMEBUFFER, frameBufferObject0);
    //cout<<glGetError()<<endl;
    // create a color attachment texture
    glGenTextures(1, &colorTextureBuffer0);
    glBindTexture(GL_TEXTURE_2D, colorTextureBuffer0);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, WIDTH0, HEIGHT0, 0, GL_RGBA,  GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorTextureBuffer0, 0);
    // create a renderbuffer object for depth and stencil attachment (we won't be sampling these)
    //更接近opengl原生数据结构，当不需要读取(比如glReadPixels)这些数据时效率非常快，一般用来depth test 和stencil test
    glGenRenderbuffers(1, &renderBufferObject0);
    glBindRenderbuffer(GL_RENDERBUFFER, renderBufferObject0);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH32F_STENCIL8, WIDTH0, HEIGHT0); // use a single renderbuffer object for both a depth AND stencil buffer.
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, renderBufferObject0); // now actually attach it
    // now that we actually created the framebuffer and added all attachments we want to check if it is actually complete now
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << endl;
    //if (glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT)
    //    cout << "ERROR::FRAMEBUFFER::GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT"<<endl;
    //if (glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT)
    //    cout << "ERROR::FRAMEBUFFER::GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT"<<endl;
    //if (glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER)
    //    cout << "ERROR::FRAMEBUFFER::GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER"<<endl;
    //if (glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER)
    //    cout << "ERROR::FRAMEBUFFER::GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER"<<endl;
    //if (glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_UNSUPPORTED)
    //    cout << "ERROR::FRAMEBUFFER::GL_FRAMEBUFFER_UNSUPPORTED"<<endl;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // framebuffer configuration
    glGenFramebuffers(1, &frameBufferObject1);
    glBindFramebuffer(GL_FRAMEBUFFER, frameBufferObject1);
    // create a color attachment texture
    glGenTextures(1, &colorTextureBuffer1);
    glBindTexture(GL_TEXTURE_2D, colorTextureBuffer1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, WIDTH1, HEIGHT1, 0, GL_RGBA,  GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorTextureBuffer1, 0);
    // create a renderbuffer object for depth and stencil attachment (we won't be sampling these)
    //更接近opengl原生数据结构，当不需要读取(比如glReadPixels)这些数据时效率非常快，一般用来depth test 和stencil test
    glGenRenderbuffers(1, &renderBufferObject1);
    glBindRenderbuffer(GL_RENDERBUFFER, renderBufferObject1);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH32F_STENCIL8, WIDTH1, HEIGHT1); // use a single renderbuffer object for both a depth AND stencil buffer.
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, renderBufferObject1); // now actually attach it
    // now that we actually created the framebuffer and added all attachments we want to check if it is actually complete now
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << endl;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    //shaders
    //....to do
    string vertexCode0("#version 330 core\n"
                       "\n"
                       "layout (location = 0) in vec3 attributePos;\n"
                       "layout (location = 1) in vec3 attributeUV;\n"
                       "\n"
                       "uniform mat4 model;\n"
                       "uniform mat4 view;\n"
                       "uniform mat4 projection;\n"
                       "\n"
                       "out vec2 UV;\n"
                       "out float Radius;\n"
                       "\n"
                       "void main()\n"
                       "{\n"
                       "    vec4 position = view * model * vec4(attributePos, 1.0f);\n"
                       "\tgl_Position = projection * position;\n"
                       "\n"
                       "\tRadius = length(position);///300.0f;//如何保证精度？？？？？\n"
                       "\tUV = vec2(attributeUV.xy);\n"
                       "}");
    string fragmentCode0("#version 330 core\n"
                         "\n"
                         "in vec2 UV;\n"
                         "in float Radius;\n"
                         "\n"
                         "uniform sampler2D texture1;\n"
                         "uniform int depthFlag;\n"
                         "\n"
                         "out vec4 FragColor;\n"
                         "\n"
                         "void main()\n"
                         "{\n"
                         "    if(depthFlag != 0)\n"
                         "    {\n"
                         "        FragColor = vec4(Radius, Radius, Radius, 1.0f);\n"
                         "    }\n"
                         "    else\n"
                         "    {\n"
                         "        FragColor = texture(texture1, UV.xy);\n"
                         "    }\n"
                         "}");
    skyboxShader.init(vertexCode0, fragmentCode0);


    string vertexCode1("#version 330 core\n"
                       "\n"
                       "layout (location = 0) in vec3 position3F;\n"
                       "layout (location = 1) in vec4 color4F;\n"
                       "\n"
                       "out vec4 PointColor;\n"
                       "\n"
                       "uniform int depthFlag;\n"
                       "uniform mat4 model;\n"
                       "uniform mat4 view;\n"
                       "\n"
                       "void main()\n"
                       "{\n"
                       "    const float PI = 3.1415926535897932384626433832795;\n"
                       "    vec4 temp = view * model * vec4(position3F, 1.0f);\n"
                       "    float y = temp.x; float z = temp.y; float x = temp.z;\n"
                       "    float r = length(vec3(x,y,z));//只用于计算theta角度，真正的半径在color4F.w\n"
                       "\n"
                       "    float theta_ = acos(z/r);\n"
                       "    float fa_;\n"
                       "        if(x>0 && y>=0)\n"
                       "            fa_ = atan(y/x);\n"
                       "        else if(x>0 && y<0)\n"
                       "            fa_ = 2*PI+atan(y/x);\n"
                       "        else if(x<=0 && y>=0)\n"
                       "            fa_ = PI + atan(y/x);\n"
                       "        else\n"
                       "            fa_ = PI + atan(y/x);\n"
                       "\n"
                       "    gl_Position = vec4(1.0-fa_/PI, (0.5-theta_/PI)*2, 0.0, 1.0);\n"
                       "    if(depthFlag == 0)\n"
                       "    {\n"
                       "        PointColor = vec4(color4F.xyz, 1.0f);\n"
                       "    }\n"
                       "    else\n"
                       "    {\n"
                       "        PointColor = color4F.wwww;\n"
                       "    }\n"
                       "\t\n"
                       "}");
    string fragmentCode1("#version 330 core\n"
                         "\n"
                         "out vec4 FragColor;\n"
                         "\n"
                         "in vec4 PointColor;\n"
                         "\n"
                         "void main()\n"
                         "{\n"
                         "\tFragColor = PointColor;\n"
                         "}");
    panoShader.init(vertexCode1, fragmentCode1);

    return 0;
}

int Pers2PanoWarper::CleanUpGL()
{
    glDeleteRenderbuffers(1, &renderBufferObject0);
    glDeleteRenderbuffers(1, &renderBufferObject1);
    glDeleteTextures(1, &colorTextureBuffer0);
    glDeleteTextures(1, &colorTextureBuffer1);
    glDeleteFramebuffers(1, &frameBufferObject0);
    glDeleteFramebuffers(1, &frameBufferObject1);

    //shaders
    //....to do
    skyboxShader.clear();
    panoShader.clear();

    glfwTerminate();
    return 0;
}

//深度图生成致密三角网格，相邻像素符合条件即可相连
int Pers2PanoWarper::GenerateTrianglesForSkybox( const cv::Mat &depth,
        const i3d::Intrinsics &intrinsics,std::vector<float> &vertices)
{
    const float MAX_DIFF = 0.05;//相邻像素深度值可相差的最大比率

    const float rows = depth.rows;
    const float cols = depth.cols;

    float cx, cy, f;
    cx = cols/2; cy = rows/2;
    f = intrinsics.f*rows/2/intrinsics.cy;

    vertices.clear();
    vertices.reserve((rows-1)*(cols-1)*6*5);

    float x, y, z, u, v;//3d coordinates and uv texture coordinates
    for (float h = 0; h < rows-1; ++h)
    {
        for (float w = 0; w < cols-1; ++w)
        {
            //参考了 https://github.com/simonfuhrmann/mve/blob/master/libs/mve/depthmap.cc#L211
            //0 1
            //2 3
            float d[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            d[0] = depth.at<float>(h, w);//depth of the current point
            d[1] = depth.at<float>((int)h, (int) w+1);//depth of the right
            d[2] = depth.at<float>((int)h+1, (int) w);//depth of the bottom
            d[3] = depth.at<float>((int)h+1, (int) w+1);//depth of the right bottom
            //At least three valid depth values are required.
            int positive_d_n = int(d[0]>0) + int(d[1]>0) + int(d[2]>0) + int(d[3]>0);
            if(positive_d_n<3) continue;
            //3,1,2;  0,2,3;  3,1,0;  0,2,1
            int tri[4] = {0, 0, 0, 0};
            for(int i=0; i<4; ++i)//if positive_d_n==3,then
            {
                if(!(d[i]>0))
                {
                    tri[i] = 1;
                    break;
                }
            }
            if(positive_d_n == 4)
            {
                if(fabs(d[0]-d[3]) > fabs(d[1]-d[2]))
                    tri[0] = tri[3] = 1;
                else
                    tri[1] = tri[2] = 1;
            }

            int tris[4][3] = {
                    { 3,1,2}, { 0,2,3 }, { 3,1,0}, { 0,2,1}
            };
            for(int i=0; i<4; ++i)
            {
                if(tri[i] == 0)
                    continue;

                int in0 = tris[i][0]; int in1 = tris[i][1]; int in2 = tris[i][2];
                if(fabs(d[in0] - d[in1]) < MAX_DIFF*max(d[in0], d[in1])
                && fabs(d[in1] - d[in2]) < MAX_DIFF*max(d[in1], d[in2])
                && fabs(d[in2] - d[in0]) < MAX_DIFF*max(d[in2], d[in0]) )
                {
                    for(int j=0; j<3; ++j)
                    {
                        z = d[tris[i][j]];
                        x = z * (w + tris[i][j]%2 - cx) / f;
                        y = z * (h + tris[i][j]/2 - cy) / f;
                        u = (w + tris[i][j]%2) / (float) (cols - 1);
                        v = 1 - (h + tris[i][j]/2) / (float) (rows - 1);
                        vertices.push_back(x); vertices.push_back(y); vertices.push_back(z);
                        vertices.push_back(u); vertices.push_back(v);
                    }
                }
            }//end of current(h,w) position's triangle generation

        }
    }//end of whole loop

    //三角形数目
    return vertices.size()/(5*3);
}

int Pers2PanoWarper::GenerateSkybox( const i3d::Frame &frame,
    const std::vector<float> &vertices, std::vector<cv::Mat> &images,
    std::vector<cv::Mat> &depths)
{
    GLuint VAO;
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);
    GLuint VBO;
    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(float), &vertices[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5*sizeof(float), (void*)0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5*sizeof(float), (void*)(3*sizeof(float)));
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    GLuint color_tex_id;
    cv::Mat color_tex_ = frame.image.clone();
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
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, color_tex.cols, color_tex.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, color_tex.data);
    glGenerateMipmap(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);

    // load and create a texture
    GLuint depth_tex_id;
    cv::Mat depth_tex = color_tex;
    glGenTextures(1, &depth_tex_id);
    glBindTexture(GL_TEXTURE_2D, depth_tex_id);
    // set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    // set texture filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);//Nearest for depth(require sharp edges)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, depth_tex.cols, depth_tex.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, depth_tex.data);
    glGenerateMipmap(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);

    //znear不能太小，否则将无法渲染稍远处的物体，因为znear与深度缓冲的精度有关
    glm::mat4 projection = glm::perspective(glm::radians(90.0f), (float)WIDTH0 / (float)HEIGHT0, 0.1f, 250.0f);

    //opencv坐标至opnegl坐标的转换，front, right, back, left, top, bottom/down
    vector<glm::mat4> views(6, glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f)));
    views[1] = glm::rotate(views[0], glm::radians(-90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    views[2] = glm::rotate(views[0], glm::radians(-180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    views[3] = glm::rotate(views[0], glm::radians(-270.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    views[4] = glm::rotate(views[0], glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    views[5] = glm::rotate(views[0], glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    //从frame.rxryrxtxtytz中恢复此图片的外参，作为model矩阵参数
    glm::mat4 model = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first
    model = glm::rotate(model, (float)frame.rx, glm::vec3(1.0f, 0.0f, 0.0f));
    model = glm::rotate(model, (float)frame.ry, glm::vec3(0.0f, 1.0f, 0.0f));
    model = glm::rotate(model, (float)frame.rz, glm::vec3(0.0f, 0.0f, 1.0f));
    model = glm::translate(model, glm::vec3(frame.tx, frame.ty, frame.tz));

    glEnable(GL_DEPTH_TEST);
    images.clear();
    depths.clear();
    //glViewport(0, 0, WIDTH0, HEIGHT0);
    glBindFramebuffer(GL_FRAMEBUFFER, frameBufferObject0);
    glViewport(0, 0, WIDTH0, HEIGHT0);
    for(int i=0; i<6; i++)
    {
        //////////////////////////////color//////////////////////////////
        glClearColor(1.0f, 1.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // bind textures on corresponding texture units
        glActiveTexture(GL_TEXTURE0);//default actived
        glBindTexture(GL_TEXTURE_2D, color_tex_id);
        //colorShader.setInt("texture1", 0);
        skyboxShader.use();
        skyboxShader.setMat4("projection", projection);
        skyboxShader.setMat4("view", views[i]);
        skyboxShader.setMat4("model", model);
        skyboxShader.setBool("depthFlag", false);

        glBindVertexArray(VAO);
        glDrawArrays(GL_TRIANGLES, 0, vertices.size() / 5);
        glBindVertexArray(0);
        glBindTexture(GL_TEXTURE_2D, 0);

        cv::Mat pixel_color(HEIGHT0, WIDTH0, CV_8UC4, cv::Scalar(0, 0, 0));
        glReadPixels(0, 0, pixel_color.cols, pixel_color.rows, GL_BGRA, GL_UNSIGNED_BYTE, pixel_color.data);
        cv::flip(pixel_color, pixel_color, 0);
        cv::cvtColor(pixel_color, pixel_color, COLOR_BGRA2BGR);
        images.push_back(pixel_color);

        //////////////////////////////depth//////////////////////////////
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        // bind textures on corresponding texture units
        glActiveTexture(GL_TEXTURE0);//default actived
        glBindTexture(GL_TEXTURE_2D, depth_tex_id);
        //depthShader.setInt("texture1", 0);
        skyboxShader.use();
        skyboxShader.setMat4("projection", projection);
        skyboxShader.setMat4("view", views[i]);
        skyboxShader.setMat4("model", model);
        skyboxShader.setBool("depthFlag", true);

        glBindVertexArray(VAO);
        glDrawArrays(GL_TRIANGLES, 0, vertices.size() / 5);
        glBindVertexArray(0);
        glBindTexture(GL_TEXTURE_2D, 0);
        cv::Mat pixel_depth(HEIGHT0, WIDTH0, CV_32FC1, cv::Scalar(0));
        glReadPixels(0, 0, pixel_depth.cols, pixel_depth.rows, GL_RED, GL_FLOAT, pixel_depth.data);
        cv::flip(pixel_depth, pixel_depth, 0);
        depths.push_back(pixel_depth);
    }

    return 0;
}

//太慢了。。。
int Pers2PanoWarper::GenerateVerticesForPanorama( const std::vector<cv::Mat> &images, const std::vector<cv::Mat> &depths,
        std::vector<std::vector<float>> &vectorOfVertices)
{
    for(int i=0; i<6; ++i)
    {
        float rows = depths[i].rows-1;
        float cols = depths[i].cols-1;
        vector<float> vertices;
        vertices.reserve((rows+1)*(cols+1)*7);

        for(int row = 0; row<rows+1; ++row)
        {
            for(int col = 0; col<cols+1; ++col)
            {

                float d = depths[i].at<float>(row, col);
                if(d>0.0f)// && d<1.0f)
                {
                    float x = (float(col)-cols/2)/cols;
                    float y = (float(row)-rows/2)/rows;
                    float z = 0.5;
                    vertices.push_back( x); vertices.push_back( y); vertices.push_back( z);
                    vertices.push_back( (float)images[i].at<Vec3b>(row, col)[2]/255.0);
                    vertices.push_back( (float)images[i].at<Vec3b>(row, col)[1]/255.0);
                    vertices.push_back( (float)images[i].at<Vec3b>(row, col)[0]/255.0);
                    vertices.push_back( d);
                }

            }
        }

        vectorOfVertices.push_back(vertices);
    }

    return 0;
}

int Pers2PanoWarper::GeneratePanorama( std::vector<std::vector<float>> &vectorOfVertices,
        cv::Mat &pano_image, cv::Mat &pano_depth)
{
    GLuint VAOs[6];
    glGenVertexArrays(6, VAOs);

    for(int i=0; i<6; i++)
    {
        vector<float> & vertices = vectorOfVertices[i];
        glBindVertexArray(VAOs[i]);
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
    glm::mat4 view = glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    vector<glm::mat4> models(6, glm::mat4(1.0f));
    models[1] = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    models[2] = glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    models[3] = glm::rotate(glm::mat4(1.0f), glm::radians(270.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    models[4] = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    models[5] = glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));

    glEnable(GL_DEPTH_TEST);
    //glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glBindFramebuffer(GL_FRAMEBUFFER, frameBufferObject1);
    glViewport(0, 0, WIDTH1, HEIGHT1);

    glClearColor(1.0f, 0.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    panoShader.use();
    for(int i=0; i<6; i++)
    {
        panoShader.setBool("depthFlag", false);
        panoShader.setMat4("view", view);
        panoShader.setMat4("model", models[i]);
        glBindVertexArray(VAOs[i]);
        glDrawArrays(GL_POINTS, 0, vectorOfVertices[i].size()/7);
        glBindVertexArray(0);
    }
    if(pano_image.empty() || pano_depth.empty())
    {
        pano_image = cv::Mat::zeros(PANO_H, PANO_W, CV_8UC3);
        pano_depth = cv::Mat::zeros(PANO_H, PANO_W, CV_32FC1);
    }
    cv::Mat pano_image_temp = cv::Mat::zeros(PANO_H, PANO_W, CV_8UC4);
    glReadPixels(0, 0, pano_image_temp.cols, pano_image_temp.rows, GL_BGRA, GL_UNSIGNED_BYTE, pano_image_temp.data);
    cv::flip(pano_image_temp, pano_image_temp, 0);
    cv::cvtColor(pano_image_temp, pano_image, COLOR_BGRA2BGR);

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    panoShader.use();
    for(int i=0; i<6; i++)
    {
        panoShader.setBool("depthFlag", true);
        panoShader.setMat4("view", view);
        panoShader.setMat4("model", models[i]);
        glBindVertexArray(VAOs[i]);
        glDrawArrays(GL_POINTS, 0, vectorOfVertices[i].size()/7);
        glBindVertexArray(0);
    }
    pano_depth = cv::Mat::zeros(PANO_H, PANO_W, CV_32FC1);
    glReadPixels(0, 0, pano_depth.cols, pano_depth.rows, GL_RED, GL_FLOAT, pano_depth.data);
    cv::flip(pano_depth, pano_depth, 0);

    return 0;
}

int Pers2PanoWarper::GenerateAABB(i3d::Frame &frame)
{
    uint minw, minh, maxw, maxh;
    minw = minh = PANO_W;
    maxw = maxh = 0;

    int rows = frame.pano_depth.rows;
    int cols = frame.pano_depth.cols;

    Mat pano_depth = frame.pano_depth;
    for(int h=0; h<rows; ++h)
    {
        for(int w=0; w<cols; ++w)
        {
            //不能简单地减枝。。。
            if(pano_depth.at<float>(h, w)>0.0f)
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

    Mat pano_error = Mat(rows, cols, CV_32FC1, Scalar(0.0f));
    frame.pano_image = frame.pano_image(cv::Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();
    frame.pano_depth = frame.pano_depth(cv::Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();
    frame.pano_error = pano_error(cv::Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();
    return 0;
}

//#define Pers2PanoWarper_debug
int Pers2PanoWarper::warpToPano(i3d::Frame &kframe, i3d::Intrinsics &intrinsics)
{
////////////////////////////////////////////////////////////////////////////
#ifdef Pers2PanoWarper_debug
    cout<<"***************GenerateTrianglesForSkybox"<<endl;
    double Time;
    Time = (double)cvGetTickCount();
#endif

    vector<Mat> images, depths;
    vector<float> vertices;
    GenerateTrianglesForSkybox(kframe.depth, intrinsics, vertices);

#ifdef Pers2PanoWarper_debug
    Time = (double)cvGetTickCount() - Time;
    printf( "run time = %gs\n", Time /(cvGetTickFrequency()*1000000) );//秒
    cout<<"***************GenerateTrianglesForSkybox: "<<vertices.size()<<" vertices"<<endl;
#endif

//////////////////////////////////////////////////////////////////////////////
#ifdef Pers2PanoWarper_debug
    cout<<"***************GenerateSkybox"<<endl;
    Time = (double)cvGetTickCount();
#endif

    GenerateSkybox(kframe, vertices, images, depths);

#ifdef Pers2PanoWarper_debug
    Time = (double)cvGetTickCount() - Time;
    printf( "run time = %gs\n", Time /(cvGetTickFrequency()*1000000) );//秒
    cout<<"***************GenerateSkybox: done!"<<endl;
    for(int i=0; i<6; ++i)
    {
        double minD, maxD;
        minMaxLoc(depths[i], &minD, &maxD);
        cout<<i<<" th depth, min, max:"<<minD<<", "<<maxD<<endl;

        cv::Mat msk = Mat::zeros(kframe.depth.size(), CV_8UC1);
        for(int h=0; h<kframe.depth.rows; ++h)
            for(int w=0; w<kframe.depth.cols; ++w)
                if(kframe.depth.at<float>(h,w) <= 0.01)
                    msk.at<uchar>(h,w) = 255;

        cv::Mat img, dps;
        resize(images[i], img, Size(800, 800));
        resize(depths[i], dps, Size(800, 800));

        minMaxLoc(dps, &minD, &maxD);
        if(maxD > minD)
            dps = (dps-minD)/(maxD-minD);
        //imshow("original img", ori_img);
        //imshow("original dps", kframe.depth);
        imshow("img", img);
        imshow("dps", dps);
        waitKey();
    }
#endif


///////////////////////////////////////////////////////////////////////////////
#ifdef Pers2PanoWarper_debug
    cout<<"***************GenerateVerticesForPanorama"<<endl;
    Time = (double)cvGetTickCount();
#endif

    vector<vector<float>> vectorOfVertices;
    GenerateVerticesForPanorama(images, depths, vectorOfVertices);//太慢了,所用时间是GeneratePanorama的6～7倍，应该使用间接渲染技术进行加速

#ifdef Pers2PanoWarper_debug
    Time = (double)cvGetTickCount() - Time;
    printf( "run time = %gs\n", Time /(cvGetTickFrequency()*1000000) );//秒
    for(int i=0; i<6; ++i)
    {
        cout<<"vectorOfVertices["<<i<<"].size(): "<<vectorOfVertices[i].size()<<endl;
    }
    cout<<"***************GenerateVerticesForPanorama: done!"<<endl;
#endif

////////////////////////////////////////////////////////////////////////////////////
#ifdef Pers2PanoWarper_debug
    cout<<"***************GeneratePanorama"<<endl;
    Time = (double)cvGetTickCount();
#endif

    GeneratePanorama(vectorOfVertices, kframe.pano_image, kframe.pano_depth);

#ifdef Pers2PanoWarper_debug
    Time = (double)cvGetTickCount() - Time;
    printf( "run time = %gs\n", Time /(cvGetTickFrequency()*1000000) );//秒
    cout<<"***************GeneratePanorama: done!"<<endl;
    cv::Mat img, dps;
    resize( kframe.pano_image, img, Size(1600, 800));
    resize( kframe.pano_depth, dps, Size(1600, 800));
    imshow("pano_img", img);
    imshow("pano_dps", dps);
    waitKey();
#endif

//////////////////////////////////////////////////////////////////
#ifdef Pers2PanoWarper_debug
    cout<<"***************GenerateAABB"<<endl;
    Time = (double)cvGetTickCount();
#endif

    GenerateAABB(kframe);
#ifdef Pers2PanoWarper_debug
    Time = (double)cvGetTickCount() - Time;
    printf( "run time = %gs\n", Time /(cvGetTickFrequency()*1000000) );//秒
    cout<<"***************GenerateAABB: done!"<<endl;
    img = kframe.pano_image;
    dps = kframe.pano_depth;
    imshow("pano_img", img);
    imshow("pano_dps", dps);
    waitKey();
#endif

    return 0;
}

