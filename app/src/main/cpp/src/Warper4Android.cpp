//
// Created by ale on 19-7-25.
//

#include "Warper4Android.h"


using namespace std;
using namespace cv;


//可以将此步骤改写至shader中，比如使用instanced rendering，在geometry shader中生成三角网格、uv坐标
int Warper4Android::GenerateTriangles(const cv::Mat &depth,
                                               const i3d::Intrinsics &intrinsics,
                                               std::vector<float> &vertices, cv::Mat& radius)
{
    const float MAX_DIFF = 0.05;//相邻像素深度值可相差的最大比率

    const float rows = depth.rows;
    const float cols = depth.cols;

    float cx, cy, f;
    cx = cols/2; cy = rows/2;
    f = intrinsics.f*rows/2/intrinsics.cy;
//    cx = intrinsics.cx; cy = intrinsics.cy; f = intrinsics.f;



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
                if(fabs(d[in0] - d[in1]) < MAX_DIFF*fmax(d[in0], d[in1])
                   && fabs(d[in1] - d[in2]) < MAX_DIFF*fmax(d[in1], d[in2])
                   && fabs(d[in2] - d[in0]) < MAX_DIFF*fmax(d[in2], d[in0]) )
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

const GLuint WIDTH1 = i3d::PANO_H;
const GLuint HEIGHT1 = i3d::PANO_H;

void Warper4Android::SKYBOX_GEN_VAO_TEXTURE_MVP(const i3d::Frame &frame, const std::vector<float> &vertices)
{
    ///////////////////////////////////////////VAO///////////////////////////////////////
    // vertex array object

    if(!glIsVertexArray(skyboxVAO))
       glGenVertexArrays(1, &skyboxVAO);
    glBindVertexArray(skyboxVAO);
    while((err = glGetError()) != GL_NO_ERROR)
        LOGE(" Warper4Android::SKYBOX_GEN_VAO_TEXTURE_MVP() : after GenVAO err == %d", err);


    if(glIsBuffer(skyboxVBO))
        glDeleteBuffers(1, &skyboxVBO);
    glGenBuffers(1, &skyboxVBO);
//    glGenBuffers(1, &skyboxVBO);
    glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(float), &vertices[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5*sizeof(float), (void*)0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5*sizeof(float), (void*)(3*sizeof(float)));
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    while((err = glGetError()) != GL_NO_ERROR)
        LOGE(" Warper4Android::SKYBOX_GEN_VAO_TEXTURE_MVP() : after GenVBO err == %d", err);


    //////////////////////////////////texture/////////////////////////////////////////////////////
    //texture of color
    cv::Mat color_tex_ = frame.image.clone();
    cv::flip(color_tex_, color_tex_, 0);
    //opencv不自动4byte对齐，但是opengl默认传入cpu的数据为4byte对齐
    cv::Mat color_tex;
    cv::cvtColor(color_tex_, color_tex, COLOR_BGR2RGBA);

    // texture of depth/radius
    cv::Mat depth_tex = frame.depth.clone();
    cv::flip(depth_tex, depth_tex, 0);
//    cv::merge(channels, depth_tex);
    minMaxLoc(depth_tex, &minV, &maxV);
    LOGE("Warper4Android::SKYBOX_GEN_VAO_TEXTURE_MVP:---depth_tex--- minV, maxV: %lf, %lf", minV, maxV);



    glBindTexture(GL_TEXTURE_2D, skyboxColorTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, color_tex.cols, color_tex.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, color_tex.data);

    while((err = glGetError()) != GL_NO_ERROR)
        LOGE(" Warper4Android::SKYBOX_GEN_VAO_TEXTURE_MVP() : after GenColorTexture2D err == %d", err);

//    glGenTextures(1, &skyboxDepthTexture);
    glBindTexture(GL_TEXTURE_2D, skyboxDepthTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, depth_tex.cols, depth_tex.rows, 0, GL_RED, GL_FLOAT, depth_tex.data);
//    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, WIDTH1, HEIGHT1, 0, GL_RED,  GL_FLOAT, NULL);
    while((err = glGetError()) != GL_NO_ERROR)
        LOGE(" Warper4Android::SKYBOX_GEN_VAO_TEXTURE_MVP() : after GenDepthTexture2D::glTexImage2D err == %d", err);


    ////////////////////////////////////////////MVP///////////////////////////////////////////////////////////
    //从frame.rxryrxtxtytz中恢复此图片的外参，作为model矩阵参数
    skyboxModel = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first
    skyboxModel = glm::rotate(skyboxModel, (float)frame.rx, glm::vec3(1.0f, 0.0f, 0.0f));
    skyboxModel = glm::rotate(skyboxModel, (float)frame.ry, glm::vec3(0.0f, 1.0f, 0.0f));
    skyboxModel = glm::rotate(skyboxModel, (float)frame.rz, glm::vec3(0.0f, 0.0f, 1.0f));
    skyboxModel = glm::translate(skyboxModel, glm::vec3(frame.tx, frame.ty, frame.tz));
}


int Warper4Android::GenerateSkybox(const i3d::Frame &frame, const std::vector<float> &vertices)
        //, std::vector<cv::Mat> &images, std::vector<cv::Mat> &depths)
{
//    const GLuint WIDTH1 = warperGlesInitializer.WIDTH1;
//    const GLuint HEIGHT1 = warperGlesInitializer.HEIGHT1;

    SKYBOX_GEN_VAO_TEXTURE_MVP(frame, vertices);

    LOGE("Warper4Android::GenerateSkybox() : before draw this");
    ///////////////////////////////////draw/////////////////////////////////////////////////////////
    glEnable(GL_DEPTH_TEST);
    //////////////////////////////color//////////////////////////////

    while((err = glGetError()) != GL_NO_ERROR)
        LOGE(" Warper4Android::GenerateSkybox() : before draw err == %d", err);

    for(int i=0; i<6; i++)
    {
        glBindFramebuffer(GL_FRAMEBUFFER, warperGlesInitializer.fboSkybox1[i]);
        glDrawBuffers(2, warperGlesInitializer.attachments);
        while((err = glGetError()) != GL_NO_ERROR)
            LOGE(" Warper4Android::GenerateSkybox() : before bind textures[%d], after drawbuffers(2,attachments) err == %d", i, err);

        // bind textures on corresponding texture units
        glActiveTexture(GL_TEXTURE0);//default actived
        glBindTexture(GL_TEXTURE_2D, skyboxColorTexture);

        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, skyboxDepthTexture);
        //warperGlesInitializer.shaderSkybox.setInt("colorTexture", 0);
        while((err = glGetError()) != GL_NO_ERROR)
            LOGE(" Warper4Android::GenerateSkybox() : before draw[%d], after bind depth err == %d", i, err);

        glViewport(0, 0, WIDTH1, HEIGHT1);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        warperGlesInitializer.shaderSkybox1.use();

        warperGlesInitializer.shaderSkybox1.setInt("colorTexture", 0);
        warperGlesInitializer.shaderSkybox1.setInt("depthTexture", 1);

        warperGlesInitializer.shaderSkybox1.setMat4("projection", skyboxProjection);
        warperGlesInitializer.shaderSkybox1.setMat4("view", skyboxViews[i]);
        warperGlesInitializer.shaderSkybox1.setMat4("model", skyboxModel);

        glBindVertexArray(skyboxVAO);
        glDrawArrays(GL_TRIANGLES, 0, vertices.size() / 5);
        glBindVertexArray(0);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, 0);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, 0);

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
    LOGE("Warper4Android::GenerateSkybox() : after draw this");

    return 0;
}


int Warper4Android::GeneratePoints(std::vector<float> &vPoints)
{
    const uint WIDTH = warperGlesInitializer.WIDTH1;
    const uint HEIGHT = warperGlesInitializer.HEIGHT1;
    vPoints.clear();
    vPoints = vector<float>(WIDTH*HEIGHT, 0);
//    for(int i=0; i<WIDTH*HEIGHT; ++i)
//        vPoints[i] = i;
    return 0;
}


int Warper4Android::GeneratePanorama( i3d::Frame &frame)
{
    LOGE(" Warper4Android::GeneratePanorama() : before draw");

    const uint WIDTH2 = warperGlesInitializer.WIDTH2;
    const uint HEIGHT2 = warperGlesInitializer.HEIGHT2;

    static std::vector<float> vPoints;
    if(vPoints.empty())
        GeneratePoints(vPoints);

    glGenVertexArrays(1, &panoVAO);
    glBindVertexArray(panoVAO);

    glGenBuffers(1, &panoVBO);
    glBindBuffer(GL_ARRAY_BUFFER, panoVBO);
    glBufferData(GL_ARRAY_BUFFER, vPoints.size()*sizeof(float), &vPoints[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 1, GL_FLOAT, GL_FALSE, 1*sizeof(float), (void*)0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    while((err = glGetError()) != GL_NO_ERROR)
        LOGE(" Warper4Android::GeneratePanorama() : after GenVAO err == %d", err);


    glEnable(GL_DEPTH_TEST);
    //glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glBindFramebuffer(GL_FRAMEBUFFER, warperGlesInitializer.fboPano1);
    glDrawBuffers(2, warperGlesInitializer.attachments);
    glViewport(0, 0, warperGlesInitializer.WIDTH2, warperGlesInitializer.HEIGHT2);

    glClearColor(0.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    warperGlesInitializer.shaderPano1.use();
    warperGlesInitializer.shaderPano1.setInt("colorTexture", 0);
    warperGlesInitializer.shaderPano1.setInt("depthTexture", 1);
    for(int i=0; i<6; i++)
    {
        warperGlesInitializer.shaderPano1.setInt("WIDTH", WIDTH1);
        warperGlesInitializer.shaderPano1.setMat4("view", panoView);
        warperGlesInitializer.shaderPano1.setMat4("model", panoModels[i]);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, warperGlesInitializer.skyboxColorTex2D[i]);

        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, warperGlesInitializer.skyboxDepthTex2D[i]);

        glBindVertexArray(panoVAO);
        glDrawArrays(GL_POINTS, 0, vPoints.size());
        glBindVertexArray(0);
    }

    glReadBuffer ( GL_COLOR_ATTACHMENT0 );
    cv::Mat pixel_color(HEIGHT2, WIDTH2, CV_8UC4, cv::Scalar(0, 0, 0, 0));
    glReadPixels(0, 0, pixel_color.cols, pixel_color.rows, GL_RGBA, GL_UNSIGNED_BYTE, pixel_color.data);
//    minMaxLoc(pixel_color, &minV, &maxV);
//    LOGE("Warper4Android::GeneratePanorama:---pixel_color--- minV, maxV: %lf, %lf", minV, maxV);
    cv::Mat color_tmp = pixel_color.clone();
    cv::cvtColor(color_tmp, color_tmp, COLOR_RGBA2BGR);
    frame.pano_image = color_tmp;



    glReadBuffer ( GL_COLOR_ATTACHMENT1 );
    cv::Mat pixel_depth(HEIGHT2, WIDTH2, CV_32FC1, cv::Scalar(1.0f, 1.0f, 1.0f, 1.0f));
    glReadPixels(0, 0, pixel_depth.cols, pixel_depth.rows, GL_RED, GL_FLOAT, pixel_depth.data);
    while((err = glGetError()) != GL_NO_ERROR)
        LOGE(" Warper4Android::GeneratePanorama() : after Read depth buffer err == %d", err);
//    minMaxLoc(pixel_depth, &minV, &maxV);
//    LOGE("Warper4Android::GeneratePanorama:---pixel_depth--- minV, maxV: %lf, %lf", minV, maxV);
    frame.pano_depth = pixel_depth;

    //unbind pano framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, 0);


    LOGE(" Warper4Android::GeneratePanorama() : after draw");
    return 0;
}


int Warper4Android::GenerateAABB(i3d::Frame &frame)
{
    LOGE(" Warper4Android::GenerateAABB() : before");
    uint minw, minh, maxw, maxh;
    minw = minh = i3d::PANO_W;
    maxw = maxh = 0;

    int rows = frame.pano_depth.rows;
    int cols = frame.pano_depth.cols;

    Mat pano_depth = frame.pano_depth;
    LOGE(" Warper4Android::GenerateAABB() : frame.pano_depth.rows, cols, empty?: %d, %d, %d", rows, cols, pano_depth.empty());
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
    LOGE(" Warper4Android::GenerateAABB() : minw, minh, maxw, maxh: %d, %d, %d, %d", minw, minh, maxw, maxh);
    frame.minw = minw;
    frame.minh = minh;
    frame.maxw = maxw;
    frame.maxh = maxh;

    Mat pano_error = Mat(rows, cols, CV_32FC1, Scalar(0.0f));
    frame.pano_image = frame.pano_image(cv::Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();
    frame.pano_depth = frame.pano_depth(cv::Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();
    frame.pano_error = pano_error(cv::Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();

    LOGE(" Warper4Android::GenerateAABB() : after");
    return 0;
}

int Warper4Android::WarpToPanorama(i3d::Frame &kframe, i3d::Intrinsics &intrinsics)
{

    vector<float> vertices;
    cv::Mat radius;
    GenerateTriangles(kframe.depth, intrinsics, vertices, radius);

    vector<Mat> images, depths;
    GenerateSkybox(kframe, vertices);

    GeneratePanorama(kframe);

    //裁剪，减少不必要的内存占用
    GenerateAABB(kframe);

    return 0;
}

int Warper4Android::InitializeReusable()
{
    skyboxProjection = glm::perspective(glm::radians(90.0f), (float)WIDTH1 / (float)HEIGHT1, 30.0f, 65555.0f);//0.03m-65m
    //opencv坐标至opnegl坐标的转换，front, right, back, left, top, bottom/down
    skyboxViews[0] = glm::rotate(glm::mat4(float(1.0f)), float(glm::radians(180.0f)), glm::vec3(float(1.0f), float(0.0f), float(0.0f)));
    skyboxViews[1] = glm::rotate(skyboxViews[0], glm::radians(-90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    skyboxViews[2] = glm::rotate(skyboxViews[0], glm::radians(-180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    skyboxViews[3] = glm::rotate(skyboxViews[0], glm::radians(-270.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    skyboxViews[4] = glm::rotate(skyboxViews[0], glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    skyboxViews[5] = glm::rotate(skyboxViews[0], glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));



    panoView = glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    panoModels[0] = glm::mat4(1.0f);
    panoModels[1] = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    panoModels[2] = glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    panoModels[3] = glm::rotate(glm::mat4(1.0f), glm::radians(270.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    panoModels[4] = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    panoModels[5] = glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));



    glGenFramebuffers(1, &inputFramebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, inputFramebuffer);

    glGenTextures(1, &skyboxColorTexture);
    glBindTexture(GL_TEXTURE_2D, skyboxColorTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, WIDTH1, HEIGHT1, 0, GL_RGBA,  GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, skyboxColorTexture, 0);
    while((err = glGetError()) != GL_NO_ERROR)
        LOGE(" Warper4Android::InitializeReusable() : ColorTex2D err == %d", err);

    //>>>>>>>>>>>>>>>>>>>>>>depth texture attachment<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
    glGenTextures(1, &skyboxDepthTexture);
    glBindTexture(GL_TEXTURE_2D, skyboxDepthTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, WIDTH1, HEIGHT1, 0, GL_RED,  GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, skyboxDepthTexture, 0);
    while((err = glGetError()) != GL_NO_ERROR)
        LOGE(" Warper4Android::InitializeReusable() : DepthTex2D err == %d", err);


    glGenRenderbuffers(1, &inputRenderbuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, inputRenderbuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH32F_STENCIL8, WIDTH1, HEIGHT1); // use a single renderbuffer object for both a depth AND stencil buffer.
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER,inputRenderbuffer); // now actually attach it
    // now that we actually created the framebuffer and added all attachments we want to check if it is actually complete now
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        LOGE(" Warper4Android::InitializeReusable(): Framebuffer is not complete!");

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    return 0;
}


Warper4Android::Warper4Android(std::vector<i3d::Frame>& kframes, i3d::Intrinsics& intrinsics):
skyboxVAO(0), skyboxVBO(0)
{
    InitializeReusable();
    for(int i=0; i<kframes.size(); ++i)
        WarpToPanorama(kframes[i], intrinsics);


    //debug................
//    for(int i=0; i<kframes.size(); ++i)
//    {
//        LOGE(" Warper4Android::Warper4Android              %d th imwrite.......", i);
//        string color_name = string("/sdcard/000i3d2/debug_panorama_outputs/color_")+to_string(i)+(".jpg");
//        string depth_name = string("/sdcard/000i3d2/debug_panorama_outputs/depth_")+to_string(i)+(".png");
//        cv::Mat depth_tmp = kframes[i].pano_depth.clone();
//        depth_tmp.convertTo(depth_tmp, CV_16U, 1000);//为了显示效果
//
//        double minV, maxV;
//        minMaxLoc(kframes[i].pano_image, &minV, &maxV);
//        LOGE("Warper4Android::Warper4Android:---kframes[%d].pano_image--- minV, maxV: %lf, %lf", i, minV, maxV);
//
//        minMaxLoc(depth_tmp, &minV, &maxV);
//        LOGE("Warper4Android::Warper4Android:---depth_tmp[%d]--- minV, maxV: %lf, %lf", i, minV, maxV);
//        imwrite(color_name, kframes[i].pano_image);
//        imwrite(depth_name, depth_tmp);
//    }
}

Warper4Android::~Warper4Android() {}
