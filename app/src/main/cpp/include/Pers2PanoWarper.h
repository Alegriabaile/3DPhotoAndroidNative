//
// Created by ale on 19-5-13.
//

#ifndef I3D_PERS2PANOWARPER_H
#define I3D_PERS2PANOWARPER_H

//perspective image to panorama image warper.
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>

#include "i3d.h"
#include "Frame.h"
#include "learnopengl/shader.h"

class Pers2PanoWarper
{
private:
    GLuint WIDTH0, WIDTH1;// = PANO_H;
    GLuint HEIGHT0, HEIGHT1;// = PANO_H;

    GLuint frameBufferObject0, frameBufferObject1;
    GLuint colorTextureBuffer0, colorTextureBuffer1;
    GLuint renderBufferObject0, renderBufferObject1;

    Shader skyboxShader, panoShader;

    //////////////////////////////////////////////////////////////////////////////////////
    //初始化opengl的context，使所有有关opengl的操作都在当前glfw设置好的context下工作
    int InitGL();
    //清除当前opengl的context及可能的GPU资源占用
    int CleanUpGL();

    /////////////////////////////////////////////////////////////////////////////////////////
    //将输入的深度图，转换为三角形顶点集与其对应的uv坐标
    int GenerateTrianglesForSkybox(const cv::Mat &depth, const i3d::Intrinsics &intrinsics,
            std::vector<float> &vertices);

    //根据GenerateTrianglesForSkybox得到的顶点属性集，渲染得到天空盒的六个面的颜色和深度值(半径r，点到原点的距离)
    int GenerateSkybox( const i3d::Frame &frame,
                        const std::vector<float> &vertices, std::vector<cv::Mat> &images,
                        std::vector<cv::Mat> &depths);

    //对天空盒中的离散点进行采样
    int GenerateVerticesForPanorama(const std::vector<cv::Mat> &images, const std::vector<cv::Mat> &depths,
                                    std::vector<std::vector<float>> &vectorOfVertices);

    //将所有采样点根据球坐标公式映射至全景图上
    int GeneratePanorama(std::vector<std::vector<float>> &vectorOfVertices,
                         cv::Mat &pano_image, cv::Mat &pano_depth);

    //将有内容的部分用边界盒包起来，丢弃无内容的部分，存储边界坐标值，节省内存与stitch时间
    int GenerateAABB(i3d::Frame& frame);//Axis-Aligned Bounding Boxes

    //调用以上操作
    int warpToPano(i3d::Frame& kframe, i3d::Intrinsics& intrinsics);


public:
    Pers2PanoWarper(std::vector<i3d::Frame>& kframes, i3d::Intrinsics& intrinsics);
    ~Pers2PanoWarper();
};

#endif //I3D_PERS2PANOWARPER_H
