//
// Created by ale on 19-7-25.
//

#ifndef NATIVE0701_WARPER4ANDROID_H
#define NATIVE0701_WARPER4ANDROID_H

#include "i3d.h"
#include "Frame.h"
#include "WarperShader.h"
#include "WarperEglManager.h"
#include "WarperGlesInitializer.h"

int warpToPano(i3d::Frame& kframe, i3d::Intrinsics& intrinsics);

class Warper4Android {
private:
    WarperGlesInitializer warperGlesInitializer;

    //从RGBD图片生成致密网格,且相邻像素值在相邻时过大时不生成三角形。未来可将此步骤放入shader中进行优化。
    int GenerateTriangles( const cv::Mat &depth,
                           const i3d::Intrinsics &intrinsics,
                           std::vector<float> &vertices,
                           cv::Mat &radius);

    int GenerateSkybox( const i3d::Frame &frame,
                        const std::vector<float> &vertices);

    int GeneratePoints( std::vector<float> &vPoints);

    int GeneratePanorama( i3d::Frame &frame);


    int InitializeReusable();
    void SKYBOX_GEN_VAO_TEXTURE_MVP(const i3d::Frame &frame, const std::vector<float> &vertices);
    //void FREE_VAO_TEXTURE();
    int GenerateAABB(i3d::Frame &frame);

    int WarpToPanorama(i3d::Frame& kframe, i3d::Intrinsics& intrinsics);


    glm::mat4 skyboxModel, skyboxViews[6], skyboxProjection;
    GLuint skyboxVAO, skyboxVBO, skyboxColorTexture, skyboxDepthTexture;


    glm::mat4 panoModels[6], panoView, projection2;
    GLuint panoVAO, panoVBO;


    GLuint inputFramebuffer, inputRenderbuffer;


    double minV, maxV;
    GLenum err;


public:
    Warper4Android(std::vector<i3d::Frame>& kframes, i3d::Intrinsics& intrinsics);
    ~Warper4Android();
};

#endif //NATIVE0701_WARPER4ANDROID_H
