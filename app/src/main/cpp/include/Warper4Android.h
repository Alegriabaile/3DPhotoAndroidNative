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

    int GenerateTriangles( const cv::Mat &depth,
                           const i3d::Intrinsics &intrinsics,
                           std::vector<float> &vertices);

    int GenerateSkybox( const i3d::Frame &frame,
                        const std::vector<float> &vertices,
                        std::vector<cv::Mat> &images,
                        std::vector<cv::Mat> &depths);

    int GeneratePoints( std::vector<float> &vPoints);

    int GeneratePanorama(const std::vector<cv::Mat> &images,
                         const std::vector<cv::Mat> &depths,
                         i3d::Frame &frame);


    int InitializeReusable();
    void SKYBOX_GEN_VAO_TEXTURE_MVP(const i3d::Frame &frame, const std::vector<float> &vertices);
    //void FREE_VAO_TEXTURE();
    int WarpToPanorama(i3d::Frame& kframe, i3d::Intrinsics& intrinsics);


    glm::mat4 skyboxModel, skyboxViews[6], skyboxProjection;
    GLuint skyboxVAO, skyboxVBO, skyboxColorTexture, skyboxDepthTexture;


    glm::mat4 panoModels[6], panoView, projection2;
    GLuint panoVAO, panoVBO, panoColorTexture, panoDepthTexture;


    GLuint inputFramebuffer, inputColorTexture, inputDepthTexture, inputRenderbuffer;


    double minV, maxV;
    GLenum err;
public:
    Warper4Android(std::vector<i3d::Frame>& kframes, i3d::Intrinsics& intrinsics);
    ~Warper4Android();

};


template <typename T>
void print_f(T i)
{
    std::cout<<i<<std::endl;
}
#endif //NATIVE0701_WARPER4ANDROID_H
