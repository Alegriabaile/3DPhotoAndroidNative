//
// Created by ale on 19-7-25.
//

#ifndef NATIVE0701_WARPERGLESINITIALIZER_H
#define NATIVE0701_WARPERGLESINITIALIZER_H

#include "i3d.h"
#include "WarperShader.h"
#include "WarperEglManager.h"

//index == 1: for skybox
//index == 2: for panorama
class WarperGlesInitializer {
public:
    GLuint WIDTH1, HEIGHT1;//
    GLuint WIDTH2, HEIGHT2;//

    GLuint fboColor1, fboDepth1;//for skybox
    GLuint fboColor2, fboDepth2;//for panorama

    Shader colorShader1, depthShader1;
    Shader colorShader2, depthShader2;

    WarperGlesInitializer();
    ~WarperGlesInitializer();

private:
    WarperEglManager warperEglManager;//manage egl environment.

    GLuint mColorTex2D1, mDepthTex2D1, mColorRenderBuffer1, mDepthRenderBuffer1;//for skybox
    GLuint mColorTex2D2, mDepthTex2D2, mColorRenderBuffer2, mDepthRenderBuffer2;//for panorama
    int InitializeGlesSrcs();
    int ReleaseGlesSrcs();
    int GenerateFbos();
    int GenerateShaders();
};


#endif //NATIVE0701_WARPERGLESINITIALIZER_H
