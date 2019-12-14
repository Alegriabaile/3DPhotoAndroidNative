//
// Created by ale on 19-12-9.
//

#ifndef MY3DPHOTO_6CUBEMAPCAPTURER_H
#define MY3DPHOTO_6CUBEMAPCAPTURER_H

#include <iostream>
#include <opencv2/opencv.hpp>

#include "6ShaderForCapturer.h"

#include "EGL/egl.h"
#include "EGL/eglplatform.h"
#include "EGL/eglext.h"
#include "GLES3/gl3.h"

namespace m3d
{

    class CubemapCapturer
    {
    private:
        const GLuint SIDELENGTH;

        GLuint attachments[6];
        GLuint cubemapFrameBuffers[6], cubemapRenderBuffer[6];
        GLuint colorAttachments[6], depthAttachments[6];//attachments.

        ShaderForCapturer shaderForCapturer;

        glm::mat4 modelMat, viewMats[6], projectionMat;

        GLuint inputFrameBuffer;//framebuffer that colorTex2d, depthTex2d attaches.
        GLuint colorTex2d, depthTex2d;

        GLuint vao, vbo;

        //to debug...
        void checkError();
        void checkError(const std::string str);

        //initialization.
        void InitFrameBuffers();
        void InitShader();
        void InitMvps();
        void InitTextures();

    public:
        CubemapCapturer(const GLuint SIDELENGTH_ = 2048);
        virtual ~CubemapCapturer();

        void draw(const std::vector<float> & vertices, const cv::Mat& colorTexture, const cv::Mat &depthTexture, const glm::mat4 &modelMat_ = glm::mat4(1.0f));
        void showResults();

        const GLuint* colorAttatches();
        const GLuint* depthAttatches();
    };

}

#endif //MY3DPHOTO_6CUBEMAPCAPTURER_H
