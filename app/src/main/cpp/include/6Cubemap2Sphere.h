//
// Created by ale on 19-12-9.
//

#ifndef MY3DPHOTO_6CUBEMAP2SPHERE_H
#define MY3DPHOTO_6CUBEMAP2SPHERE_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "EGL/egl.h"
#include "EGL/eglplatform.h"
#include "EGL/eglext.h"
#include "GLES3/gl3.h"

#include "6ShaderForCapturer.h"

namespace m3d
{
    class Cubemap2Sphere
    {
    private:
        const GLuint HEIGHT, WIDTH;
//        GLuint textureID[12];
        GLuint attachments[6];

        GLuint sphereFrameBuffer, sphereRenderBuffer;
        GLuint colorAttachment, depthAttachment;//attachments.

        GLuint vao, vbo;
//        glm::mat4 modelMat, viewMat, projectionMat;

        ShaderForCapturer shaderForCapturer;

        std::vector<float> vertices;
        //to debug...
        void checkError();
        void checkError(const std::string str);
        std::string to_string(size_t value);

        void InitFrameBuffers();
        void InitShader();
        void InitMvps();
        void InitTextures();
        void InitVertices();

    public:
        Cubemap2Sphere(const GLuint HEIGHT_=2048, const GLuint WIDTH_=4096);
        virtual ~Cubemap2Sphere();

        void draw(const GLuint colorTex2ds[6], const GLuint depthTex2ds[6], cv::Mat &pano_image, cv::Mat &pano_depth);
        void showResults();
    };

}

#endif //MY3DPHOTO_6CUBEMAP2SPHERE_H
