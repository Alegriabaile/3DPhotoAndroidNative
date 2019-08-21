//
// Created by ale on 19-8-8.
//

#ifndef NATIVE0701_RENDERER_H
#define NATIVE0701_RENDERER_H

#include "Camera.h"
#include "WarperShader.h"

class Renderer {
private:
    //buffers...
    Camera mCamera;
    Shader mShader;
    GLuint mVAO, mVBO, mTexture;
    GLuint mVerticesSize;
    //attributes
    GLuint mWidth, mHeight;

    //states:
    bool mStateInitialized;
public:
    Renderer();
    ~Renderer();

    void initialize(std::vector<float>& vArray, cv::Mat &texture);
    bool isInitialized() const;

    void reshape(GLsizei width, GLsizei height);
    void display();


    void rotateCamera(float xoffset, float yoffset);
    void translateCamera(GLint camera_Movement, float step);
    void zoomCamera(float zoffset);

    void resetR();
    void resetT();
    void resetCamera();

    const std::string & getErrorInfo() const;
};


#endif //NATIVE0701_RENDERER_H
