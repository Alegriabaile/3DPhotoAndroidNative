//
// Created by ale on 19-8-8.
//

#include <i3d.h>
#include "Renderer.h"

static std::string strVertexCode1("#version 300 es                                                \n"
                                  "precision highp float;                                         \n"
                                  "                                                               \n"
                                  "layout (location = 0) in vec3 attributePos;                    \n"
                                  "layout (location = 1) in vec3 attributeUV;                     \n"
                                  "                                                               \n"
                                  "uniform mat4 model;                                            \n"
                                  "uniform mat4 view;                                             \n"
                                  "uniform mat4 projection;                                       \n"
                                  "                                                               \n"
                                  "out vec2 UV;                                                   \n"
                                  "                                                               \n"
                                  "void main()                                                    \n"
                                  "{                                                              \n"
                                  "    vec4 position = view * model * vec4(attributePos, 1.0f);   \n"
                                  "    gl_Position = projection * position;                       \n"
                                  "                                                               \n"
                                  "    UV = vec2(attributeUV.xy);                                 \n"
                                  "}                                                              \n");

static std::string strFragmentCode1("#version 300 es                                                \n"
                                    "precision highp float;                                         \n"
                                    "                                                               \n"
                                    "in vec2 UV;                                                    \n"
                                    "                                                               \n"
                                    "uniform sampler2D colorTexture;                                \n"
                                    "                                                               \n"
                                    "layout(location = 0) out vec4 colorAttachment;                 \n"
                                    "                                                               \n"
                                    "void main()                                                    \n"
                                    "{                                                              \n"
                                    "    colorAttachment = texture(colorTexture, UV.xy);            \n"
                                    "}                                                              \n");


Renderer::Renderer():
        mStateInitialized(false)
{

}
Renderer::~Renderer()
{

}

void Renderer::initialize(std::vector<float>& vertices, cv::Mat &texture)
{
    mStateInitialized = false;

    GLenum err;

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    //shader...
    if(glIsProgram(mShader.ID))
        mShader.clear();
    mShader.init(strVertexCode1, strFragmentCode1);
    if(!glIsProgram(mShader.ID) || (err = glGetError()) != GL_NO_ERROR)
    {
        LOGE("Renderer::initialize(): Shader::init() failed... err = %d...", err);
        return;
    }


    //vao, vbo
    mVerticesSize = vertices.size()/5;
    if(vertices.empty())
    {
        LOGE("Renderer::initialize(): gen vao failed... mVerticesSize = %d...", mVerticesSize);
    } else
        LOGE("Renderer::initialize(): gen vao ... mVerticesSize = %d...", mVerticesSize);
    if(glIsBuffer(mVBO))
    {
        glDeleteBuffers(1, &mVBO);
        mVBO = 0;
    }

    if(glIsVertexArray(mVAO))
    {
        glDeleteVertexArrays(1, &mVAO);
        mVAO = 0;
    }

    glGenVertexArrays(1, &mVAO);
    glBindVertexArray(mVAO);

    glGenBuffers(1, &mVBO);
    glBindBuffer(GL_ARRAY_BUFFER, mVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(float), &vertices[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5*sizeof(float), (void*)0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5*sizeof(float), (void*)(3*sizeof(float)));

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    if((err = glGetError()) != GL_NO_ERROR)
    {
        LOGE("Renderer::initialize(): generate vao,vbo failed... err = %d...", err);
        return;
    }


    //texture...
    if(texture.empty())
    {
        LOGE("Renderer::initialize(): generate texture id failed... err = %d...", err);
        return;
    }
    glGenTextures(1, &mTexture);
    glBindTexture(GL_TEXTURE_2D, mTexture);
    cv::Mat texture_tmp;
    cv::cvtColor(texture, texture_tmp, cv::COLOR_BGR2RGBA);//避免四字节对齐问题。
    cv::flip(texture_tmp, texture_tmp, 0);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texture_tmp.cols, texture_tmp.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, texture_tmp.data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    if((err = glGetError()) != GL_NO_ERROR)
    {
        LOGE("Renderer::initialize(): generate texture2d failed... err = %d...", err);
        return;
    }


    mStateInitialized = true;
}

bool Renderer::isInitialized() const
{
    return mStateInitialized;
}

void Renderer::reshape(GLsizei width, GLsizei height)
{
    glViewport(0, 0, mWidth = width, mHeight = height);
}


void Renderer::display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    mShader.use();
    mShader.setInt("colorTexture", 0);
    // view/projection transformations
    glm::mat4 projection = glm::perspective(glm::radians(mCamera.Zoom), (float)mWidth / (float)mHeight, 0.1f, 65555.0f);
    glm::mat4 view = mCamera.GetViewMatrix();
    glm::mat4 model = glm::mat4(1.0f);
//    model = glm::translate(model, glm::vec3(0.0f, 0.0f, 1.0f)); // translate it down so it's at the center of the scene
    //model = glm::scale(model, glm::vec3(0.2f, 0.2f, 0.2f));	// it's a bit too big for our scene, so scale it down
    mShader.setMat4("projection", projection);
    mShader.setMat4("view", view);
    mShader.setMat4("model", model);

    //draw................
    // bind textures on corresponding texture units
    glActiveTexture(GL_TEXTURE0);//default actived
    glBindTexture(GL_TEXTURE_2D, mTexture);

    glBindVertexArray(mVAO);


    glDrawArrays(GL_TRIANGLES, 0, mVerticesSize);


    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Renderer::rotateCamera(float xoffset, float yoffset)
{
    mCamera.ProcessMouseMovement(xoffset, yoffset);
}
void Renderer::translateCamera(GLint camera_Movement, float step)
{
    mCamera.ProcessKeyboard(Camera_Movement(camera_Movement), step);
}
void Renderer::zoomCamera(float zoffset)
{
    mCamera.ProcessMouseScroll(zoffset);
}

void Renderer::resetR()
{
    mCamera.ResetR();
}
void Renderer::resetT()
{
    mCamera.ResetT();
}
void Renderer::resetCamera()
{
    mCamera.Reset();
}

const std::string& Renderer::getErrorInfo() const
{

    return std::string();
}