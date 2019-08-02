//
// Created by ale on 19-7-25.
//

#include "WarperGlesInitializer.h"

int WarperGlesInitializer::GenerateFbos()
{
    GLenum err;
    GLuint attachments_tmp[6] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2,  GL_COLOR_ATTACHMENT3, GL_COLOR_ATTACHMENT4, GL_COLOR_ATTACHMENT5};
    for(int i=0; i<6; ++i)
        attachments[i] = attachments_tmp[i];
    //..............................................................................//
    //......................skybox frame buffer objects.............................//
    //..............................................................................//

    // framebuffer configuration
    glGenFramebuffers(6, fboSkybox1);
    for(int i=0; i<6; ++i)
    {
        glBindFramebuffer(GL_FRAMEBUFFER, fboSkybox1[i]);

        //>>>>>>>>>>>>>>>>>>>>>>color texture attachment<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
        // create a color attachment texture
        glGenTextures(1, &skyboxColorTex2D[i]);
        glBindTexture(GL_TEXTURE_2D, skyboxColorTex2D[i]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, WIDTH1, HEIGHT1, 0, GL_RGBA,  GL_UNSIGNED_BYTE, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, skyboxColorTex2D[i], 0);
        while((err = glGetError()) != GL_NO_ERROR)
            LOGE(" WarperGlesInitializer::GenerateFbos() : mSkyboxColorTex2D err == %d", err);

        //>>>>>>>>>>>>>>>>>>>>>>depth texture attachment<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
        glGenTextures(1, &skyboxDepthTex2D[i]);
        glBindTexture(GL_TEXTURE_2D, skyboxDepthTex2D[i]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, WIDTH1, HEIGHT1, 0, GL_RED,  GL_FLOAT, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, skyboxDepthTex2D[i], 0);
        while((err = glGetError()) != GL_NO_ERROR)
            LOGE(" WarperGlesInitializer::GenerateFbos() : mSkyboxDepthTex2D err == %d", err);

        glGenRenderbuffers(1, &mSkyboxRenderBuffer[i]);
        glBindRenderbuffer(GL_RENDERBUFFER, mSkyboxRenderBuffer[i]);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH32F_STENCIL8, WIDTH1, HEIGHT1); // use a single renderbuffer object for both a depth AND stencil buffer.
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, mSkyboxRenderBuffer[i]); // now actually attach it
        // now that we actually created the framebuffer and added all attachments we want to check if it is actually complete now
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            LOGE("WarperGlesInitializer::GenerateFbos(): ERROR::FRAMEBUFFER:: Framebuffer is not complete!");

        //不用DrawBuffers，则只会画Attachment0所在的颜色附着。
        glDrawBuffers(2, attachments);

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    //..............................................................................//
    //......................panorama frame buffer objects...........................//
    //..............................................................................//
    //////////////////////////////////color//////////////////////////////////////////////////
    glGenFramebuffers(1, &fboPano1);
    glBindFramebuffer(GL_FRAMEBUFFER, fboPano1);

    // create a color attachment texture
    glGenTextures(1, &mPanoColorTex2D);
    glBindTexture(GL_TEXTURE_2D, mPanoColorTex2D);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, WIDTH2, HEIGHT2, 0, GL_RGBA,  GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mPanoColorTex2D, 0);
    while((err = glGetError()) != GL_NO_ERROR)
        LOGE(" WarperGlesInitializer::GenerateFbos() : mSkyboxColorTex2D err == %d", err);


    // create a depth attachment texture
    glGenTextures(1, &mPanoDepthTex2D);
    glBindTexture(GL_TEXTURE_2D, mPanoDepthTex2D);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, WIDTH2, HEIGHT2, 0, GL_RED,  GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, mPanoDepthTex2D, 0);
    while((err = glGetError()) != GL_NO_ERROR)
        LOGE(" WarperGlesInitializer::GenerateFbos() : mSkyboxColorTex2D err == %d", err);

    // create a renderbuffer object for depth and stencil attachment (we won't be sampling these)
    //更接近opengl原生数据结构，当不需要读取(比如glReadPixels)这些数据时效率非常快，一般用来depth test 和stencil test
    glGenRenderbuffers(1, &mPanoRenderBuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, mPanoRenderBuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH32F_STENCIL8, WIDTH2, HEIGHT2); // use a single renderbuffer object for both a depth AND stencil buffer.
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, mPanoRenderBuffer); // now actually attach it
    // now that we actually created the framebuffer and added all attachments we want to check if it is actually complete now
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        LOGE("WarperGlesInitializer::GenerateFbos(): ERROR::FRAMEBUFFER:: Framebuffer is not complete!");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    return 0;
}

int WarperGlesInitializer::GenerateShaders()
{
    //..............................................................................//
    //......................skybox color and depth shaders..........................//
    //..............................................................................//
    std::string skyboxVertexCode1("#version 300 es                                                \n"
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

    std::string skyboxFragmentCode1("#version 300 es                                                \n"
                                    "precision highp float;                                         \n"
                                    "                                                               \n"
                                    "in vec2 UV;                                                    \n"
                                    "                                                               \n"
                                    "uniform sampler2D colorTexture;                                \n"
                                    "uniform sampler2D depthTexture;                                \n"
                                    "                                                               \n"
                                    "layout(location = 0) out vec4 colorAttachment;                 \n"
                                    "layout(location = 1) out vec4 depthAttachment;                 \n"
                                    "                                                               \n"
                                    "void main()                                                    \n"
                                    "{                                                              \n"
                                    "    colorAttachment = texture(colorTexture, UV.xy);            \n"
                                    "    depthAttachment = texture(depthTexture, UV.xy);            \n"
                                    "}                                                              \n");

    //color
    shaderSkybox1.init(skyboxVertexCode1, skyboxFragmentCode1);
    //depth
//    shaderSkybox2.init(skyboxVertexCode2, skyboxFragmentCode2);



    //..............................................................................//
    //......................panorama color and depth shaders........................//
    //..............................................................................//
    std::string panoVertexCode1("#version 300 es                                                     \n"
                                "precision highp float;                                              \n"
                                "                                                                    \n"
                                "layout (location = 0) in float attributePos;                        \n"
                                "                                                                    \n"
                                "uniform mat4 model;                                                 \n"
                                "uniform mat4 view;                                                  \n"
                                "uniform int WIDTH;                                                  \n"
                                "                                                                    \n"
                                "out vec2 UV;                                                        \n"
                                "                                                                    \n"
                                "void main()                                                         \n"
                                "{                                                                   \n"
                                "    const float PI = 3.1415926535897932384626433832795;             \n"
                                "    float height = float(WIDTH);                                    \n"
                                "    float width = float(WIDTH);                                     \n"
                                "                                                                    \n"
                                "    float h = float(gl_VertexID/WIDTH);                             \n"
                                "    float w = float(gl_VertexID%WIDTH);                             \n"
                                "    float tx = (w-width/2.0+0.5/width)/width;                       \n"
                                "    float ty = (h-width/2.0+0.5/width)/width;                       \n"
                                "    float tz = 0.5f;                                                \n"
                                "                                                                    \n"
                                "    UV = vec2(tx+0.5, ty+0.5);                                      \n"
                                "                                                                    \n"
                                "    vec4 temp = view * model * vec4(tx, ty, tz, 1.0f);              \n"
                                "    float y = temp.x; float z = temp.y; float x = temp.z;           \n"
                                "    float r = length(vec3(x,y,z));                                  \n"
                                "                                                                    \n"
                                "    float theta_ = acos(z/r);                                       \n"
                                "    float fa_;                                                      \n"
                                "                                                                    \n"
                                "    if(x>0.0 && y>=0.0)                                             \n"
                                "        fa_ = atan(y/x);                                            \n"
                                "    else if(x>0.0 && y<0.0)                                         \n"
                                "        fa_ = 2.0*PI+atan(y/x);                                     \n"
                                "    else if(x<=0.0 && y>=0.0)                                       \n"
                                "        fa_ = PI + atan(y/x);                                       \n"
                                "    else                                                            \n"
                                "        fa_ = PI + atan(y/x);                                       \n"
                                "                                                                    \n"
                                "    gl_Position = vec4(1.0-fa_/PI, (0.5-theta_/PI)*2.0, 0.0, 1.0);  \n"
                                "                                                                    \n"
                                "}                                                                   \n");

    std::string panoFragmentCode1("#version 300 es                                                     \n"
                                  "precision highp float;                                              \n"
                                  "                                                                    \n"
                                  "uniform sampler2D colorTexture;                                     \n"
                                  "uniform sampler2D depthTexture;                                     \n"
                                  "                                                                    \n"
                                  "in vec2 UV;                                                         \n"
                                  "                                                                    \n"
                                  "layout(location = 0) out vec4 colorAttachment;                      \n"
                                  "layout(location = 1) out vec4 depthAttachment;                      \n"
                                  "                                                                    \n"
                                  "void main()                                                         \n"
                                  "{                                                                   \n"
                                  "    colorAttachment = texture(colorTexture, UV.xy);                 \n"
                                  "    depthAttachment = texture(depthTexture, UV.xy);                 \n"
                                  "    //depthAttachment = vec4(100.0f, 0.0f, 0.0f, 0.0f);             \n"
                                  "}                                                                   \n");


    LOGE(" WarperGlesInitializer::GenerateShaders(): before shaderPano1...");
    shaderPano1.init(panoVertexCode1, panoFragmentCode1);
//    LOGE(" WarperGlesInitializer::GenerateShaders(): before shaderPano2...");
//    shaderPano2.init(panoVertexCode2, panoFragmentCode2);
    return 0;
}

int WarperGlesInitializer::InitializeGlesSrcs()
{
    //输出opengles版本
    const char* glesVersion = (const char*)glGetString(GL_VERSION);
    LOGE(" WarperGlesInitializer(): glGetString(): gles version: %s", glesVersion);

    GenerateFbos();
    GenerateShaders();

    //若存在错误，则在此输出
    GLenum err;
    while((err = glGetError()) != GL_NO_ERROR)
    {
        LOGE(" WarperGlesInitializer() :    err == %d", err);
    }
    return 0;
}

int WarperGlesInitializer::ReleaseGlesSrcs()
{
    //release skybox resources...
//    glDeleteRenderbuffers(1, &mColorRenderBuffer1);
//    glDeleteRenderbuffers(1, &mDepthRenderBuffer1);
//    glDeleteTextures(1, &mColorTex2D1);
//    glDeleteTextures(1, &mDepthTex2D1);
//    glDeleteFramebuffers(1, &fboColor1);
//    glDeleteFramebuffers(1, &fboDepth1);

    //release panorama resources...
    //todo
    return 0;
}

WarperGlesInitializer::WarperGlesInitializer():
WIDTH1(i3d::PANO_H), HEIGHT1(i3d::PANO_H), WIDTH2(i3d::PANO_W), HEIGHT2(i3d::PANO_H)

{
    InitializeGlesSrcs();
}

WarperGlesInitializer::~WarperGlesInitializer()
{
    ReleaseGlesSrcs();
}

