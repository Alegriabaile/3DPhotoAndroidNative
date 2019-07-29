//
// Created by ale on 19-7-25.
//

#include "WarperGlesInitializer.h"

int WarperGlesInitializer::GenerateFbos()
{
    //..............................................................................//
    //......................skybox frame buffer objects.............................//
    //..............................................................................//

    //>>>>>>>>>>>>>>>>>>>>>>color frame buffer<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
    // framebuffer configuration
    glGenFramebuffers(1, &fboColor1);
    glBindFramebuffer(GL_FRAMEBUFFER, fboColor1);

    // create a color attachment texture
    glGenTextures(1, &mColorTex2D1);
    glBindTexture(GL_TEXTURE_2D, mColorTex2D1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, WIDTH1, HEIGHT1, 0, GL_RGBA,  GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mColorTex2D1, 0);

    // create a depth attachment texture
    glGenTextures(1, &mDepthTex2D1);
    glBindTexture(GL_TEXTURE_2D, mColorTex2D1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, WIDTH1, HEIGHT1, 0, GL_RGBA,  GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mColorTex2D1, 0);
    // create a renderbuffer object for depth and stencil attachment (we won't be sampling these)
    //更接近opengl原生数据结构，当不需要读取(比如glReadPixels)这些数据时效率非常快，一般用来depth test 和stencil test
    glGenRenderbuffers(1, &mColorRenderBuffer1);
    glBindRenderbuffer(GL_RENDERBUFFER, mColorRenderBuffer1);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH32F_STENCIL8, WIDTH1, HEIGHT1); // use a single renderbuffer object for both a depth AND stencil buffer.
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, mColorRenderBuffer1); // now actually attach it
    // now that we actually created the framebuffer and added all attachments we want to check if it is actually complete now
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        LOGE("ERROR::FRAMEBUFFER:: Framebuffer is not complete!");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    //>>>>>>>>>>>>>>>>>>>>>>depth frame buffer<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
    // framebuffer configuration
    glGenFramebuffers(1, &fboDepth1);
    glBindFramebuffer(GL_FRAMEBUFFER, fboDepth1);

    // create a color attachment texture
    glGenTextures(1, &mDepthTex2D1);
    glBindTexture(GL_TEXTURE_2D, mDepthTex2D1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, WIDTH1, HEIGHT1, 0, GL_RGBA,  GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mDepthTex2D1, 0);
    // create a renderbuffer object for depth and stencil attachment (we won't be sampling these)
    //更接近opengl原生数据结构，当不需要读取(比如glReadPixels)这些数据时效率非常快，一般用来depth test 和stencil test
    glGenRenderbuffers(1, &mDepthRenderBuffer1);
    glBindRenderbuffer(GL_RENDERBUFFER, mDepthRenderBuffer1);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH32F_STENCIL8, WIDTH1, HEIGHT1); // use a single renderbuffer object for both a depth AND stencil buffer.
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, mDepthRenderBuffer1); // now actually attach it
    // now that we actually created the framebuffer and added all attachments we want to check if it is actually complete now
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        LOGE("ERROR::FRAMEBUFFER:: Framebuffer is not complete!");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);


    //..............................................................................//
    //......................panorama frame buffer objects...........................//
    //..............................................................................//

    return 0;
}

int WarperGlesInitializer::GenerateShaders()
{
    //..............................................................................//
    //......................skybox color and depth shaders..........................//
    //..............................................................................//

    std::string colorVertexCode1("#version 300 es                                                \n"
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

    std::string colorFragmentCode1("#version 300 es                                                \n"
                                   "precision highp float;                                         \n"
                                   "                                                               \n"
                                   "uniform sampler2D colorTexture;                                \n"
                                   "uniform sampler2D depthTexture;                                \n"
                                   "                                                               \n"
                                   "layout(position = 0) out vec4 colorAttachment;                 \n"
                                   "layout(position = 1) out vec4 depthAttachment;                 \n"
                                   "                                                               \n"
                                   "in vec2 UV;                                                    \n"
                                   "                                                               \n"
                                   "out vec4 FragColor;                                            \n"
                                   "                                                               \n"
                                   "void main()                                                    \n"
                                   "{                                                              \n"
                                   "    colorAttachment = texture(colorTexture, UV.xy);            \n"
                                   "    depthAttachment = texture(depthTexture, UV.xy);            \n"
                                   "}                                                              \n");
//
//    std::string depthVertexCode1("#version 300 es                                                \n"
//                                 "precision highp float;                                         \n"
//                                 "                                                               \n"
//                                 "layout (location = 0) in vec3 attributePos;                    \n"
//                                 "layout (location = 1) in vec3 attributeUV;                     \n"
//                                 "                                                               \n"
//                                 "uniform mat4 model;                                            \n"
//                                 "uniform mat4 view;                                             \n"
//                                 "uniform mat4 projection;                                       \n"
//                                 "                                                               \n"
//                                 "out float Radius;                                              \n"
//                                 "                                                               \n"
//                                 "void main()                                                    \n"
//                                 "{                                                              \n"
//                                 "    vec4 position = view * model * vec4(attributePos, 1.0f);   \n"
//                                 "    gl_Position = projection * position;                       \n"
//                                 "                                                               \n"
//                                 "    Radius = length(position);///300.0f;//如何保证精度？？？？？  \n"
//                                 "}                                                              \n");
//    std::string depthFragmentCode1("#version 300 es                                                \n"
//                                   "precision highp float;                                         \n"
//                                   "                                                               \n"
//                                   "                                                               \n"
//                                   "uniform sampler2D texture1;                                    \n"//决定插值方法（最近邻）
//                                   "                                                               \n"
//                                   "in float Radius;                                               \n"
//                                   "                                                               \n"
//                                   "out vec4 FragColor;                                            \n"
//                                   "                                                               \n"
//                                   "void main()                                                    \n"
//                                   "{                                                              \n"
//                                   "    FragColor = vec4(Radius, Radius, Radius, 1.0f);            \n"
//                                   "}                                                              \n");
    colorShader1.init(colorVertexCode1, colorFragmentCode1);
//    depthShader1.init(depthVertexCode1, depthFragmentCode1);



    //..............................................................................//
    //......................panorama color and depth shaders........................//
    //..............................................................................//
    std::string panoVertexCode("#version 300 es                                                     \n"
                               "precision highp float;                                              \n"
                               "                                                                    \n"
                               "layout (location = 0) in vec3 attributePos;                         \n"
                               "layout (location = 1) in vec3 attributeUV;                          \n"
                               "                                                                    \n"
                               "uniform mat4 model;                                                 \n"
                               "uniform mat4 view;                                                  \n"
                               "uniform int SIDE_LENGTH;                                            \n"
                               "                                                                    \n"
                               "out vec2 UV;                                                        \n"
                               "                                                                    \n"
                               "void main()                                                         \n"
                               "{                                                                   \n"
                               "    int h = gl_InstanceID/SIDE_LENGTH;                              \n"
                               "    int w = gl_InstanceID%SIDE_LENGTH;                              \n"
                               "    float tx = float(w-SIDE_LENGTH/2)/float(SIDE_LENGTH);           \n"
                               "    float ty = float(h-SIDE_LENGTH/2)/float(SIDE_LENGTH);           \n"
                               "    float tz = 0.5f;                                                \n"
                               "                                                                    \n"
                               "    UV = vec2(tx, ty);                                              \n"
                               "                                                                    \n"
                               "    vec4 temp = view * model * vec4(tx, ty, tz, 1.0f);              \n"
                               "    float y = temp.x; float z = temp.y; float x = temp.z;           \n"
                               "    float r = length(vec3(x,y,z));                                  \n"
                               "                                                                    \n"
                               "    float theta_ = acos(z/r);                                       \n"
                               "    float fa_;                                                      \n"
                               "                                                                    \n"
                               "    if(x>0 && y>=0)                                                 \n"
                               "        fa_ = atan(y/x);                                            \n"
                               "    else if(x>0 && y<0)                                             \n"
                               "        fa_ = 2*PI+atan(y/x);                                       \n"
                               "    else if(x<=0 && y>=0)                                           \n"
                               "        fa_ = PI + atan(y/x);                                       \n"
                               "    else                                                            \n"
                               "        fa_ = PI + atan(y/x);                                       \n"
                               "                                                                    \n"
                               "    gl_Position = vec4(1.0-fa_/PI, (0.5-theta_/PI)*2, 0.0, 1.0);    \n"
                               "                                                                    \n"
                               "}                                                                   \n");

    std::string panoFragmentCode("#version 300 es                                                     \n"
                                 "precision highp float;                                              \n"
                                 "                                                                    \n"
                                 "uniform texture2d colorTexture;                                     \n"
                                 "uniform texture2d depthTexture;                                     \n"
                                 "                                                                    \n"
                                 "in vec2 UV;                                                         \n"
                                 "                                                                    \n"
                                 "layout(position = 0) out vec4 colorAttachment;                      \n"
                                 "layout(position = 1) out vec4 depthAttachment;                      \n"
                                 "                                                                    \n"
                                 "void main()                                                         \n"
                                 "{                                                                   \n"
                                 "    colorAttachment = texture(colorTexture, UV.xy);                 \n"
                                 "    depthAttachment = texture(depthTexture, UV.xy);                 \n"
                                 "}                                                                   \n");

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
    glDeleteRenderbuffers(1, &mColorRenderBuffer1);
    glDeleteRenderbuffers(1, &mDepthRenderBuffer1);
    glDeleteTextures(1, &mColorTex2D1);
    glDeleteTextures(1, &mDepthTex2D1);
    glDeleteFramebuffers(1, &fboColor1);
    glDeleteFramebuffers(1, &fboDepth1);

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

