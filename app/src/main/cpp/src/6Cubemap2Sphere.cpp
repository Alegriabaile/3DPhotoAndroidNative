//
// Created by ale on 19-12-9.
//

#include "6Cubemap2Sphere.h"

namespace m3d
{
    Cubemap2Sphere::Cubemap2Sphere(const GLuint HEIGHT_, const GLuint WIDTH_)
    : HEIGHT(HEIGHT_), WIDTH(WIDTH_)
    {
        InitFrameBuffers();
        InitShader();
        InitMvps();
        InitTextures();
        InitVertices();
    }
    Cubemap2Sphere::~Cubemap2Sphere() {}

    void Cubemap2Sphere::checkError()
    {
        GLenum err;
        while((err = glGetError()) != GL_NO_ERROR)
        {
            printf(" Cubemap2Sphere::checkError() :    err == %d \n", err);
        }
    }
    void Cubemap2Sphere::checkError(const std::string str)
    {
        GLenum err;
        while((err = glGetError()) != GL_NO_ERROR)
            std::cout<<str<<"  :    err == "<<err<<"\n";
    }

    void Cubemap2Sphere::draw(const GLuint colorTex2ds[6], const GLuint depthTex2ds[6], cv::Mat &pano_image, cv::Mat &pano_depth)
    {
        checkError("Cubemap2Sphere::draw(): before initialization. ");

        glEnable(GL_DEPTH_TEST);

        GLuint GLTEXTUREIDs[12] = {GL_TEXTURE0, GL_TEXTURE1, GL_TEXTURE2, GL_TEXTURE3, GL_TEXTURE4, GL_TEXTURE5,
                                   GL_TEXTURE6, GL_TEXTURE7, GL_TEXTURE8, GL_TEXTURE9, GL_TEXTURE10, GL_TEXTURE11};

        glBindFramebuffer(GL_FRAMEBUFFER, sphereFrameBuffer);
        glDrawBuffers(2, attachments);

        shaderForCapturer.use();
        for(size_t i = 0; i < 6; i++)
        {
            // bind textures on corresponding texture units
            glActiveTexture(GLTEXTUREIDs[i]);//default actived
            glBindTexture(GL_TEXTURE_2D, colorTex2ds[i]);

            glActiveTexture(GLTEXTUREIDs[i+6]);
            glBindTexture(GL_TEXTURE_2D, depthTex2ds[i]);

            std::string colorName = std::string("colorTexture") + to_string(i);
            std::string depthName = std::string("depthTexture") + to_string(i);
            shaderForCapturer.setInt( colorName.c_str(), i);
            shaderForCapturer.setInt( depthName.c_str(), i+6);
        }
        checkError("Cubemap2Sphere::draw(): after draw: ");

        glViewport(0, 0, WIDTH, HEIGHT);
        glClearColor(0.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glBindVertexArray(vao);

        glDrawArrays(GL_TRIANGLES, 0, vertices.size() / 5);

        glBindVertexArray(0);

        for(size_t i = 0; i < 6; i++)
        {
            // bind textures on corresponding texture units
            glActiveTexture(GLTEXTUREIDs[i]);//default actived
            glBindTexture(GL_TEXTURE_2D, 0);

            glActiveTexture(GLTEXTUREIDs[i+6]);
            glBindTexture(GL_TEXTURE_2D, 0);
        }

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        
        
        
        //read the results.
        cv::Mat pixel_color(HEIGHT, WIDTH, CV_8UC4, cv::Scalar(0, 0, 0, 0));
        cv::Mat pixel_depth(HEIGHT, WIDTH, CV_32FC1, cv::Scalar(0.0f, 0.0f, 0.0f, 0.0f));

        //bind pano framebuffer
        glBindFramebuffer(GL_FRAMEBUFFER, sphereFrameBuffer);

        glReadBuffer ( GL_COLOR_ATTACHMENT0 );
        glReadPixels(0, 0, pixel_color.cols, pixel_color.rows, GL_RGBA, GL_UNSIGNED_BYTE, pixel_color.data);
        checkError("Cubemap2Sphere::showResults(): after read color buffer: ");

        glReadBuffer ( GL_COLOR_ATTACHMENT1 );
        glReadPixels(0, 0, pixel_depth.cols, pixel_depth.rows, GL_RED, GL_FLOAT, pixel_depth.data);
        checkError("Cubemap2Sphere::showResults(): after read depth buffer: ");
        //unbind pano framebuffer
        glBindFramebuffer(GL_FRAMEBUFFER, 0);


        cv::cvtColor(pixel_color, pano_image, cv::COLOR_RGBA2BGR);
        cv::flip(pano_image, pano_image, 0);
        cv::flip(pixel_depth, pano_depth, 0);
    }

    void Cubemap2Sphere::InitFrameBuffers()
    {
        GLuint attachments_[6] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2,  GL_COLOR_ATTACHMENT3, GL_COLOR_ATTACHMENT4, GL_COLOR_ATTACHMENT5};
        for(size_t i = 0; i < 6; ++i)
            attachments[i] = attachments_[i];

        glGenFramebuffers(1, &sphereFrameBuffer);
        glBindFramebuffer(GL_FRAMEBUFFER, sphereFrameBuffer);

        // create a color attachment texture
        glGenTextures(1, &colorAttachment);
        glBindTexture(GL_TEXTURE_2D, colorAttachment);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, WIDTH, HEIGHT, 0, GL_RGBA,  GL_UNSIGNED_BYTE, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorAttachment, 0);
        checkError("Cubemap2Sphere::InitFrameBuffers(): after generate color texture[i]: ");

        // create a depth attachment texture
        glGenTextures(1, &depthAttachment);
        glBindTexture(GL_TEXTURE_2D, depthAttachment);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, WIDTH, HEIGHT, 0, GL_RED,  GL_FLOAT, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, depthAttachment, 0);
        checkError("Cubemap2Sphere::InitFrameBuffers(): after generate depth texture[i]: ");

        glGenRenderbuffers(1, &sphereRenderBuffer);
        glBindRenderbuffer(GL_RENDERBUFFER, sphereRenderBuffer);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH32F_STENCIL8, WIDTH, HEIGHT); // use a single renderbuffer object for both a depth AND stencil buffer.
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, sphereRenderBuffer); // now actually attach it
        // now that we actually created the framebuffer and added all attachments we want to check if it is actually complete now
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            printf("OpenglManagerForWarper::GenerateFbos(): ERROR::FRAMEBUFFER:: Framebuffer is not complete! \n");

        //不用DrawBuffers，则只会画Attachment0所在的颜色附着。
        glDrawBuffers(2, attachments);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void Cubemap2Sphere::InitShader()
    {
        checkError("Cubemap2Sphere::InitShader(): before Init shader: ");

        std::string sphereVertexCode1("#version 300 es                                                \n"
                                      "precision highp float;                                         \n"
                                      "                                                               \n"
                                      "layout (location = 0) in vec3 attributePos;                    \n"
                                      "layout (location = 1) in vec3 attributeUV;                     \n"
                                      "                                                               \n"
                                      "out vec2 UV;                                                   \n"
                                      "                                                               \n"
                                      "void main()                                                    \n"
                                      "{                                                              \n"
                                      "    gl_Position = vec4(attributePos, 1.0f);                    \n"
                                      "    UV = vec2(attributeUV.xy);                                 \n"
                                      "}                                                              \n");


        std::string sphereFragmentCode1("#version 300 es                                                \n"
                                        "precision highp float;                                         \n"
                                        "                                                               \n"
                                        "#define M_PI 3.1415926535897932384626433832795                 \n"
                                        "uniform sampler2D colorTexture0;//front                        \n"
                                        "uniform sampler2D colorTexture1;//right                        \n"
                                        "uniform sampler2D colorTexture2;//back                         \n"
                                        "uniform sampler2D colorTexture3;//left                         \n"
                                        "uniform sampler2D colorTexture4;//top                          \n"
                                        "uniform sampler2D colorTexture5;//bottom                       \n"
                                        "                                                               \n"
                                        "uniform sampler2D depthTexture0;                               \n"
                                        "uniform sampler2D depthTexture1;                               \n"
                                        "uniform sampler2D depthTexture2;                               \n"
                                        "uniform sampler2D depthTexture3;                               \n"
                                        "uniform sampler2D depthTexture4;                               \n"
                                        "uniform sampler2D depthTexture5;                               \n"
                                        "                                                               \n"
                                        "in vec2 UV;                                                    \n"
                                        "                                                               \n"
                                        "layout(location = 0) out vec4 colorAttachment;                 \n"
                                        "layout(location = 1) out vec4 depthAttachment;                 \n"
                                        "                                                               \n"
                                        "void main()                                                    \n"
                                        "{                                                              \n"
                                        "    //from 3rd/cube2equirect-master/shaders/cube2equirect.frag \n"
                                        "    float theta = UV.x * M_PI;                                 \n"
                                        "    float phi = (UV.y * M_PI) / 2.0;                           \n"
                                        "                                                               \n"
                                        "    float x = cos(phi) * sin(theta);                           \n"
                                        "    float y = sin(phi);                                        \n"
                                        "    float z = cos(phi) * cos(theta);                           \n"
                                        "                                                               \n"
                                        "    float scale;                                               \n"
                                        "    vec2 px;                                                   \n"
                                        "    vec4 src;                                                  \n"
                                        "    vec4 src_depth;                                            \n"
                                        "                                                               \n"
                                        "    if (abs(x) >= abs(y) && abs(x) >= abs(z)) {                \n"
                                        "      if (x < 0.0) {                                           \n"
                                        "        scale = -1.0 / x;                                      \n"
                                        "        px.x = ( z*scale + 1.0) / 2.0;                         \n"
                                        "        px.y = ( y*scale + 1.0) / 2.0;                         \n"
                                        "        src = texture(colorTexture3, px);                      \n"
                                        "        src_depth = texture(depthTexture3, px);                \n"
                                        "      }                                                        \n"
                                        "      else{                                                    \n"
                                        "        scale = 1.0 / x;                                       \n"
                                        "        px.x = (-z*scale + 1.0) / 2.0;                         \n"
                                        "        px.y = ( y*scale + 1.0) / 2.0;                         \n"
                                        "        src = texture(colorTexture1, px);                      \n"
                                        "        src_depth = texture(depthTexture1, px);                \n"
                                        "      }                                                        \n"
                                        "    }                                                          \n"
                                        "    else if (abs(y) >= abs(z)) {                               \n"
                                        "      if (y < 0.0) {                                           \n"
                                        "        scale = -1.0 / y;                                      \n"
                                        "        px.x = ( x*scale + 1.0) / 2.0;                         \n"
                                        "        px.y = ( z*scale + 1.0) / 2.0;                         \n"
                                        "        src = texture(colorTexture5, px);                      \n"
                                        "        src_depth = texture(depthTexture5, px);                \n"
                                        "      }                                                        \n"
                                        "      else {                                                   \n"
                                        "        scale = 1.0 / y;                                       \n"
                                        "        px.x = ( x*scale + 1.0) / 2.0;                         \n"
                                        "        px.y = (-z*scale + 1.0) / 2.0;                         \n"
                                        "        src = texture(colorTexture4, px);                      \n"
                                        "        src_depth = texture(depthTexture4, px);                \n"
                                        "      }                                                        \n"
                                        "    }                                                          \n"
                                        "    else {                                                     \n"
                                        "      if (z < 0.0) {                                           \n"
                                        "        scale = -1.0 / z;                                      \n"
                                        "        px.x = (-x*scale + 1.0) / 2.0;                         \n"
                                        "        px.y = ( y*scale + 1.0) / 2.0;                         \n"
                                        "        src = texture(colorTexture2, px);                      \n"
                                        "        src_depth = texture(depthTexture2, px);                \n"
                                        "      }                                                        \n"
                                        "      else {                                                   \n"
                                        "        scale = 1.0 / z;                                       \n"
                                        "        px.x = ( x*scale + 1.0) / 2.0;                         \n"
                                        "        px.y = ( y*scale + 1.0) / 2.0;                         \n"
                                        "        src = texture(colorTexture0, px);                      \n"
                                        "        src_depth = texture(depthTexture0, px);                \n"
                                        "      }                                                        \n"
                                        "    }                                                          \n"
                                        "                                                               \n"
                                        "    colorAttachment = src;                                     \n"
                                        "    depthAttachment = src_depth;                               \n"
                                        "}                                                              \n");
        //color
        shaderForCapturer.init(sphereVertexCode1, sphereFragmentCode1);
        checkError("Cubemap2Sphere::InitShader(): after Init shader: ");
    }

    void Cubemap2Sphere::InitMvps(){}

    void Cubemap2Sphere::InitTextures()
    {
    }

    void Cubemap2Sphere::InitVertices()
    {
        std::vector<float> vertices_ = {
                -1.0, -1.0, 0.0,  // left,  bottom, back
                -1.0, -1.0,  // left,  bottom
                1.0,  1.0, 0.0,   // right, top,    back
                1.0,  1.0,   // right, top
                -1.0,  1.0, 0.0,  // left,  top,    back
                -1.0,  1.0,  // left,  top

                1.0,  1.0, 0.0,   // right, top,    back
                1.0,  1.0,   // right, top
                -1.0, -1.0, 0.0,  // left,  bottom, back
                -1.0, -1.0,  // left,  bottom
                1.0, -1.0, 0.0,  // right, bottom, back
                1.0, -1.0,  // right, bottom
        };
        std::swap(vertices_, vertices);
        //vertices.

        glGenVertexArrays(1, &vao);
        checkError("CubemapCapturer::draw(): after glGenVertexArrays(): ");
        glBindVertexArray(vao);

        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);

        glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(float), &vertices[0], GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5*sizeof(float), (void*)0);

        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5*sizeof(float), (void*)(3*sizeof(float)));

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        checkError("Cubemap2Sphere::InitVertices(): after initialize the vertices. ");
    }

    void Cubemap2Sphere::showResults()
    {
        cv::Mat pixel_color(HEIGHT, WIDTH, CV_8UC4, cv::Scalar(0, 0, 0, 0));
        cv::Mat pixel_depth(HEIGHT, WIDTH, CV_32FC1, cv::Scalar(0.0f, 1.0f, 1.0f, 1.0f));


        //bind pano framebuffer
        glBindFramebuffer(GL_FRAMEBUFFER, sphereFrameBuffer);

        glReadBuffer ( GL_COLOR_ATTACHMENT0 );
        glReadPixels(0, 0, pixel_color.cols, pixel_color.rows, GL_RGBA, GL_UNSIGNED_BYTE, pixel_color.data);
        checkError("Cubemap2Sphere::showResults(): after read color buffer: ");

        glReadBuffer ( GL_COLOR_ATTACHMENT1 );
        glReadPixels(0, 0, pixel_depth.cols, pixel_depth.rows, GL_RED, GL_FLOAT, pixel_depth.data);
        checkError("Cubemap2Sphere::showResults(): after read depth buffer: ");
        //unbind pano framebuffer
        glBindFramebuffer(GL_FRAMEBUFFER, 0);



        cv::Mat color_tmp;// = pixel_color.clone();
        cv::cvtColor(pixel_color, color_tmp, cv::COLOR_RGBA2BGR);
        cv::flip(color_tmp, color_tmp, 0);
        cv::imshow("color", color_tmp);

        double minV, maxV;
        minMaxLoc(pixel_depth, &minV, &maxV);
        printf("Cubemap2Sphere::showResults():---pixel_depth--- minV, maxV: %lf, %lf\n", minV, maxV);

        cv::flip(pixel_depth, pixel_depth, 0);
        cv::imshow("depth", pixel_depth);
        cv::waitKey();
    }

#include <string>
#include <sstream>

    std::string Cubemap2Sphere::to_string(size_t value)
    {
        std::ostringstream os ;
        os << value ;
        return os.str() ;
    }


}