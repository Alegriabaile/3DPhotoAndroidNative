//
// Created by ale on 19-12-9.
//

#include "6CubemapCapturer.h"

namespace m3d
{
    CubemapCapturer::CubemapCapturer(const GLuint SIDELENGTH_)
    : SIDELENGTH(SIDELENGTH_)
    {
        InitFrameBuffers();
        InitShader();
        InitMvps();
        InitTextures();
    }
    CubemapCapturer::~CubemapCapturer() {}

    void CubemapCapturer::checkError()
    {
        GLenum err;
        while((err = glGetError()) != GL_NO_ERROR)
            printf(" CubemapCapturer::checkError() :    err == %d \n", err);
    }
    void CubemapCapturer::checkError(const std::string str)
    {
        GLenum err;
        while((err = glGetError()) != GL_NO_ERROR)
            std::cout<<str<<"  :    err == "<<err<<"\n";
    }

    void CubemapCapturer::draw(const std::vector<float> &vertices, const cv::Mat &color_, const cv::Mat &depth_, const glm::mat4 &modelMat_)
    {
        //vertices.
        if(!glIsVertexArray(vao))
            glGenVertexArrays(1, &vao);
        checkError("CubemapCapturer::draw(): after glGenVertexArrays(): ");

        if(glIsBuffer(vbo))
            glDeleteBuffers(1, &vbo);

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
        checkError("CubemapCapturer::draw(): after initialize the vertices. ");

        //textures.
        cv::Mat color;
        cv::flip(color_, color, 0);
        cv::cvtColor(color, color, CV_BGR2RGBA);

        cv::Mat depth;
        cv::flip(depth_, depth, 0);

        glBindTexture(GL_TEXTURE_2D, colorTex2d);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, color.cols, color.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, color.data);

        glBindTexture(GL_TEXTURE_2D, depthTex2d);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, depth.cols, depth.rows, 0, GL_RED, GL_FLOAT, depth.data);
        checkError("CubemapCapturer::draw(): after initialize the textures. ");
        glBindTexture(GL_TEXTURE_2D, 0);

        glEnable(GL_DEPTH_TEST);

        modelMat = modelMat_;
        for(int i=0; i<6; i++)
        {
            glBindFramebuffer(GL_FRAMEBUFFER, cubemapFrameBuffers[i]);
            glDrawBuffers(2, attachments);
            // bind textures on corresponding texture units
            glActiveTexture(GL_TEXTURE0);//default actived
            glBindTexture(GL_TEXTURE_2D, colorTex2d);

            glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_2D, depthTex2d);

            glViewport(0, 0, SIDELENGTH, SIDELENGTH);
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            shaderForCapturer.use();
            shaderForCapturer.setInt("colorTexture", 0);
            shaderForCapturer.setInt("depthTexture", 1);
            shaderForCapturer.setMat4("projection", projectionMat);
            shaderForCapturer.setMat4("view", viewMats[i]);
            shaderForCapturer.setMat4("model", modelMat);

            glBindVertexArray(vao);

            glDrawArrays(GL_TRIANGLES, 0, vertices.size() / 5);

            glBindVertexArray(0);
            glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_2D, 0);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, 0);

            glBindFramebuffer(GL_FRAMEBUFFER, 0);
        }
        checkError("CubemapCapturer::draw(): after draw: ");
    }

    void CubemapCapturer::InitFrameBuffers()
    {
        GLuint attachments_[6] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2,  GL_COLOR_ATTACHMENT3, GL_COLOR_ATTACHMENT4, GL_COLOR_ATTACHMENT5};
        for(size_t i = 0; i < 6; ++i)
            attachments[i] = attachments_[i];

        glGenFramebuffers(6, cubemapFrameBuffers);

        for(size_t i = 0; i < 6; ++i)
        {
            glBindFramebuffer(GL_FRAMEBUFFER, cubemapFrameBuffers[i]);

            // create a color attachment texture
            glGenTextures(1, &colorAttachments[i]);
            glBindTexture(GL_TEXTURE_2D, colorAttachments[i]);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, SIDELENGTH, SIDELENGTH, 0, GL_RGBA,  GL_UNSIGNED_BYTE, NULL);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorAttachments[i], 0);
            checkError("CubemapCapturer::InitFrameBuffers(): after generate color texture[i]: ");

            // create a depth attachment texture
            glGenTextures(1, &depthAttachments[i]);
            glBindTexture(GL_TEXTURE_2D, depthAttachments[i]);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, SIDELENGTH, SIDELENGTH, 0, GL_RED,  GL_FLOAT, NULL);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, depthAttachments[i], 0);
            checkError("CubemapCapturer::InitFrameBuffers(): after generate depth texture[i]: ");

            glGenRenderbuffers(1, &cubemapRenderBuffer[i]);
            glBindRenderbuffer(GL_RENDERBUFFER, cubemapRenderBuffer[i]);
            glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH32F_STENCIL8, SIDELENGTH, SIDELENGTH); // use a single renderbuffer object for both a depth AND stencil buffer.
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, cubemapRenderBuffer[i]); // now actually attach it

            // now that we actually created the framebuffer and added all attachments we want to check if it is actually complete now
            if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
                printf("OpenglManagerForWarper::GenerateFbos(): ERROR::FRAMEBUFFER:: Framebuffer is not complete!  \n");

            //不用DrawBuffers，则只会画Attachment0所在的颜色附着。
            glDrawBuffers(2, attachments);

            glBindFramebuffer(GL_FRAMEBUFFER, 0);
        }
    }

    void CubemapCapturer::InitShader()
    {
        checkError("CubemapCapturer::InitShader(): before Init shader: ");
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
        shaderForCapturer.init(skyboxVertexCode1, skyboxFragmentCode1);

        checkError("CubemapCapturer::InitShader(): after Init shader: ");
    }

    void CubemapCapturer::InitMvps()
    {
//        modelMat = glm::mat4(1.0f);

        viewMats[0] = glm::rotate(glm::mat4(float(1.0f)), float(glm::radians(180.0f)), glm::vec3(float(1.0f), float(0.0f), float(0.0f)));
        viewMats[1] = glm::rotate(viewMats[0], glm::radians(-90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        viewMats[2] = glm::rotate(viewMats[0], glm::radians(-180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        viewMats[3] = glm::rotate(viewMats[0], glm::radians(-270.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        viewMats[4] = glm::rotate(viewMats[0], glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
        viewMats[5] = glm::rotate(viewMats[0], glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));

        projectionMat = glm::perspective(glm::radians(90.0f), (float)SIDELENGTH / (float)SIDELENGTH, 0.1f, 65500.0f);//0.03m-65m
    }

    void CubemapCapturer::InitTextures()
    {
        glGenFramebuffers(1, &inputFrameBuffer);
        glBindFramebuffer(GL_FRAMEBUFFER, inputFrameBuffer);

        glGenTextures(1, &colorTex2d);
        glBindTexture(GL_TEXTURE_2D, colorTex2d);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, SIDELENGTH, SIDELENGTH, 0, GL_RGBA,  GL_UNSIGNED_BYTE, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorTex2d, 0);
        checkError("CubemapCapturer::InitTextures(): after generate color texture: ");

        //>>>>>>>>>>>>>>>>>>>>>>depth texture attachment<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
        glGenTextures(1, &depthTex2d);
        glBindTexture(GL_TEXTURE_2D, depthTex2d);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, SIDELENGTH, SIDELENGTH, 0, GL_RED,  GL_FLOAT, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, depthTex2d, 0);
        checkError("CubemapCapturer::InitTextures(): after generate depth texture: ");

        glBindTexture(GL_TEXTURE_2D, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void CubemapCapturer::showResults()
    {
        cv::Mat pixel_color(SIDELENGTH, SIDELENGTH, CV_8UC4, cv::Scalar(0, 0, 0, 0));
        cv::Mat pixel_depth(SIDELENGTH, SIDELENGTH, CV_32FC1, cv::Scalar(1.0f, 1.0f, 1.0f, 1.0f));
        for(size_t i = 0; i < 6; ++i)
        {
            glBindFramebuffer(GL_FRAMEBUFFER, cubemapFrameBuffers[i]);

            glReadBuffer ( GL_COLOR_ATTACHMENT0 );
            glReadPixels(0, 0, pixel_color.cols, pixel_color.rows, GL_RGBA, GL_UNSIGNED_BYTE, pixel_color.data);
            checkError("CubemapCapturer::showResults(): after read color buffer: ");

            cv::Mat color_tmp;// = pixel_color.clone();
            cv::cvtColor(pixel_color, color_tmp, cv::COLOR_RGBA2BGR);
            cv::flip(color_tmp, color_tmp, 0);
            cv::imshow("color", color_tmp);

            glReadBuffer ( GL_COLOR_ATTACHMENT1 );
            glReadPixels(0, 0, pixel_depth.cols, pixel_depth.rows, GL_RED, GL_FLOAT, pixel_depth.data);
            checkError("CubemapCapturer::showResults(): after read depth buffer: ");

            double minV, maxV;
            minMaxLoc(pixel_depth, &minV, &maxV);
            printf("CubemapCapturer::showResults():---pixel_depth--- minV, maxV: %lf, %lf\n", minV, maxV);

            cv::flip(pixel_depth, pixel_depth, 0);
            cv::imshow("depth", pixel_depth);

            //unbind pano framebuffer
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            cv::waitKey();
        }
    }

    const GLuint* CubemapCapturer::colorAttatches()
    {
        return colorAttachments;
    }
    const GLuint* CubemapCapturer::depthAttatches()
    {
        return depthAttachments;
    }


}