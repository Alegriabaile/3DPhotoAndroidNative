//
// Created by ale on 19-7-25.
//

#ifndef NATIVE0701_WARPEREGLMANAGER_H
#define NATIVE0701_WARPEREGLMANAGER_H

#include "i3d.h"
#include "EGL/egl.h"
#include "EGL/eglplatform.h"
#include "EGL/eglext.h"

class WarperEglManager {
private:

    EGLConfig eglConfig;
    EGLSurface eglSurface;
    EGLContext eglContext;
    EGLDisplay eglDisplay;

    int InitializeEGL();
    int ReleaseEGL();

public:
    WarperEglManager();
    ~WarperEglManager();
};


#endif //NATIVE0701_WARPEREGLMANAGER_H
