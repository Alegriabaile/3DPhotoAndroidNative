//
// Created by ale on 19-7-25.
//

#include "WarperEglManager.h"

static const int width = 500;
static const int height = 500;

int WarperEglManager::InitializeEGL()
{
    EGLint numConfigs;
    EGLBoolean SUCCEED = EGL_FALSE;

    /* get an EGL display connection */
    eglDisplay = eglGetDisplay(EGL_DEFAULT_DISPLAY);

    if( eglDisplay == EGL_NO_DISPLAY)
        LOGE(" eglGetDisplay(): Unable to open connection to local windowing system");



    /* initialize the EGL display connection */
    EGLint majorVersion, minorVersion;
    SUCCEED = eglInitialize(eglDisplay, &majorVersion, &minorVersion);
    LOGE("EGL version: %d.%d", majorVersion, minorVersion);

    if(SUCCEED)
        ;//LOGI(" eglInitialize(): initialize EGL successful");
    else
        LOGE(" eglInitialize(): Unable to initialize EGL");



    /* get an appropriate EGL frame buffer configuration */
    static EGLint const configAttributes[] = {
            EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT_KHR,// very important!
            EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
            EGL_RED_SIZE, 8,    EGL_GREEN_SIZE, 8,    EGL_BLUE_SIZE, 8,
            EGL_NONE };

    SUCCEED = SUCCEED && eglChooseConfig(eglDisplay, configAttributes, &eglConfig, 1, &numConfigs);

    if(SUCCEED)
        ;//LOGI(" eglChooseConfig(): successful");
    else
        LOGE(" eglChooseConfig(): Unable to choose config");



    /* create an EGL rendering context */
    const EGLint contextAttributes[] = { EGL_CONTEXT_CLIENT_VERSION, 3,    EGL_NONE};

    eglContext = eglCreateContext(eglDisplay, eglConfig, EGL_NO_CONTEXT, contextAttributes);

    if( eglContext == EGL_NO_CONTEXT)
        LOGE(" eglCreateContext(): Unable to create context");




    /* create an EGL pixel buffer surface */
    const EGLint surfaceAttributes[] = { EGL_WIDTH,width,    EGL_HEIGHT,height,    EGL_NONE };

    eglSurface = eglCreatePbufferSurface(eglDisplay, eglConfig, surfaceAttributes);

    if( eglSurface == EGL_NO_SURFACE)
        LOGE(" eglCreatePbufferSurface(): Unable to create context");




    /* connect the context to the surface */
    SUCCEED = SUCCEED && eglMakeCurrent(eglDisplay, eglSurface, eglSurface, eglContext);
    if(SUCCEED)
        ;//LOGI(" eglMakeCurrent(): successful");
    else
        LOGE(" eglMakeCurrent(): Unable to make current");

    if(SUCCEED)
        return true;

    return false;
}


int WarperEglManager::ReleaseEGL()
{
    eglMakeCurrent(eglDisplay, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroyContext(eglDisplay, eglContext);
    eglDestroySurface(eglDisplay, eglSurface);
    eglTerminate(eglDisplay);

    eglDisplay = EGL_NO_DISPLAY;
    eglSurface = EGL_NO_SURFACE;
    eglContext = EGL_NO_CONTEXT;
    return true;
}

WarperEglManager::WarperEglManager()
{
    InitializeEGL();
}

WarperEglManager::~WarperEglManager()
{
    ReleaseEGL();
}

