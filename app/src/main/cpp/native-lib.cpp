#ifndef NATIVE0618_NATIVE_LIB_CPP
#define NATIVE0618_NATIVE_LIB_CPP


#include <jni.h>
#include <android/bitmap.h>
#include <android/log.h>


#include "debug_main.h"
#include "3d_results.h"
#include "viewer/Renderer.h"


#ifdef __cplusplus
extern "C"
{
#endif

JNIEXPORT jint JNICALL
Java_i3d_native0701_MainActivity_getImage(JNIEnv *env, jclass, jobject bitmap, jstring javaString) {

    //cv::Mat image = cv::imread("/storage/emulated/0/DCIM/Camera/test.png", cv::IMREAD_UNCHANGED);
    using namespace cv;
    using namespace std;
    using namespace i3d;

    static int index = 0;
    static string root_dir;
    static vector<Frame> frames;

    jboolean isCopy;
    const char* str;
    str = (*env).GetStringUTFChars(javaString, &isCopy);
    string root_dir_(str);
    delete[] str;

    bool debug_glsurfaceview = true;
    if(!debug_glsurfaceview)
    {
        vertices = {
                //front face
                -1, 1, 1, 0.25, 0.33,
                1, 1, 1, 0.5,  0.33,
                1, -1, 1, 0.5, 0.66,
//                1, -1, 1, 0.5, 0.66,
//                -1, -1, 1, 0.25, 0.66,
//                -1, 1, 1, 0.25, 0.33,
//                //right face
//                1, 1, 1, 0.5, 0.33,
//                1, 1, -1, 0.75, 0.33,
//                1, -1, -1, 0.75, 0.66,
//                1, -1, -1, 0.75, 0.66,
//                1, -1, 1, 0.5, 0.66,
//                1, 1, 1, 0.5, 0.33,
//                //back face
//                1, 1, -1, 0.75, 0.33,
//                -1, 1, -1, 1.0, 0.33,
//                -1, -1, -1, 1.0, 0.66,
//                -1, -1, -1, 1.0, 0.66,
//                1, -1, -1, 0.75, 0.66,
//                1, 1, -1, 0.75, 0.33,
//                //left face
//                -1, 1, -1, 0.0, 0.33,
//                -1, 1, 1, 0.25, 0.33,
//                -1, -1, 1, 0.25, 0.66,
//                -1, -1, 1, 0.25, 0.66,
//                -1, -1, -1, 0.0, 0.66,
//                -1, 1, -1, 0.0, 0.33,
//                //top face
//                -1, -1, 1, 0.25, 0.66,
//                1, -1, 1, 0.5, 0.66,
//                1, -1, -1, 0.5, 1.0,
//                1, -1, -1, 0.5, 1.0,
//                -1, -1, -1, 0.25, 1.0,
//                -1, -1, 1, 0.25, 0.66,
//                //bottom face
//                -1, 1, -1, 0.25, 0.0,
//                1, 1, -1, 0.5, 0.0,
//                1, 1, 1, 0.5, 0.33,
//                1, 1, 1, 0.5, 0.33,
//                -1, 1, 1, 0.25, 0.33,
//                -1, 1, -1, 0.25, 0.0,
        };

        texture = cv::Mat(800, 800, CV_8UC3, Scalar(0, 0, 255, 0));
        if(frames.empty())
        {
            Frame frame;
            frame.image = texture.clone();
            frames.push_back(frame);
        }

    }
    else
    if(root_dir.empty() || root_dir.compare(root_dir_))
    {
        root_dir.assign(root_dir_.begin(), root_dir_.end());
        LOGW("debug_initInputData start.........");
        debug_main(root_dir, frames, vertices, texture);
        LOGW("debug_initInputData finished.............");
    }

    index = (index+1)%frames.size();
    Mat image = frames[index].image;

    image.convertTo(image, CV_8UC4);
    cv::cvtColor(image, image, cv::COLOR_BGRA2RGBA);
    //h,w,type
    cv::Mat bmp(image.rows, image.cols, CV_8UC4);
    AndroidBitmap_lockPixels(env, bitmap, (void **)&bmp.data);
    image.copyTo(bmp);
    AndroidBitmap_unlockPixels(env, bitmap);



    return frames.size();
}





//viewer
Renderer renderer;

JNIEXPORT void JNICALL
Java_i3d_native0701_GLUtils_initialize(JNIEnv *env, jclass)
{
    renderer.initialize(vertices, texture);
}

JNIEXPORT jboolean JNICALL
Java_i3d_native0701_GLUtils_isInitialized(JNIEnv *env, jclass)
{
    return renderer.isInitialized();
}

JNIEXPORT void JNICALL
Java_i3d_native0701_GLUtils_reshape(JNIEnv *env, jclass, jint width, jint height)
{
    renderer.reshape(width, height);
}

JNIEXPORT void JNICALL
Java_i3d_native0701_GLUtils_display(JNIEnv *env, jclass)
{
    renderer.display();
}

JNIEXPORT void JNICALL
Java_i3d_native0701_GLUtils_translateCamera(JNIEnv *env, jclass, jint camera_Movement, jfloat step)
{
    renderer.translateCamera(camera_Movement, step);
}

JNIEXPORT void JNICALL
Java_i3d_native0701_GLUtils_rotateCamera(JNIEnv *env, jclass, jfloat xoffset, jfloat yoffset)
{
    renderer.rotateCamera(xoffset, yoffset);
}

JNIEXPORT void JNICALL
Java_i3d_native0701_GLUtils_resetPose(JNIEnv *env, jclass)
{
    renderer.resetCamera();
}





#ifdef __cplusplus
}
#endif//extern "C"




#endif //NATIVE0618_NATIVE_LIB_CPP