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
Java_i3d_native0701_MainActivity_processI3d(JNIEnv *env, jclass clazz, jstring javaString) {

    //cv::Mat image = cv::imread("/storage/emulated/0/DCIM/Camera/test.png", cv::IMREAD_UNCHANGED);
    using namespace cv;
    using namespace std;
    using namespace i3d;


    static string root_dir;

    jboolean isCopy;
    const char* str;
    str = (*env).GetStringUTFChars(javaString, &isCopy);
    string root_dir_(str);
    delete[] str;

    bool debug_glsurfaceview = true;
    if(!debug_glsurfaceview)
    {
        /*
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
         */
    }
    else
    if(root_dir.empty() || root_dir.compare(root_dir_))
    {
        root_dir.assign(root_dir_.begin(), root_dir_.end());
        LOGW("debug_initInputData start.........");
        debug_main(root_dir, frames, vertices, texture);
        LOGW("debug_initInputData finished.............");
    }

    if(frames.empty())
        return 0;

    //h,w,type

//
//
//    LOGE("JNI getImgSize():: start GetFieldID()");
//    jfieldID  nameFieldId_cols, nameFieldId_rows ;
////    jclass cls = env->GetObjectClass(obj);  //获得Java层该对象实例的类引用，即HelloJNI类引用
//    nameFieldId_cols = (*env).GetFieldID(clazz , "COLS" , "I"); //获得属性句柄
//    nameFieldId_rows = (*env).GetFieldID(clazz , "ROWS" , "I"); //获得属性句柄
//    LOGE("JNI getImgSize():: finish GetFieldID()");
//
//    if(nameFieldId_cols == NULL || nameFieldId_rows == NULL)
//    {
//        LOGE("JNI can't GetFieldID COLS and ROWS");
//        return frames.size();
//    }
//
//    LOGE("JNI getImgSize():: start GetFieldID()");
//    (*env).SetStaticIntField(clazz, nameFieldId_cols, image.cols);
//    (*env).SetStaticIntField(clazz, nameFieldId_rows, image.rows);


    return frames.size();
}



JNIEXPORT jint JNICALL
Java_i3d_native0701_MainActivity_getImageW(JNIEnv *env, jclass clazz) {

    if(frames.empty())
        return 0;

    cv::Mat image = frames[0].image;
    return image.cols;
}

JNIEXPORT jint JNICALL
Java_i3d_native0701_MainActivity_getImageH(JNIEnv *env, jclass clazz) {

    if(frames.empty())
        return 0;
    cv::Mat image = frames[0].image;
    return image.rows;
}

JNIEXPORT jint JNICALL
Java_i3d_native0701_MainActivity_getImage(JNIEnv *env, jclass clazz, jobject bitmap) {

    //cv::Mat image = cv::imread("/storage/emulated/0/DCIM/Camera/test.png", cv::IMREAD_UNCHANGED);
    using namespace cv;
    using namespace std;
    using namespace i3d;

    static int index = -1;

    if(frames.empty())
        return 0;

    index = (index+1)%frames.size();
    Mat image = frames[index].image;

    image.convertTo(image, CV_8UC4);
    cv::cvtColor(image, image, cv::COLOR_BGRA2RGBA);
    //h,w,type
    cv::Mat img_show = image;
//    resize(image, img_show, Size(640, 480));

    cv::Mat bmp(img_show.rows, img_show.cols, CV_8UC4);
    AndroidBitmap_lockPixels(env, bitmap, (void **)&bmp.data);
    img_show.copyTo(bmp);
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
Java_i3d_native0701_GLUtils_zoomCamera(JNIEnv *env, jclass, jfloat zoffset)
{
    renderer.zoomCamera(zoffset);
}

JNIEXPORT void JNICALL
Java_i3d_native0701_GLUtils_resetPose(JNIEnv *env, jclass)
{
    renderer.resetCamera();
}

JNIEXPORT void JNICALL
Java_i3d_native0701_GLUtils_resetR(JNIEnv *env, jclass)
{
    renderer.resetR();
}

JNIEXPORT void JNICALL
Java_i3d_native0701_GLUtils_resetT(JNIEnv *env, jclass)
{
    renderer.resetT();
}



#ifdef __cplusplus
}
#endif//extern "C"




#endif //NATIVE0618_NATIVE_LIB_CPP