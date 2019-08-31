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
Java_i3d_native0701_ProcessTask_processI3d(JNIEnv *env, jobject obj, jstring javaString) {

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

    if(root_dir.empty() || root_dir.compare(root_dir_))
    {
        root_dir.assign(root_dir_.begin(), root_dir_.end());
//        LOGW("debug_initInputData start.........");
//        debug_main(root_dir, frames, vertices, texture);
//        LOGW("debug_initInputData finished.............");


        jclass claz = env->GetObjectClass(obj);
        jmethodID jmethodID1=env->GetMethodID(claz,"notifyMainActivity","()I");

        Intrinsics intrinsics;
        debug_initInputData(root_dir, frames, intrinsics);
        env->CallIntMethod(obj, jmethodID1);

        debug_estimatePoses(frames, intrinsics);
        env->CallIntMethod(obj, jmethodID1);

        debug_warpToPanoramas(frames, intrinsics);
        env->CallIntMethod(obj, jmethodID1);


        Frame pano;
        debug_stitchAllPanos(frames, pano);
        env->CallIntMethod(obj, jmethodID1);

        genCompactTri(pano, texture, vertices);
        env->CallIntMethod(obj, jmethodID1);
    }

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