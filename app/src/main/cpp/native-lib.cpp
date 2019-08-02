#ifndef NATIVE0618_NATIVE_LIB_CPP
#define NATIVE0618_NATIVE_LIB_CPP

#include <jni.h>
#include <android/bitmap.h>
#include <android/log.h>
#include "debug_main.h"

#ifndef TAG_MY_LOG
#define TAG_MY_LOG

#define TAG "...................TAG_MY_LOG..................."
#define LOGV(...) __android_log_print(ANDROID_LOG_VERBOSE, TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)
//#undef TAG

#endif



#ifdef __cplusplus
extern "C"
{
#endif

JNIEXPORT jint JNICALL
Java_i3d_native0701_MainActivity_getImage(JNIEnv *env, jclass, jobject bitmap) {

    //cv::Mat image = cv::imread("/storage/emulated/0/DCIM/Camera/test.png", cv::IMREAD_UNCHANGED);
    using namespace cv;
    using namespace std;
    using namespace i3d;

    static bool isInit = true;
    static int index = 0;
    static vector<Frame> frames;
    string root_dir("/sdcard/000i3d2");
    if(isInit)
    {
        LOGW("debug_initInputData start.........");
        debug_main(root_dir, frames);
        isInit = false;
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

#ifdef __cplusplus
}
#endif


#endif //NATIVE0618_NATIVE_LIB_CPP