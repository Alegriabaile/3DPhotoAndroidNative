//
// Created by ale on 18-12-18.
//

#ifndef I3D_I3D_H
#define I3D_I3D_H

#include <vector>
#include "opencv2/opencv.hpp"

namespace i3d
{
    struct Intrinsics
    {
        double f;
        double cx;
        double cy;
    };

    class Edge
    {
    public:
        int src, dst;
        double rx, ry, rz, tx, ty, tz;
        double cost;//Rt偏移量，越小说明越可靠
        std::vector<cv::Point2f> psrc, pdst;

        Edge(){}
        Edge(const i3d::Edge& _edge)
        : src(_edge.src), dst(_edge.dst)
        , rx(_edge.rx), ry(_edge.ry), rz(_edge.rz)
        , tx(_edge.tx), ty(_edge.ty), tz(_edge.tz)
        , psrc(_edge.psrc), pdst(_edge.pdst)
        , cost(_edge.cost) {}

    };


    const uint PANO_H = 2048;
    const uint PANO_W = PANO_H*2;

    const double MAX_DEPTH_VALUE = 10000;

    //const uint PANO_W = 2048;
    //const uint PANO_H = 1024;
}

#include <android/log.h>

//DEBUG IN ANDROID
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

#include <string>
#include <sstream>
template <typename T>
std::string to_string(T value)
{
    std::ostringstream os ;
    os << value ;
    return os.str() ;
}

#endif //I3D_I3D_H
