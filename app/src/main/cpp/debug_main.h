//
// Created by ale on 19-7-1.
//

#ifndef NATIVE0701_DEBUG_INITINPUTDATA_H
#define NATIVE0701_DEBUG_INITINPUTDATA_H
#include <iostream>
#include <string>

#include "Frame.h"
#include "i3d.h"

#include "1FrameReader.h"

#include "initInputData.h"
#include "genFeatures.h"
#include "genInitialGraph.h"
#include "genKeyFrames.h"
#include "genGlobalByMst.h"

#include "RigidProblem.h"
#include "DeformableProblem.h"

#include "6PanoramaCapturer.h"
#include "stitchAllPanos.h"
#include "genCompactTri.h"


int debug_initInputData(std::string root_dir, std::vector<i3d::Frame> &kframes, i3d::Intrinsics& intrinsics)
{
    using namespace std;
    using namespace cv;
    using namespace i3d;

    LOGW("                                       ");
    LOGW("                                       ");
//    std::vector<i3d::Frame> frames;
    LOGW("start initFrames");
//    Intrinsics intrinsics;
//    initFrames(root_dir, kframes, intrinsics);
    m3d::FrameReader frameReader(root_dir, kframes, intrinsics);
    LOGW("finish initFrames");


    LOGW("                                       ");
    LOGW("                                       ");
    //*********compute feature points(corners and descriptors)***************//
    LOGW("start computeFeatures...");
    computeFeatures(kframes);//opencv内部利用了多线程
    LOGE("kframes.size():  %d.", kframes.size());

    LOGW("finish computeFeatures");
    return 0;
}


int debug_estimatePoses(std::vector<i3d::Frame> &frames, i3d::Intrinsics& intrinsics)
{
    using namespace std;
    using namespace cv;
    using namespace i3d;

    LOGW("                                       ");
    LOGW("                                       ");
    //*********determine match pairs*************************************//
    LOGW("start genInitGraph...");
    vector<Edge> edges;
    genInitialGraph(frames, intrinsics, edges);//有并发提升空间
//    genRelativePoses(frames, intrinsics, edges);//有并发提升空间
    LOGW("finish genInitGraph");

    LOGW("                                       ");
    LOGW("                                       ");
    //*********generate key frames*******************************//
    //delete standalone frames(those without valuable edges)
    LOGW("start genKeyFrames1...");
    vector<Frame> tkframes;
    genKeyFrames(frames, edges, tkframes);
    LOGE("kframes.size():  %d.", tkframes.size());
    LOGW("finish genKeyFrames1...");

    LOGW("                                       ");
    LOGW("                                       ");
    //*********g2o 粗糙全局位恣求解********************************//
    LOGW("start genGlobalByMst...");
    vector<Edge> kedges;
    genGlobalByMst(edges, tkframes, kedges);
    //generate the key frames that contribute to connected graph.
    vector<Frame> kframes;
    LOGW("start genKeyFrames2...");
    genKeyFrames(tkframes, kedges, kframes);
    LOGE("kframes.size():  %d.", kframes.size());
    LOGW("finish genKeyFrames2...");


    LOGW("finish genGlobalByMst...");
    //****更快的、更好用的粗糙全局位恣求解，可以用最小耗费生成树算法******//

    LOGW("                                       ");
    LOGW("                                       ");
    //*********ceres lib 最小二乘法优化 *******************************//
    LOGW("start solveProblem...");
//    char length[5] = {0};
//    sprintf(length, "%2d", kframes.size());
    string str("kframes.size(): ");
    str.append(to_string(kframes.size()));
    LOGW("         %s    ", str.c_str());

    if(kframes[0].image.cols >= 1920)
        i3d_rp::RigidProblem rigidProblem(kframes, kedges, intrinsics);
    else
        i3d_dp::DeformableProblem<2,2> deformableProblemWxh(kframes, kedges, intrinsics);
    LOGW("finish solveProblem...");

    frames = kframes;

    return 0;
}

int debug_warpToPanoramas(std::vector<i3d::Frame> &kframes, i3d::Intrinsics& intrinsics)
{

    LOGW("                                       ");
    LOGW("                                       ");
    //*********gles warp perspective frames to panorama   *************//
    LOGW("start warper4Android...");
    m3d::PanoramaCapturer panoramaCapturer(kframes, intrinsics);
//    Warper4Android warper4Android(kframes, intrinsics);
    LOGW("finish warper4Android...");

    return 0;
}


int debug_stitchAllPanos(std::vector<i3d::Frame> &kframes, i3d::Frame& pano)
{
    LOGW("                                       ");
    LOGW("                                       ");
    //*********gles warp perspective frames to panorama   *************//
    LOGW("start stitchAllPanos...");
    stitchAllPanos(kframes, pano);
    LOGW("finish stitchAllPanos...");

    return 0;
}


//int debug_main(std::string root_dir, std::vector<i3d::Frame> &kframes, std::vector<float>& vertices, cv::Mat& texture)
//{
//    using namespace std;
//    using namespace cv;
//    using namespace i3d;
//
//    Intrinsics intrinsics;
//    debug_initInputData(root_dir, kframes, intrinsics);
//
//
//    debug_estimatePoses(kframes, intrinsics);
//
//
//    debug_warpToPanoramas(kframes, intrinsics);
//
//
//
//    Frame pano;
//    debug_stitchAllPanos(kframes, pano);
//
//    LOGE("debug_main(): save pano result.");
//    string res_pano_color = root_dir + string("/debug_res_pano/pano_color.jpg");
//    imwrite(res_pano_color, pano.pano_image);
//
////    string tri_out_dir = root_dir + string("/debug_compact_tri");
////    genCompactTri(tri_out_dir, pano);//icount==30
//
////    Mat finalTexture;
////    vector<float> finalArray;
//    LOGE("debug_main(): save texture result.");
//    genCompactTri(pano, texture, vertices);
//    string texture_name = root_dir + string("/debug_compact_tri/texture.jpg");
//    imwrite(texture_name, texture);
//
//    return 0;
//}

#endif //NATIVE0701_DEBUG_INITINPUTDATA_H
