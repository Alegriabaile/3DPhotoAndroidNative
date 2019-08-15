//
// Created by ale on 19-7-1.
//

#ifndef NATIVE0701_DEBUG_INITINPUTDATA_H
#define NATIVE0701_DEBUG_INITINPUTDATA_H
#include <iostream>
#include <string>
#include <include/Warper4Android.h>

#include "Frame.h"
#include "i3d.h"
#include "initInputData.h"
#include "genFeatures.h"
//#include "genRelativePoses.h"
#include "genInitialGraph.h"
#include "genKeyFrames.h"
#include "genGlobalByMst.h"

#include "DProblem1.h"
//#include "Pers2PanoWarper.h"
#include "stitchAllPanos.h"
#include "genCompactTri.h"


int debug_initInputData(std::string root_dir, std::vector<i3d::Frame> &kframes, i3d::Intrinsics& intrinsics)
{
    using namespace std;
    using namespace cv;
    using namespace i3d;

    LOGW("[                                     ]");
    LOGW("[                                     ]");
//    std::vector<i3d::Frame> frames;
    LOGW("start initFrames");
//    Intrinsics intrinsics;
    initFrames(root_dir, kframes, intrinsics);
    LOGW("finish initFrames");
    return 0;
}


int debug_estimatePoses(std::vector<i3d::Frame> &frames, i3d::Intrinsics& intrinsics)
{
    using namespace std;
    using namespace cv;
    using namespace i3d;

    LOGW("[                                     ]");
    LOGW("[                                     ]");
    //*********compute feature points(corners and descriptors)***************//
    LOGW("start computeFeatures...");
    computeFeatures(frames);//opencv内部利用了多线程
    LOGW("finish computeFeatures");

    LOGW("[                                     ]");
    LOGW("[                                     ]");
    //*********determine match pairs*************************************//
    LOGW("start genInitGraph...");
    vector<Edge> edges;
    genInitGraph(frames, intrinsics, edges);//有并发提升空间
//    genRelativePoses(frames, intrinsics, edges);//有并发提升空间
    LOGW("finish genInitGraph");

    LOGW("[                                     ]");
    LOGW("[                                     ]");
    //*********generate key frames*******************************//
    //delete standalone frames(those without valuable edges)
    LOGW("start genKeyFrames1...");
    vector<Frame> tkframes;
    genKeyFrames(frames, edges, tkframes);
    LOGW("finish genKeyFrames1...");

    LOGW("[                                     ]");
    LOGW("[                                     ]");
    //*********g2o 粗糙全局位恣求解********************************//
    LOGW("start genGlobalByMst...");
    vector<Edge> kedges;
    genGlobalByMst(edges, tkframes, kedges);
    //generate the key frames that contribute to connected graph.
    vector<Frame> kframes;
    LOGW("start genKeyFrames2...");
    genKeyFrames(tkframes, kedges, kframes);
    LOGW("finish genKeyFrames2...");
    LOGW("finish genGlobalByMst...");
    //****更快的、更好用的粗糙全局位恣求解，可以用最小耗费生成树算法******//

    LOGW("[                                     ]");
    LOGW("[                                     ]");
    //*********ceres lib 最小二乘法优化 *******************************//
    LOGW("start solveProblem...");
//    char length[5] = {0};
//    sprintf(length, "%2d", kframes.size());
    string str("kframes.size(): ");
    str.append(to_string(kframes.size()));
    LOGW("         %s    ", str.c_str());
    DProblem1 rigidProblem(kframes, kedges, intrinsics, 2, 2, 10);
    LOGW("finish solveProblem...");

    frames = kframes;

    return 0;
}

int debug_warpToPanoramas(std::vector<i3d::Frame> &kframes, i3d::Intrinsics& intrinsics)
{

    LOGW("[                                     ]");
    LOGW("[                                     ]");
    //*********gles warp perspective frames to panorama   *************//
    LOGW("start warper4Android...");
    Warper4Android warper4Android(kframes, intrinsics);
    LOGW("finish warper4Android...");

    return 0;
}


int debug_stitchAllPanos(std::vector<i3d::Frame> &kframes, i3d::Frame& pano)
{
    LOGW("[                                     ]");
    LOGW("[                                     ]");
    //*********gles warp perspective frames to panorama   *************//
    LOGW("start stitchAllPanos...");
    stitchAllPanos(kframes, pano);
    LOGW("finish stitchAllPanos...");

    return 0;
}


int debug_genCompactTri()
{

    return 0;
}


int debug_main(std::string root_dir, std::vector<i3d::Frame> &kframes, std::vector<float>& vertices, cv::Mat& texture)
{
    using namespace std;
    using namespace cv;
    using namespace i3d;

    Intrinsics intrinsics;
    debug_initInputData(root_dir, kframes, intrinsics);


    debug_estimatePoses(kframes, intrinsics);


    debug_warpToPanoramas(kframes, intrinsics);



    Frame pano;
    debug_stitchAllPanos(kframes, pano);

    string res_pano_color = root_dir + string("/debug_res_pano/pano_color.jpg");
    imwrite(res_pano_color, pano.pano_image);

//    string tri_out_dir = root_dir + string("/debug_compact_tri");
//    genCompactTri(tri_out_dir, pano);//icount==30

//    Mat finalTexture;
//    vector<float> finalArray;
    genCompactTri(pano, texture, vertices);

    return 0;
}

#endif //NATIVE0701_DEBUG_INITINPUTDATA_H
