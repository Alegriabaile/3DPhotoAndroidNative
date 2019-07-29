//
// Created by ale on 19-3-14.
//

//
// Created by ale on 19-2-28.
//

//
// Created by ale on 19-2-19.
//

//
// Created by ale on 19-2-17.
//


#include <iostream>
#include <genRts.h>
#include "Frame.h"
#include "i3d.h"

#include "initInputData.h"

#include "genFeatures.h"
#include "genInitialGraph.h"
#include "genKeyFrames.h"
#include "genGlobalByMst.h"
#include "DeformableProblem.h"
#include "warpToPano.h"
#include "stitchAllPanos.h"
#include "genObjFromPano.h"
#include "genObjFromPixels.h"

using namespace i3d;
using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    //在argv.txt中设置输入数据源路径，从而只需要更改txt文件（不需要再次编译源程序）即可测试其他数据源的效果
    cout<<"start loading input/output path"<<endl;
    ifstream fin( "argv.txt" );
    if (!fin)
    {
        cerr<<"parameter file does not exist."<<endl;
        return -1;
    }
    std::string root_dir;
    getline( fin, root_dir );
    fin.close();
    cout<<"finish loading input/output path"<<endl;

    //**********read input data*************//
    cout<<"start init..."<<endl;
    vector<Frame> frames;
    initFrames(root_dir, frames);
    Intrinsics intrinsics;
    initIntrinsics(root_dir, frames, intrinsics);
    cout<<"finish init."<<endl;

    //*********compute feature points(corners and descriptors)***************//
    cout<<"start computeFeatures..."<<endl;
    computeFeatures(frames);//opencv内部利用了多线程
    cout<<"finish computeFeatures"<<endl;

    //*********determine match pairs*************************************//
    cout<<"start genInitGraph..."<<endl;
    vector<Edge> edges;
    genInitGraph(frames, intrinsics, edges);//有并发提升空间
    cout<<"finish genInitGraph"<<endl;

    //*********generate key frames*******************************//
    //delete standalone frames(those without valuable edges)
    cout<<"start genKeyFrames1..."<<endl;
    vector<Frame> tkframes;
    genKeyFrames(frames, edges, tkframes);
    cout<<"finish genKeyFrames1..."<<endl;

    //****初始全局位恣求解，使用最小耗费生成树算法******//
    cout<<"start genGlobalByMst..."<<endl;
    vector<Edge> kedges;
    genGlobalByMst(edges, tkframes, kedges);
    //generate the key frames that contribute to connected graph.
    vector<Frame> kframes;
    genKeyFrames(tkframes, kedges, kframes);
    cout<<"finish genGlobalByMst..."<<endl;

    //*********ceres lib 最小二乘法优化 *******************************//
    cout<<"start solveProblem..."<<endl;
    cout<<"kframes.size(): "<<kframes.size()<<endl;
    google::InitGoogleLogging(argv[0]);
    //*********ceres lib 最小二乘法优化 *******************************//
    cout<<"start solveProblem..."<<endl;
    NaiveRtProblem naiveRtProblem;
    initRts(kframes, kedges, intrinsics, naiveRtProblem);
    genRts(naiveRtProblem);
    getOptimizedRts(naiveRtProblem, kframes);
    cout<<"finish solveProblem..."<<endl;
    //DeformableProblem<2,2> deformableProblemWxh(kframes, kedges, intrinsics);
    //cout<<"finish solveProblem..."<<endl;

    //***********warp perspective image to panorama with rasterization pipeline**************//
    cout<<"start warpToPanos..."<<endl;
    warpToPanos(kframes, intrinsics);
    cout<<"finish warpToPanos..."<<endl;

    //***********stitch all panoramas to one frame*************************//
    cout<<"start stitchAllPanos..."<<endl;
    Frame pano;
    stitchAllPanos(kframes, pano);
    cout<<"finish stitchAllPanos..."<<endl;

    //***********mesh simplification and exporting(to .obj+.mtl+texture.jpg)
    cout<<"start mesh simplification and exporting"<<endl;
    //save pano rgbd data to root_dir
    Mat pano_image, pano_depth_, pano_depth;
    pano_depth_ = pano.pano_depth.clone();//(cv::Range(pano.minh, pano.maxh+1), cv::Range(pano.minw, pano.maxw+1)).clone();
    pano_image = pano.pano_image.clone();//(cv::Range(pano.minh, pano.maxh+1), cv::Range(pano.minw, pano.maxw+1)).clone();
    pano_depth_.convertTo(pano_depth, CV_16UC1);
    vector<int> compression_params_jpg;
    compression_params_jpg.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params_jpg.push_back(100); //
    imwrite(root_dir+"/pano_result.jpg", pano_image, compression_params_jpg); //保存图片</span>
    vector<int> compression_params_png;
    compression_params_png.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params_png.push_back(0); // 无压缩png.
    imwrite(root_dir+"/pano_result.png", pano_depth, compression_params_png); //保存图片</span>

    //generate .obj .mtl texture.jpg to root_dir
    genNormalMap(pano);
    //genObjFromPano(root_dir, pano);
    genObjFromPixels(root_dir, pano);

    cout<<"finish mesh simplification and exporting"<<endl;

    double maxVal, minVal;
    minMaxLoc(pano.pano_depth, &minVal, &maxVal);
    int count = 0;
    for(int h=0; h<pano.pano_depth.rows; ++h)
    {
        for(int w=0; w<pano.pano_depth.cols; ++w)
        {
            if(pano.pano_depth.at<float>(h,w) < 1e-6f)
                pano.pano_depth.at<float>(h,w) = maxVal/2.0;
            //if(pano.pano_image.at<Vec3b>(h,w)[0] == 0)
            //    pano.pano_image.at<Vec3b>(h,w) = Vec3b(128,128,128);
            if(pano.pano_depth.at<float>(h,w)>196.0f && pano.pano_depth.at<float>(h,w)<200.0f)
                count++;
        }
    }
    cout<<"count: "<<count<<endl;
    //double minVal, maxVal;
    minMaxLoc(pano.pano_depth, &minVal, &maxVal);
    pano.pano_depth = (pano.pano_depth-minVal)/(maxVal-minVal);
    imshow("pano_depth_normalized", pano.pano_depth);
    waitKey();

    cout<<"minVal, maxVal: "<<minVal<<", "<<maxVal<<endl;
    cout<<"minh, minw, maxh, maxw: "<<endl;
    cout<<pano.minh<<", "<<pano.minw<<", "<<pano.maxh<<", "<<pano.maxw<<endl;

    return 0;
}