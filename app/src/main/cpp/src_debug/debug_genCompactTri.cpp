//
// Created by ale on 19-3-24.
//

//
// Created by ale on 19-3-20.
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
#include "genCompactTri.h"

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

    //***********mesh simplification and exporting(to .obj+.mtl+texture.jpg)**********//
    cout<<"start mesh simplification and exporting"<<endl;
    genCompactTri(root_dir, pano);
    cout<<"finish mesh simplification and exporting"<<endl;

    return 0;
}