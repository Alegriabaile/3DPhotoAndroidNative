//
// Created by ale on 19-5-14.
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
#include "Frame.h"
#include "i3d.h"

#include "initInputData.h"

#include "genFeatures.h"
#include "genRelativePoses.h"
#include "genKeyFrames.h"
#include "genGlobalByMst.h"
#include "DeformableProblem.h"
#include "DProblem2.h"
#include "RProblem2.h"
#include "Pers2PanoWarper.h"
#include "stitchAllPanos.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv/cxeigen.hpp>
#include <opencv2/core/eigen.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

using namespace i3d;
using namespace cv;
using namespace std;
using namespace pcl;

static void BackprojectToPano(Frame& frame, Intrinsics& intrinsics, PointCloud<PointXYZRGBA>::Ptr pc)
{
    //Rotate and Translate matrix.
    cv::Mat R = (Mat_<double>(3, 3));
    cv::Mat rvec = (Mat_<double>(3, 1) << frame.rx, frame.ry, frame.rz);
    cv::Rodrigues(rvec, R);
    double tx = frame.tx;
    double ty = frame.ty;
    double tz = frame.tz;
    int rows = frame.depth.rows;
    int cols = frame.depth.cols;
    Intrinsics _intrinsics;
    _intrinsics.cx = (float)frame.depth.cols/2;
    _intrinsics.cy = (float)frame.depth.rows/2;
    _intrinsics.f = intrinsics.f*_intrinsics.cx/intrinsics.cx;
    for (int row = 0; row < rows; ++row)
        for (int col = 0; col < cols; ++col)
        {
            double d = frame.depth.ptr<float>(row)[col];
            if(d>=150 || d<0)
            {
                d = 160;
                frame.depth.ptr<float>(row)[col] = d;
            }
            if (d > 0 && d < 300.0f)
            {
                double x = (col - _intrinsics.cx) * d / _intrinsics.f;
                double y = (row - _intrinsics.cy) * d / _intrinsics.f;
                double z = d;
                double t_x = R.at<double>(0, 0) * x + R.at<double>(0, 1) * y + R.at<double>(0, 2) * z + tx;
                double t_y = R.at<double>(1, 0) * x + R.at<double>(1, 1) * y + R.at<double>(1, 2) * z + ty;
                double t_z = R.at<double>(2, 0) * x + R.at<double>(2, 1) * y + R.at<double>(2, 2) * z + tz;
                x = t_x;
                y = t_y;
                z = t_z;

                int h = round(row*(float)frame.image.rows/(float)rows);
                int w = round(col*(float)frame.image.cols/(float)cols);
                uint8_t b = frame.image.ptr<uchar>(h)[w * 3];
                uint8_t g = frame.image.ptr<uchar>(h)[w * 3 + 1];
                uint8_t r = frame.image.ptr<uchar>(h)[w * 3 + 2];

                PointXYZRGBA p;
                p.x = x;
                p.y = y;
                p.z = z;
                p.r = r;
                p.g = g;
                p.b = b;
                //if(y>-700.0f && y < 1300.0f)
                pc->points.push_back(p);
            }
        }

}


int main(int argc, char** argv)
{


    //在argv.txt中设置输入数据源路径，从而只需要更改txt文件（不需要再次编译源程序）即可测试其他数据源的效果
    ifstream fin( "argv.txt" );
    if (!fin)
    {
        cerr<<"parameter file does not exist."<<endl;
        return -1;
    }
    std::string root_dir;
    getline( fin, root_dir );
    fin.close();
    //**********read input data*************//
    cout<<"start init..."<<endl;
    vector<Frame> frames;
    Intrinsics intrinsics;
    initFrames(root_dir, frames, intrinsics);
    cout<<"finish init."<<endl;
    ////////////////////////////////////////////////////////////////

    //*********compute feature points(corners and descriptors)***************//
    cout<<"start computeFeatures..."<<endl;
    computeFeatures(frames);//opencv内部利用了多线程
    cout<<"finish computeFeatures"<<endl;

    //*********determine match pairs*************************************//
    cout<<"start genInitGraph..."<<endl;
    vector<Edge> edges;
    //genInitGraph(frames, intrinsics, edges);//有并发提升空间
    genRelativePoses(frames, intrinsics, edges);//有并发提升空间
    cout<<"finish genInitGraph"<<endl;

    //*********generate key frames*******************************//
    //delete standalone frames(those without valuable edges)
    cout<<"start genKeyFrames1..."<<endl;
    vector<Frame> tkframes;
    genKeyFrames(frames, edges, tkframes);
    cout<<"finish genKeyFrames1..."<<endl;

    //*********g2o 粗糙全局位恣求解********************************//
    cout<<"start genGlobalByMst..."<<endl;
    vector<Edge> kedges;
    genGlobalByMst(edges, tkframes, kedges);
    //generate the key frames that contribute to connected graph.
    vector<Frame> kframes;
    genKeyFrames(tkframes, kedges, kframes);
    cout<<"finish genGlobalByMst..."<<endl;
    //****更快的、更好用的粗糙全局位恣求解，可以用最小耗费生成树算法******//

    //*********ceres lib 最小二乘法优化 *******************************//
    cout<<"start solveProblem..."<<endl;
    cout<<"kframes.size(): "<<kframes.size()<<endl;
    for(int i=0; i<kframes.size(); ++i)
    {
        cout<<i<<" th rx,ty,tz, tx,ty,tz: "
            <<kframes[i].rx<<","<<kframes[i].ry<<","<<kframes[i].rz<<", "
            <<kframes[i].tx<<","<<kframes[i].ty<<","<<kframes[i].tz<<endl;
    }
    google::InitGoogleLogging(argv[0]);
    //RProblem2 RigidProblem(kframes, kedges, intrinsics, 50);
    //DProblem2 deformableProblemWxh(kframes, kedges, intrinsics, 2, 2, 50);
    for(int i=0; i<kframes.size(); ++i)
    {
        cout<<i<<" th rx,ty,tz, tx,ty,tz: "
            <<kframes[i].rx<<","<<kframes[i].ry<<","<<kframes[i].rz<<", "
            <<kframes[i].tx<<","<<kframes[i].ty<<","<<kframes[i].tz<<endl;
    }
    cout<<"finish solveProblem..."<<endl;

    //***********warp perspective image to panorama with rasterization pipeline**************//
    cout<<"start warpToPanos..."<<endl;
    Pers2PanoWarper(kframes, intrinsics);
    cout<<"finish warpToPanos..."<<endl;

    //***********stitch all panoramas to one frame*************************//
    cout<<"start stitchAllPanos..."<<endl;
    Frame pano;
    stitchAllPanos(kframes, pano);
    cout<<"finish stitchAllPanos..."<<endl;

    //debug and show...............
    double maxVal, minVal;
    minMaxLoc(pano.pano_depth, &minVal, &maxVal);
    minMaxLoc(pano.pano_depth, &minVal, &maxVal);
    Mat temp_pano_depth = (pano.pano_depth-minVal)/(maxVal-minVal);

    Mat img = pano.pano_image;
    Mat dps = pano.pano_depth;
    imshow("img", img);
    imshow("dps", dps);

    Mat pano_img(PANO_H, PANO_W, CV_8UC3, Scalar(0,0,0));
    Size dsize = Size(1024, 512);

    int minh = pano.minh;
    int minw = pano.minw;
    for(int h = minh; h<=pano.maxh; ++h)
        for(int w = minw; w<=pano.maxw; ++w)
            if(pano.pano_depth.at<float>(h-minh, w-minw) > 0.0f)
            {pano_img.at<Vec3b>(h,w) = Vec3b(img.at<Vec3b>(h-minh, w-minw));}
    cv::resize(pano_img, pano_img, dsize);
    imshow("pano_img", pano_img);

    //imshow("pano image", pano_img);
    //imshow("pano depth", pano_dps);
    while(waitKey());


    return 0;
}