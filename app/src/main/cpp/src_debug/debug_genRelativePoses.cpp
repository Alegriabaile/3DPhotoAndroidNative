//
// Created by ale on 18-12-19.
//



//自定义数据结构/自定义封装
#include "Frame.h"
#include "i3d.h"

//功能实现
#include "initInputData.h"
#include "genFeatures.h"
#include "genRelativePoses.h"
#include "genKeyFrames.h"
//依赖项
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


int main()
{
    //google::InitGoogleLogging(argv[0]);

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
    cout<<"start init..."<<endl;
    vector<Frame> frames;
    Intrinsics intrinsics;
    initFrames(root_dir, frames, intrinsics);
    cout<<"finish init."<<endl;

    //*********compute feature points(corners and descriptors)***************//
    computeFeatures(frames);

    //*********determine match pairs*************************************//
    vector<Edge> edges;
    genRelativePoses(frames, intrinsics, edges);

    vector<Frame> kframes;
    genKeyFrames(frames, edges, kframes);

    for(int i=0; i<edges.size(); ++i)//debug pcl
    {
        cout<<i<<" th rx,ty,tz, tx,ty,tz: "
            <<edges[i].rx<<","<<edges[i].ry<<","<<edges[i].rz<<", "
            <<edges[i].tx<<","<<edges[i].ty<<","<<edges[i].tz<<endl;
        cv::Mat R;
        cv::Mat rvec = (Mat_<double>(3,1) << edges[i].rx, edges[i].ry, edges[i].rz);
        cv::Rodrigues( rvec, R );
        Eigen::Matrix3d r;
        cv::cv2eigen(R, r);

        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        Eigen::AngleAxisd angle(r);
        T = angle;
        T(0,3) = edges[i].tx;
        T(1,3) = edges[i].ty;
        T(2,3) = edges[i].tz;

        PointCloud<PointXYZRGBA>::Ptr cloud1( new PointCloud<PointXYZRGBA>);
        BackprojectToPano(kframes[edges[i].dst], intrinsics, cloud1);
        PointCloud<PointXYZRGBA>::Ptr cloud2( new PointCloud<PointXYZRGBA>);
        BackprojectToPano(kframes[edges[i].src], intrinsics, cloud2);

        PointCloud<PointXYZRGBA>::Ptr cali_cloud2(new PointCloud<PointXYZRGBA>);
        pcl::transformPointCloud( *cloud2, *cali_cloud2, T.matrix());//src -> dst
        *cloud1 += *cali_cloud2;
        cloud2->clear();
        cali_cloud2->clear();


        pcl::visualization::CloudViewer viewer( "viewer" );
        viewer.showCloud( cloud1 );
        while( !viewer.wasStopped() )
        {
        }

        cloud1->clear();

        cout<<edges[i].src<<" "<<edges[i].dst<<endl;
    }

    return 0;
}