//
// Created by ale on 19-3-14.
//

#ifndef I3D_GENOBJFROMPANO_H
#define I3D_GENOBJFROMPANO_H

#include "i3d.h"
#include "Frame.h"
#include "floodFillWithoutHoles.h"
#include "genSimplePolygonContours.h"
#include "triangulateSimplePolygon.h"
#include "exportTriangles2Obj.h"

#ifndef I3D_GEN_NORMAL_MAP
#define I3D_GEN_NORMAL_MAP
int genNormalMap(i3d::Frame& pano)
{
    using namespace cv;
    cv::Mat srcDepth = pano.pano_depth;//CV_32FC1
    if(srcDepth.type() != CV_32FC1)
        srcDepth.convertTo(srcDepth, CV_32FC1);

    //计算法向量
    Mat depth = Mat(srcDepth.size()+Size(2,2), CV_32FC1, Scalar(0.0f));
    Mat ROI=depth(Rect( 1, 1, srcDepth.cols, srcDepth.rows));//x,y,w,h    xy坐标，宽度，高度
    srcDepth.copyTo(ROI);
    if(depth.type() != CV_32FC1)
        depth.convertTo(depth, CV_32FC1);

    Mat normals(depth.size(), CV_32FC3);

    for(int x = 1; x < depth.rows-1; ++x)
    {
        for(int y = 1; y < depth.cols-1; ++y)
        {
            // use float instead of double otherwise you will not get the correct result
            // check my updates in the original post. I have not figure out yet why this
            // is happening.
            float dzdx = (depth.at<float>(x+1, y) - depth.at<float>(x-1, y)) / 2.0;
            float dzdy = (depth.at<float>(x, y+1) - depth.at<float>(x, y-1)) / 2.0;

            Vec3f d(-dzdx, -dzdy, 1.0f);

            Vec3f n = normalize(d);
            normals.at<Vec3f>(x, y) = n;
        }
    }

    normals = normals(Rect( 1, 1, srcDepth.cols, srcDepth.rows)).clone();
    pano.pano_normal = normals.clone();

    return 0;
}
#endif

int genObjFromPano(const std::string& outputDir, const i3d::Frame& pano)
{
    using namespace std;
    using namespace cv;

    const cv::Mat srcImage = pano.pano_image;//CV_8UC3
    const cv::Mat srcDepth = pano.pano_depth;//CV_32FC1
    cv::Mat depthImage, maskImage;

    int minw = pano.minw;
    int minh = pano.minh;
    int maxw = pano.maxw;
    int maxh = pano.maxw;

    if (srcImage.empty() || srcDepth.empty())
    {
        cout<<"pano.rgb/d is empty!"<<endl;
        return -1;
    }
    depthImage = srcDepth.clone();//(Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();//运算过程中会改变
    maskImage = Mat(depthImage.size(), CV_8UC1, Scalar(0));//用原图尺寸初始化掩膜mask

    //特殊的漫水填充得到限制大小的chart（保证生成简单多边形），
    //然后对chart进行三角剖分，现在使用的是最简单的ear clipping算法，
    //直到所有chart都被剖分为三角网格
    cout<<"triangles generation started: "<<endl;
    double minVal, maxVal;
    Point minLoc, maxLoc;
    minMaxLoc(depthImage, &minVal, &maxVal, &minLoc, &maxLoc);
    vector<Point> triangles;
    while(maxVal > 0)
    {
        Mat mask_draw = maskImage.clone();
        double diff = 0.05*depthImage.at<float>(maxLoc.y, maxLoc.x);
        int temp = floodFillWithoutHoles(depthImage, mask_draw, maxLoc, diff, diff, false);

        if(temp != 0)
        {
            cout<<"temp: "<<temp<<endl;
            return -1;
        }

        //将depthImage清零
        for(int i=0; i<mask_draw.rows; ++i)
            for(int j=0; j<mask_draw.cols; ++j)
                if(mask_draw.at<uchar>(i,j) > 0)
                {
                    depthImage.at<float>(i, j) = 0;
                }
        //生成简单多边形的外轮廓
        vector<Point> contour;
        clockWiseContour(mask_draw, maxLoc, contour);
        genSimplePolygonContours(mask_draw, maxLoc, contour, true);

        //cout<<contour.size()<<" contour: "<<endl;
        //for(auto i:contour)
        //    cout<<i<<" ";
        //cout<<endl;
        //简化轮廓，用更少的点表示离散轮廓
        vector<Point> contour_approx;
        cv::approxPolyDP(contour, contour_approx, 3, false);

        //从简单多边形生成三角形
        vector<Point> triangles_temp;
        triangulateSimplePolygon(contour_approx, triangles_temp);
        for(int i=0; i<triangles_temp.size(); ++i)
            triangles.emplace_back(triangles_temp[i]);

        //为下一轮作准备，若深度图上的所有点都已经遍历，则 (maxVal>0) == false, 跳出循环
        minMaxLoc(depthImage, &minVal, &maxVal, &minLoc, &maxLoc);
    }
    cout<<"triangles generation finished: "<<triangles.size()<<" tris generized"<<endl;
    //depthImage = srcDepth.clone();//(Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();


    //imshow("normals", normals);
    //waitKey();

    //将最终的模型信息写入.obj和.mtl文件中，并将贴图保存
    writePanoToSimpleObj(pano, triangles, outputDir);

    return 0;
}


#endif //I3D_GENOBJFROMPANO_H
