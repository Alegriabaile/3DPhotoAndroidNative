//
// Created by ale on 19-3-20.
//

#ifndef I3D_GENERATETRIANGLES_H
#define I3D_GENERATETRIANGLES_H

#include "i3d.h"
#include "Frame.h"
#include "floodFillWithoutHoles.h"
#include "genSimplePolygonContours.h"
#include "triangulateSimplePolygon.h"


int GenerateSimplifiedTriangles(const cv::Mat& pano_depth, std::vector<cv::Point>& triangles)
{
    using namespace std;
    using namespace cv;

    const cv::Mat srcDepth = pano_depth.clone();//CV_32FC1
    cv::Mat depthImage, maskImage;

    if (srcDepth.empty())
    {
        cout<<"pano.pano_depth is empty!"<<endl;
        return -1;
    }
    depthImage = srcDepth.clone();//(Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();//运算过程中会改变
    maskImage = Mat(depthImage.size(), CV_8UC1, Scalar(0));//用原图尺寸初始化掩膜mask

    //特殊的漫水填充得到限制大小的chart（保证生成简单多边形），
    //然后对chart进行三角剖分，现在使用的是最简单的ear clipping算法，
    //直到所有chart都被剖分为三角网格
    //cout<<"triangles generation started: "<<endl;
    double minVal, maxVal;
    Point minLoc, maxLoc;
    minMaxLoc(depthImage, &minVal, &maxVal, &minLoc, &maxLoc);
    //vector<Point> triangles;
    triangles.clear();
    while(maxVal > 0)
    {
        Mat mask_draw = maskImage.clone();
        double diff = 0.05*depthImage.at<float>(maxLoc.y, maxLoc.x);
        int temp = floodFillWithoutHoles(depthImage, mask_draw, maxLoc, diff, diff, false);

        if(temp != 0)
        {
            cout<<"floodFillWithoutHoles: temp: "<<temp<<endl;
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
    //cout<<"triangles generation finished: "<<triangles.size()<<" tris generized"<<endl;

    return 0;
}

int GenerateCompactTriangles(const cv::Mat& pano_depth, std::vector<cv::Point>& triangles)
{
    using namespace cv;
    using namespace std;

    const float MAX_DIFF = 0.05;
    const cv::Mat depth = pano_depth;
    if(depth.empty())
    {
        cout<<"GenerateCompactTriangles: depth.empty! "<<endl;
        return -1;
    }

    triangles.clear();
    for (float h = 0; h < depth.rows; ++h)
    {
        for (float w = 0; w < depth.cols; ++w)
        {
            float d = depth.at<float>(h, w);
            float x, y, z;

            if(!(d>0))
                continue;

            if (h > 0 && w > 0)
            {
                float dleft = depth.at<float>(h, (int) w - 1);
                float dtop = depth.at<float>(h - 1, (int) w);
                if (fabs(dleft - d) < MAX_DIFF*d && fabs(dtop - d) < MAX_DIFF*d)//dleft > 0 && dtop > 0 &&
                {
                    triangles.emplace_back(Point(w, h));
                    triangles.emplace_back(Point(w, h-1));
                    triangles.emplace_back(Point(w-1, h));
                }
            }

            if (h + 1 < depth.rows && w + 1 < depth.cols)
            {
                float dright = depth.ptr<float>(h)[(int) w + 1];
                float dbottom = depth.ptr<float>(h + 1)[(int) w];
                if (fabs(dright - d) < MAX_DIFF*d && fabs(dbottom - d) < MAX_DIFF*d)
                {
                    triangles.emplace_back(Point(w, h));
                    triangles.emplace_back(Point(w, h+1));
                    triangles.emplace_back(Point(w+1, h));
                }
            }

        }//for w=0...
    }//for h=0...

    return 0;
}//end of GenerateCompactTriangles

#endif //I3D_GENERATETRIANGLES_H
