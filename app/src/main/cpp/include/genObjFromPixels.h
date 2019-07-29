//
// Created by ale on 19-3-18.
//

#ifndef I3D_GENOBJFROMPIXELS_H
#define I3D_GENOBJFROMPIXELS_H

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

static void GenerateCompactTriangles(const i3d::Frame& pano, std::vector<cv::Point> &triangles)
{
    using namespace cv;
    const float MAX_DIFF = 0.05;
    const cv::Mat depth = pano.pano_depth;
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
                if (abs(dleft - d) < MAX_DIFF*d && abs(dtop - d) < MAX_DIFF*d)//dleft > 0 && dtop > 0 &&
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
                if (abs(dright - d) < MAX_DIFF*d && abs(dbottom - d) < MAX_DIFF*d)
                {
                    triangles.emplace_back(Point(w, h));
                    triangles.emplace_back(Point(w, h+1));
                    triangles.emplace_back(Point(w+1, h));
                }
            }

        }//for w=0...
    }//for h=0...
}//end of GenerateCompactTriangles

int genObjFromPixels(const std::string& outputDir, const i3d::Frame& pano)
{
    using namespace std;
    using namespace cv;

    const cv::Mat srcImage = pano.pano_image;//CV_8UC3
    const cv::Mat srcDepth = pano.pano_depth;//CV_32FC1

    vector<Point> triangles;

    GenerateCompactTriangles(pano, triangles);

    cout<<"triangles generation finished: "<<triangles.size()<<" tris generized"<<endl;
    //depthImage = srcDepth.clone();//(Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();


    //imshow("normals", normals);
    //waitKey();

    //将最终的模型信息写入.obj和.mtl文件中，并将贴图保存
    writePanoToSimpleObj(pano, triangles, outputDir);

    return 0;
}

#endif //I3D_GENOBJFROMPIXELS_H
