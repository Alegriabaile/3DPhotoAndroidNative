//
// Created by ale on 19-3-20.
//

#include "genSimplifiedTri.h"
#include "GenerateTriangles.h"
#include "WriteFbToObj.h"
#include "WriteFbToTxt.h"
using namespace std;
using namespace cv;

static int GenerateSimplifiedTrianglesFront(const cv::Mat& pano_depth, std::vector<cv::Point>& triangles, std::vector<cv::Point>& contour_edges)
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
    contour_edges.clear();
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
        contour_edges.insert(contour_edges.end(), contour.begin(), contour.end());
        //简化轮廓，用更少的点表示离散轮廓
        vector<Point> contour_approx;
        cv::approxPolyDP(contour, contour_approx, 1, false);

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

static int GenerateSimplifiedTrianglesBack(const cv::Mat& pano_depth, std::vector<cv::Point>& triangles)
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

static int InitializeMultiLayer(i3d::Frame& pano, const int icount = 30)
{
    cv::Mat pano_image_f = pano.pano_image;
    cv::Mat pano_depth_f = pano.pano_depth;
    //init
    cv::Mat image_f(pano_image_f.size()+Size(icount*2, icount*2), CV_8UC3, Scalar(0,0,0));
    cv::Mat image_b(pano_image_f.size()+Size(icount*2, icount*2), CV_8UC3, Scalar(0,0,0));
    cv::Mat depth_f(pano_depth_f.size()+Size(icount*2, icount*2), CV_32FC1, Scalar(0.0f));
    cv::Mat depth_b(pano_depth_f.size()+Size(icount*2, icount*2), CV_32FC1, Scalar(0.0f));

    //cv::medianBlur(pano_depth_f.clone(), pano_depth_f, 7);//ksize较大时只能使用CV_8U类型
    //image_f(Rect(icount, icount, pano_image_f.cols, pano_image_f.rows)) = pano_image_f.clone();
    //depth_f(Rect(icount, icount, pano_depth_f.cols, pano_depth_f.rows)) = pano_depth_f.clone();
    Mat roi= image_f(Rect(icount, icount, pano_image_f.cols, pano_image_f.rows));
    pano_image_f.copyTo(roi);
    roi = depth_f(Rect(icount, icount, pano_depth_f.cols, pano_depth_f.rows));
    pano_depth_f.copyTo(roi);

    pano.pano_image = image_f;
    pano.pano_depth = depth_f;
    pano.pano_image_b = image_b;
    pano.pano_depth_b = depth_b;

    pano.minh -= icount;
    pano.minw -= icount;
    pano.maxh += icount;
    pano.maxw += icount;

    return 0;
}

static int GenerateBackLayerContents(i3d::Frame& pano, std::vector<cv::Point>& contour_edges, const int icount = 30)
{
    cv::Mat depth_f = pano.pano_depth;
    cv::Mat depth_b = pano.pano_depth_b;
    for(auto i:contour_edges)
        depth_b.at<float>(i.y, i.x) = depth_f.at<float>(i.y, i.x);

    float d, dl_b, dr_b, dt_b, db_b;//current depth, left-right-top-bottom depth.
    float dl_f, dr_f, dt_f, db_f;
    for(int k = 0; k<icount-1; ++k)
    {
        cv::Mat depth_b_t = depth_b.clone();
        for(int h=1; h<depth_b.rows-1; ++h)
        {
            for(int w=1; w<depth_b.cols-1; ++w)
            {
                //只处理有深度值的邻域，否则跳过
                d = depth_b.at<float>(h, w);
                if(!(d > 1e-6f))
                    continue;
                dl_b = depth_b.at<float>(h, w-1);
                dr_b = depth_b.at<float>(h, w+1);
                dt_b = depth_b.at<float>(h-1, w);
                db_b = depth_b.at<float>(h+1, w);

                dl_f = depth_f.at<float>(h, w-1);
                dr_f = depth_f.at<float>(h, w+1);
                dt_f = depth_f.at<float>(h-1, w);
                db_f = depth_f.at<float>(h+1, w);
                //left
                if(!(dl_b > 1e-6f) || dl_b < d )//&& d <= depth_f.at<float>(h, w-1))
                    if(dl_f>1e-6f&&dl_f<d || !(dl_f>1e-6f))
                        depth_b_t.at<float>(h, w-1) = d;
                //right
                if(!(dr_b > 1e-6f) || dr_b < d )//&& d <= depth_f.at<float>(h, w+1))
                    if(dr_f>1e-6f&&dr_f<d || !(dr_f>1e-6f))
                        depth_b_t.at<float>(h, w+1) = d;
                //top
                if(!(dt_b > 1e-6f) || dt_b < d)//&& d <= depth_f.at<float>(h-1, w))
                    if(dt_f>1e-6f&&dt_f<d || !(dt_f>1e-6f))
                        depth_b_t.at<float>(h-1, w) = d;
                //bottom
                if(!(db_b > 1e-6f) || db_b < d)//&& d <= depth_f.at<float>(h+1, w))
                    if(db_f>1e-6f&&db_f<d || !(db_f>1e-6f))
                        depth_b_t.at<float>(h+1, w) = d;
            }
        }
        depth_b = depth_b_t;
    }


    cv::Mat mask;
    mask = (depth_f<=1e-6f);
    inpaint(pano.pano_image, mask, pano.pano_image, 3, CV_INPAINT_NS);
    mask = depth_b > 0;
    inpaint(pano.pano_image, mask, pano.pano_image_b, 3, CV_INPAINT_NS);

    pano.pano_depth = depth_f;
    pano.pano_depth_b = depth_b;

    //debug
    /*
    imshow("image_f", pano.pano_image);
    imshow("depth_f", pano.pano_depth);
    imshow("image_b", pano.pano_image_b);
    imshow("depth_b", pano.pano_depth_b);
    waitKey();*/

    return 0;
}

int genSimplifiedTri(const std::string& outputDir, i3d::Frame& pano, const int icount)
{
    vector<Point> triangles_f, triangles_b;
    vector<Point> contour_edges;

    InitializeMultiLayer(pano, icount);
    GenerateSimplifiedTrianglesFront(pano.pano_depth, triangles_f, contour_edges);

    GenerateBackLayerContents(pano, contour_edges, icount);
    GenerateSimplifiedTrianglesBack(pano.pano_depth_b, triangles_b);

    cout<<"f tri size(): "<<triangles_f.size()<<".  b tri size(): "<<triangles_b.size()<<endl;
    //WriteFbToObj(pano, triangles_f, triangles_b, outputDir);
    WriteFbToTxt(pano, triangles_f, triangles_b, outputDir);
    return 0;
}