//
// Created by ale on 19-3-14.
//

#ifndef I3D_GENSIMPLEPOLYGONCONTOURS_H
#define I3D_GENSIMPLEPOLYGONCONTOURS_H
//
// Created by ale on 19-3-11.
//

#include <vector>
#include <opencv2/opencv.hpp>

static cv::Point GetPointUp(const cv::Mat& mask, const cv::Point& current)
{
    if(current.y-1 >= 0 && mask.at<uchar>(current.y-1, current.x)>0)
        return GetPointUp(mask, cv::Point(current.x, current.y-1));

    return current;
}
static cv::Point GetPointDown(const cv::Mat& mask, const cv::Point& current)
{
    if(current.y+1 < mask.rows && mask.at<uchar>(current.y+1, current.x)>0)
        return GetPointDown(mask, cv::Point(current.x, current.y+1));

    return current;
}
static cv::Point GetPointLeft(const cv::Mat& mask, const cv::Point& current)
{
    if(current.x-1 >= 0 && mask.at<uchar>(current.y, current.x-1)>0)
        return GetPointLeft(mask, cv::Point(current.x-1, current.y));

    return current;
}
static cv::Point GetPointRight(const cv::Mat& mask, const cv::Point& current)
{
    if(current.x+1 < mask.cols && mask.at<uchar>(current.y, current.x+1)>0)
        return GetPointRight(mask, cv::Point(current.x+1, current.y));

    return current;
}

int antiClockContour(const cv::Mat& mask, const cv::Point& seed, std::vector<cv::Point>& contour)
{
    if(mask.empty())
        return -1;
    if(mask.type() != CV_8UC1)
        return -2;
    if( seed.x>mask.cols-1 || seed.x<0 ||
        seed.y>mask.rows-1 || seed.y<0 ||
        !(mask.at<uchar>(seed.y, seed.x)>0) )
        return -3;

    contour.clear();

    cv::Point top = GetPointUp(mask, seed);
    cv::Point bottom = GetPointDown(mask, seed);

    for(int i=0; i<=bottom.y-top.y; ++i)
        contour.push_back(GetPointLeft(mask, top+cv::Point(0, i)));
    for(int i=0; i<=bottom.y-top.y; ++i)
        contour.push_back(GetPointRight(mask, bottom-cv::Point(0, i)));

    return 0;
}

/*opencv坐标系
 *   o —— > x
 *   |
 *   v
 *  y
 *  由于与普通坐标系不一致，故在处理向量夹角时要将上下翻转，
 *  等效做法可以将轮廓顶点集顺时针输出
 * */
int clockWiseContour(const cv::Mat& mask, const cv::Point& seed, std::vector<cv::Point>& contour)
{
    if(mask.empty())
        return -1;
    if(mask.type() != CV_8UC1)
        return -2;
    if( seed.x>mask.cols-1 || seed.x<0 ||
        seed.y>mask.rows-1 || seed.y<0 ||
        !(mask.at<uchar>(seed.y, seed.x)>0) )
        return -3;

    contour.clear();

    cv::Point top = GetPointUp(mask, seed);
    cv::Point bottom = GetPointDown(mask, seed);

    for(int i=0; i<=bottom.y-top.y; ++i)
        contour.push_back(GetPointRight(mask, top+cv::Point(0, i)));
    for(int i=0; i<=bottom.y-top.y; ++i)
        contour.push_back(GetPointLeft(mask, bottom-cv::Point(0, i)));

    return 0;
}

int genSimplePolygonContours(const cv::Mat& mask, const cv::Point& seed, std::vector<cv::Point>& contour, const bool isClockWise = true)
{
    if(isClockWise)
        clockWiseContour(mask, seed, contour);
    else
        antiClockContour(mask, seed, contour);

    return 0;
}
#endif //I3D_GENSIMPLEPOLYGONCONTOURS_H
