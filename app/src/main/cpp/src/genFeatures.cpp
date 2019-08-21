//
// Created by ale on 18-12-19.
//

#include "genFeatures.h"

using namespace std;
using namespace cv;
namespace i3d
{
    static void ComputeKeypointsAndDescriptor(const Mat& image, const Mat& depth, vector<KeyPoint>& keypoints, Mat& descriptor)
    {
        int maxCornerNumber = floor(sqrt(image.rows*image.cols)/2);
        maxCornerNumber = maxCornerNumber>1000?1000:maxCornerNumber;
        double qualityLevel = 0.01;//角点检测可接受的最小特征值
        double minDistance = 10;//角点之间的最小距离
        int blockSize = 3;//计算导数自相关矩阵时指定的邻域范围
        double k = 0.04;//权重系数

        // Compute DAISY descriptors for both images (goodFeaturesToTrack)
        vector<Point2f> p2f;
        Mat gray;
        cvtColor(image, gray, COLOR_BGR2GRAY);

        Size size = gray.size();
        double scale_min = 0.05;

        //ROI感兴趣区域，即去除一定的边界
        Rect roi(size.width*scale_min, size.height*scale_min, size.width*(1.0-2.0*scale_min), size.height*(1.0-2.0*scale_min));
        Mat mask = Mat::zeros(size, CV_8UC1);
        mask(roi).setTo(255);
        //深度值为0的不在考虑中，同时要考虑边界
        Mat depth_resize;
        resize(depth, depth_resize, size);
        mask = (depth_resize > 1e-6) & mask;

        goodFeaturesToTrack(gray,//输入图像
                            p2f,//检测到的角点的输出向量
                            maxCornerNumber,//角点的最大数量
                            qualityLevel,//角点检测可接受的最小特征值
                            minDistance,//角点之间的最小距离
                            mask,//感兴趣区域
                            blockSize,//计算导数自相关矩阵时指定的邻域范围
                            false,//不使用Harris角点检测
                            k);//权重系数

        const float keypoint_diameter = 1.0f;
        for (vector<Point2f>::iterator i = p2f.begin(); i<p2f.end(); i++)
            keypoints.push_back(KeyPoint(i->x, i->y, keypoint_diameter));

        Ptr<cv::xfeatures2d::DAISY> descriptor_extractor = cv::xfeatures2d::DAISY::create();
        descriptor_extractor->compute(image, keypoints, descriptor);
    }

    int computeFeatures(vector<i3d::Frame> & frames)
    {
        for(int i=0; i<frames.size(); ++i)
        {
            ComputeKeypointsAndDescriptor(frames[i].image, frames[i].depth, frames[i].keypoints, frames[i].descriptor);
        }

        return 0;
    }
}
