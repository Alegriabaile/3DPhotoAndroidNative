//
// Created by ale on 19-7-25.
//

#ifndef NATIVE0701_WARPER4ANDROID_H
#define NATIVE0701_WARPER4ANDROID_H

#include "i3d.h"
#include "Frame.h"
#include "WarperShader.h"
#include "WarperEglManager.h"
#include "WarperGlesInitializer.h"

int warpToPano(i3d::Frame& kframe, i3d::Intrinsics& intrinsics);

class Warper4Android {
private:
    WarperGlesInitializer warperGlesInitializer;

    int GenerateTriangles( const cv::Mat &depth,
                           const i3d::Intrinsics &intrinsics,
                           std::vector<float> &vertices);

    int GenerateSkybox( const i3d::Frame &frame,
                        const std::vector<float> &vertices,
                        std::vector<cv::Mat> &images,
                        std::vector<cv::Mat> &depths);

    int GeneratePoints( const std::vector<cv::Mat> &images,
                        const std::vector<cv::Mat> &depths,
                        std::vector<std::vector<float>> &vPoints);

    int GeneratePanorama(const std::vector<cv::Mat> &images,
                         const std::vector<cv::Mat> &depths,
                         const i3d::Frame &frame);


    int WarpToPanorama(i3d::Frame& kframe, i3d::Intrinsics& intrinsics);
public:
    Warper4Android(std::vector<i3d::Frame>& kframes, i3d::Intrinsics& intrinsics);
    ~Warper4Android();

};


template <typename T>
void print_f(T i)
{
    std::cout<<i<<std::endl;
}
#endif //NATIVE0701_WARPER4ANDROID_H
