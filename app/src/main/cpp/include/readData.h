#ifndef INC_3D_DEMO2_READDATA_H
#define INC_3D_DEMO2_READDATA_H



#include <vector>
#include <string>
#include "opencv2/opencv.hpp"
#include "i3d.h"

void readRGBD(const std::string mainDir, std::vector<cv::Mat>& images, std::vector<cv::Mat>& depths);
void readIntrinsics(const std::string mainDir, i3d::Intrinsics& intrinsics);

#endif