//
// Created by ale on 18-12-19.
//

#ifndef I3D_COMPUTEFEATURES_H
#define I3D_COMPUTEFEATURES_H

#include<opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <vector>
#include <string>

#include "Frame.h"
namespace i3d
{
    int computeFeatures(std::vector<i3d::Frame> & frames);
}


#endif //I3D_COMPUTEFEATURES_H
