//
// Created by ale on 18-12-18.
//

#ifndef I3D_FRAME_H
#define I3D_FRAME_H

#include "opencv2/opencv.hpp"
#include <vector>

namespace i3d
{
    class Frame
    {
    public:
        cv::Mat image, depth, error;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptor;

        double rx, ry, rz, tx, ty, tz;
        //bool isKey;
        uint minw, minh, maxw, maxh;
        cv::Mat pano_image, pano_depth, pano_error, pano_disparity, pano_normal;
        cv::Mat pano_depth_b, pano_image_b, pano_normal_b;
        cv::Mat pano_label, mask_label, mask_bound, soft_mask;//20190524---feathering...

        Frame()
                : rx(0), ry(0), rz(0)
                , tx(0), ty(0), tz(0)
                //, isKey(_frame.isKey)
                , minw(0), minh(0), maxw(0), maxh(0)
        {}

        Frame(const Frame& _frame)
                : image(_frame.image), depth(_frame.depth), error(_frame.error)
                , keypoints(_frame.keypoints)
                , descriptor(_frame.descriptor)
                , rx(_frame.rx), ry(_frame.ry), rz(_frame.rz)
                , tx(_frame.tx), ty(_frame.ty), tz(_frame.tz)
                //, isKey(_frame.isKey)
                , minw(_frame.minw), minh(_frame.minh), maxw(_frame.maxw), maxh(_frame.maxh)
                , pano_image(_frame.pano_image)
                , pano_depth(_frame.pano_depth)
                , pano_error(_frame.pano_error)
                , pano_normal(_frame.pano_normal)
                , pano_disparity(_frame.pano_disparity)
                , pano_depth_b(_frame.pano_depth_b)
                , pano_image_b(_frame.pano_image_b)
                , pano_normal_b(_frame.pano_normal_b)
                , pano_label(_frame.pano_label)
                , mask_bound(_frame.mask_bound)
                , mask_label(_frame.mask_label)
                , soft_mask(_frame.soft_mask) {}

    };


}


#endif //I3D_FRAME_H
