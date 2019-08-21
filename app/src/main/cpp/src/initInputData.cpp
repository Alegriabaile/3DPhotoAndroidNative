//
// Created by ale on 18-12-18.
//

#include "initInputData.h"

namespace i3d
{

    int initFrames(const std::string & mainDir, std::vector<Frame>& frames, i3d::Intrinsics& intrinsic)
    {
        //using namespace cv;
        std::vector<cv::Mat> images, depths;
        readRGBD(mainDir, images, depths);

        for(int i=0; i<images.size(); i++)
        {
            Frame frame;
            frame.image = images[i];

            //将深度图/视差图大小resize至与颜色图大小相同
            cv::Mat _depth = depths[i];
            cv::resize(_depth, _depth, images[i].size());
            frame.depth = _depth;

            if(frame.image.empty())
                std::cout<<"initFrames()::frame.image.empty()!!!"<<std::endl;
            if(frame.depth.empty())
                std::cout<<"initFrames()::frame.depth.empty()!!!"<<std::endl;

            frames.push_back(frame);
        }

        if(frames.size() < 1)
        {
            std::cout<<"initInputData: frames.size() < 1 before read intrinsics !!!"<<std::endl;
            exit(-1);
        }

        //对齐内参的尺寸
        readIntrinsics(mainDir, intrinsic);
        //f(color)/f = cx(color)/cx = cols(color)/(cx*2)
//        intrinsic.f = intrinsic.f*frames[0].image.cols/(intrinsic.cx*2);
//        intrinsic.cx = frames[0].image.cols/2;
//        intrinsic.cy = frames[0].image.rows/2;

        intrinsic.colorw = frames[0].image.cols;
        intrinsic.colorh = frames[0].image.rows;
        intrinsic.depthw = frames[0].depth.cols;
        intrinsic.depthh = frames[0].depth.rows;

        return 0;
    }

}