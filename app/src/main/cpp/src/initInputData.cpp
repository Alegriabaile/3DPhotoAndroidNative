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


            cv::Mat mv[3];
            if(depths[i].channels()>1)
                cv::split(depths[i], mv);
            else
                mv[0] = depths[i];
            cv::Mat _depth;
            mv[0].convertTo(_depth, CV_32FC1);
            //将深度图转换为归一化的单通道浮点表示方法
            //double minVal, maxVal;
            //minMaxLoc(_depth, &minVal, &maxVal);
            //_depth = _depth/255.0f;//(_depth-float(minVal))/float(maxVal-minVal);

            //将深度图/视差图大小resize至与颜色图大小相同
            _depth = _depth*(images[i].cols/_depth.cols);//深度值是否需要变化？？？
            cv::resize(_depth, _depth, images[i].size());
            frame.depth = _depth;
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
        intrinsic.f = intrinsic.f*frames[0].image.cols/(intrinsic.cx*2);
        intrinsic.cx = frames[0].image.cols/2;
        intrinsic.cy = frames[0].image.rows/2;


        return 0;
    }

}