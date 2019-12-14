//
// Created by ale on 19-11-7.
//

#ifndef MY3DPHOTO_1FRAMEREADER_H
#define MY3DPHOTO_1FRAMEREADER_H

#include "i3d.h"
#include "Frame.h"
#include "1FileNameExtractor.h"

namespace m3d
{
    class FrameReader
    {
    private:
        std::string dataDir;
        unsigned int state;

        void readArgv(std::string argv1);

        void readDefaultIntrinsic(const std::string &paramDefault, m3d::IntrinsicD& intrinsicD);

        void readImage(const std::string& imageFileName, cv::Mat& image);
        void readImages(const std::vector<std::string>& imageFileNames, std::vector<i3d::Frame> &Frames);

        void readDepth(const std::string& depthFileName, i3d::Frame &frame);
        void readDepths(const std::vector<std::string>& depthFileNames, std::vector<i3d::Frame> &Frames);

        void readParameter(const std::string &paramFileName, i3d::Frame &frame);
//        void readParameters(const std::vector<std::string>& paramFileNames,  i3d::IntrinsicD &defaultIntrinsicD, std::vector<i3d::Frame> &Frames);

        void readData(std::string argv1, std::vector<i3d::Frame> &frames, i3d::Intrinsics& intrinsic);


    public:
        FrameReader(std::string argv1, std::vector<i3d::Frame> &frames, i3d::Intrinsics& intrinsic);

        ~FrameReader();

    };




}



#endif //MY3DPHOTO_1FRAMEREADER_H
