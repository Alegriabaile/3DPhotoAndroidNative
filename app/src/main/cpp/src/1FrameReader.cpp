//
// Created by ale on 19-11-8.
//
#include "1FrameReader.h"

namespace m3d
{

    void FrameReader::readArgv(std::string argv1)
    {
//        std::ifstream inFile(argv1);
//        if (!inFile) {
////            LOG("FrameReader::readArgv", "argv file does not exist.");
//            exit(-1);
//        }
//        inFile >> dataDir >> state;
//        inFile.close();
        dataDir = argv1;
        state = 1;
    }

    void FrameReader::readDefaultIntrinsic(const std::string &paramDefault, m3d::IntrinsicD &intrinsicD)
    {
        double fx, fy, cx, cy;

        std::ifstream inFile(paramDefault);
        if (!inFile) {
//            LOG("FrameReader::readDefaultIntrinsic", "paramDefault file does not exist.");
            exit(-1);
        }
        inFile >> fx >> fy >> cx >> cy;
        inFile.close();

        intrinsicD.setIntrinsic(fx, fy, cx, cy);
    }

    void FrameReader::readImage(const std::string &imageFileName, cv::Mat &image)
    {
        image = cv::imread(imageFileName, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
        if (image.empty()) {
//            LOG("FrameReader::readImage", "read image failed");
            exit(-1);
        }
    }

    void FrameReader::readImages(const std::vector<std::string> &imageFileNames, std::vector<i3d::Frame> &frames)
    {
        frames.clear();
        frames.resize(imageFileNames.size());

        for (int i = 0; i < imageFileNames.size(); ++i)
        {
            readImage(imageFileNames[i], frames[i].image);
            frames[i].imageFileName.assign(imageFileNames[i]);
        }
    }

    void FrameReader::readDepth(const std::string &depthFileName, i3d::Frame &frame)
    {
        cv::Mat dTmp = cv::imread(depthFileName, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
        if (dTmp.empty())
        {
//            LOG("FrameReader::readDepth", "read depth failed");
            exit(-1);
        }
        unsigned int channels_ = dTmp.channels();
        if (channels_ != 3 && channels_ != 1) {
//            LOG("FrameReader::readDepth", "depth.channels() != (1 or 3)");
            exit(-1);
        }

        frame.depth = cv::Mat(dTmp.rows, dTmp.cols, CV_32FC1, cv::Scalar(0));
        frame.disparity = cv::Mat(dTmp.rows, dTmp.cols, CV_32FC1, cv::Scalar(0));

        if (state == 0) {
            for (unsigned int h = 0; h < dTmp.rows; h++) {
                for (unsigned int w = 0; w < dTmp.cols; w++) {
                    unsigned int d = dTmp.ptr<uchar>(h)[w * channels_];
                    frame.disparity.at<float>(h, w) = float(d);
                }
            }
            frame.depth = 1 / frame.disparity;
        } else if (state == 1) {
            for (unsigned int h = 0; h < dTmp.rows; h++) {
                for (unsigned int w = 0; w < dTmp.cols; w++) {
                    float d = dTmp.ptr<ushort>(h)[w];
                    frame.depth.at<float>(h, w) = d;
                }
            }
            frame.disparity = 1 / frame.depth;
        }
    }

    void FrameReader::readDepths(const std::vector<std::string> &depthFileNames, std::vector<i3d::Frame> &frames)
    {
        for (int i = 0; i < depthFileNames.size(); ++i)
        {
            readDepth(depthFileNames[i], frames[i]);
            frames[i].depthFileName.assign(depthFileNames[i]);
        }
    }

    void FrameReader::readParameter(const std::string &paramFileName, i3d::Frame &frame)
    {
        frame.paramFileName.assign(paramFileName);
        //read standalone intrinsics and the given extrinsics.
        //todo
//    m3d::IntrinsicD intrinsicD;
//    m3d::ExtrinsicD extrinsicD;
    }


    void FrameReader::readData(std::string argv1, std::vector<i3d::Frame> &frames, i3d::Intrinsics& intrinsic)
    {
        readArgv(argv1);

        std::string paramDefault;
        std::vector<std::string> imageFileNames, depthFileNames, paramFileNames;

        FileNameExtractor fileNameExtractor(dataDir, imageFileNames, depthFileNames, paramFileNames, paramDefault,
                                            state);

        m3d::IntrinsicD intrinDefault;
        readDefaultIntrinsic(paramDefault, intrinDefault);

        readImages(imageFileNames, frames);
        readDepths(depthFileNames, frames);

        intrinsic.f = intrinDefault.intrinsic_c()[0];
        intrinsic.cx = intrinDefault.intrinsic_c()[2];
        intrinsic.cy = intrinDefault.intrinsic_c()[3];

        intrinsic.colorh = frames[0].image.rows;
        intrinsic.colorw = frames[0].image.cols;
        intrinsic.depthh = frames[0].depth.rows;
        intrinsic.depthw = frames[0].depth.cols;
    }

    FrameReader::FrameReader(std::string argv1, std::vector<i3d::Frame> &frames, i3d::Intrinsics& intrinsic)
    {
        readData(argv1, frames, intrinsic);
    }

    FrameReader::~FrameReader() {}

}