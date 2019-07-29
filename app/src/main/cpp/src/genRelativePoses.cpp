//
// Created by ale on 19-4-18.
//
#include "genRelativePoses.h"
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace std;
using namespace i3d;
using namespace cv;

static int ComputeMatches(const Frame& frame1, const Frame& frame2, vector<Point2f>& pp1, vector<Point2f>& pp2)
{
    const Mat desc1 = frame1.descriptor;
    const Mat desc2 = frame2.descriptor;

    // For each descriptor in image1, find 2 closest matched in image2 (note: couldn't get BF matcher to work here at all)
    FlannBasedMatcher flannmatcher;
    flannmatcher.add(desc1);
    flannmatcher.train();
    vector <vector<DMatch>> matches;
    flannmatcher.knnMatch(desc2, matches, 2);

    // ignore matches with high ambiguity -- i.e. second closest match not much worse than first
    // push all remaining matches back into DMatch Vector "good_matches" so we can draw them using DrawMatches
    int num_good = 0;
    //vector<DMatch>      good_matches;

    const float nn_match_ratio = 0.7f;      // Nearest neighbor matching ratio
    const vector<KeyPoint> keypoints1 = frame1.keypoints;
    const vector<KeyPoint> keypoints2 = frame2.keypoints;
    for (int i = 0; i < matches.size(); i++)
    {
        DMatch first = matches[i][0];
        DMatch second = matches[i][1];
        if (first.distance < nn_match_ratio * second.distance)
        {
            pp1.push_back(Point2f(keypoints1[first.trainIdx].pt.x, keypoints1[first.trainIdx].pt.y));
            pp2.push_back(Point2f(keypoints2[first.queryIdx].pt.x, keypoints2[first.queryIdx].pt.y));
            //good_matches.push_back(DMatch(num_good, num_good, 0));
            //good_matches.push_back(DMatch(first.trainIdx, first.queryIdx, 0));
            num_good++;
        }
    }

    return num_good;
}

#define GEN_RELATIVEPOSES_DEBUG
int genRelativePoses(const vector<Frame>& frames, const Intrinsics& intrinsics, vector<Edge>& edges)
{
    int fsize = frames.size();
    if(fsize<=1)
    {
        LOGE("[                                     ]");
        LOGE("[                                     ]");
        LOGE("genRelativePoses(): frame size less than 2");
        cout<<"genRelativePoses(): frame size less than 2"<<endl;
        return -1;
    }

    //尺寸与rgb图像保持一致
    Mat K = Mat::zeros(3,3,CV_64FC1);
    K.at<double>(0,0) = intrinsics.f;
    K.at<double>(0,2) = intrinsics.cx;
    K.at<double>(1,1) = intrinsics.f;
    K.at<double>(1,2) = intrinsics.cy;
    K.at<double>(2,2) = 1;

    for(int i=0; i<fsize-1; ++i)
        for(int j=i+1; j<fsize; ++j)
        {
            vector<Point2f> ppi, ppj;
            int psize = ComputeMatches(frames[i], frames[j], ppi, ppj );
            if(psize<8) continue;

            //codes from https://github.com/gaoxiang12/slambook/blob/master/ch7/pose_estimation_2d2d.cpp
            //-- 计算本质矩阵
            Point2d principal_point ( intrinsics.cx, intrinsics.cy );	//相机光心
            double focal_length = intrinsics.f;			//相机焦距
            Mat essential_matrix;
            essential_matrix = findEssentialMat ( ppi, ppj, focal_length, principal_point );
            if(essential_matrix.rows > 3)
            {
#ifdef GEN_RELATIVEPOSES_DEBUG
                cout<<i<<","<<j<<" :"<<"find essential_matrix failed !!!"<<endl;
#endif
                continue;
            }
            //-- 从本质矩阵中恢复旋转和平移信息.
            Mat R,t;
            recoverPose ( essential_matrix, ppi, ppj, R, t, focal_length, principal_point );

#ifdef GEN_RELATIVEPOSES_DEBUG
            //cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;
            //cout<<"R is "<<endl<<R<<endl;
            //cout<<"t is "<<endl<<t<<endl;
#endif
            ///////////////////////////////////////////////////////////////////////////////////////////////
            Mat rvec = Mat::zeros(3, 1, CV_64FC1);
            Mat tvec = Mat::zeros(3, 1, CV_64FC1);
            Rodrigues(R, rvec);
            //cout<<" recoverPose t: "<<t.type()<<endl;CV_64FC1
            tvec.at<double>(0) = t.at<double>(0)/1000;
            tvec.at<double>(1) = t.at<double>(1)/1000;
            tvec.at<double>(2) = t.at<double>(2)/1000;
            //why use this????
            double translation = fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec))) ;//+ fabs(cv::norm(tvec/10));//fabs(cv::norm(tvec/1000.0))
#ifdef GEN_RELATIVEPOSES_DEBUG
            cout<<"************************************************************"<<endl;
            cout<<i<<","<<j<<" :   feature_points matched_points translations : "<<endl
            <<"          "<< frames[i].keypoints.size()<< " "<< ppi.size()<< " "<< translation
            << " "<<endl;
            //cout<<rvec<<endl;
            //cout<<t<<endl;
            //cout<<"rx, ry, rz, tx, ty, tz: "<<rvec<<endl<<tvec<<endl;
#endif
            const double MIN_MATCHED_FEATURE_POINTS = 40;//double(frames[i].keypoints.size())/10.0;//20
            const double MIN_TRANSLATION = 0.11;
            const double MAX_TRANSLATION = 0.55;//0.5
            if( ppi.size() <= MIN_MATCHED_FEATURE_POINTS)
            {
#ifdef GEN_RELATIVEPOSES_DEBUG
                cout<<"matched_points less than min !!!"<<endl;
#endif
                continue;
            }//MIN_MATCHED_FEATURE_POINTS
            if( translation > MAX_TRANSLATION || translation < MIN_TRANSLATION)
            {
#ifdef GEN_RELATIVEPOSES_DEBUG
                cout<<"translation out of range !!!"<<endl;

#endif
                continue;
            }

            i3d::Edge edge;
            edge.src = i; edge.dst = j;
            edge.rx = rvec.at<double>(0); edge.ry = rvec.at<double>(1); edge.rz = rvec.at<double>(2);
            edge.tx = tvec.at<double>(0); edge.ty = tvec.at<double>(1); edge.tz = tvec.at<double>(2);
            edge.psrc.clear(); edge.pdst.clear();
            edge.psrc = ppi;
            edge.pdst = ppj;

            edge.cost = translation;
            edges.push_back(edge);
        }

    return 0;
}