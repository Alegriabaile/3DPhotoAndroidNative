//
// Created by ale on 18-12-19.
//
#include "genInitialGraph.h"
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

//#define GEN_INIT_GRAPH

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

static int BackprojectTo3dPoints(const vector<Point2f>& pp, const Frame& frame, const Mat& K, vector<Point3f> & ppp)
{
    double fx = K.at<double>(0,0);
    double fy = K.at<double>(1,1);
    double cx = K.at<double>(0,2);
    double cy = K.at<double>(1,2);

    Mat depth = frame.depth;
    Size size = frame.image.size();
    Mat depth_resize;
    resize(depth, depth_resize, size);

    for(int i=0; i<pp.size(); i++)
    {
        double u = pp[i].x;
        double v = pp[i].y;
        double d = depth_resize.at<float>(v,u);

        double x = d*(u-cx)/fx;
        double y = d*(v-cy)/fy;

        ppp.push_back(Point3f(x,y,d));
    }

    return 0;
}

#define GEN_INITIAL_GRAPH
int SolvePnPProblem(std::vector<Point3f> &vPt3D, std::vector<Point2f> &vPt2D, cv::Mat &K, cv::Mat & rvec, cv::Mat & tvec, cv::Mat & inliers)
{
    //std::vector<Point3f> vPt3D；// 世界坐标：x=(u-u0)*depth/fx; y = (v-v0)*depth/fy; z=depth;
    //std::vector<Point2f> vPt2D；//像素坐标，不需要归一化，会受噪声影响
    //cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32FC1); // output rotation std::vector
    //cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32FC1);

    //RANSAC parameters
    int iterationsCount = 200;// number of Ransac iterations.
    float reprojectionError = 1.0;// maximum allowed distance to consider it an inlier.
    double confidence = 0.95;//0.95;// ransac successful confidence.

    //点对个数必须大于4
    bool b = solvePnPRansac(vPt3D, vPt2D, K, cv::Mat(), rvec, tvec, false, iterationsCount, reprojectionError, confidence, inliers, SOLVEPNP_ITERATIVE);

    //if(inliers.rows<5)
    //    cout<<"inliers.rows less than 5, inliers.rows = "<<inliers.rows<<endl;
    //else
    //    cout<<"inliers.rows = "<<inliers.rows<<endl;
#ifdef GEN_INITIAL_GRAPH
    //cout<<"************************************************************"<<endl
    //<<"confidence, inliers.rows, inliers: "<<confidence<<", "<<inliers.rows<<endl;
    /*
    transpose(inliers, inliers);
    cout<<inliers<<endl;
    transpose(inliers, inliers);
    cout<<"[";
    for(int i=0; i<inliers.rows; ++i)
        cout<<inliers.at<int>(i)<<", ";
    cout<<"]"<<endl;
    */
#endif

    return inliers.rows;
}


int genInitialGraph(const vector<Frame>& frames, const Intrinsics& intrinsics, vector<Edge>& edges)
{
    int fsize = frames.size();
    if(fsize<1)
        return -1;

    //尺寸与rgb图像保持一致
    Size size = frames[0].image.size();
    double cx = size.width/2;
    double cy = size.height/2;
    double f = intrinsics.f*size.width/intrinsics.cx/2;
    Mat K = Mat::zeros(3,3,CV_64FC1);
    K.at<double>(0,0) = f;
    K.at<double>(0,2) = cx;
    K.at<double>(1,1) = f;
    K.at<double>(1,2) = cy;
    K.at<double>(2,2) = 1;

    for(int i=0; i<fsize-1; ++i)
        for(int j=i+1; j<fsize; ++j)
        {
            vector<Point2f> ppi, ppj;
            int psize = ComputeMatches(frames[i], frames[j], ppi, ppj );
            if(psize<4) continue;

            vector<Point3f> pppi;
            BackprojectTo3dPoints(ppi, frames[i], K, pppi);
            //solvePnPRansac(p3d, p2d..), R,t transforms object point3d to camera coordinate.
            Mat rvec = Mat::zeros(3, 1, CV_64FC1);
            Mat tvec = Mat::zeros(3, 1, CV_64FC1);
            Mat inliers;
            int n_inliers =SolvePnPProblem(pppi, ppj, K, rvec, tvec, inliers);//(i-----transform(rvec,tvec)----->j)

            double translation = fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec/1000.0));//why use this????

#ifdef GEN_INITIAL_GRAPH
            cout<<"************************************************************"<<endl;
            cout<<i<<","<<j<<" :   feature_points matched_points inliers translations inliers_rate : "<<endl
                <<"          "<< frames[i].keypoints.size()<< " "<< ppi.size()<< " "<< n_inliers<<" "<<translation
                << " "<<n_inliers/(double)(ppi.size())<<endl;
            //cout<<"rx, ry, rz, tx, ty, tz: "<<rvec<<endl<<tvec<<endl;
#endif
            const double MIN_MATCHED_FEATURE_POINTS = double(frames[i].keypoints.size())/6.0;//20
            const double MIN_INLIER_ROWS = double(frames[i].keypoints.size())/12.0;//11
            const double MIN_INLIER_MATCHED_RATE0 = 0.33;
            const double MIN_INLIER_MATCHED_RATE1 = 0.50;
            const double MIN_TRANSLATION = 0.05;
            const double MAX_TRANSLATION = 0.55;//0.5
            if( ppi.size() <= MIN_MATCHED_FEATURE_POINTS)continue;//MIN_MATCHED_FEATURE_POINTS
            if( n_inliers <= MIN_INLIER_ROWS)continue;//MIN_INLIER_ROWS..not reliable???
            if( translation > MAX_TRANSLATION || translation < MIN_TRANSLATION)continue;
            if( ppi.size()<MIN_MATCHED_FEATURE_POINTS*2 && n_inliers/(double)(ppi.size())<MIN_INLIER_MATCHED_RATE1) continue;
            if( ppi.size()>=MIN_MATCHED_FEATURE_POINTS*2 && n_inliers/(double)(ppi.size())<MIN_INLIER_MATCHED_RATE0) continue;
            i3d::Edge edge;
            edge.src = i; edge.dst = j;
            edge.rx = rvec.at<double>(0); edge.ry = rvec.at<double>(1); edge.rz = rvec.at<double>(2);
            edge.tx = tvec.at<double>(0); edge.ty = tvec.at<double>(1); edge.tz = tvec.at<double>(2);
            edge.psrc.clear(); edge.pdst.clear();
            for(int k=0; k<inliers.rows; ++k)
            {
                edge.psrc.push_back(ppi[inliers.at<int>(k)]);
                edge.pdst.push_back(ppj[inliers.at<int>(k)]);
            }
            edge.cost = translation;
            edges.push_back(edge);
        }

    return 0;
}
