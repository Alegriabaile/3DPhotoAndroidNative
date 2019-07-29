//
// Created by ale on 18-12-19.
//



#include <iostream>
#include "Frame.h"
#include "i3d.h"

#include "initInputData.h"

#include "genFeatures.h"

using namespace i3d;
using namespace cv;
using namespace std;

static vector<DMatch> ComputeMatches(const Frame& frame1, const Frame& frame2, vector<KeyPoint>& pp1, vector<KeyPoint>& pp2)
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
    vector<DMatch>      good_matches;

    const float nn_match_ratio = 0.7f;      // Nearest neighbor matching ratio
    const vector<KeyPoint> keypoints1 = frame1.keypoints;
    const vector<KeyPoint> keypoints2 = frame2.keypoints;
    for (int i = 0; i < matches.size(); i++)
    {
        DMatch first = matches[i][0];
        DMatch second = matches[i][1];
        if (first.distance < nn_match_ratio * second.distance)
        {
            pp1.push_back(keypoints1[first.trainIdx]);
            pp2.push_back(keypoints2[first.queryIdx]);
            good_matches.push_back(DMatch(num_good, num_good, 0));
            //good_matches.push_back(DMatch(first.trainIdx, first.queryIdx, 0));
            num_good++;
        }
    }

    return good_matches;
}

int main()
{
    //google::InitGoogleLogging(argv[0]);
    //在argv.txt中设置输入数据源路径，从而只需要更改txt文件（不需要再次编译源程序）即可测试其他数据源的效果
    ifstream fin( "argv.txt" );
    if (!fin)
    {
        cerr<<"parameter file does not exist."<<endl;
        return -1;
    }
    std::string root_dir;
    getline( fin, root_dir );
    fin.close();
    //**********input data*************//
    vector<Frame> frames;
    initFrames(root_dir, frames);
    Intrinsics intrinsics;
    initIntrinsics(root_dir, frames, intrinsics);

    //*********compute feature points(corners and descriptors)***************//
    computeFeatures(frames);
    for(int i=0; i<frames.size()-1; ++i)
    {
        for(int j=i+1; j<frames.size(); ++j)
        {
            Mat img1 = frames[i].image.clone();
            Mat img2 = frames[j].image.clone();
            const vector<KeyPoint> kp1= frames[i].keypoints;
            const vector<KeyPoint> kp2 = frames[j].keypoints;
            //vector<Point2f> p1, p2;
            for(int k=0; k<kp1.size(); k++)
            {
                Point2f p2f;
                p2f.x = kp1[k].pt.x;
                p2f.y = kp1[k].pt.y;
                circle(img1, p2f, 5, Scalar(0,0,255));//BGR,choose red

            }
            for(int k=0; k<kp2.size(); k++)
            {
                Point2f p2f;
                p2f.x = kp2[k].pt.x;
                p2f.y = kp2[k].pt.y;
                circle(img2, p2f, 5, Scalar(0,0,255));
            }
            //imshow("img1", img1);
            //imshow("img2", img2);

            vector<KeyPoint> pp1, pp2;
            vector<DMatch> goodmatches = ComputeMatches(frames[i], frames[j], pp1, pp2);
            Mat dst;
            drawMatches(img1, pp1, img2, pp2, goodmatches, dst);
            string dst_name("dst:");
            dst_name.append(to_string(i)).append(", ").append(to_string(j));
            imshow(dst_name, dst);

            waitKey();
            destroyWindow(dst_name);
            //while(waitKey());
        }
    }


    return 0;
}