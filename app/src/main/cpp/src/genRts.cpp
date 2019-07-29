//
// Created by ale on 18-12-20.
//

// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: keir@google.com (Keir Mierle)
//
// A minimal, self-contained bundle adjuster using Ceres, that reads
// files from University of Washington' Bundle Adjustment in the Large dataset:
// http://grail.cs.washington.edu/projects/bal
//
// This does not use the best configuration for solving; see the more involved
// bundle_adjuster.cc file for details.

#include "genRts.h"


using namespace std;
using namespace cv;
using namespace i3d;

struct double_6dim
{
    double ax, ay, ad, bx, by, bd;
};
const double d_scale = 1;

int initRts(const vector<Frame> & kframes, const vector<Edge> & edges, const Intrinsics & intrinsics, NaiveRtProblem & naiveRtProblem)
{
    vector<struct double_6dim > matches;
    vector<int> camera_i, camera_j;

    Mat K = Mat::zeros(3,3,CV_64FC1);
    K.at<double>(0,0) = intrinsics.f;
    K.at<double>(0,2) = intrinsics.cx;
    K.at<double>(1,1) = intrinsics.f;
    K.at<double>(1,2) = intrinsics.cy;
    K.at<double>(2,2) = 1;

    if(kframes.size()<1)
        return -1;

    float ratioh = (float)kframes[0].depth.rows/(float)kframes[0].image.rows;
    float ratiow = (float)kframes[0].depth.cols/(float)kframes[0].image.cols;
    for(int i=0; i<edges.size(); ++i)
    {
        //cout<<"matches"<<i<<","<<j<<endl;
        for(int k=0; k<edges[i].psrc.size(); ++k)
        {
            //cout<<"camera_i/j .push_back()"<<k<<endl;
            camera_i.push_back(edges[i].src);
            camera_j.push_back(edges[i].dst);

            //cout<<"matches.push_back()"<<endl;
            struct double_6dim temp_match;// not safe
            temp_match.ax = edges[i].psrc[k].x;//x,y为image坐标空间，而d为depth坐标空间
            temp_match.ay = edges[i].psrc[k].y;
            temp_match.ad = kframes[edges[i].src].depth.at<float >(floor(temp_match.ay*ratioh),floor(temp_match.ax*ratiow))/d_scale;
            temp_match.bx = edges[i].pdst[k].x;
            temp_match.by = edges[i].pdst[k].y;
            temp_match.bd = kframes[edges[i].dst].depth.at<float >(floor(temp_match.by*ratioh),floor(temp_match.bx*ratiow))/d_scale;
            matches.push_back(temp_match);
        }

    }

    //cout<<"naiveRtProblem 调用重载运算符"<<endl;
    Intrinsics  _intrinsics;
    _intrinsics.cx = kframes[0].image.cols/2;
    _intrinsics.cy = kframes[0].image.rows/2;
    _intrinsics.f = intrinsics.f*_intrinsics.cx/intrinsics.cx;
    _intrinsics.colorw = intrinsics.colorw;
    _intrinsics.colorh = intrinsics.colorh;
    _intrinsics.depthw = intrinsics.depthw;
    _intrinsics.depthh = intrinsics.depthh;

    naiveRtProblem(matches.size(), kframes.size(), _intrinsics);
    for(int k = 0; k<matches.size(); ++k)
    {
        double match[6];
        //for(int i=0; i<6; ++i)
        //    match[i] = matches[k][i];
        match[0] = matches[k].ax;
        match[1] = matches[k].ay;
        match[2] = matches[k].ad;
        match[3] = matches[k].bx;
        match[4] = matches[k].by;
        match[5] = matches[k].bd;
        naiveRtProblem.InsertMatch(k, camera_i[k], camera_j[k], match);
        //cout<<"delete:"<<k<<endl;
    }

    for(int k=0; k<kframes.size(); ++k)
    {
        double *cameras = naiveRtProblem.mutable_cameras();
        cameras[k*6] = kframes[k].rx;
        cameras[k*6+1] = kframes[k].ry;
        cameras[k*6+2] = kframes[k].rz;
        cameras[k*6+3] = kframes[k].tx;
        cameras[k*6+4] = kframes[k].ty;
        cameras[k*6+5] = kframes[k].tz;
    }
    return 0;
}

int genRts(NaiveRtProblem& naiveRtProblem)
{
    const double* matches = naiveRtProblem.const_matches();
    const int num_matches = naiveRtProblem.num_matches();
    const int num_cameras = naiveRtProblem.num_cameras();

    // Create residuals for each observation in the bundle adjustment problem. The
    // parameters for cameras and points are added automatically.
    ceres::Problem problem;
    for (int k = 0; k < num_matches; ++k)
    {
        // Each Residual block takes a point and a camera as input and outputs a 2
        // dimensional residual. Internally, the cost function stores the observed
        // image location and compares the reprojection against the observation.

        ceres::CostFunction* cost_function =
                NaiveReprojectionError::Create(*(matches+6*k), *(matches+6*k+1),*(matches+6*k+2),
                                               *(matches+6*k+3),*(matches+6*k+4),*(matches+6*k+5), naiveRtProblem.intrinsics);
        problem.AddResidualBlock(cost_function,
                                 new ceres::CauchyLoss(0.5) /* squared loss */,
                                 naiveRtProblem.mutable_camera_i(k),
                                 naiveRtProblem.mutable_camera_j(k));
    }

    // Make Ceres automatically detect the bundle structure. Note that the
    // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
    // for standard bundle adjustment problems.
    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type = ceres::DENSE_QR;//ceres::DENSE_SCHUR; //ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = true;
    //options.max_num_iterations = 300;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    return 0;
}

static double my_square(double x)
{
    return x*x;
}
static int point_transfer(const Vec3f& point0, const Vec3f& rvec, const Vec3f& tvec, Vec3f& point1)
{
    //Rotate and Translate matrix.
    //matrix: |Point1; 1| = |R, t|*|point0; 1|
    cv::Mat R = (Mat_<double>(3,3));
    cv::Mat rv = (Mat_<double>(3,1) << rvec[0], rvec[1], rvec[2]);
    cv::Rodrigues( rv, R );
    double x = point0[0];
    double y = point0[1];
    double z = point0[2];

    point1[0] = R.at<double>(0, 0) * x + R.at<double>(0, 1) * y + R.at<double>(0, 2) * z + tvec[0];
    point1[1] = R.at<double>(1, 0) * x + R.at<double>(1, 1) * y + R.at<double>(1, 2) * z + tvec[1];
    point1[2] = R.at<double>(2, 0) * x + R.at<double>(2, 1) * y + R.at<double>(2, 2) * z + tvec[2];

    //cout<<R<<point1<<endl;
    return 0;
}
int getOptimizedRts(NaiveRtProblem& naiveRtProblem, vector<Frame> & kframes)
{
    const double * parameters = naiveRtProblem.mutable_cameras();
    for(int i=0; i<kframes.size(); ++i)
    {
        kframes[i].rx = parameters[i*6];
        kframes[i].ry = parameters[i*6+1];
        kframes[i].rz = parameters[i*6+2];
        kframes[i].tx = parameters[i*6+3];
        kframes[i].ty = parameters[i*6+4];
        kframes[i].tz = parameters[i*6+5];
    }

    //**instant 3d photography**
    //we compute a center of projection for the panorama
    //by tracing the camera front vectors backwards and
    //finding the 3D point that minimizes the distance to all of them

    //实现：找到中心点尽量使[中心点，相机点]的向量与对应相机点的向量平行，即叉积为0
    //令方向归一化，则使得所有的相机，下式：sin(theta)
    // = |n X vector(center - camera_original)|/(|n|*|vector(center - camera_original)|)
    //最小，为使计算简单，去掉|n|*|vector(center - camera_original)|项，即只要优化
    //|n X vector(center - camera_original)|使其最小即可。
    uint ksize = kframes.size();
    vector<Mat> N(ksize);
    vector<Mat> NNT(ksize);
    vector<Mat> NNTo(ksize);
    Mat A = Mat::zeros(3,3,CV_32FC1);
    Mat b = Mat(3,1,CV_32FC1, Scalar(0));
    for(int i=0; i<kframes.size(); ++i)
    {
        Vec3f point0(0,0,100);
        Vec3f orig0(0,0,0);
        Vec3f point1, orig1;
        Vec3f rvec, tvec;

        rvec[0] = parameters[i*6];
        rvec[1] = parameters[i*6+1];
        rvec[2] = parameters[i*6+2];
        tvec[0] = parameters[i*6+3];
        tvec[1] = parameters[i*6+4];
        tvec[2] = parameters[i*6+5];

        point_transfer(point0, rvec, tvec, point1);
        point_transfer(orig0, rvec, tvec, orig1);

        Vec3f n(normalize(point1-orig1));
        N[i] = (Mat_<float>(3,3) <<   0, -n[2], n[1]
                                    , n[2], 0, -n[0]
                                    , -n[1], n[0], 0 );
        Mat o = (Mat_<float>(3,1) <<  orig1[0], orig1[1], orig1[2]);
        NNT[i] = N[i]*(-N[i]);
        NNTo[i] = NNT[i]*o;

        A += NNT[i];
        b += NNTo[i];

        //cout<<orig1<<", "<<point1<<", "<<n<<";"<<endl;
    }

    Mat A_inverse;
    invert(A, A_inverse);
    Mat center = A_inverse*b;
    //cout<<A<<endl;
    //cout<<center<<endl;

    double rx, ry, rz, tx, ty, tz;
    rx = ry = rz = 0;
    tx = center.at<float>(0);
    ty = center.at<float>(1);
    tz = center.at<float>(2);
    for(int i=0; i<kframes.size(); ++i)
    {
        rx += kframes[i].rx;
        ry += kframes[i].ry;
        rz += kframes[i].rz;
    }
    for(int i=0; i<kframes.size(); ++i)
    {
        kframes[i].rx -= rx/kframes.size();
        kframes[i].ry -= ry/kframes.size();
        kframes[i].rz -= rz/kframes.size();
        kframes[i].tx -= tx;
        kframes[i].ty -= ty;
        kframes[i].tz -= tz;
    }
    /*
    //改用平行旋转的平均值
    double rx, ry, rz, tx, ty, tz;
    double min = 1e10;//足够大
    int k = 0;
    for(int i=0; i<kframes.size(); ++i)
    {
        double sum = 0;
        for(int j=0; j<kframes.size(); ++j)
        {
            if(j!=i)
                sum += my_square( ( kframes[j].tx-kframes[i].tx)/1000.0)
                        + my_square( (kframes[j].ty-kframes[i].ty)/1000.0)
                        + my_square( ( kframes[j].tz-kframes[i].tz)/1000.0)
                        + my_square( kframes[j].rx - kframes[i].rx)
                        + my_square( kframes[j].ry - kframes[i].ry)
                        + my_square( kframes[j].rz - kframes[i].rz);
        }
        k = min<sum?k:i;
        min = min<sum?min:sum;
        rx += kframes[k].rx;
        ry += kframes[k].ry;
        rz += kframes[k].rz;
        tx += kframes[k].tx;
        ty += kframes[k].ty;
        tz += kframes[k].tz;
    }

    for(int i=0; i<kframes.size(); ++i)
    {
        kframes[i].rx -= rx/kframes.size();
        kframes[i].ry -= ry/kframes.size();
        kframes[i].rz -= rz/kframes.size();
        kframes[i].tx -= tx/kframes.size();
        kframes[i].ty -= ty/kframes.size();
        kframes[i].tz -= tz/kframes.size();
    }
     */



    return 0;
}
