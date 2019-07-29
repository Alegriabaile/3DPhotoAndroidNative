//
// Created by ale on 19-3-24.
//

#ifndef I3D_RIGIDPROBLEM_H
#define I3D_RIGIDPROBLEM_H

#include "i3d.h"
#include "Frame.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

class RigidProblem
{
public:
    i3d::Intrinsics intrinsics;

    ~RigidProblem()
    {
        //if(camera_i_ != nullptr)//可以delete[]空指针
        delete[] camera_i_;
        //if(camera_j_ != nullptr)
        delete[] camera_j_;
        //if(matches_ != nullptr)
        delete[] matches_;
        //if(parameters_ != nullptr)
        delete[] parameters_;
    }

    RigidProblem()
            : num_matches_(0), num_cameras_(0)
            , camera_i_(nullptr), camera_j_(nullptr)
            , matches_(nullptr), parameters_(nullptr){}

    RigidProblem(std::vector<i3d::Frame> & kframes, std::vector<i3d::Edge> & kedges, i3d::Intrinsics & intrinsics)
            : num_matches_(0), num_cameras_(0)
            , camera_i_(nullptr), camera_j_(nullptr)
            , matches_(nullptr), parameters_(nullptr)
    {
        initParameters(kframes, kedges, intrinsics);
        solveProblem();
        updateParameters(kframes);
    }

    int initParameters(const std::vector<i3d::Frame> & kframes, const std::vector<i3d::Edge> & edges, const i3d::Intrinsics & intrinsics);
    int updateParameters(std::vector<i3d::Frame> & kframes);
    int solveProblem();

private:
    int insert(const int k, const int i, const int j, const double * matches)
    {
        if(k<0 || k>=num_matches_)
            return -1;
        if(i<0 || i>num_cameras_ || j<0 || j>num_cameras_)
            return -2;

        for(int index = 0; index<6; index++)
            *(matches_+k*6+index) = matches[index];

        camera_i_[k] = i;
        camera_j_[k] = j;
    }
    bool operator()(int num_matches_, int num_cameras_, i3d::Intrinsics intrinsics1)
    {
        this->num_matches_  = num_matches_;
        this->num_cameras_ = num_cameras_;
        this->intrinsics = intrinsics1;

        delete[] camera_i_; delete[] camera_j_;
        delete[] matches_; delete[] parameters_;

        camera_i_ = new int[num_matches_]; camera_j_ = new int[num_matches_];
        matches_  = new double[num_matches_*6];//a_xyd, b_xyd
        parameters_ = new double[num_cameras_*6];//rotate_angles, translation
        return true;
    }

    int camera_i_index(int k) const { return camera_i_[k];}
    int camera_j_index(int k) const { return camera_j_[k];}
    int num_matches() const { return num_matches_; }
    int num_cameras() const { return num_cameras_; }
    const double * const const_matches() const { return matches_; }

    double* mutable_cameras() { return parameters_; }

    double* mutable_camera_i(int k){ return mutable_cameras() + camera_i_[k] * 6;}
    double* mutable_camera_j(int k){ return mutable_cameras() + camera_j_[k] * 6;}

    int point_transfer(const cv::Vec3f& point0, const cv::Vec3f& rvec, const cv::Vec3f& tvec, cv::Vec3f& point1);
    int depth_transfer(const double *const scales, const double *const offsets, cv::Mat & depth);

/////////////////////////////variables declaring/////////////////////////////

private:
    int num_cameras_;
    int num_matches_;

    int* camera_i_;
    int* camera_j_;
    double* matches_;
    //mutable data, (Independent variables to be optimized)
    double* parameters_;//camera extrisic
/////////////////////////////////////////////////////////////////////////
};


//////////////////////////模板代码不能放到cpp文件中，因为其模板参数必须在编译时给出，而cpp文件在编译时已经被编译为源代码////////////////////
struct double_6
{
    double ax, ay, ad, bx, by, bd;
};
const double d_scale = 1;

int RigidProblem::initParameters(
        const std::vector<i3d::Frame> &kframes,
        const std::vector<i3d::Edge> &edges,
        const i3d::Intrinsics &intrinsics)
{
    std::vector<struct double_6 > matches;
    std::vector<int> camera_i, camera_j;

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
            struct double_6 temp_match;// not safe
            temp_match.ax = edges[i].psrc[k].x;//x,y为image坐标空间，而d为depth坐标空间
            temp_match.ay = edges[i].psrc[k].y;
            temp_match.ad = kframes[edges[i].src].depth.at<float >
                    (floor(temp_match.ay*ratioh),floor(temp_match.ax*ratiow))/d_scale;
            temp_match.bx = edges[i].pdst[k].x;
            temp_match.by = edges[i].pdst[k].y;
            temp_match.bd = kframes[edges[i].dst].depth.at<float >
                    (floor(temp_match.by*ratioh),floor(temp_match.bx*ratiow))/d_scale;
            matches.push_back(temp_match);
        }
    }

    //cout<<"naiveRtProblem 调用重载运算符"<<endl;
    i3d::Intrinsics  _intrinsics = intrinsics;

    this->operator()(matches.size(), kframes.size(), _intrinsics);
    for(int k = 0; k<matches.size(); ++k)
    {
        double match[6];
        match[0] = matches[k].ax; match[1] = matches[k].ay; match[2] = matches[k].ad;
        match[3] = matches[k].bx; match[4] = matches[k].by; match[5] = matches[k].bd;
        this->insert(k, camera_i[k], camera_j[k], match);
        //cout<<"delete:"<<k<<endl;
    }

    for(int k=0; k<kframes.size(); ++k)
    {
        double *cameras = this->mutable_cameras();
        cameras[k*6] = kframes[k].rx; cameras[k*6+1] = kframes[k].ry; cameras[k*6+2] = kframes[k].rz;
        cameras[k*6+3] = kframes[k].tx; cameras[k*6+4] = kframes[k].ty; cameras[k*6+5] = kframes[k].tz;
    }
    return 0;
}
int RigidProblem::point_transfer(const cv::Vec3f& point0, const cv::Vec3f& rvec, const cv::Vec3f& tvec, cv::Vec3f& point1)
{
    //Rotate and Translate matrix.
    //matrix: |Point1; 1| = |R, t|*|point0; 1|
    cv::Mat R = (cv::Mat_<double>(3,3));
    cv::Mat rv = (cv::Mat_<double>(3,1) << rvec[0], rvec[1], rvec[2]);
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

int RigidProblem::updateParameters(std::vector<i3d::Frame> & kframes)
{
    const double * parameters = this->mutable_cameras();
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
    std::vector<cv::Mat> N(ksize);
    std::vector<cv::Mat> NNT(ksize);
    std::vector<cv::Mat> NNTo(ksize);
    cv::Mat A = cv::Mat::zeros(3,3,CV_32FC1);
    cv::Mat b = cv::Mat(3,1,CV_32FC1, cv::Scalar(0));
    for(int i=0; i<kframes.size(); ++i)
    {
        cv::Vec3f point0(0,0,100);
        cv::Vec3f orig0(0,0,0);
        cv::Vec3f point1, orig1;
        cv::Vec3f rvec, tvec;

        rvec[0] = parameters[i*6]; rvec[1] = parameters[i*6+1]; rvec[2] = parameters[i*6+2];
        tvec[0] = parameters[i*6+3]; tvec[1] = parameters[i*6+4]; tvec[2] = parameters[i*6+5];

        point_transfer(point0, rvec, tvec, point1);
        point_transfer(orig0, rvec, tvec, orig1);

        cv::Vec3f n(normalize(point1-orig1));
        N[i] = (cv::Mat_<float>(3,3) <<
                                     0, -n[2], n[1], n[2], 0, -n[0], -n[1], n[0], 0 );
        cv::Mat o = (cv::Mat_<float>(3,1) <<  orig1[0], orig1[1], orig1[2]);
        NNT[i] = N[i]*(-N[i]); NNTo[i] = NNT[i]*o;

        A += NNT[i]; b += NNTo[i];
        //cout<<orig1<<", "<<point1<<", "<<n<<";"<<endl;
    }

    cv::Mat A_inverse;
    invert(A, A_inverse);
    cv::Mat center = A_inverse*b;
    //cout<<A<<endl;
    //cout<<center<<endl;

    double rx, ry, rz, tx, ty, tz;
    rx = ry = rz = 0;
    tx = center.at<float>(0); ty = center.at<float>(1); tz = center.at<float>(2);
    for(int i=0; i<kframes.size(); ++i)
    {
        rx += kframes[i].rx; ry += kframes[i].ry; rz += kframes[i].rz;
    }
    for(int i=0; i<kframes.size(); ++i)
    {
        kframes[i].rx -= rx/kframes.size();
        kframes[i].ry -= ry/kframes.size();
        kframes[i].rz -= rz/kframes.size();
        kframes[i].tx -= tx; kframes[i].ty -= ty; kframes[i].tz -= tz;
    }

    return 0;
}

////////////about ceres functor and solver////////////////////////////
class CostFunctor1
{
private:
    double a_x, a_y, a_d;
    double b_x, b_y, b_d;
    double f, cx, cy;

public:
    //const values.
    CostFunctor1(
            double a_x, double a_y, double a_d,
            double b_x, double b_y, double b_d,
            i3d::Intrinsics intrinsics)
            : a_x(a_x), a_y(a_y), a_d(a_d)
            , b_x(b_x), b_y(b_y), b_d(b_d)
            , f(intrinsics.f), cx(intrinsics.cx), cy(intrinsics.cy) {}
    //, f(intrinsics.f*intrinsics.colorw/intrinsics.cx/2), cx(intrinsics.colorw/2), cy(intrinsics.colorh/2) {}

    // camera[0,1,2] are the angle-axis rotation.
    // camera[3,4,5] are the translation.
    template <typename T>
    bool operator()(const T* const camera_a, const T* const camera_b, T* residuals) const
    {
        //i_point's 3d position in camera_a's coordinate.
        T point[3];
        point[0] = T(a_d*(a_x-cx)/f );
        point[1] = T(a_d*(a_y-cy)/f );
        point[2] = T(a_d );

        //i_point's position in global coordinate.
        T p[3];
        //RodriguesRotate<T>(camera_a_, point, p);
        ceres::AngleAxisRotatePoint(camera_a, point, p);
        //T a_matrix[3][3];
        //ceres::AngleAxisToRotationMatrix(camera_a_, a_matrix);
        p[0] += camera_a[3];
        p[1] += camera_a[4];
        p[2] += camera_a[5];

        //i_point's 3d position in camera_b's coordinate.
        p[0] -= camera_b[3];
        p[1] -= camera_b[4];
        p[2] -= camera_b[5];
        //RodriguesRotate<T>(camera_b, p, point);
        T camera_b_[3];
        camera_b_[0] = -camera_b[0];
        camera_b_[1] = -camera_b[1];
        camera_b_[2] = -camera_b[2];
        ceres::AngleAxisRotatePoint(camera_b_, p, point);
        //i_point's 2d position from camera_a to camera_b.
        const T x=point[0]/point[2];
        const T y=point[1]/point[2];
        T a_u = f*x+cx;// u = (f*x + cx*z)/d
        T a_v = f*y+cy;// v = (f*y + cy*z)/d

        // The error is the difference between the 2d points aligned in camera_b, difference(Pa->b(i_point), j_point) ).
        T b_u = T(b_x);
        T b_v = T(b_y);
        residuals[0] = a_u - b_u;
        residuals[1] = a_v - b_v;
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const double a_x, const double a_y, const double a_d,
                                       const double b_x, const double b_y, const double b_d,
                                       i3d::Intrinsics intrinsics)
    {
        return (new ceres::AutoDiffCostFunction<CostFunctor1, 2, 6, 6>(
                new CostFunctor1(a_x, a_y, a_d, b_x, b_y, b_d, intrinsics)));
    }

};

int RigidProblem::solveProblem()
{
    const double* const matches = this->const_matches();
    const int num_matches = this->num_matches();
    const int num_cameras = this->num_cameras();

    // Create residuals for each observation in the bundle adjustment problem. The
    // parameters for cameras and points are added automatically.
    ceres::Problem problem;

    for (int k = 0; k < num_matches; ++k)
    {
        // Each Residual block takes a point and a camera as input and outputs a 2
        // dimensional residual. Internally, the cost function stores the observed
        // image location and compares the reprojection against the observation.

        ceres::CostFunction* cost_function =
                CostFunctor1::Create(*(matches+6*k), *(matches+6*k+1),*(matches+6*k+2),
                                     *(matches+6*k+3),*(matches+6*k+4),*(matches+6*k+5), this->intrinsics);
        problem.AddResidualBlock(cost_function,
                                 new ceres::CauchyLoss(0.5) /* squared loss */,
                                 this->mutable_camera_i(k),
                                 this->mutable_camera_j(k));
    }



    // Make Ceres automatically detect the bundle structure. Note that the
    // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
    // for standard bundle adjustment problems.
    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type = ceres::DENSE_QR;//ceres::DENSE_SCHUR; //ceres::LEVENBERG_MARQUARDT;

    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 50;//100;
    options.num_threads = 4;
    options.num_linear_solver_threads = 4;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    return 0;
}
#endif //I3D_RIGIDPROBLEM_H
