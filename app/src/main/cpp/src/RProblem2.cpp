//
// Created by ale on 19-5-15.
//

//
// Created by ale on 19-5-15.
//

#include "RProblem2.h"

#ifndef I3D_DOUBLE_6
#define I3D_DOUBLE_6
struct double_6
{
    double ax, ay, ad, bx, by, bd;
};
const double d_scale = 1;
#endif

int RProblem2::point_transfer(const cv::Vec3f& point0, const cv::Vec3f& rvec, const cv::Vec3f& tvec, cv::Vec3f& point1)
{
    //Rotate and Translate matrix.
    //matrix: |Point1; 1| = |R, t|*|point0; 1|
    cv::Mat R = (cv::Mat_<float>(3,3));
    cv::Mat rv = (cv::Mat_<float>(3,1) << rvec[0], rvec[1], rvec[2]);
    cv::Rodrigues( rv, R );
    double x = point0[0];
    double y = point0[1];
    double z = point0[2];

    point1[0] = R.at<float>(0, 0) * x + R.at<float>(0, 1) * y + R.at<float>(0, 2) * z + tvec[0];
    point1[1] = R.at<float>(1, 0) * x + R.at<float>(1, 1) * y + R.at<float>(1, 2) * z + tvec[1];
    point1[2] = R.at<float>(2, 0) * x + R.at<float>(2, 1) * y + R.at<float>(2, 2) * z + tvec[2];

    //cout<<R<<point1<<endl;
    return 0;
}

#define DEPTH_TRANSFER_DEBUG
int RProblem2::depth_transfer(const double & scale, const double & offset, cv::Mat& depth)
{
#ifdef DEPTH_TRANSFER_DEBUG
    using namespace std;
    cout<<endl;
    cout<<"scale, offset: "<<scale<<", "<<offset<<endl;
    double maxVal, minVal;
    minMaxLoc(depth, &minVal, &maxVal);
    cout<<"original min, max: "<<minVal<<", "<<maxVal<<endl;
    //imshow("ori dps", depth);
#endif

    int rows = depth.rows;
    int cols = depth.cols;
    for(int h = 0; h < rows; ++h)
    {
        for(int w = 0; w < cols; ++w)
        {
            double d = depth.at<float>(h, w);
            double d_new = 1.0f/(scale*d+offset);//转换公式
            depth.at<float>(h, w) = d_new;
        }
    }

#ifdef DEPTH_TRANSFER_DEBUG
    minMaxLoc(depth, &minVal, &maxVal);
    cout<<"deformed min, max: "<<minVal<<", "<<maxVal<<endl;
    cout<<"rows, cols: "<<rows<<", "<<cols<<endl;
    imshow("cged dps/max", depth/maxVal);
    cv::waitKey();
#endif
    return 0;
}
#undef DEPTH_TRANSFER_DEBUG


int RProblem2::updateParameters(std::vector<i3d::Frame> & kframes)
{
    //更新模型矩阵
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
        N[i] = (cv::Mat_<float>(3,3) << 0, -n[2], n[1], n[2], 0, -n[0], -n[1], n[0], 0 );
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

    //更新变形后的深度值
    for(int i=0; i<kframes.size(); ++i)
    {
        double * scales = this->mutable_scales(i);
        double * offsets = this->mutable_offsets(i);
        this->depth_transfer(scales[0], offsets[0], kframes[i].depth);
    }

    return 0;
}


RProblem2::~RProblem2()
{
    clean_up();
}

RProblem2::RProblem2(std::vector<i3d::Frame> &kframes, std::vector<i3d::Edge> &kedges,
                     i3d::Intrinsics &intrinsics, int iter_num)
        : num_matches_(0), num_cameras_(0)
        , camera_i_(nullptr), camera_j_(nullptr)
        , matches_(nullptr), parameters_(nullptr)
        , scales_(nullptr), offsets_(nullptr)
        , iter_n(iter_num)
{
    initParameters(kframes, kedges, intrinsics);
    solveProblem();
    updateParameters(kframes);
    clean_up();
}

int RProblem2::initParameters(const std::vector<i3d::Frame> &kframes,
                              const std::vector<i3d::Edge> &edges, const i3d::Intrinsics &intrinsics)
{
    std::vector<struct double_6 > matches;
    std::vector<int> camera_i, camera_j;

    if(kframes.size()<1)
        return -1;

    double ratioh = (double)kframes[0].depth.rows/(double)kframes[0].image.rows;
    double ratiow = (double)kframes[0].depth.cols/(double)kframes[0].image.cols;
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
    this->operator()(matches.size(), kframes.size(), intrinsics);
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
    for(int k=0; k<kframes.size(); ++k)
    {
        double *scales = this->mutable_scales(k);
        double *offsets = this->mutable_offsets(k);
        scales[0] = 0.1; offsets[0] = 0.1;
    }

    return 0;
}

////////////about ceres functor and solver////////////////////////////
class D2CostFunctor1
{
private:
    double a_x, a_y, a_d;
    double b_x, b_y, b_d;
    double f, cx, cy;

public:
    //const values.
    D2CostFunctor1(
            double a_x, double a_y, double a_d,
            double b_x, double b_y,
            i3d::Intrinsics intrinsics)
            : a_x(a_x), a_y(a_y), a_d(a_d)
            , b_x(b_x), b_y(b_y)
            , f(intrinsics.f)
            , cx(intrinsics.cx), cy(intrinsics.cy)
    {}

    // camera[0,1,2] are the angle-axis rotation.
    // camera[3,4,5] are the translation.
    template <typename T>
    bool operator()(const T* const camera_a, const T* const camera_b,
                    const T* const scales_a0, const T* const offsets_a0,
                    T* residuals) const
    {
        //a_x, a_y, b_x, b_y, based on color image size

        //Bilinear Interpolation
        T scale_weight, offset_weight;
        scale_weight = scales_a0[0];
        offset_weight = offsets_a0[0];
        //T a_d_ = T(1.0) / (scale_weight * (T(1.0) /T(a_d)) + offset_weight );
        //T a_d_ = T(a_d) / (scale_weight + offset_weight*T(a_d) );
        T a_d_ = T(1.0) / (scale_weight*T(a_d) + offset_weight);

        //i_point's 3d position in camera_a's coordinate.
        T point[3];
        point[0] = T(a_d_*(a_x-cx)/f );
        point[1] = T(a_d_*(a_y-cy)/f );
        point[2] = T(a_d_ );

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
        T a_u = T(f)*x+T(cx);// u = (f*x + cx*z)/d
        T a_v = T(f)*y+T(cy);// v = (f*y + cy*z)/d

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
                                       const double b_x, const double b_y,
                                       i3d::Intrinsics intrinsics)
    {
        return (new ceres::AutoDiffCostFunction<D2CostFunctor1, 2, 6, 6, 1, 1>(
                new D2CostFunctor1(a_x, a_y, a_d, b_x, b_y, intrinsics)));
    }

};

//E_smoothness
struct D2CostFunctor2
{
    D2CostFunctor2(double l1_=1e3): l1(l1_){}

    //在一个加了const限定符的成员函数中，不能够调用 非const成员函数
    template <typename T> bool operator()(const T* const scales1, const T* const scales2,
                                          const T* const offsets1, const T* const offsets2, T* residual) const
    {
        residual[0] = T(l1) * (scales1[0] - scales2[0]);
        residual[1] = T(l1) * (offsets1[0] - offsets2[0]);
        return true;
    }

    static ceres::CostFunction* Create(const double l1=1e3)
    {
        return ( new ceres::AutoDiffCostFunction < D2CostFunctor2, 2, 1,1,1,1> ( new D2CostFunctor2(l1)));
    }
private:
    double l1;
};


//E_scale
struct D2CostFunctor3
{
    D2CostFunctor3(double l2=1e-2): l2(l2) {}
    template <typename T>
    bool operator()(const T* const scales, T* residual) const
    {
        residual[0] = T(l2) * ceres::sqrt(T(1.0) / scales[0]);
        return true;
    }

    static ceres::CostFunction* Create(const double l2=1e-2)
    {
        return (new ceres::AutoDiffCostFunction<D2CostFunctor3, 1, 1>(new D2CostFunctor3(l2)));
    }

private:
    double l2;
};

int RProblem2::solveProblem()
{
    const double *const matches = this->const_matches();
    const int num_matches = this->num_matches();
    const int num_cameras = this->num_cameras();

    // Create residuals for each observation in the bundle adjustment problem. The
    // parameters for cameras and points are added automatically.
    ceres::Problem problem;
    //Eprojection
    for (int i = 0; i < num_matches; ++i)
    {
        double a_x, a_y, a_d;
        double b_x, b_y, b_d;
        a_x = *(matches + 6 * i);
        a_y = *(matches + 6 * i + 1);
        a_d = *(matches + 6 * i + 2);
        b_x = *(matches + 6 * i + 3);
        b_y = *(matches + 6 * i + 4);
        b_d = *(matches + 6 * i + 5);

        ceres::CostFunction *costFunctor1_0 =
                D2CostFunctor1::Create(a_x, a_y, a_d,
                                       b_x, b_y, this->intrinsics);
        double *scales = this->mutable_scales(this->camera_i_index(i));
        double *offsets = this->mutable_offsets(this->camera_i_index(i));
        problem.AddResidualBlock(costFunctor1_0,
                                 new ceres::CauchyLoss(0.5) /* squared loss */,
                                 this->mutable_camera_i(i),
                                 this->mutable_camera_j(i),
                                 scales, offsets);

        //主要为了求解B图像的scales、offsets因子
        ceres::CostFunction *costFunctor1_1 =
                D2CostFunctor1::Create(b_x, b_y, b_d,
                                       a_x, a_y, this->intrinsics);
        scales = this->mutable_scales(this->camera_j_index(i));
        offsets = this->mutable_offsets(this->camera_j_index(i));
        problem.AddResidualBlock(costFunctor1_1,
                                 new ceres::CauchyLoss(0.5),
                                 this->mutable_camera_j(i),
                                 this->mutable_camera_i(i),
                                 scales, offsets);


    }

    //l1, l2最终会经过平方
    double depth_expande = 1.0f;
    std::cout << "num_matches : " << num_matches << std::endl;
    for (int c = 0; c < num_cameras; ++c)
    {
        double *scales = this->mutable_scales(c);
        double *offsets = this->mutable_offsets(c);
        //Esmothness

        //Escale
        double l2 = 1e-4 * depth_expande;//1e-2, 5,

        ceres::CostFunction *costFunctor3 = D2CostFunctor3::Create(l2);
        problem.AddResidualBlock(costFunctor3, NULL, scales);
    }

    // Make Ceres automatically detect the bundle structure. Note that the
    // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
    // for standard bundle adjustment problems.
    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type = ceres::DENSE_QR;//ceres::DENSE_SCHUR; //ceres::LEVENBERG_MARQUARDT;

    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = iter_n;//100;
    options.num_threads = 4;
    options.num_linear_solver_threads = 4;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //std::cout << summary.FullReport() << "\n";
    return 0;
}