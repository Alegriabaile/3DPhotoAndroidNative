//
// Created by ale on 19-2-28.
//

#ifndef I3D_DEFORMABLEPROBLEM_H
#define I3D_DEFORMABLEPROBLEM_H
//
// Created by ale on 19-2-21.
//
#include "i3d.h"
#include "Frame.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

namespace i3d_dp {
    template<int gridw, int gridh>
    class DeformableProblem {
    public:
        i3d::Intrinsics intrinsics;

        ~DeformableProblem() {
            if (camera_i_ != nullptr)
                delete[] camera_i_;
            if (camera_j_ != nullptr)
                delete[] camera_j_;
            if (matches_ != nullptr)
                delete[] matches_;
            if (parameters_ != nullptr)
                delete[] parameters_;
            if (scales_ != nullptr)
                delete[] scales_;
            if (offsets_ != nullptr)
                delete[] offsets_;

            camera_i_ = camera_j_ = nullptr;
            matches_ = parameters_ = scales_ = offsets_ = nullptr;
        }

        DeformableProblem()
                : num_matches_(0), num_cameras_(0), camera_i_(nullptr), camera_j_(nullptr),
                  matches_(nullptr), parameters_(nullptr), scales_(nullptr), offsets_(nullptr),
                  grid_w(gridw), grid_h(gridh) {}

        DeformableProblem(std::vector<i3d::Frame> &kframes, std::vector<i3d::Edge> &kedges,
                          i3d::Intrinsics &intrinsics)
                : num_matches_(0), num_cameras_(0), camera_i_(nullptr), camera_j_(nullptr),
                  matches_(nullptr), parameters_(nullptr), scales_(nullptr), offsets_(nullptr),
                  grid_w(gridw), grid_h(gridh) {
            LOGE("DeformableProblem()::before initParameters()...");
            initParameters(kframes, kedges, intrinsics);
            LOGE("DeformableProblem()::before solveProblem()...");
            solveProblem();
            LOGE("DeformableProblem()::before updateParameters()...");
            updateParameters(kframes);
            LOGE("DeformableProblem()::after updateParameters()...");
        }

        int
        initParameters(const std::vector<i3d::Frame> &kframes, const std::vector<i3d::Edge> &edges,
                       const i3d::Intrinsics &intrinsics);

        int updateParameters(std::vector<i3d::Frame> &kframes);

        int solveProblem();

    private:
        int insert(const int k, const int i, const int j, const double *matches) {

            if (k < 0 || k >= num_matches_) {
                LOGE("insert():: k<0 || k>=num_matches_)");
                return -1;
            }


            if (i < 0 || i > num_cameras_ || j < 0 || j > num_cameras_) {
                LOGE("insert():: i<0 || i>num_cameras_ || j<0 || j>num_cameras_");
                return -2;
            }

//        LOGE("insert():: for(int index = 0; index<6; index++)");
            for (int index = 0; index < 6; index++)
                *(matches_ + k * 6 + index) = matches[index];

//        LOGE("insert():: camera_i_[k] = i; camera_j_[k] = j;");
            camera_i_[k] = i;
            camera_j_[k] = j;

//        LOGE("insert():: finish...");

            return 0;
        }

        int bilinearWeight(double w, double h, double cols, double rows, double &w0, double &w1,
                           double &w2, double &w3) {
            int k0 = std::floor(w / cols * (grid_w - 1)) * grid_h +
                     std::floor(h / rows * (grid_h - 1));
            //求双线性插值权重
            double x1 = cols / (grid_w - 1) * std::floor(w / cols * (grid_w - 1));
            double y1 = rows / (grid_h - 1) * std::floor(h / rows * (grid_h - 1));
            double x2 = x1 + cols / (grid_w - 1);
            double y2 = y1 + rows / (grid_h - 1);

            w0 = (x2 - w) * (y2 - h) / ((x2 - x1) * (y2 - y1));
            w1 = (x2 - w) * (h - y1) / ((x2 - x1) * (y2 - y1));
            w2 = (w - x1) * (y2 - h) / ((x2 - x1) * (y2 - y1));
            w3 = (w - x1) * (h - y1) / ((x2 - x1) * (y2 - y1));

            return k0;
        }

        bool operator()(int num_matches_, int num_cameras_, i3d::Intrinsics intrinsics1) {
//        LOGE("operator(): start... ");
            this->num_matches_ = num_matches_;
            this->num_cameras_ = num_cameras_;
            this->intrinsics = intrinsics1;

//        LOGE("operator(): delete !nullptr... ");
            if (camera_i_ != nullptr)
                delete[] camera_i_;
            if (camera_j_ != nullptr)
                delete[] camera_j_;
            if (matches_ != nullptr)
                delete[] matches_;
            if (parameters_ != nullptr)
                delete[] parameters_;
            if (scales_ != nullptr)
                delete[] scales_;
            if (offsets_ != nullptr)
                delete[] offsets_;

//        LOGE("operator(): !nullptr = nullptr... ");
            camera_i_ = camera_j_ = nullptr;
            matches_ = parameters_ = scales_ = offsets_ = nullptr;

//        LOGE("operator(): new sth... num_matches_:%d, num_cameras:%d, gridw:%d, gridh:%d", num_matches_, num_cameras_, gridw, gridh);
            camera_i_ = new int[num_matches_];
            camera_j_ = new int[num_matches_];
            matches_ = new double[num_matches_ * 6];//a_xyd, b_xyd
            parameters_ = new double[num_cameras_ * 6];//rotate_angles, translation
            scales_ = new double[num_cameras_ * gridw * gridh];
            offsets_ = new double[num_cameras_ * gridw * gridh];
//        LOGE("operator(): finish... ");
            return true;
        }

        int camera_i_index(int k) const { return camera_i_[k]; }

        int camera_j_index(int k) const { return camera_j_[k]; }

        int num_matches() const { return num_matches_; }

        int num_cameras() const { return num_cameras_; }

        const double *const const_matches() const { return matches_; }

        double *mutable_cameras() { return parameters_; }

        double *mutable_scales(int k) { return scales_ + gridw * gridh * k; }

        double *mutable_offsets(int k) { return offsets_ + gridw * gridh * k; }

        double *mutable_camera_i(int k) { return mutable_cameras() + camera_i_[k] * 6; }

        double *mutable_camera_j(int k) { return mutable_cameras() + camera_j_[k] * 6; }

        int point_transfer(const cv::Vec3f &point0, const cv::Vec3f &rvec, const cv::Vec3f &tvec,
                           cv::Vec3f &point1);

        int depth_transfer(const double *const scales, const double *const offsets, cv::Mat &depth);

/////////////////////////////variables declaring/////////////////////////////
    public:
        const double grid_w;
        const double grid_h;

    private:
        int num_cameras_;
        int num_matches_;

        int *camera_i_;
        int *camera_j_;
        double *matches_;
        //mutable data, (Independent variables to be optimized)
        double *parameters_;//camera extrisic
        double *scales_;
        double *offsets_;
/////////////////////////////////////////////////////////////////////////
    };


//////////////////////////模板代码不能放到cpp文件中，因为其模板参数必须在编译时给出，而cpp文件在编译时已经被编译为源代码////////////////////
    struct double_6 {
        double ax, ay, ad, bx, by, bd;
    };
    const double d_scale = 1;

    template<int gridw, int gridh>
    int DeformableProblem<gridw, gridh>::initParameters(
            const std::vector<i3d::Frame> &kframes,
            const std::vector<i3d::Edge> &edges,
            const i3d::Intrinsics &intrinsics) {
//    LOGE("initParameters(): start...");
        std::vector<struct double_6> matches;
        std::vector<int> camera_i, camera_j;

        if (kframes.size() < 1)
            return -1;

//    LOGE("initParameters(): ratioh and ratiow, from edges to matches...");
        float ratioh = (float) kframes[0].depth.rows / (float) kframes[0].image.rows;
        float ratiow = (float) kframes[0].depth.cols / (float) kframes[0].image.cols;
        for (int i = 0; i < edges.size(); ++i) {
            //cout<<"matches"<<i<<","<<j<<endl;
            for (int k = 0; k < edges[i].psrc.size(); ++k) {
                //cout<<"camera_i/j .push_back()"<<k<<endl;
                camera_i.push_back(edges[i].src);
                camera_j.push_back(edges[i].dst);

                //cout<<"matches.push_back()"<<endl;
                struct double_6 temp_match;// not safe
                temp_match.ax = edges[i].psrc[k].x;//x,y为image坐标空间，而d为depth坐标空间
                temp_match.ay = edges[i].psrc[k].y;
                temp_match.ad = kframes[edges[i].src].depth.at<float>
                        (floor(temp_match.ay * ratioh), floor(temp_match.ax * ratiow)) / d_scale;
                temp_match.bx = edges[i].pdst[k].x;
                temp_match.by = edges[i].pdst[k].y;
                temp_match.bd = kframes[edges[i].dst].depth.at<float>
                        (floor(temp_match.by * ratioh), floor(temp_match.bx * ratiow)) / d_scale;
                matches.push_back(temp_match);
            }
        }

//    LOGE("initParameters(): intrinsics...");
        i3d::Intrinsics _intrinsics;
        _intrinsics.cx = kframes[0].image.cols / 2;
        _intrinsics.cy = kframes[0].image.rows / 2;
        _intrinsics.f = intrinsics.f * _intrinsics.cx / intrinsics.cx;
        _intrinsics.colorw = intrinsics.colorw;
        _intrinsics.colorh = intrinsics.colorh;
        _intrinsics.depthw = intrinsics.depthw;
        _intrinsics.depthh = intrinsics.depthh;


//    LOGE("initParameters(): initialize matches to this,,, operator()... ");
        this->operator()(matches.size(), kframes.size(), _intrinsics);

//    LOGE("initParameters(): initialize matches to this,,, insert()... ");
        for (int k = 0; k < matches.size(); ++k) {
//        LOGE("initParameters(): k=%d<matches.size()... ", k);
            double match[6];
            match[0] = matches[k].ax;
            match[1] = matches[k].ay;
            match[2] = matches[k].ad;
            match[3] = matches[k].bx;
            match[4] = matches[k].by;
            match[5] = matches[k].bd;
            this->insert(k, camera_i[k], camera_j[k], match);
            //cout<<"delete:"<<k<<endl;
        }
//    LOGE("initParameters(): initialize cameras to this... ");
        for (int k = 0; k < kframes.size(); ++k) {
            double *cameras = this->mutable_cameras();
            cameras[k * 6] = kframes[k].rx;
            cameras[k * 6 + 1] = kframes[k].ry;
            cameras[k * 6 + 2] = kframes[k].rz;
            cameras[k * 6 + 3] = kframes[k].tx;
            cameras[k * 6 + 4] = kframes[k].ty;
            cameras[k * 6 + 5] = kframes[k].tz;
        }
//    LOGE("initParameters(): initialize scales and offsets to this... ");
        for (int k = 0; k < kframes.size(); ++k) {
            double *scales = this->mutable_scales(k);
            double *offsets = this->mutable_offsets(k);
            for (int i = 0; i < gridw * gridh; ++i) {
                scales[i] = 0.1;
                offsets[i] = 0.0;
            }
        }

//    LOGE("initParameters(): finish... ");
        return 0;
    }

    template<int gridw, int gridh>
    int
    DeformableProblem<gridw, gridh>::point_transfer(const cv::Vec3f &point0, const cv::Vec3f &rvec,
                                                    const cv::Vec3f &tvec, cv::Vec3f &point1) {
        //Rotate and Translate matrix.
        //matrix: |Point1; 1| = |R, t|*|point0; 1|
        cv::Mat R = (cv::Mat_<double>(3, 3));
        cv::Mat rv = (cv::Mat_<double>(3, 1) << rvec[0], rvec[1], rvec[2]);
        cv::Rodrigues(rv, R);
        double x = point0[0];
        double y = point0[1];
        double z = point0[2];

        point1[0] =
                R.at<double>(0, 0) * x + R.at<double>(0, 1) * y + R.at<double>(0, 2) * z + tvec[0];
        point1[1] =
                R.at<double>(1, 0) * x + R.at<double>(1, 1) * y + R.at<double>(1, 2) * z + tvec[1];
        point1[2] =
                R.at<double>(2, 0) * x + R.at<double>(2, 1) * y + R.at<double>(2, 2) * z + tvec[2];

        //cout<<R<<point1<<endl;
        return 0;
    }

#define DEPTH_TRANSFER_DEBUG

    template<int gridw, int gridh>
    int DeformableProblem<gridw, gridh>::depth_transfer(const double *const scales,
                                                        const double *const offsets,
                                                        cv::Mat &depth) {
#ifdef DEPTH_TRANSFER_DEBUG
        using namespace std;
        LOGE("depth_transfer():  scales and offsets:  ...............");
        for (int j = 0; j < gridw * gridh; ++j) {
            LOGE("%lf ", scales[j]);//"("<<std::atan(scales[j])/(3.141592654)+(0.5)<<")"<<" ";
        }

        for (int j = 0; j < gridw * gridh; ++j) {
            LOGE("%lf ", offsets[j]);//"("<<std::atan(offsets[j])/(3.141592654)+(0.5)<<")"<<" ";
        }

        double maxVal, minVal;
        minMaxLoc(depth, &minVal, &maxVal);
        LOGE("original min, max: %lf, %lf", minVal, maxVal);
#endif
        double rows = depth.rows;
        double cols = depth.cols;
        for (double h = 0; h < rows; ++h)
            for (double w = 0; w < cols; ++w) {

                double w0, w2;
                double w1, w3;
                int k0 = bilinearWeight(w, h, cols, rows, w0, w1, w2, w3);
                double scale = scales[k0] * w0 + scales[k0 + 1] * w1 + scales[k0 + gridh] * w2 +
                               scales[k0 + gridh + 1] * w3;
                double offset = offsets[k0] * w0 + offsets[k0 + 1] * w1 + offsets[k0 + gridh] * w2 +
                                offsets[k0 + gridh + 1] * w3;

                double d = depth.at<float>(h, w);
                double d_new = d / (scale + offset * d);
                depth.at<float>(h, w) = d_new;
            }
#ifdef DEPTH_TRANSFER_DEBUG
        minMaxLoc(depth, &minVal, &maxVal);
        LOGE("original min, max: %lf, %lf", minVal, maxVal);
#endif
        return 0;
    }

#undef DEPTH_TRANSFER_DEBUG

    template<int gridw, int gridh>
    int DeformableProblem<gridw, gridh>::updateParameters(std::vector<i3d::Frame> &kframes) {
        const double *parameters = this->mutable_cameras();
        for (int i = 0; i < kframes.size(); ++i) {
            kframes[i].rx = parameters[i * 6];
            kframes[i].ry = parameters[i * 6 + 1];
            kframes[i].rz = parameters[i * 6 + 2];
            kframes[i].tx = parameters[i * 6 + 3];
            kframes[i].ty = parameters[i * 6 + 4];
            kframes[i].tz = parameters[i * 6 + 5];
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
//    uint ksize = kframes.size();
//    std::vector<cv::Mat> N(ksize);
//    std::vector<cv::Mat> NNT(ksize);
//    std::vector<cv::Mat> NNTo(ksize);
//    cv::Mat A = cv::Mat::zeros(3,3,CV_32FC1);
//    cv::Mat b = cv::Mat(3,1,CV_32FC1, cv::Scalar(0));
//    for(int i=0; i<kframes.size(); ++i)
//    {
//        cv::Vec3f point0(0,0,100);
//        cv::Vec3f orig0(0,0,0);
//        cv::Vec3f point1, orig1;
//        cv::Vec3f rvec, tvec;
//
//        rvec[0] = parameters[i*6]; rvec[1] = parameters[i*6+1]; rvec[2] = parameters[i*6+2];
//        tvec[0] = parameters[i*6+3]; tvec[1] = parameters[i*6+4]; tvec[2] = parameters[i*6+5];
//
//        point_transfer(point0, rvec, tvec, point1);
//        point_transfer(orig0, rvec, tvec, orig1);
//
//        cv::Vec3f n(normalize(point1-orig1));
//        N[i] = (cv::Mat_<float>(3,3) <<
//                                 0, -n[2], n[1], n[2], 0, -n[0], -n[1], n[0], 0 );
//        cv::Mat o = (cv::Mat_<float>(3,1) <<  orig1[0], orig1[1], orig1[2]);
//        NNT[i] = N[i]*(-N[i]); NNTo[i] = NNT[i]*o;
//
//        A += NNT[i]; b += NNTo[i];
//        //cout<<orig1<<", "<<point1<<", "<<n<<";"<<endl;
//    }
//
//    cv::Mat A_inverse;
//    invert(A, A_inverse);
//    cv::Mat center = A_inverse*b;
//    //cout<<A<<endl;
//    //cout<<center<<endl;
//
//    double rx, ry, rz, tx, ty, tz;
//    rx = ry = rz = 0;
//    tx = center.at<float>(0); ty = center.at<float>(1); tz = center.at<float>(2);
//    for(int i=0; i<kframes.size(); ++i)
//    {
//        rx += kframes[i].rx; ry += kframes[i].ry; rz += kframes[i].rz;
//    }
//    for(int i=0; i<kframes.size(); ++i)
//    {
//        kframes[i].rx -= rx/kframes.size();
//        kframes[i].ry -= ry/kframes.size();
//        kframes[i].rz -= rz/kframes.size();
//        kframes[i].tx -= tx; kframes[i].ty -= ty; kframes[i].tz -= tz;
//    }

        for (int i = 0; i < kframes.size(); ++i) {
            double *scales = this->mutable_scales(i);
            double *offsets = this->mutable_offsets(i);
            this->depth_transfer(scales, offsets, kframes[i].depth);
        }

        return 0;
    }

////////////about ceres functor and solver////////////////////////////
    class CostFunctor1 {
    private:
        double a_x, a_y, a_d;
        double b_x, b_y, b_d;
        double f, cx, cy;

        double w0, w2;
        double w1, w3;

    public:
        //const values.
        CostFunctor1(
                double a_x, double a_y, double a_d,
                double b_x, double b_y,
                i3d::Intrinsics intrinsics,
                double w0, double w1, double w2, double w3)
                : a_x(a_x), a_y(a_y), a_d(a_d), b_x(b_x), b_y(b_y),
                  f(intrinsics.f * intrinsics.colorw / intrinsics.cx / 2),
                  cx(intrinsics.colorw / 2), cy(intrinsics.colorh / 2), w0(w0), w1(w1), w2(w2),
                  w3(w3) {}

        // camera[0,1,2] are the angle-axis rotation.
        // camera[3,4,5] are the translation.
        template<typename T>
        bool operator()(const T *const camera_a, const T *const camera_b,
                        const T *const scales_a0, const T *const scales_a1,
                        const T *const scales_a2, const T *const scales_a3,
                        const T *const offsets_a0, const T *const offsets_a1,
                        const T *const offsets_a2, const T *const offsets_a3,
                        T *residuals) const {
            //a_x, a_y, b_x, b_y, based on color image size

            //Bilinear Interpolation
            T scale_weight =
                    scales_a0[0] * w0 + scales_a1[0] * w1 + scales_a2[0] * w2 + scales_a3[0] * w3;
            T offset_weight = offsets_a0[0] * w0 + offsets_a1[0] * w1 + offsets_a2[0] * w2 +
                              offsets_a3[0] * w3;
            //T a_d_ = T(1.0) / (scale_weight * (T(1.0) /T(a_d)) + offset_weight );
            T a_d_ = T(a_d) / (scale_weight + offset_weight * T(a_d));

            //i_point's 3d position in camera_a's coordinate.
            T point[3];
            point[0] = T(a_d_ * (a_x - cx) / f);
            point[1] = T(a_d_ * (a_y - cy) / f);
            point[2] = T(a_d_);

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
            const T x = point[0] / point[2];
            const T y = point[1] / point[2];
            T a_u = T(f) * x + T(cx);// u = (f*x + cx*z)/d
            T a_v = T(f) * y + T(cy);// v = (f*y + cy*z)/d

            // The error is the difference between the 2d points aligned in camera_b, difference(Pa->b(i_point), j_point) ).
            T b_u = T(b_x);
            T b_v = T(b_y);

            residuals[0] = a_u - b_u;
            residuals[1] = a_v - b_v;
            return true;
        }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction *Create(const double a_x, const double a_y, const double a_d,
                                           const double b_x, const double b_y,
                                           i3d::Intrinsics intrinsics,
                                           const double w0, const double w1, const double w2,
                                           const double w3) {
            return (new ceres::AutoDiffCostFunction<CostFunctor1, 2, 6, 6, 1, 1, 1, 1, 1, 1, 1, 1>(
                    new CostFunctor1(a_x, a_y, a_d, b_x, b_y, intrinsics, w0, w1, w2, w3)));
        }

    };

//E_smoothness
    struct CostFunctor2 {
        CostFunctor2(double l1_ = 1e3) : l1(l1_) {}

        //在一个加了const限定符的成员函数中，不能够调用 非const成员函数
        template<typename T>
        bool operator()(const T *const scales1, const T *const scales2,
                        const T *const offsets1, const T *const offsets2, T *residual) const {
            residual[0] = T(l1) * (scales1[0] - scales2[0]);
            residual[1] = T(l1) * (offsets1[0] - offsets2[0]);
            return true;
        }

        static ceres::CostFunction *Create(const double l1 = 1e3) {
            return (new ceres::AutoDiffCostFunction<CostFunctor2, 2, 1, 1, 1, 1>(
                    new CostFunctor2(l1)));
        }

    private:
        double l1;
    };


//E_scale
    struct CostFunctor3 {
        CostFunctor3(double l2 = 1e-2) : l2(l2) {}

        template<typename T>
        bool operator()(const T *const scales, T *residual) const {
            residual[0] = T(l2) * ceres::sqrt(T(1.0) / scales[0]);
            return true;
        }

        static ceres::CostFunction *Create(const double l2 = 1e-2) {
            return (new ceres::AutoDiffCostFunction<CostFunctor3, 1, 1>(new CostFunctor3(l2)));
        }

    private:
        double l2;
    };

    template<int gridw, int gridh>
    int DeformableProblem<gridw, gridh>::solveProblem() {
        const double *const matches = this->const_matches();
        const int num_matches = this->num_matches();
        const int num_cameras = this->num_cameras();

        // Create residuals for each observation in the bundle adjustment problem. The
        // parameters for cameras and points are added automatically.
        ceres::Problem problem;

        //Eprojection
        for (int i = 0; i < num_matches; ++i) {
            double a_x, a_y, a_d;
            double b_x, b_y, b_d;
            a_x = *(matches + 6 * i);
            a_y = *(matches + 6 * i + 1);
            a_d = *(matches + 6 * i + 2);
            b_x = *(matches + 6 * i + 3);
            b_y = *(matches + 6 * i + 4);
            b_d = *(matches + 6 * i + 5);

            double colorw, colorh;
            colorw = this->intrinsics.colorw;
            colorh = this->intrinsics.colorh;
            int k0;
            double w0, w2;
            double w1, w3;
            k0 = this->bilinearWeight(a_x, a_y, colorw, colorh, w0, w1, w2, w3);
            ceres::CostFunction *costFunctor1_0 =
                    CostFunctor1::Create(a_x, a_y, a_d,
                                         b_x, b_y, this->intrinsics,
                                         w0, w1, w2, w3);
            double *scales = this->mutable_scales(this->camera_i_index(i));
            double *offsets = this->mutable_offsets(this->camera_i_index(i));
            problem.AddResidualBlock(costFunctor1_0,
                                     new ceres::CauchyLoss(0.5) /* squared loss */,
                                     this->mutable_camera_i(i),
                                     this->mutable_camera_j(i),
                                     scales + k0, scales + k0 + 1, scales + k0 + gridh,
                                     scales + k0 + gridh + 1,
                                     offsets + k0, offsets + k0 + 1, offsets + k0 + gridh,
                                     offsets + k0 + gridh + 1);

            //主要为了求解B图像的scales、offsets因子
            k0 = this->bilinearWeight(b_x, b_y, colorw, colorh, w0, w1, w2, w3);
            ceres::CostFunction *costFunctor1_1 =
                    CostFunctor1::Create(b_x, b_y, b_d,
                                         a_x, a_y, this->intrinsics,
                                         w0, w1, w2, w3);
            scales = this->mutable_scales(this->camera_j_index(i));
            offsets = this->mutable_offsets(this->camera_j_index(i));
            problem.AddResidualBlock(costFunctor1_1,
                                     new ceres::CauchyLoss(0.5),
                                     this->mutable_camera_j(i),
                                     this->mutable_camera_i(i),
                                     scales + k0, scales + k0 + 1, scales + k0 + gridh,
                                     scales + k0 + gridh + 1,
                                     offsets + k0, offsets + k0 + 1, offsets + k0 + gridh,
                                     offsets + k0 + gridh + 1);
        }

        //l1, l2最终会经过平方
        //目前只适用于kinect深度
        const double depth_expande = sqrt(25 * 450 / double(gridw * gridh) /
                                          double(num_matches));//*num_matches/500.0);//*num_matches/500.0);//4/gridw/gridh;//sqrt(4*num_matches/1000/gridw/gridh);//sqrt(i3d::MAX_DEPTH_VALUE*4*num_matches/1000/gridw/gridh);
        LOGE("num_matches : %d ", num_matches);
        for (int c = 0; c < num_cameras; ++c) {
            double *scales = this->mutable_scales(c);
            double *offsets = this->mutable_offsets(c);
            //Esmothness
            double l1 = 1e3 * depth_expande;//
            for (int w = 0; w < gridw; ++w) {
                for (int h = 0; h < gridh - 1; ++h) {

                    int k = w * gridh + h;
                    ceres::CostFunction *costFunctor2_h =
                            CostFunctor2::Create(l1);
                    problem.AddResidualBlock(costFunctor2_h, NULL,
                                             scales + k, scales + k + 1,
                                             offsets + k, offsets + k + 1);
                }
            }
            for (int w = 0; w < gridw - 1; ++w) {
                for (int h = 0; h < gridh; ++h) {
                    int k = w * gridh + h;
                    ceres::CostFunction *costFunctor2_w =
                            CostFunctor2::Create(l1);
                    problem.AddResidualBlock(costFunctor2_w, NULL,
                                             scales + k, scales + k + gridh,
                                             offsets + k, offsets + k + gridh);
                }
            }

            //Escale
            double l2 = 1e-2 * depth_expande;
            for (int k = 0; k < gridw * gridh; ++k) {
                ceres::CostFunction *costFunctor3 =
                        CostFunctor3::Create(l2);
                problem.AddResidualBlock(costFunctor3, NULL, scales + k);
            }
        }

        // Make Ceres automatically detect the bundle structure. Note that the
        // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
        // for standard bundle adjustment problems.
        ceres::Solver::Options options;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.linear_solver_type = ceres::DENSE_QR;//ceres::DENSE_SCHUR; //ceres::LEVENBERG_MARQUARDT;

//    options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 50;//100;
//    options.num_threads = 4;
//    options.num_linear_solver_threads = 4;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
//    std::cout << summary.FullReport() << "\n";
        return 0;
    }
}

#endif //I3D_DEFORMABLEPROBLEM_H
