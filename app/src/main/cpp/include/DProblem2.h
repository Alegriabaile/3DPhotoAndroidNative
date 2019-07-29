//
// Created by ale on 19-5-4.
//

#ifndef I3D_DPROBLEM2_H
#define I3D_DPROBLEM2_H

//align the disparity to depth.
////////////////////////// d' = 1/(scale*d+offset)

#include "i3d.h"
#include "Frame.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

class DProblem2
{
public:
    DProblem2(std::vector<i3d::Frame> & kframes, std::vector<i3d::Edge> & kedges,
            i3d::Intrinsics & intrinsics, int gridw = 2, int gridh = 2, int iter_num = 10);
    ~DProblem2();


private:
    int initParameters(const std::vector<i3d::Frame> & kframes,
            const std::vector<i3d::Edge> & edges, const i3d::Intrinsics & intrinsics);
    int updateParameters(std::vector<i3d::Frame> & kframes);
    int solveProblem();

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
    int bilinearWeight(double w, double h, double cols, double rows,
                       double& w0, double&w1, double& w2, double& w3)
    {
        if(grid_w<=1 || grid_h<=1)
            return -1;

        int k0 = std::floor(w/cols*(grid_w-1))*grid_h + std::floor(h/rows*(grid_h-1));
        //求双线性插值权重
        double x1 = cols/(grid_w-1) * std::floor(w/cols*(grid_w-1));
        double y1 = rows/(grid_h-1) * std::floor(h/rows*(grid_h-1));
        double x2 = x1 + cols/(grid_w-1);
        double y2 = y1 + rows/(grid_h-1);

        w0 = (x2-w)*(y2-h)/( (x2-x1)*(y2-y1));
        w1 = (x2-w)*(h-y1)/( (x2-x1)*(y2-y1));
        w2 = (w-x1)*(y2-h)/( (x2-x1)*(y2-y1));
        w3 = (w-x1)*(h-y1)/( (x2-x1)*(y2-y1));

        return k0;
    }
    bool operator()(int num_matches_, int num_cameras_, i3d::Intrinsics intrinsics1)
    {
        this->num_matches_  = num_matches_;
        this->num_cameras_ = num_cameras_;
        this->intrinsics = intrinsics1;

        delete[] camera_i_; delete[] camera_j_;
        delete[] matches_; delete[] parameters_;
        delete[] scales_; delete[] offsets_;

        camera_i_ = new int[num_matches_]; camera_j_ = new int[num_matches_];
        matches_  = new double[num_matches_*6];//a_xyd, b_xyd
        parameters_ = new double[num_cameras_*6];//rotate_angles, translation
        scales_ = new double[num_cameras_*grid_w*grid_h];
        offsets_ = new double[num_cameras_*grid_w*grid_h];
        return true;
    }

    //release memory
    int clean_up()
    {
        //if(camera_i_ != nullptr)//可以delete[]空指针
        delete[] camera_i_;
        //if(camera_j_ != nullptr)
        delete[] camera_j_;
        //if(matches_ != nullptr)
        delete[] matches_;
        //if(parameters_ != nullptr)
        delete[] parameters_;

        delete[] scales_;
        delete[] offsets_;

        return 0;
    }

    int camera_i_index(int k) const { return camera_i_[k];}
    int camera_j_index(int k) const { return camera_j_[k];}
    int num_matches() const { return num_matches_; }
    int num_cameras() const { return num_cameras_; }
    const double * const const_matches() const { return matches_; }

    double* mutable_cameras() { return parameters_; }
    double* mutable_scales(int k){ return scales_ + grid_w*grid_h*k;}
    double* mutable_offsets(int k){ return offsets_ + grid_w*grid_h*k;}

    double* mutable_camera_i(int k){ return mutable_cameras() + camera_i_[k] * 6;}
    double* mutable_camera_j(int k){ return mutable_cameras() + camera_j_[k] * 6;}

    int point_transfer(const cv::Vec3f& point0, const cv::Vec3f& rvec, const cv::Vec3f& tvec, cv::Vec3f& point1);
    int depth_transfer(const double *const scales, const double *const offsets, cv::Mat & depth);

/////////////////////////////variables declaring/////////////////////////////

private:
    int grid_w;
    int grid_h;

    int num_cameras_;
    int num_matches_;

    int* camera_i_;
    int* camera_j_;
    double* matches_;
    //mutable data, (Independent variables to be optimized)
    double* parameters_;//camera extrisic
    double* scales_;
    double* offsets_;

    int iter_n;//ceres lib迭代次数

    i3d::Intrinsics intrinsics;
/////////////////////////////////////////////////////////////////////////
};


#endif //I3D_DPROBLEM2_H
