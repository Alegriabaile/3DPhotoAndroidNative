//
// Created by ale on 19-7-4.
//

#ifndef I3D_RPROBLEM3_H
#define I3D_RPROBLEM3_H
////////////////////////// d' = scale*d+offset

#include "i3d.h"
#include "Frame.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

class RProblem3
{
public:
    RProblem3(std::vector<i3d::Frame> & kframes, std::vector<i3d::Edge> & kedges,
              i3d::Intrinsics & intrinsics, int iter_num = 10);
    ~RProblem3();


private:
    int initParameters(const std::vector<i3d::Frame> & kframes,
                       const std::vector<i3d::Edge> & edges,
                       const i3d::Intrinsics & intrinsics);
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
        scales_ = new double[num_cameras_];
        offsets_ = new double[num_cameras_];
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
    double* mutable_scales(int k){ return scales_ + k;}
    double* mutable_offsets(int k){ return offsets_ + k;}

    double* mutable_camera_i(int k){ return mutable_cameras() + camera_i_[k] * 6;}
    double* mutable_camera_j(int k){ return mutable_cameras() + camera_j_[k] * 6;}

    int point_transfer(const cv::Vec3f& point0, const cv::Vec3f& rvec, const cv::Vec3f& tvec, cv::Vec3f& point1);
    int depth_transfer(const double & scale, const double & offset, cv::Mat& depth);

/////////////////////////////variables declaring/////////////////////////////

private:
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

#endif //I3D_RPROBLEM3_H
