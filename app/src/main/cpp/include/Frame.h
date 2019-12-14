//
// Created by ale on 18-12-18.
//

#ifndef I3D_FRAME_H
#define I3D_FRAME_H

#include "opencv2/opencv.hpp"
#include <vector>

namespace i3d
{
    class Frame
    {
    public:
        std::string imageFileName;
        std::string depthFileName;
        std::string paramFileName;

        cv::Mat image, depth, error, disparity;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptor;

        double rx, ry, rz, tx, ty, tz;
        //bool isKey;
        uint minw, minh, maxw, maxh;
        cv::Mat pano_image, pano_depth, pano_error, pano_disparity, pano_normal;
        cv::Mat pano_depth_b, pano_image_b, pano_normal_b;
        cv::Mat pano_label, mask_label, mask_bound, soft_mask;//20190524---feathering...

        Frame()
                : rx(0), ry(0), rz(0)
                , tx(0), ty(0), tz(0)
                //, isKey(_frame.isKey)
                , minw(0), minh(0), maxw(0), maxh(0)
        {}

        Frame(const Frame& _frame)
                : image(_frame.image), depth(_frame.depth), error(_frame.error)
                , keypoints(_frame.keypoints)
                , descriptor(_frame.descriptor)
                , rx(_frame.rx), ry(_frame.ry), rz(_frame.rz)
                , tx(_frame.tx), ty(_frame.ty), tz(_frame.tz)
                //, isKey(_frame.isKey)
                , minw(_frame.minw), minh(_frame.minh), maxw(_frame.maxw), maxh(_frame.maxh)
                , pano_image(_frame.pano_image)
                , pano_depth(_frame.pano_depth)
                , pano_error(_frame.pano_error)
                , pano_normal(_frame.pano_normal)
                , pano_disparity(_frame.pano_disparity)
                , pano_depth_b(_frame.pano_depth_b)
                , pano_image_b(_frame.pano_image_b)
                , pano_normal_b(_frame.pano_normal_b)
                , pano_label(_frame.pano_label)
                , mask_bound(_frame.mask_bound)
                , mask_label(_frame.mask_label)
                , soft_mask(_frame.soft_mask) {}

    };

}

namespace m3d
{
    template<typename T>
    class Intrinsic
    {
    private:
        T intri_c[4];//fx, fy, ppx, ppy;
        T intri_d[4];
    public:
//        Intrinsic():fx(0), fy(0), ppx(0), ppy(0) {}
        Intrinsic(T _fx_c = 0, T _fy_c = 0, T _ppx_c = 0, T _ppy_c = 0) {
            intri_c[0] = _fx_c;
            intri_c[1] = _fy_c;
            intri_c[2] = _ppx_c;
            intri_c[3] = _ppy_c;

            for (int i = 0; i < 4; ++i)
                intri_d[i] = intri_c[i];
        }

        Intrinsic(T _intrinsic_c[4], T _intrinsic_d[4]) {
            for (int i = 0; i < 4; ++i)
                intri_c[i] = _intrinsic_c[i];
            for (int i = 0; i < 4; ++i)
                intri_d[i] = _intrinsic_d[i];
        }

        virtual ~Intrinsic() {}

        void setIntrinsic(T _fx_c = 0, T _fy_c = 0, T _ppx_c = 0, T _ppy_c = 0) {
            intri_c[0] = _fx_c;
            intri_c[1] = _fy_c;
            intri_c[2] = _ppx_c;
            intri_c[3] = _ppy_c;

            for (int i = 0; i < 4; ++i)
                intri_d[i] = intri_c[i];
        }

        void setIntrinsic(T _intrinsic_c[4], T _intrinsic_d[4]) {
            for (int i = 0; i < 4; ++i)
                intri_c[i] = _intrinsic_c[i];
            for (int i = 0; i < 4; ++i)
                intri_d[i] = _intrinsic_d[i];
        }

        T *intrinsic_c() { return intri_c; }

        T *intrinsic_d() { return intri_d; }

    };
    typedef Intrinsic<double> IntrinsicD;
}

#endif //I3D_FRAME_H
