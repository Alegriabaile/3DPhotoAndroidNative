//
// Created by ale on 19-12-10.
//

#ifndef MY3DPHOTO_6PANORAMACAPTURER_H
#define MY3DPHOTO_6PANORAMACAPTURER_H

#include "i3d.h"
#include "Frame.h"

#include "WarperEglManager.h"
#include "6CubemapCapturer.h"
#include "6Cubemap2Sphere.h"

namespace m3d
{
    class PanoramaCapturer
    {
    private:
//        const m3d::Graph &graph;
//        std::vector<m3d::Frame> &frames;

        const i3d::Intrinsics &intrinsics;
        std::vector<i3d::Frame> &kframes;

        size_t GenerateTriangles( const cv::Mat &depth,
                                  const double * const intrinsics,//intrinsics[4]
                                  std::vector<float> &vertices,
                                  cv::Mat &radius);
        void transformMatrixFromExtrinsics(const double * const extrinsics, glm::mat4 & transformMat);

        void GenerateAABB(i3d::Frame &frame);

        void GeneratePanoramasFromScenes();

    public:
        PanoramaCapturer( std::vector<i3d::Frame>& kframes, const i3d::Intrinsics& intrinsics);

        virtual ~PanoramaCapturer();

    };

}

#endif //MY3DPHOTO_6PANORAMACAPTURER_H
