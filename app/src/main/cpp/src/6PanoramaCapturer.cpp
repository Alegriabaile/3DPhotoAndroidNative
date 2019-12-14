//
// Created by ale on 19-12-10.
//

#include "6PanoramaCapturer.h"

namespace m3d
{
    PanoramaCapturer::PanoramaCapturer(std::vector<i3d::Frame>& kframes_, const i3d::Intrinsics& intrinsics_)
    : kframes(kframes_), intrinsics(intrinsics_)
    {
        GeneratePanoramasFromScenes();
    }

    PanoramaCapturer::~PanoramaCapturer() {}

    size_t PanoramaCapturer::GenerateTriangles(const cv::Mat &depth, const double *const intrinsics,
                                             std::vector<float> &vertices, cv::Mat &radius)
    {
        if(radius.empty())
            radius = cv::Mat(depth.size(), CV_32FC1, cv::Scalar(0.0f));
        const float MAX_DIFF = 0.05;//相邻像素深度值可相差的最大比率

        const float rows = depth.rows;
        const float cols = depth.cols;

        float fx = intrinsics[0];
        float fy = intrinsics[1];
        float cx = intrinsics[2];
        float cy = intrinsics[3];

        fx = fx * cols / cx / 2.0f;
        fy = fy * rows / cy / 2.0f;
        cx = cols / 2.0f;
        cy = rows / 2.0f;

        vertices.clear();
        vertices.reserve((rows - 1) * (cols - 1) * 6 * 5);

        float x, y, z, u, v;//3d coordinates and uv texture coordinates
        for (float h = 0; h < rows - 1; ++h)
        {
            for (float w = 0; w < cols - 1; ++w)
            {
                //参考了 https://github.com/simonfuhrmann/mve/blob/master/libs/mve/depthmap.cc#L211
                //0 1
                //2 3
                float d[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                d[0] = depth.at<float>(h, w);//depth of the current point
                d[1] = depth.at<float>((int) h, (int) w + 1);//depth of the right
                d[2] = depth.at<float>((int) h + 1, (int) w);//depth of the bottom
                d[3] = depth.at<float>((int) h + 1, (int) w + 1);//depth of the right bottom
                //At least three valid depth values are required.
                int positive_d_n = int(d[0] > 0) + int(d[1] > 0) + int(d[2] > 0) + int(d[3] > 0);
                if (positive_d_n < 3) continue;
                //3,1,2;  0,2,3;  3,1,0;  0,2,1
                int tri[4] = {0, 0, 0, 0};
                for (int i = 0; i < 4; ++i)//if positive_d_n==3,then
                {
                    if (!(d[i] > 0)) {
                        tri[i] = 1;
                        break;
                    }
                }
                if (positive_d_n == 4) {
                    if (fabs(d[0] - d[3]) > fabs(d[1] - d[2]))
                        tri[0] = tri[3] = 1;
                    else
                        tri[1] = tri[2] = 1;
                }

                int tris[4][3] = {
                        {3, 1, 2},
                        {0, 2, 3},
                        {3, 1, 0},
                        {0, 2, 1}
                };
                for (int i = 0; i < 4; ++i) {
                    if (tri[i] == 0)
                        continue;

                    int in0 = tris[i][0];
                    int in1 = tris[i][1];
                    int in2 = tris[i][2];
                    if (fabs(d[in0] - d[in1]) < MAX_DIFF * fmax(d[in0], d[in1])
                        && fabs(d[in1] - d[in2]) < MAX_DIFF * fmax(d[in1], d[in2])
                        && fabs(d[in2] - d[in0]) < MAX_DIFF * fmax(d[in2], d[in0])) {
                        for (int j = 0; j < 3; ++j) {
                            z = d[tris[i][j]];
                            x = z * (w + tris[i][j] % 2 - cx) / fx;
                            y = z * (h + tris[i][j] / 2 - cy) / fy;
                            u = (w + 0.5 + tris[i][j] % 2) / (float) (cols);
                            v = 1 - (h + 0.5 + tris[i][j] / 2) / (float) (rows);
                            vertices.push_back(x);
                            vertices.push_back(y);
                            vertices.push_back(z);
                            vertices.push_back(u);
                            vertices.push_back(v);
                        }
                    }
                }//end of current(h,w) position's triangle generation

                z = d[0];
                x = z * (w - cx) / fx;
                y = z * (h - cy) / fy;
                radius.at<float>((size_t)h, (size_t)w) = std::sqrt(x*x + y*y + z*z);
            }
        }//end of whole loop

        //三角形数目
        return vertices.size() / (5 * 3);
    }

    void PanoramaCapturer::transformMatrixFromExtrinsics(const double * const extrinsics, glm::mat4 & transformMat)
    {
        transformMat = glm::mat4(1.0f);
        //relative R, t.
        cv::Mat rVec = (cv::Mat_<double>(3, 1) << extrinsics[0], extrinsics[1], extrinsics[2]);
        cv::Mat R = cv::Mat(3,3,CV_64FC1);
        cv::Rodrigues(rVec, R);

        //glm: matlab type, vertical prior.
        for(size_t h = 0; h < 3; ++h)
            for(size_t w = 0; w < 3; ++w)
                transformMat[w][h] = R.at<double>(h, w);

        for(size_t  i = 0; i < 3; ++i)
            transformMat[3][i] = extrinsics[3+i];
    }

    void PanoramaCapturer::GenerateAABB(i3d::Frame &frame)
    {
        LOGE(" Warper4Android::GenerateAABB() : before");
        uint minw, minh, maxw, maxh;
        minw = minh = i3d::PANO_W;
        maxw = maxh = 0;

        int rows = frame.pano_depth.rows;
        int cols = frame.pano_depth.cols;

        cv::Mat pano_depth = frame.pano_depth;
        LOGE(" Warper4Android::GenerateAABB() : frame.pano_depth.rows, cols, empty?: %d, %d, %d", rows, cols, pano_depth.empty());
        for(int h=0; h<rows; ++h)
        {
            for(int w=0; w<cols; ++w)
            {
                //不能简单地减枝。。。
                if(pano_depth.at<float>(h, w) > 0.0f)
                {
                    minw = minw < w ? minw : w;
                    minh = minh < h ? minh : h;
                    maxw = maxw > w ? maxw : w;
                    maxh = maxh > h ? maxh : h;
                }
            }
        }
        LOGE(" Warper4Android::GenerateAABB() : minw, minh, maxw, maxh: %d, %d, %d, %d", minw, minh, maxw, maxh);
        frame.minw = minw;
        frame.minh = minh;
        frame.maxw = maxw;
        frame.maxh = maxh;

        cv::Mat pano_error = cv::Mat(rows, cols, CV_32FC1, cv::Scalar(0.0f));
        frame.pano_image = frame.pano_image(cv::Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();
        frame.pano_depth = frame.pano_depth(cv::Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();
        frame.pano_error = pano_error(cv::Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();

        LOGE(" Warper4Android::GenerateAABB() : after");

    }

    void PanoramaCapturer::GeneratePanoramasFromScenes()
    {
        WarperEglManager warperEglManager;
        m3d::CubemapCapturer cubemapCapturer(i3d::PANO_H);
        m3d::Cubemap2Sphere cubemap2Sphere(i3d::PANO_H, i3d::PANO_W);

        for(size_t i = 0; i < kframes.size(); ++i)
        {
            cv::Mat &color = kframes[i].image;
            cv::Mat &depth = kframes[i].depth;
            std::vector<float> vertices;
            cv::Mat radius;//use or not to use...
            double intriK[4] = {intrinsics.f, intrinsics.f, intrinsics.cx, intrinsics.cy};
            GenerateTriangles(depth, intriK, vertices, radius);

            glm::mat4 modelMat;
            double rts[6] = {kframes[i].rx, kframes[i].ry, kframes[i].rz,
                             kframes[i].tx, kframes[i].ty, kframes[i].tz};
            transformMatrixFromExtrinsics( rts, modelMat);
            cubemapCapturer.draw(vertices, color, radius /*or depth?*/, modelMat);

            const GLuint *colorAttaches = cubemapCapturer.colorAttatches();
            const GLuint *depthAttaches = cubemapCapturer.depthAttatches();
            cv::Mat &pano_image = kframes[i].pano_image;
            cv::Mat &pano_depth = kframes[i].pano_depth;
            cubemap2Sphere.draw(colorAttaches, depthAttaches, pano_image, pano_depth);

            GenerateAABB(kframes[i]);
        }

    }




}