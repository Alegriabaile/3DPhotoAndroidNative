//
// Created by ale on 19-3-20.
//

#include "genTwoLayerContents.h"

using namespace i3d;
using namespace cv;
using namespace std;

static int GenerateNormalMap(const Mat& pano_depth, Mat& pano_normal)
{
    using namespace cv;
    const cv::Mat srcDepth = pano_depth;//CV_32FC1
    if(srcDepth.type() != CV_32FC1)
    {
        std::cout<<"GenerateNormalMap: depth.type is not CV_32FC1"<<endl;
        return -1;
    }


    //计算法向量
    Mat depth = Mat(srcDepth.size()+Size(2,2), CV_32FC1, Scalar(0.0f));
    Mat ROI=depth(Rect( 1, 1, srcDepth.cols, srcDepth.rows));//x,y,w,h    xy坐标，宽度，高度
    srcDepth.copyTo(ROI);
    if(depth.type() != CV_32FC1)
        depth.convertTo(depth, CV_32FC1);

    Mat normals(depth.size(), CV_32FC3);

    for(int x = 1; x < depth.rows-1; ++x)
    {
        for(int y = 1; y < depth.cols-1; ++y)
        {
            // use float instead of double otherwise you will not get the correct result
            // check my updates in the original post. I have not figure out yet why this
            // is happening.
            float dzdx = (depth.at<float>(x+1, y) - depth.at<float>(x-1, y)) / 2.0;
            float dzdy = (depth.at<float>(x, y+1) - depth.at<float>(x, y-1)) / 2.0;

            Vec3f d(-dzdx, -dzdy, 1.0f);

            Vec3f n = normalize(d);
            normals.at<Vec3f>(x, y) = n;
        }
    }

    normals = normals(Rect( 1, 1, srcDepth.cols, srcDepth.rows)).clone();
    pano_normal = normals.clone();

    return 0;
}


static int GenerateBackLayer(i3d::Frame& pano, const int icount=30)
{
    cv::Mat pano_image_f = pano.pano_image;
    cv::Mat pano_depth_f = pano.pano_depth;
    //init
    cv::Mat image_f(pano_image_f.size()+Size(icount*2, icount*2), CV_8UC3, Scalar(0,0,0));
    cv::Mat image_b(pano_image_f.size()+Size(icount*2, icount*2), CV_8UC3, Scalar(0,0,0));
    cv::Mat depth_f(pano_depth_f.size()+Size(icount*2, icount*2), CV_32FC1, Scalar(0.0f));
    cv::Mat depth_b(pano_depth_f.size()+Size(icount*2, icount*2), CV_32FC1, Scalar(0.0f));

    //cv::medianBlur(pano_depth_f.clone(), pano_depth_f, 7);//ksize较大时只能使用CV_8U类型
    //image_f(Rect(icount, icount, pano_image_f.cols, pano_image_f.rows)) = pano_image_f.clone();
    //depth_f(Rect(icount, icount, pano_depth_f.cols, pano_depth_f.rows)) = pano_depth_f.clone();
    Mat roi= image_f(Rect(icount, icount, pano_image_f.cols, pano_image_f.rows));
    pano_image_f.copyTo(roi);
    roi = depth_f(Rect(icount, icount, pano_depth_f.cols, pano_depth_f.rows));
    pano_depth_f.copyTo(roi);

    //imshow("pano_image_f", pano_image_f);
    //imshow("pano_depth_f", pano_depth_f);
    //imshow("image_f", image_f);
    //imshow("depth_f", depth_f);
    //waitKey();

    float d, dl, dr, dt, db;//current depth, left-right-top-bottom depth.
    float diff = 0.05;
    for(int h=0; h<pano_depth_f.rows; ++h)
    {
        for(int w=0; w<pano_depth_f.cols; ++w)
        {
            //只处理有深度值的邻域，否则跳过
            d = depth_f.at<float>(h+icount, w+icount);
            if(!(d > 1e-6f))
                continue;
            dl = depth_f.at<float>(h+icount, w-1+icount);
            dr = depth_f.at<float>(h+icount, w+1+icount);
            dt = depth_f.at<float>(h-1+icount, w+icount);
            db = depth_f.at<float>(h+1+icount, w+icount);
            //left
            if( dl > 1e-6f && (d-dl>diff*dl) || !(dl > 1e-6f))
                depth_b.at<float>(h+icount, w-1+icount) = d;
            //right
            if( dr > 1e-6f && (d-dr>diff*dr) || !(dr > 1e-6f))
                depth_b.at<float>(h+icount, w+1+icount) = d;
            //top
            if( dt > 1e-6f && (d-dt>diff*dt) || !(dt > 1e-6f))
                depth_b.at<float>(h-1+icount, w+icount) = d;
            //bottom
            if( db > 1e-6f && (d-db>diff*db) || !(db > 1e-6f))
                depth_b.at<float>(h+1+icount, w+icount) = d;

            if( dl > 1e-6f && (d-dl>diff*dl) || !(dl > 1e-6f) || dr > 1e-6f && (d-dr>diff*dr) || !(dr > 1e-6f) ||
                dt > 1e-6f && (d-dt>diff*dt) || !(dt > 1e-6f) || db > 1e-6f && (d-db>diff*db) || !(db > 1e-6f))
            {
                depth_b.at<float>(h+icount, w+icount) = d;
                ;
            }
        }
    }

    //cout<<"begin iterate"<<endl;
    //iterate icount-1
    for(int k = 0; k<icount-1; ++k)
    {
        cv::Mat depth_b_t = depth_b.clone();
        for(int h=1; h<depth_b.rows-1; ++h)
        {
            for(int w=1; w<depth_b.cols-1; ++w)
            {
                //只处理有深度值的邻域，否则跳过
                d = depth_b.at<float>(h, w);
                if(!(d > 1e-6f))
                    continue;
                dl = depth_b.at<float>(h, w-1);
                dr = depth_b.at<float>(h, w+1);
                dt = depth_b.at<float>(h-1, w);
                db = depth_b.at<float>(h+1, w);

                float dl_f = depth_f.at<float>(h, w-1);
                float dr_f = depth_f.at<float>(h, w+1);
                float dt_f = depth_f.at<float>(h-1, w);
                float db_f = depth_f.at<float>(h+1, w);
                //left
                if(!(dl > 1e-6f) || dl<d )//&& d <= depth_f.at<float>(h, w-1))
                    if(dl_f>1e-6f&&dl_f<d || !(dl_f>1e-6f))
                        depth_b_t.at<float>(h, w-1) = d;
                //right
                if(!(dr > 1e-6f) || dr<d )//&& d <= depth_f.at<float>(h, w+1))
                    if(dr_f>1e-6f&&dr_f<d || !(dr_f>1e-6f))
                        depth_b_t.at<float>(h, w+1) = d;
                //top
                if(!(dt > 1e-6f) || dt<d)//&& d <= depth_f.at<float>(h-1, w))
                    if(dt_f>1e-6f&&dt_f<d || !(dt_f>1e-6f))
                        depth_b_t.at<float>(h-1, w) = d;
                //bottom
                if(!(db > 1e-6f) || db<d)//&& d <= depth_f.at<float>(h+1, w))
                    if(db_f>1e-6f&&db_f<d || !(db_f>1e-6f))
                        depth_b_t.at<float>(h+1, w) = d;
            }
        }

        depth_b = depth_b_t;
    }

    //color inpainting
    cv::Mat mask = (depth_f<=1e-6f);
    inpaint(image_f, mask, image_f, 3, CV_INPAINT_NS);
    mask = depth_b > 0;
    inpaint(image_f, mask, image_b, 3, CV_INPAINT_NS);

    pano.pano_image = image_f;
    pano.pano_depth = depth_f;
    pano.pano_image_b = image_b;
    pano.pano_depth_b = depth_b;

    pano.minh -= icount;
    pano.minw -= icount;
    pano.maxh += icount;
    pano.maxw += icount;


    //debug

    imshow("image_f", image_f);
    imshow("depth_f", depth_f);
    imshow("image_b", image_b);
    imshow("depth_b", depth_b);
    waitKey();//*/
    return 0;
}

int genTwoLayerContents(Frame& pano)
{
    //generate front layer normal map.
    if(pano.pano_depth.empty() || pano.pano_image.empty())
    {
        cout<<"genTwoLayerContents: pano_depth || pano_image is empty!!!"<<endl;
        return -1;
    }
    GenerateNormalMap(pano.pano_depth, pano.pano_normal);

    //generate back layer depth, color and normal map.
    GenerateBackLayer(pano);
    if(pano.pano_depth_b.empty() || pano.pano_image_b.empty())
    {
        cout<<"genTwoLayerContents: pano_depth_b || pano_image_b is empty!!!"<<endl;
        return -2;
    }
    GenerateNormalMap(pano.pano_depth_b, pano.pano_normal_b);

    /*
    //debug, show the results.
    imshow("front normal map", pano.pano_normal);
    imshow("back normal map", pano.pano_normal_b);

    imshow("front image", pano.pano_image);
    imshow("back image", pano.pano_image_b);

    imshow("front depth", pano.pano_depth);
    imshow("back depth", pano.pano_depth_b);

    waitKey();
    */
    return 0;
}