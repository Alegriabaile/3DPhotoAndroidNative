//
// Created by ale on 19-2-19.
//
#include "stitchAllPanos.h"
#include "guided-filter/guidedfilter.h"

using namespace std;
using namespace cv;
using namespace i3d;

static int GenConsensusCount(const int &h, const int &w, const float &d, const int &in, vector<Frame>& kframes)
{
    float cnt_consesus = 0;//count of how many other views n(p, αp) are at similar depth
    for(int image_i =0; image_i<kframes.size(); image_i++)
    {
        if (image_i != in
            && h + kframes[in].minh >= kframes[image_i].minh
            && h + kframes[in].minh <= kframes[image_i].maxh
            && w + kframes[in].minw >= kframes[image_i].minw
            && w + kframes[in].minw <= kframes[image_i].maxw)
        {
            float d_tmp = kframes[image_i].pano_depth.at<float>(
                    h + kframes[in].minh - kframes[image_i].minh,
                    w + kframes[in].minw - kframes[image_i].maxh);

            if (d_tmp > 0 && d_tmp < 1.0f)
            {
                float ratio = d / d_tmp;
                //计数，(h, w)点深度一致性的图片个数
                cnt_consesus += (ratio <= 1.1f && ratio >= 0.9f) ? 1 : 0;
            }
        }
    }

    return cnt_consesus;
}

//对深度一致性不够的像素进行惩罚
static int GenConsensusErrors(const int in, vector<Frame>& kframes)
{
    float t = 3;//τ（发音：/tau/）
    for(int h=0; h<kframes[in].pano_depth.rows; ++h)
    {
        for(int w=0; w<kframes[in].pano_depth.cols; ++w)
        {
            float cnt_consesus = 0;
            float d = kframes[in].pano_depth.at<float>(h,w);
            if( d > 0.0f )
                cnt_consesus = GenConsensusCount(h, w, d, in, kframes);

            //Efinal = Econsensus + Lambda3*Eboundary + Lambda4*Esaturated
            //Econsensus(p, αp) = max{1 − τnconsensus (p,αp) , 0},
            float e_consensus = 1 - cnt_consesus/t;
            kframes[in].pano_error.at<float>(h,w) += e_consensus>0?e_consensus:0;
        }
    }

    return 0;
}

//对过曝的像素进行惩罚
static int GenSaturatedErrors(Frame& frame)
{
    const Mat &pano_image = frame.pano_image;
    const Mat &pano_depth = frame.pano_depth;
    for(int h=0; h<pano_depth.rows; ++h)
    {
        for(int w=0; w<pano_depth.cols; ++w)
        {
            if (pano_depth.at<float>(h, w) > 0.0f)
            {
                float e = 0;
                float r, g, b;
                b = pano_image.at<cv::Vec3b>(h, w)[0];
                g = pano_image.at<cv::Vec3b>(h, w)[1];
                r = pano_image.at<cv::Vec3b>(h, w)[2];
                float LAB_L = 0.4124 * r + 0.3576 * g + 0.1805 * b;
                float LAB_L_FULL = 0.4124 * 255 + 0.3576 * 255 + 0.1805 * 255;
                //e = e_boundary + e_saturated.
                //Edata = Econsensus + λ3 Eboundary + λ4 Esaturated, λ4 = 3;
                //Esaturated(p, αp) = (1 if l(p, αp) > τsaturated, 0 otherwise)
                e = 3 * (LAB_L_FULL - LAB_L < 0.2 * LAB_L_FULL);
                frame.pano_error.at<float>(h, w) += e;
                //cout << row << "," << col << "     ";
            }
        }
    }

    return 0;
}

//对边界像素进行惩罚
//使用全景图边界替代透视图边界，跳过透视图转全景图时的边界信息tracking
static int GenBoundaryErrors(Frame& frame)
{
    const Mat &pano_image = frame.pano_image;
    const Mat &pano_depth = frame.pano_depth;

    const int rows = pano_depth.rows;
    const int cols = pano_depth.cols;
    Mat boundary_errors = Mat(rows, cols, CV_32FC1, Scalar(1.0f));

    //Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height)
    float scale_min = 0.05f;
    float scale_max = 1.0f - scale_min*2.0f;
    Rect roi(cols*scale_min, rows*scale_min, cols*scale_max, rows*scale_max);
    boundary_errors(roi).setTo(0.0f);

    frame.pano_error = frame.pano_error + boundary_errors;
    return 0;
}

static int GenPanoLabels(vector<Frame>& kframes, Frame& pano)
{
    uint minh, minw, maxh, maxw;
    minh = pano.minh; minw = pano.minw;
    maxh = pano.maxh; maxw = pano.maxw;

    pano.pano_image = Mat(PANO_H, PANO_W, CV_8UC3, Scalar(0,0,0) );
    pano.pano_depth = Mat(PANO_H, PANO_W, CV_32FC1, cv::Scalar(0.0) );
    pano.pano_label = Mat(PANO_H, PANO_W, CV_8UC1, Scalar(255));
    for(int h=minh; h<maxh+1; ++h)
    {
        for(int w=minw; w<maxw+1; ++w)
        {
            float error = 10;
            uint label = kframes.size();
            for(int i=0; i<kframes.size(); ++i)
            {
                if(h >= kframes[i].minh && h <= kframes[i].maxh
                   && w >= kframes[i].minw && w <= kframes[i].maxw
                   && kframes[i].pano_depth.at<float>(h-kframes[i].minh, w-kframes[i].minw) > 0.0f)
                {
                    float error_t = kframes[i].pano_error.at<float>(h-kframes[i].minh, w-kframes[i].minw);
                    label = error_t<error?i:label;
                    error = error_t<error?error_t:error;
                }
            }

            if(label<kframes.size())
            {
                pano.pano_depth.at<float>(h,w)
                        = kframes[label].pano_depth.at<float>(h-kframes[label].minh, w-kframes[label].minw);
                pano.pano_image.at<cv::Vec3b>(h,w)
                        = kframes[label].pano_image.at<cv::Vec3b>(h-kframes[label].minh, w-kframes[label].minw);
                //pano.pano_image.at<Vec3b>(h,w)
                //        = kframes[label].pano_image.at<Vec3b>(h-kframes[label].minh, w-kframes[label].minw);
                //pano.pano_image.at<cv::Vec3b>(h,w)[1]
                //        = kframes[label].pano_image.at<cv::Vec3b>(h-kframes[label].minh, w-kframes[label].minw)[1];
                //pano.pano_image.at<cv::Vec3b>(h,w)[2]
                //        = kframes[label].pano_image.at<cv::Vec3b>(h-kframes[label].minh, w-kframes[label].minw)[2];

                pano.pano_label.at<uchar>(h,w) = label;
            }
        }
    }

    pano.pano_image = pano.pano_image(Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();
    pano.pano_depth = pano.pano_depth(Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();
    pano.pano_label = pano.pano_label(Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();

    return 0;
}

//feathering........
static int GenSoftMask(const Frame & pano, const int iLabel,  Frame& frame)
{
    uint rows, cols;

    //generate soft image (boundary) mask...
    rows = frame.pano_depth.rows;
    cols = frame.pano_depth.cols;

    cv::Size sz = frame.pano_depth.size();
    frame.mask_bound = cv::Mat(sz, CV_32FC1, Scalar(0.0f));
    frame.mask_label = cv::Mat(sz, CV_32FC1, Scalar(0.0f));
    frame.soft_mask = cv::Mat(sz, CV_32FC1, Scalar(0.0f));

    const cv::Mat &pano_depth =  frame.pano_depth;
    for (int h = 0; h < rows; ++h)
    {
        for(int w = 0; w < cols; ++w)
        {
            if(pano_depth.at<float>(h,w) > 0.0f)
                if( h-1 >=0 && pano_depth.at<float>(h-1,w) <= 1e-6f
                 || h+1 < rows && pano_depth.at<float>(h+1,w) <= 1e-6f
                 || w-1 >=0 && pano_depth.at<float>(h,w-1) <= 1e-6f
                 || w+1 < rows && pano_depth.at<float>(h,w+1) <= 1e-6f)
                    frame.mask_bound.at<float>(h,w) = 1.0f;

        }
    }
    int ksize;
    ksize = rows<cols?rows:cols;
    ksize = ksize*0.15*2+1;
    cv::Mat bkernel = getGaussianKernel( ksize, -1, CV_32F);
    filter2D(frame.mask_bound, frame.mask_bound, -1, bkernel);
    frame.mask_bound = 1.0f - frame.mask_bound;

    //generate soft label mask
    //可以只遍历一次，不需要遍历kframe.size()次
    rows = pano.pano_depth.rows;
    cols = pano.pano_depth.cols;
    const cv::Mat & pano_label = pano.pano_label;
    for (int h = 0; h < rows; ++h)
    {
        for(int w = 0; w < cols; ++w)
        {
            if(iLabel == pano_label.at<uchar>(h,w))
            if(h + 1 < rows && pano_label.at<uchar>( h, w) != iLabel
               || h - 1 >= 0   && pano_label.at<uchar>( h - 1,  w) != iLabel
               || w + 1 < cols && pano_label.at<uchar>( h, w + 1) != iLabel
               || w - 1 >= 0   && pano_label.at<uchar>( h, w - 1) != iLabel)
                frame.mask_label.at<float>(h+pano.minh-frame.minh, w+pano.minw-frame.minw) = 1.0f;
        }
    }
    ksize = 50*PANO_W/8196;
    cv::Mat lkernel = Mat::ones(ksize, ksize, CV_32FC1)/float(ksize*ksize);
    filter2D(frame.mask_label, frame.mask_label, -1, lkernel);
    frame.mask_label = 1.0f - frame.mask_label;

    //generate final soft mask from label mask and image boundary mask.
    rows = frame.pano_depth.rows;
    cols = frame.pano_depth.cols;
    for (int h = 0; h < rows; ++h)
    {
        for(int w = 0; w < cols; ++w)
        {
            if(pano_depth.at<float>(h,w) > 0.0f)
            {
                float l = frame.mask_label.at<float>(h,w);
                float b = frame.mask_bound.at<float>(h,w);
                frame.soft_mask.at<float>(h,w) = std::fmin(l, b);
            }
        }
    }

    imshow("mask_label", frame.mask_label);
    imshow("mask_bound", frame.mask_bound);
    imshow("soft_mask", frame.soft_mask);
    waitKey();

    return 0;
}
static int GenSoftMasks( const Frame& pano, vector<Frame>& kframes)
{
    for(int i=0; i<kframes.size(); ++i)
        GenSoftMask(pano, i, kframes[i]);

    return 0;
}

int stitchAllPanos(vector<Frame>& kframes, Frame& pano)
{
    //generate penalties...
    uint minh, minw, maxh, maxw;
    minw = minh = PANO_W;
    maxw = maxh = 0;
    for(int i=0; i<kframes.size(); ++i)
    {
        GenConsensusErrors(i, kframes);
        //GenSaturatedErrors(kframes[i]);
        GenBoundaryErrors(kframes[i]);

        //we use a filter footprint that spans 2.5% of the image width
        //and set the edge-aware parameter ϵ = 10−7.
        //I:guide, p:input, q:output. q = guidedFilter(I, p, ...);
        //cv::Mat I = kframes[i].pano_depth.clone();
        //cv::Mat p = kframes[i].pano_error.clone();
        //double minVal, maxVal;
        //minMaxLoc(I, &minVal, &maxVal);
        //I = (I-float(minVal))/float(maxVal - minVal);
        //I = 1.0f/(I+1.0f);
        //kframes[i].pano_error = guidedFilter(I, p, I.cols*0.025, 1e-7);

        //record the edge of output pano
        minw = minw < kframes[i].minw ? minw : kframes[i].minw;
        minh = minh < kframes[i].minh ? minh : kframes[i].minh;
        maxw = maxw > kframes[i].maxw ? maxw : kframes[i].maxw;
        maxh = maxh > kframes[i].maxh ? maxh : kframes[i].maxh;
    }

    //generate pano_depth and pano_label(for pano_image generating)
    pano.minh = minh; pano.minw = minw;
    pano.maxh = maxh; pano.maxw = maxw;
    GenPanoLabels(kframes, pano);




    /////////////////////////////////feathering//////////////////////////////
    /*
    GenSoftMasks(pano, kframes);

    Mat pimg = pano.pano_image.clone();
    Mat pmsk;
    GetLabelEdges(pano_label, pmsk);
    Mat pmsk_part = pmsk(Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();
    Mat elementCross=getStructuringElement(MORPH_CROSS,Size(3,3),Point(-1,-1));
    for(int i=0; i<1; ++i)
        dilate(pmsk_part, pmsk_part, elementCross);
    imshow("pmsk_part", pmsk_part);

    Mat blured = pano.pano_image.clone();
    GaussianBlur(blured.clone(), blured, Size(21,21), 0);
    imshow("blured", blured);
    imshow("orig", pano.pano_image);
    blured.copyTo(pano.pano_image, pmsk_part);
    imshow("comp", pano.pano_image);
    //blur(blured, blured, )
    //*/

    //debug, show each pano.image/depth/error, show last pano.image/depth/label
    /*
    Mat label_color(PANO_H, PANO_W, CV_8UC3, Scalar(255, 255, 255));
    if(true)
    for(int i=0; i<kframes.size(); ++i)
    {
        int b = (unsigned)theRNG() & 255;//随机返回一个0~255之间的值
        int g = (unsigned)theRNG() & 255;
        int r = (unsigned)theRNG() & 255;
        Vec3b v3b = Vec3b(b, g, r);
        for(int h=0; h<pano_label.rows; ++h)
            for(int w=0; w<pano_label.cols; ++w)
                if(pano_label.at<uchar>(h,w) == i)
                    label_color.at<Vec3b>(h,w) = v3b;

        imshow("image", kframes[i].pano_image);
        imshow("depth", kframes[i].pano_depth);
        imshow("error", kframes[i].pano_error/8.0);
        cout<<kframes[i].minw<<" "<<kframes[i].minh<<" "<<kframes[i].maxw<<" "<<kframes[i].maxh
        <<"    "<<kframes[i].depth.cols<<","<<kframes[i].depth.rows<<endl;
        //waitKey();
        destroyWindow("image");
        destroyWindow("depth");
        destroyWindow("error");
    }
    Mat label_color_part = label_color(Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();
    resize(label_color, label_color, Size(1000, 500));
    imshow("label_color_part", label_color_part);
    imshow("label_color", label_color);
    imshow("image", pano.pano_image);
    imshow("depth", pano.pano_depth);
    cout<<minh<<" "<<maxh<<" "<<minw<<" "<<maxw<<endl;
    waitKey();
    destroyWindow("label_color_part");
    destroyWindow("label_color");
    destroyWindow("image");
    destroyWindow("depth");
    //*/

    return 0;
}