//
// Created by ale on 19-3-24.
//

//
// Created by ale on 19-3-20.
//

#include "genCompactTri.h"
#include "WriteFbToObj.h"
#include "WriteFbToTxt.h"
using namespace std;
using namespace cv;

static int GenerateTwoLayerContents(i3d::Frame& pano, const int icount1=30)
{
    const int icount = 0;
    cv::Mat pano_image_f = pano.pano_image;
    cv::Mat pano_depth_f = pano.pano_depth;
    //init
    cv::Mat image_f(pano_image_f.size()+Size(icount*2, icount*2), CV_8UC3, Scalar(255,255,255));
    cv::Mat image_b(pano_image_f.size()+Size(icount*2, icount*2), CV_8UC3, Scalar(255,255,255));
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
            {
//                image_f.at<Vec3b>(h, w) = Vec3b(255,255,255);
                continue;
            }

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
    for(int k = 0; k<icount1-1; ++k)
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
//    inpaint(image_f, mask, image_f, 3, CV_INPAINT_NS);
    mask = depth_b > 0.0f;
    inpaint(image_f, mask, image_b, 3, CV_INPAINT_NS);
    mask = mask==0;
    image_b.setTo(Vec3b(0,0,0), mask);

    pano.pano_image = image_f;
    pano.pano_depth = depth_f;
    pano.pano_image_b = image_b;
    pano.pano_depth_b = depth_b;

    //小心越界。。。尤其是下标
    pano.minh -= icount;
    pano.minw -= icount;
    pano.maxh += icount;
    pano.maxw += icount;


    //debug

//    imshow("image_f", image_f);
//    imshow("depth_f", depth_f);
//    imshow("image_b", image_b);
//    imshow("depth_b", depth_b);
//    waitKey();//*/


    if(pano.minh<0 || pano.minw<0 || pano.maxh>=i3d::PANO_H || pano.maxw>=i3d::PANO_W)
        return -3;
    return 0;
}

static int GenerateCompactTriangles(const cv::Mat& pano_depth, std::vector<cv::Point>& triangles)
{
    using namespace cv;
    using namespace std;

    const float MAX_DIFF = 0.05;
    const cv::Mat depth = pano_depth;
    if(depth.empty())
    {
        cout<<"GenerateCompactTriangles: depth.empty! "<<endl;
        return -1;
    }

    triangles.clear();
    for (float h = 0; h < depth.rows; ++h)
    {
        for (float w = 0; w < depth.cols; ++w)
        {
            float d = depth.at<float>(h, w);
            float x, y, z;

            if(!(d>0))
                continue;

            if (h > 0 && w > 0)
            {
                float dleft = depth.at<float>(h, (int) w - 1);
                float dtop = depth.at<float>(h - 1, (int) w);
                if (fabs(dleft - d) < MAX_DIFF*d && fabs(dtop - d) < MAX_DIFF*d)//dleft > 0 && dtop > 0 &&
                {
                    triangles.emplace_back(Point(w, h));
                    triangles.emplace_back(Point(w, h-1));
                    triangles.emplace_back(Point(w-1, h));
                }
            }

            if (h + 1 < depth.rows && w + 1 < depth.cols)
            {
                float dright = depth.ptr<float>(h)[(int) w + 1];
                float dbottom = depth.ptr<float>(h + 1)[(int) w];
                if (fabs(dright - d) < MAX_DIFF*d && fabs(dbottom - d) < MAX_DIFF*d)
                {
                    triangles.emplace_back(Point(w, h));
                    triangles.emplace_back(Point(w, h+1));
                    triangles.emplace_back(Point(w+1, h));
                }
            }

        }//for w=0...
    }//for h=0...

    return 0;
}//end of GenerateCompactTriangles


int genCompactTri(const std::string& outputDir, i3d::Frame& pano, const int icount)
{
    vector<Point> triangles_f, triangles_b;
//    vector<Point> contour_edges;

    if(GenerateTwoLayerContents(pano)!=0)
    {
        LOGE("genCompactTri(): GenerateTwoLayerContents(): min max h w out of range...No triangles generated.");
        return -3;
    }


    GenerateCompactTriangles(pano.pano_depth, triangles_f);
    GenerateCompactTriangles(pano.pano_depth_b, triangles_b);

//    cout<<"f tri size(): "<<triangles_f.size()<<".  b tri size(): "<<triangles_b.size()<<endl;
    LOGE("f tri size(): %d;      b tri size(): %d   ", triangles_f.size(), triangles_b.size());
    //WriteFbToObj(pano, triangles_f, triangles_b, outputDir);
    WriteFbToTxt(pano, triangles_f, triangles_b, outputDir);
    return 0;
}



static int GenerateResults(const i3d::Frame& pano, const std::vector<cv::Point>& triangles_f, const std::vector<cv::Point>& triangles_b,
                           cv::Mat & f_b_texture, std::vector<float>& f_b_vertices)
{
    using namespace std;
    using namespace cv;

    int minw = pano.minw;
    int minh = pano.minh;
    int maxw = pano.maxw;
    int maxh = pano.maxh;
    const cv::Mat pano_image = pano.pano_image.clone();//(cv::Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();
    const cv::Mat pano_depth = pano.pano_depth;//(cv::Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();
    const cv::Mat pano_depth_b = pano.pano_depth_b;
    const cv::Mat pano_image_b = pano.pano_image_b;


    f_b_vertices.clear();
    f_b_vertices.reserve((triangles_b.size()+triangles_f.size())*5);

    float theta, phi, r;//theta:0-180, phi:0-360
    float x, y, z, u, v;
    //front vertices
    for(int i=0; i<triangles_f.size(); ++i)
    {
        cv::Point pt = triangles_f[i];
        r = pano_depth.at<float>(pt.y, pt.x);
        theta = float(pt.y+minh+0.5)/float(i3d::PANO_H)*M_PI;
        phi = float(pt.x+minw+0.5)/float(i3d::PANO_W)*M_PI*2;

        x = r*sin(theta)*sin(2*M_PI-phi);
        y = -r*cos(theta);
        z = -r*sin(theta)*cos(2*M_PI-phi);
        u = float(pt.x) / float(maxw - minw);
        v = 1 - float(pt.y) / float(maxh - minh)/2.0f;

        f_b_vertices.push_back(x); f_b_vertices.push_back(y); f_b_vertices.push_back(z);
        f_b_vertices.push_back(u); f_b_vertices.push_back(v);
    }
    //back vertices
    for(int i=0; i<triangles_b.size(); ++i)
    {
        cv::Point pt = triangles_b[i];
        r = pano_depth_b.at<float>(pt.y, pt.x);
        theta = float(pt.y+minh+0.5)/float(i3d::PANO_H)*M_PI;
        phi = float(pt.x+minw+0.5)/float(i3d::PANO_W)*M_PI*2;

        x = r*sin(theta)*sin(2*M_PI-phi);
        y = -r*cos(theta);
        z = -r*sin(theta)*cos(2*M_PI-phi);
        u = float(pt.x) / float(maxw - minw);
        v = 1 - ( float(pt.y) / float(maxh - minh)/2.0f+0.5f );

        f_b_vertices.push_back(x); f_b_vertices.push_back(y); f_b_vertices.push_back(z);
        f_b_vertices.push_back(u); f_b_vertices.push_back(v);
    }


    cv::Mat multilayer_texture(pano_image.rows*2, pano_image.cols, CV_8UC3, Scalar(0,0,0));
    cv::Mat Roi_f = multilayer_texture(Rect(0, 0, pano_image.cols, pano_image.rows));//x, y, w, h
    cv::Mat Roi_b = multilayer_texture(Rect(0, pano_image.rows, pano_image.cols, pano_image.rows));
    pano_image.copyTo(Roi_f);
    pano_image_b.copyTo(Roi_b);
    f_b_texture = multilayer_texture;

    return 0;
}
//f_b_vertices:
// vertice0 = [x0,y0,z0,u0,v0].
// triangle0 = [vertices0, vertices1, vertices2].
// num_of_triangles = f_b_vertices.size()/(5*3)
int genCompactTri(i3d::Frame& pano, cv::Mat & f_b_texture, std::vector<float>& f_b_vertices, const int icount)
{
    vector<Point> triangles_f, triangles_b;
//    vector<Point> contour_edges;

    if(GenerateTwoLayerContents(pano)!=0)
    {
        LOGE("genCompactTri(): GenerateTwoLayerContents(): min max h w out of range...No triangles generated.");
        return -3;
    }


    GenerateCompactTriangles(pano.pano_depth, triangles_f);
    GenerateCompactTriangles(pano.pano_depth_b, triangles_b);

//    cout<<"f tri size(): "<<triangles_f.size()<<".  b tri size(): "<<triangles_b.size()<<endl;
    LOGE("f tri size(): %d;      b tri size(): %d   ", triangles_f.size(), triangles_b.size());
    //WriteFbToObj(pano, triangles_f, triangles_b, outputDir);


//    WriteFbToTxt(pano, triangles_f, triangles_b, outputDir);
    GenerateResults(pano, triangles_f, triangles_b, f_b_texture, f_b_vertices);
    return 0;
}