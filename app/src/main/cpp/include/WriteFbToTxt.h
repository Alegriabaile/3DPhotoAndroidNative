//
// Created by ale on 19-3-20.
//

#ifndef I3D_WRITEFBTOTXT_H
#define I3D_WRITEFBTOTXT_H

#include "i3d.h"
#include "Frame.h"
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <ostream>

//write triangles from front layer and back layer to .obj file,
//and calculate uvs, generate texture image.
int WriteFbToTxt(const i3d::Frame& pano, const std::vector<cv::Point>& triangles_f, const std::vector<cv::Point>& triangles_b,
                         const std::string& outputPath, const std::string& fileName = "customized", const std::string& texFileName = "customized.jpg")
{
    using namespace std;
    using namespace cv;

    if(pano.pano_depth.empty() || pano.pano_depth_b.empty())
    {
        cout<<"WriteFbToTxt: pano_depth or pano_depth_b is empty!"<<endl;
        return -1;
    }

    if(triangles_f.empty() || triangles_b.empty())
    {
        cout<<"WriteFbToTxt: triangles f/b is empty!"<<endl;
        return -2;
    }


    std::string txtFileName(fileName);
    txtFileName.append(".txt");

    FILE *txtFileStream;

    txtFileStream = fopen((outputPath+"/"+txtFileName).c_str(), "w");
    if(txtFileStream == nullptr)
    {
        printf("无法打开txt文件! \n" );
        return -3;
    }
    int minw = pano.minw;
    int minh = pano.minh;
    int maxw = pano.maxw;
    int maxh = pano.maxh;
    const cv::Mat pano_image = pano.pano_image.clone();//(cv::Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();
    const cv::Mat pano_depth = pano.pano_depth;//(cv::Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();
    const cv::Mat pano_depth_b = pano.pano_depth_b;
    const cv::Mat pano_image_b = pano.pano_image_b;


    fprintf(txtFileStream, "%lu %d\n", triangles_f.size()+triangles_b.size(), 5);
    //fout vertices
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

        fprintf(txtFileStream, "%.6f %.6f %.6f %0.6f %0.6f\n", x/1000.0f, -y/1000.0f, -z/1000.0f, u, v);
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

        fprintf(txtFileStream, "%.6f %.6f %.6f %0.6f %0.6f\n", x/1000.0f, -y/1000.0f, -z/1000.0f, u, v);
    }
    fclose(txtFileStream);

    cv::Mat multilayer_texture(pano_image.rows*2, pano_image.cols, CV_8UC3, Scalar(0,0,0));
    cv::Mat Roi_f = multilayer_texture(Rect(0, 0, pano_image.cols, pano_image.rows));//x, y, w, h
    cv::Mat Roi_b = multilayer_texture(Rect(0, pano_image.rows, pano_image.cols, pano_image.rows));
    pano_image.copyTo(Roi_f);
    pano_image_b.copyTo(Roi_b);
    std::string imageFileName(outputPath);
    imageFileName.append("/").append(texFileName);
    cv::imwrite(imageFileName, multilayer_texture);

    return 0;
}
#endif //I3D_WRITEFBTOTXT_H
