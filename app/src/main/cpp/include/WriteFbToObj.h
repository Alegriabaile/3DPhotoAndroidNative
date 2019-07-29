//
// Created by ale on 19-3-20.
//

#ifndef I3D_WRITEFBTOOBJ_H
#define I3D_WRITEFBTOOBJ_H

#include "i3d.h"
#include "Frame.h"
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <ostream>

//write triangles from front layer and back layer to .obj file,
//and calculate uvs, generate texture image.
int WriteFbToObj(const i3d::Frame& pano, const std::vector<cv::Point>& triangles_f, const std::vector<cv::Point>& triangles_b,
                         const std::string& outputPath, const std::string& fileName = "multilayer_result", const std::string& texFileName = "multilayer_texture.jpg")
{
    using namespace std;
    using namespace cv;

    if(pano.pano_depth.empty() || pano.pano_depth_b.empty())
    {
        cout<<"WriteFbToObj: pano_depth or pano_depth_b is empty!"<<endl;
        return -1;
    }

    if(triangles_f.empty() || triangles_b.empty())
    {
        cout<<"WriteFbToObj: triangles f/b is empty!"<<endl;
        return -2;
    }


    std::string objFileName(fileName), mtlFileName(fileName);
    objFileName.append(".obj");
    mtlFileName.append(".mtl");

    FILE *objFileStream, *mtlFileStream;;

    objFileStream = fopen((outputPath+"/"+objFileName).c_str(), "w");
    if(objFileStream == nullptr)
    {
        printf("无法打开obj文件! \n" );
        return -3;
    }
    mtlFileStream=fopen((outputPath+"/"+mtlFileName).c_str(),"w");
    if(mtlFileStream == nullptr)
    {
        printf("无法打开mtl文件! \n" );
        return -4;
    }

    //fprintf(objFileStream, "# SimpleObjWriter by Alegriabaile\n\n");

    fprintf(objFileStream, "mtllib %s\n\n",mtlFileName.c_str());
    //fout vertices
    float theta, phi, r;//theta:0-180, phi:0-360
    float x, y, z;
    int minw = pano.minw;
    int minh = pano.minh;
    int maxw = pano.maxw;
    int maxh = pano.maxh;
    const cv::Mat pano_image = pano.pano_image.clone();//(cv::Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();
    const cv::Mat pano_depth = pano.pano_depth;//(cv::Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();
    const cv::Mat pano_normal = pano.pano_normal;
    const cv::Mat pano_depth_b = pano.pano_depth_b;
    const cv::Mat pano_image_b = pano.pano_image_b;
    const cv::Mat pano_normal_b = pano.pano_normal_b;

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

        fprintf(objFileStream, "v  %.6f %.6f %.6f\n", x, -y, -z);
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

        fprintf(objFileStream, "v  %.6f %.6f %.6f\n", x, -y, -z);
    }

    //fout texture uvs, front
    fprintf(objFileStream, "\n");
    float u, v;
    for(int i=0; i<triangles_f.size(); ++i)
    {
        cv::Point pt = triangles_f[i];
        u = float(pt.x) / float(maxw - minw);
        v = 1 - float(pt.y) / float(maxh - minh)/2.0f;

        fprintf(objFileStream, "vt %.6f %.6f\n", u, v);
    }
    fprintf(objFileStream, "\n");
    //fout texture uvs, back
    for(int i=0; i<triangles_b.size(); ++i)
    {
        cv::Point pt = triangles_b[i];
        u = float(pt.x) / float(maxw - minw);
        v = 1 - ( float(pt.y) / float(maxh - minh)/2.0f+0.5f );

        fprintf(objFileStream, "vt %.6f %.6f\n", u, v);
    }
    fprintf(objFileStream, "\n");

    //with normal or without normal
    std::string mtlName("texture_para");
    if(!pano_normal.empty() && !pano_normal_b.empty())
    {
        //fout normals, front
        fprintf(objFileStream, "\n");
        float dx, dy, dz;
        for(int i=0; i<triangles_f.size(); ++i)
        {
            cv::Point pt = triangles_f[i];
            dx = pano_normal.at<cv::Vec3f>(pt.y, pt.x)[0];
            dy = pano_normal.at<cv::Vec3f>(pt.y, pt.x)[1];
            dz = pano_normal.at<cv::Vec3f>(pt.y, pt.x)[2];
            fprintf(objFileStream, "vn %.6f %.6f %.6f\n", dx, dy, dz);
        }
        fprintf(objFileStream, "\n");
        //fout normals, back
        for(int i=0; i<triangles_f.size(); ++i)
        {
            cv::Point pt = triangles_f[i];
            dx = pano_normal.at<cv::Vec3f>(pt.y, pt.x)[0];
            dy = pano_normal.at<cv::Vec3f>(pt.y, pt.x)[1];
            dz = pano_normal.at<cv::Vec3f>(pt.y, pt.x)[2];
            fprintf(objFileStream, "vn %.6f %.6f %.6f\n", dx, dy, dz);
        }
        fprintf(objFileStream, "\n");

        //fout face indices
        fprintf(objFileStream, "usemtl %s\n\n", mtlName.c_str());
        for(int i=0; i<(triangles_f.size()+triangles_b.size())/3; ++i)
        {
            fprintf(objFileStream, "f %d/%d/%d %d/%d/%d %d/%d/%d\n",
                    i*3+1, i*3+1, i*3+1,
                    i*3+2, i*3+2, i*3+2,
                    i*3+3, i*3+3, i*3+3);
        }
    } else{
        //fout face indices
        fprintf(objFileStream, "usemtl %s\n\n", mtlName.c_str());
        for(int i=0; i<(triangles_f.size()+triangles_b.size())/3; ++i)
        {
            fprintf(objFileStream, "f %d/%d %d/%d %d/%d\n",
                    i*3+1, i*3+1,
                    i*3+2, i*3+2,
                    i*3+3, i*3+3);
        }
    }

    ///////////////////////////fout .mtl file///////////////////////////
    fprintf(mtlFileStream, "# SimpleObjWriter by Alegriabaile\n\n");
    float Tr = 0.0;
    //cv::Vec3f Tf = cv::Vec3f(1.0, 1.0, 1.0);
    cv::Vec3f Ka(1.0, 1.0, 1.0), Kd(1.0, 1.0, 1.0);
    fprintf(mtlFileStream, "newmtl %s\n", mtlName.c_str());

    fprintf(mtlFileStream, "    Tr %.4f\n", Tr);
    //fprintf(mtlFileStream, "    Tf %.4f %.4f %.4f\n", Tf[0], Tf[1], Tf[2]);
    fprintf(mtlFileStream, "    Ka %.4f %.4f %.4f\n", Ka[0], Ka[1], Ka[2]);
    fprintf(mtlFileStream, "    Kd %.4f %.4f %.4f\n", Kd[0], Kd[1], Kd[2]);
    fprintf(mtlFileStream, "    map_Ka %s\n", texFileName.c_str());
    fprintf(mtlFileStream, "    map_Kd %s\n", texFileName.c_str());

    fclose(objFileStream);
    fclose(mtlFileStream);

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
#endif //I3D_WRITEFBTOOBJ_H
