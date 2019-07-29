//
// Created by ale on 19-3-14.
//

#ifndef I3D_EXPORT2OBJ_H
#define I3D_EXPORT2OBJ_H

//
// Created by ale on 19-3-12.
//
#include "i3d.h"
#include "Frame.h"
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <ostream>

int writePanoToSimpleObj(const i3d::Frame& pano, const std::vector<cv::Point>& triangles,
        const std::string& filePath, const std::string& fileName = "pano_result", const std::string& texFileName = "texture.jpg")
{
    if(pano.pano_depth.empty())
        return -1;
    if(triangles.empty())
        return -2;

    std::string objFileName(fileName), mtlFileName(fileName);
    objFileName.append(".obj");
    mtlFileName.append(".mtl");

    FILE *objFileStream, *mtlFileStream;;

    objFileStream = fopen((filePath+"/"+objFileName).c_str(), "w");
    if(objFileStream == nullptr)
    {
        printf("无法打开obj文件! \n" );
        return -3;
    }
    mtlFileStream=fopen((filePath+"/"+mtlFileName).c_str(),"w");
    if(mtlFileStream == nullptr)
    {
        printf("无法打开mtl文件! \n" );
        return -4;
    }

    fprintf(objFileStream, "# SimpleObjWriter by Alegriabaile\n\n");

    fprintf(objFileStream, "mtllib %s\n\n",mtlFileName.c_str());
    //fout vertices
    float theta, phi, r;//theta:0-180, phi:0-360
    float x, y, z;
    int minw = pano.minw;
    int minh = pano.minh;
    int maxw = pano.maxw;
    int maxh = pano.maxh;
    const cv::Mat pano_image = pano.pano_image.clone();//(cv::Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();
    const cv::Mat pano_depth = pano.pano_depth.clone();//(cv::Range(minh, maxh+1), cv::Range(minw, maxw+1)).clone();
    const cv::Mat pano_normal = pano.pano_normal.clone();
    for(int i=0; i<triangles.size(); ++i)
    {
        cv::Point pt = triangles[i];
        r = pano_depth.at<float>(pt.y, pt.x);
        theta = float(pt.y+minh+0.5)/float(i3d::PANO_H)*M_PI;
        phi = float(pt.x+minw+0.5)/float(i3d::PANO_W)*M_PI*2;

        x = r*sin(theta)*sin(2*M_PI-phi);
        y = -r*cos(theta);
        z = -r*sin(theta)*cos(2*M_PI-phi);

        fprintf(objFileStream, "v  %.6f %.6f %.6f\n", x, y, z);
    }
    
    //fout texture uvs
    fprintf(objFileStream, "\n");
    float u, v;
    for(int i=0; i<triangles.size(); ++i)
    {
        cv::Point pt = triangles[i];
        u = float(pt.x) / float(maxw - minw);
        v = 1 - float(pt.y) / float(maxh - minh);

        fprintf(objFileStream, "vt %.6f %.6f\n", u, v);
    }
    fprintf(objFileStream, "\n");

    //fout normals
    fprintf(objFileStream, "\n");
    float dx, dy, dz;
    for(int i=0; i<triangles.size(); ++i)
    {
        cv::Point pt = triangles[i];
        dx = pano_normal.at<cv::Vec3f>(pt.y, pt.x)[0];
        dy = pano_normal.at<cv::Vec3f>(pt.y, pt.x)[1];
        dz = pano_normal.at<cv::Vec3f>(pt.y, pt.x)[2];
        fprintf(objFileStream, "vn %.6f %.6f %.6f\n", dx, dy, dz);
    }
    fprintf(objFileStream, "\n");
    
    //fout face indices
    std::string mtlName("texture_para");
    fprintf(objFileStream, "usemtl %s\n\n", mtlName.c_str());
    for(int i=0; i<triangles.size()/3; ++i)
    {
        fprintf(objFileStream, "f %d/%d/%d %d/%d/%d %d/%d/%d\n",
                i*3+1, i*3+1, i*3+1,
                i*3+2, i*3+2, i*3+2,
                i*3+3, i*3+3, i*3+3);
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

    std::string imageFileName(filePath);
    imageFileName.append("/").append(texFileName);
    cv::imwrite(imageFileName, pano_image);

    return 0;
}

int writeToSimpleObj(const cv::Mat& depth, const std::vector<cv::Point>& triangles, const float& f,
                     const std::string& filePath, const std::string& fileName = "temp", const std::string& texFileName = "texture.jpg")
{
    if(depth.empty())
        return -1;
    if(triangles.empty())
        return -2;

    std::string objFileName(filePath), mtlFileName(filePath);
    objFileName.append(fileName).append(".obj");
    mtlFileName.append(fileName).append(".mtl");

    FILE *objFileStream, *mtlFileStream;;

    objFileStream = fopen(objFileName.c_str(), "w");
    if(objFileStream == nullptr)
    {
        printf("无法打开obj文件! \n" );
        return -3;
    }
    mtlFileStream=fopen(mtlFileName.c_str(),"w");
    if(mtlFileStream == nullptr)
    {
        printf("无法打开mtl文件! \n" );
        return -4;
    }

    fprintf(objFileStream, "# SimpleObjWriter by Alegriabaile\n\n");

    fprintf(objFileStream, "mtllib %s\n\n",mtlFileName.c_str());
    //fout vertices
    float cx = (float)(depth.cols)/2;
    float cy = (float)(depth.rows)/2;
    for(int i=0; i<triangles.size(); ++i)
    {
        cv::Point pt = triangles[i];
        cv::Point3f p3f;
        p3f.z = depth.at<float>(pt.y, pt.x);
        p3f.x = p3f.z*(pt.x-cx)/f;
        p3f.y = p3f.z*(pt.y-cy)/f;

        p3f = p3f/1000.0f;
        fprintf(objFileStream, "v  %.4f %.4f %.4f\n", p3f.x, p3f.y, p3f.z);
    }
    //fout texture uvs
    fprintf(objFileStream, "\n");
    float u, v;
    for(int i=0; i<triangles.size(); ++i)
    {
        cv::Point pt = triangles[i];
        u = pt.x / (cx*2 - 1);
        v = 1 - pt.y / (cy*2 - 1);

        fprintf(objFileStream, "vt %.4f %.4f\n", u, v);
    }
    fprintf(objFileStream, "\n");

    //fout face indices
    std::string mtlName("texture_para");
    fprintf(objFileStream, "usemtl %s\n\n", mtlName.c_str());
    for(int i=0; i<triangles.size()/3; ++i)
    {
        fprintf(objFileStream, "f %d/%d %d/%d %d/%d\n",
                i*3+1, i*3+1, i*3+2, i*3+2, i*3+3, i*3+3);
    }

    ///////////////////////////fout .mtl file///////////////////////////
    fprintf(mtlFileStream, "# SimpleObjWriter by Alegriabaile\n\n");
    float Tr = 0.0;
    cv::Vec3f Tf = cv::Vec3f(1.0, 1.0, 1.0);
    cv::Vec3f Ka(1.0, 1.0, 1.0), Kd(1.0, 1.0, 1.0);
    fprintf(mtlFileStream, "newmtl %s\n", mtlName.c_str());
    fprintf(mtlFileStream, "    Tr %.4f\n", Tr);
    fprintf(mtlFileStream, "    Tf %.4f %.4f %.4f\n", Tf[0], Tf[1], Tf[2]);
    fprintf(mtlFileStream, "    Ka %.4f %.4f %.4f\n", Ka[0], Ka[1], Ka[2]);
    fprintf(mtlFileStream, "    Kd %.4f %.4f %.4f\n", Kd[0], Kd[1], Kd[2]);
    fprintf(mtlFileStream, "    map_Ka %s\n", texFileName.c_str());
    fprintf(mtlFileStream, "    mat_Kd %s\n", texFileName.c_str());

    fclose(objFileStream);
    fclose(mtlFileStream);

    /*
    ofstream objOfstream, mtlOfstream;
    objOfstream.open(objFileName, ios_base::out|ios_base::trunc);
    mtlOfstream.open(mtlFileName, ios_base::out|ios_base::trunc);
    if(!objOfstream.is_open())
    {
        cout<<"无法打开obj文件"<<endl;
        return -3;
    }
    if(!mtlOfstream.is_open())
    {
        cout<<"无法打开mtl文件"<<endl;
        return -4;
    }

    ///////////////////////////fout .obj file///////////////////////////
    objOfstream<<"# SimpleObjWriter by Alegriabaile\n\n";
    objOfstream<<"mtllib "<<mtlFileName<<"\n\n";
    //fout vertices
    float cx = (float)(depth.cols)/2;
    float cy = (float)(depth.rows)/2;
    for(int i=0; i<triangles.size(); ++i)
    {
        cv::Point pt = triangles[i];
        cv::Point3f p3f;
        p3f.z = depth.at<float>(pt.y, pt.x);
        p3f.x = p3f.z*(pt.x-cx)/f;
        p3f.y = p3f.z*(pt.y-cy)/f;

        p3f = p3f/1000.0f;
        objOfstream<<"v  "<<p3f.x<<" "<<p3f.y<<" "<<p3f.z<<"\n";
    }
    //fout texture uvs
    objOfstream<<"\n";
    float u, v;
    for(int i=0; i<triangles.size(); ++i)
    {
        cv::Point pt = triangles[i];
        u = pt.x / (cx*2 - 1);
        v = 1 - pt.y / (cy*2 - 1);

        objOfstream<<"vt "<<u<<" "<<v<<"\n";
    }
    //fout face indices
    objOfstream<<"\n";
    objOfstream<<"g my_pano\n";
    std::string mtlName("texture_para");
    objOfstream<<"usemtl "<<mtlName<<"\n"
    <<"s 2"<<"\n";
    for(int i=0; i<triangles.size()/3; ++i)
    {
        objOfstream<<"f "<<i*3+1<<"/"<<i*3+1<<" "
        <<i*3+2<<"/"<<i*3+2<<" "<<i*3+3<<"/"<<i*3+3<<" \n";
    }

    ///////////////////////////fout .mtl file///////////////////////////
    mtlOfstream<<"# SimpleObjWriter by Alegriabaile\n\n";
    float Tr = 0.0;
    cv::Vec3f Tf = cv::Vec3f(1.0, 1.0, 1.0);
    int illum = 2;
    cv::Vec3f Ka(1.0, 1.0, 1.0), Kd(1.0, 1.0, 1.0);
    mtlOfstream<<"newmtl "<<mtlName<<"\n"
            <<"    Tr "<<Tr<<"\n"
            <<"    Tf "<<Tf[0]<<" "<<Tf[1]<<" "<<Tf[2]<<"\n"
            <<"    illum "<<illum<<"\n"
            <<"    Ka "<<Ka[0]<<" "<<Ka[1]<<" "<<Ka[2]<<"\n"
            <<"    Kd "<<Kd[0]<<" "<<Kd[1]<<" "<<Kd[2]<<"\n"
            <<"    map_Ka "<<texFileName<<"\n"
            <<"    map_Kd "<<texFileName<<"\n";

    objOfstream.close();
    mtlOfstream.close();
    */

    return 0;
}

int writeToCustomizedFile(const cv::Mat& depth, const std::vector<cv::Point>& triangles, const float& f,
                          const std::string& filePath, const std::string& fileName = "temp")
{
    if(depth.empty())
        return -1;
    if(triangles.empty())
        return -2;

    std::string objFileName(filePath);
    objFileName.append(fileName).append(".txt");

    FILE *objFileStream;

    objFileStream = fopen(objFileName.c_str(), "w");
    if(objFileStream == nullptr)
    {
        printf("无法打开txt文件! \n" );
        return -3;
    }

    //fout vertices
    const float cx = (float)(depth.cols)/2;
    const float cy = (float)(depth.rows)/2;
    float u, v;
    fprintf(objFileStream, "%d\n", int(triangles.size()) );
    for(int i=0; i<triangles.size(); ++i)
    {
        cv::Point pt = triangles[i];
        cv::Point3f p3f;
        p3f.z = depth.at<float>(pt.y, pt.x);
        p3f.x = p3f.z*(pt.x-cx)/f;
        p3f.y = p3f.z*(pt.y-cy)/f;

        p3f = p3f/1000.0f;
        u = pt.x / (cx*2 - 1);
        v = 1 - pt.y / (cy*2 - 1);
        fprintf(objFileStream, "%.4f %.4f %.4f %.4f %.4f\n", p3f.x, p3f.y, p3f.z, u, v);
    }

    return 0;
}

#endif //I3D_EXPORT2OBJ_H
