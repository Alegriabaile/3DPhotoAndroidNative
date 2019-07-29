//
// Created by ale on 18-12-18.
//

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <sys/stat.h>

#include "readData.h"

using namespace std;
static void getCameraIntrinsics(const std::string filename, std::map<std::string, double> & camera_intrinsic)
{
    //cnblogs@半闲居士
    ifstream fin( filename.c_str() );
    if (!fin)
    {
        cerr<<"camera intrinsic parameters file does not exist."<<endl;
        return;
    }
    while(!fin.eof())
    {
        std::string str;
        getline( fin, str );
        if (str[0] == '#')
        {
            // 以‘＃’开头的是注释
            continue;
        }

        int pos = str.find("=");
        if (pos == -1)
            continue;
        std::string key = str.substr( 0, pos );
        std::string svalue = str.substr( pos+1, str.length() );
        double value = atof(const_cast<const char *>(svalue.c_str()));
        camera_intrinsic[key] = value;
        if ( !fin.good() )
            break;
    }
    fin.close();
}

//@folderPath: input folder to find @files;
//@type: RGB_img("IMG_*.jpg"), depth_img("npy_IMG_*.png") or camera rotation("TXT_*.txt").
//@files: vector of file names found from @folderPath.
static void findFilesFromFolder(const std::string folderPath, std::vector<std::string>& files)
{
    const char * dir_name = folderPath.c_str();
    files.clear();
    // check the parameter !
    if( NULL == dir_name )
    {
        cout<<" dir_name is null ! "<<endl;
        return;
    }

    // check if dir_name is a valid dir
    struct stat s;
    lstat( dir_name , &s );
    if( ! S_ISDIR( s.st_mode ) )
    {
        cout<<"dir_name is not a valid directory !"<<endl;
        return;
    }

    struct dirent * filename;    // return value for readdir()
    DIR * dir;                   // return value for opendir()
    dir = opendir( dir_name );
    if( NULL == dir )
    {
        cout<<"Can not open dir "<<dir_name<<endl;
        return;
    }

    /* read all the files in the dir ~ */
    while( ( filename = readdir(dir) ) != NULL )
    {
        // get rid of "." and ".."
        if( strcmp( filename->d_name , "." ) == 0 ||
            strcmp( filename->d_name , "..") == 0)
            continue;
        std::string file(filename->d_name);
        //if (files[i].find("Resized_IMG_") == 0 && files[i].rfind(".png") == files[i].size() - 4)
        files.push_back(filename->d_name);
    }
    closedir(dir);
}

void readRGBD(const std::string mainDir, std::vector<cv::Mat>& images, std::vector<cv::Mat>& depths)
{
    std::vector<std::string> files;
    std::string subDir1, subDir2;

    subDir1.assign(mainDir).append("/Images");
    findFilesFromFolder(subDir1, files);
    subDir2.assign(mainDir).append("/Depths");

    for (int i = 0; i < files.size(); i++)
    {
        char ind[5];
        int temp = 1000+i;
        sprintf(ind, "%4d", temp);
        std::string inds(ind);
        std::string filedir;

        filedir.assign(subDir1).append("/").append(inds).append(".jpg");
        cv::Mat img = cv::imread(filedir);
        if(img.empty())
            cout<<"ReadData.cpp: ReadData() imread(img) failed..."<<endl;
        images.push_back(img);

        filedir.assign(subDir2).append("/").append(inds).append(".jpg");
        cv::Mat dpt = cv::imread(filedir, CV_LOAD_IMAGE_ANYCOLOR|CV_LOAD_IMAGE_ANYDEPTH);

        if(dpt.empty())
        {
            //cout<<"ReadData.cpp: ReadData() imread(dpt--->jpg) failed..."<<endl;
            filedir.assign(subDir2).append("/").append(inds).append(".png");
            dpt = cv::imread(filedir, CV_LOAD_IMAGE_ANYCOLOR|CV_LOAD_IMAGE_ANYDEPTH);
        }
        if(dpt.empty())
            cout<<"ReadData.cpp: ReadData() imread(dpt--->jpg and png) failed..."<<endl;

        cv::Mat dpt_float;
        dpt.convertTo(dpt_float, CV_32FC1);

        //使最大深度值小于等于10000
        //for(int h=0; h<dpt_float.rows; ++h)
        //    for(int w=0; w<dpt_float.cols; ++w)
        //        dpt_float.at<float>(h,w) = dpt_float.at<float>(h,w)>1e4?1e4:dpt_float.at<float>(h,w);

        depths.push_back(dpt_float);
    }
}
void readIntrinsics(const std::string mainDir, i3d::Intrinsics& intrinsics)
{
    std::string subDir3;
    subDir3.assign(mainDir).append("/intrinsic.txt");
    std::map<std::string, double> camera_intrinsic;
    getCameraIntrinsics(subDir3, camera_intrinsic);

    if( camera_intrinsic.find("f") == camera_intrinsic.end()||
        camera_intrinsic.find("cx") == camera_intrinsic.end()||
        camera_intrinsic.find("cy") == camera_intrinsic.end())
    {
        cout<<"readIntrinsics(): No f || cx || cy in intrinsic.txt"<<endl;
        return;
    }

    intrinsics.f = camera_intrinsic["f"];
    intrinsics.cx = camera_intrinsic["cx"];
    intrinsics.cy = camera_intrinsic["cy"];
}