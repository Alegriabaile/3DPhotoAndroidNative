//
// Created by ale on 19-11-7.
//

#ifndef MY3DPHOTO_1FILENAMEEXTRACTOR_H
#define MY3DPHOTO_1FILENAMEEXTRACTOR_H

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <sys/stat.h>


namespace m3d
{
    class FileNameExtractor
    {
    private:
        std::string dataDir;
        std::vector<std::string> & imageFileNames;
        std::vector<std::string> & depthFileNames;
        std::vector<std::string> & paramFileNames;
        std::string &paramDefault;

        //0: default, disparity with .jpg
        //1: depth with .png
        size_t state;

        void findFileNamesFromFolder(const std::string folderPath, std::vector<std::string>& files)
        {
            const char * dir_name = folderPath.c_str();
            files.clear();
            // check the parameter !
            if( NULL == dir_name )
            {
//                m3d::LOG("FileNameExtractor::findFileNamesFromFolder", "dir_name is null");
                return;
            }

            // check if dir_name is a valid dir
            struct stat s;
            lstat( dir_name , &s );
            if( ! S_ISDIR( s.st_mode ) )
            {
//                m3d::LOG("FileNameExtractor::findFileNamesFromFolder", "dir_name is not a dir");
                return;
            }

            struct dirent * filename;    // return value for readdir()
            DIR * dir;                   // return value for opendir()
            dir = opendir( dir_name );
            if( NULL == dir )
            {
//                m3d::LOG("FileNameExtractor::findFileNamesFromFolder", "(DIR*)dir is null");
                return;
            }

            /* read all the files in the dir ~ */
            while( ( filename = readdir(dir) ) != NULL )
            {
                // get rid of "." and ".."
                if( strcmp( filename->d_name , "." ) == 0 ||
                    strcmp( filename->d_name , "..") == 0)
                    continue;

                std::string file(folderPath);
                file.append(filename->d_name);
                files.push_back(file);
            }
            closedir(dir);
        }

        void extractImageFileNames()
        {
            if(dataDir.empty())
            {
//                m3d::LOG("FileNameExtractor::extractImageFileNames", "dataDir.empty()");
                return;
            }

            while(dataDir.size()>1 && dataDir.back()=='/')
                dataDir.pop_back();

            std::string fileFolder(dataDir);
            fileFolder.append("/Images/");
            findFileNamesFromFolder(fileFolder, imageFileNames);
        }
        void extractDepthFileNames()
        {
            for(int i=0; i<imageFileNames.size(); ++i)
            {
                std::string depthFileName(imageFileNames[i]);
                depthFileName.replace(dataDir.size(), 8, "/Depths/");
                if(state == 1)
                {
                    unsigned int pos = depthFileName.find_last_of(".");
                    depthFileName.replace(pos, 4, ".png");
                }

                depthFileNames.push_back(depthFileName);
            }
        }
        void extractParamFileNames()
        {
            for(int i=0; i<imageFileNames.size(); ++i)
            {
                std::string paramFileName(imageFileNames[i]);
                paramFileName.replace(dataDir.size(), 8, "/Params/");

                unsigned int pos = paramFileName.find_last_of(".");
                paramFileName.replace(pos, 4, ".txt");
                paramFileNames.push_back(paramFileName);
            }
        }
        void extractParamDefaultFileName()
        {
            paramDefault.assign(dataDir);
            paramDefault.append("/Params/default.txt");
        }

        void extractFileNames()
        {
            extractImageFileNames();
            extractDepthFileNames();
            extractParamFileNames();
            extractParamDefaultFileName();
        }



    public:
        FileNameExtractor(const std::string &dataDir_,
                          std::vector<std::string> & imageFileNames_,
                          std::vector<std::string> & depthFileNames_,
                          std::vector<std::string> & paramFileNames_,
                          std::string &paramDefault_,
                          unsigned int states_ = 1)
                : dataDir(dataDir_)
                , imageFileNames(imageFileNames_)
                , depthFileNames(depthFileNames_)
                , paramFileNames(paramFileNames_)
                , paramDefault(paramDefault_)
        {
            state = states_;
            extractFileNames();
        }

        virtual ~FileNameExtractor(){}

    };
}

#endif //MY3DPHOTO_1FILENAMEEXTRACTOR_H
