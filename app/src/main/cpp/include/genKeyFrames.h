//
// Created by ale on 18-12-20.
//

#ifndef I3D_GENKEYFRAMES_H
#define I3D_GENKEYFRAMES_H

#include "i3d.h"
#include "Frame.h"
#include <vector>
#include <opencv2/opencv.hpp>

//从frames中剔除未在edges中出现的帧，并重新编好序号，存入kframes中
void genKeyFrames(const std::vector<i3d::Frame>& frames, std::vector<i3d::Edge>& edges, std::vector<i3d::Frame>& kframes)
{
    using namespace std;
    using namespace cv;

    //for(int i=0; i<edges.size(); i++)
    //    cout<<"edges "<<i<<": "<<edges[i].src<<" "<<edges[i].dst<<endl;

    vector<bool> key_vertexs;
    key_vertexs.resize(frames.size(), false);

    for(int i=0; i<edges.size(); i++)
    {
        key_vertexs[edges[i].src] = true;
        key_vertexs[edges[i].dst] = true;
        //cout<<"edges "<<i<<": "<<edges[i].src<<" "<<edges[i].dst<<endl;
    }

    vector<int> indexs_of_key;
    indexs_of_key.resize(frames.size());
    int index = 0;
    for(int i=0; i<frames.size(); i++)
    {
        if(key_vertexs[i] == true)
            indexs_of_key[i] = index++;
    }

    for(int i=0; i<edges.size(); i++)
    {
        int src = edges[i].src;
        int dst = edges[i].dst;

        edges[i].src = indexs_of_key[src];
        edges[i].dst = indexs_of_key[dst];
    }

    for(int i=0; i<frames.size(); i++)
    {
        if(key_vertexs[i] == true)
        {
            kframes.push_back(frames[i]);
        }
    }

    //debug
    //cout<<"GetKeyFrames(): kframes.size() and  index: "<<kframes.size()<<" "<<index<<endl;
    //for(int i=0; i<edges.size(); i++)
    //    cout<<"edges "<<i<<": "<<edges[i].src<<" "<<edges[i].dst<<endl;
}

#endif //I3D_GENKEYFRAMES_H
