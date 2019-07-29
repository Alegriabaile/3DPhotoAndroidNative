//
// Created by ale on 18-12-20.
//
#include "genGlobalByMst.h"

#include <set>

using namespace std;
using namespace i3d;

int genGlobalByMst(const std::vector<i3d::Edge>& edges, std::vector<i3d::Frame>& kframes, std::vector<i3d::Edge>& kedges)
{
    if(edges.empty())
        return -1;

    vector<Edge> sortedEdges;
    sortedEdges.assign(edges.begin(), edges.end());

    double cost_max =2.0;
    for(int i=0; i<sortedEdges.size(); ++i)
    {
        sortedEdges[i].cost = cost_max;
    }

    //边长排序
    vector<bool> edgesBool(edges.size(), false);
    for(int i=0; i<edges.size(); ++i)
    {
        int k = i;
        for(int j=0; j<edges.size(); ++j)
        {
            if(edgesBool[j]==false && sortedEdges[i].cost>edges[j].cost)
            {
                sortedEdges[i] = edges[j];
                k=j;
            }
        }
        edgesBool[k] = true;
    }
    //debug 边长排序
    //for(int i=0; i<edges.size(); ++i)
    //{
    //    cout<<edges[i].cost<<" "<<sortedEdges[i].cost<<endl;
    //}

    vector<Edge> selectedEdges;
    selectedEdges.push_back(sortedEdges[0]);

    set<int> index;
    index.insert(selectedEdges[0].src);
    index.insert(selectedEdges[0].dst);

    vector<bool> sortedEdgesSelected(sortedEdges.size(), false);
    sortedEdgesSelected[0] = true;//sortedEdges[0] selected and inserted.

    //Prim算法，并检测是否连通
    int sum = 2;
    while(sum<kframes.size())
    {
        bool isFullyConnected = true;
        int i=1;
        for(; i<sortedEdges.size(); ++i)
        {
            int src = sortedEdges[i].src;
            int dst = sortedEdges[i].dst;
            if(sortedEdgesSelected[i]==false && index.find(src)!=index.end() && index.find(dst) == index.end())
            {
                selectedEdges.push_back(sortedEdges[i]);
                index.insert(dst);
                sortedEdgesSelected[i] = true;
                sum++;
                break;
            }
            else if(sortedEdgesSelected[i]==false && index.find(src)==index.end() && index.find(dst) != index.end())
            {
                selectedEdges.push_back(sortedEdges[i]);
                index.insert(src);
                sortedEdgesSelected[i] = true;
                sum++;
                break;
            }
        }

        if(i == sortedEdges.size())
        {
            cout<<"genGlobalByMst.cpp: 初始位恣图不连通！"<<endl;
            break;
        }
        if(isFullyConnected==false)
        {
            cout<<"genGlobalByMst.cpp: 初始位恣图不连通！"<<endl;
            break;
        }

    }

    //debug Prim算法
    //for(int i=0; i<selectedEdges.size(); ++i)
    //    cout<<selectedEdges[i].src<<" "<<selectedEdges[i].dst<<endl;

    //由最小生成树递归生成每一帧的全局位恣
    index.clear();
    index.insert(selectedEdges[0].src);
    index.insert(selectedEdges[0].dst);
    kframes[selectedEdges[0].src].rx = 0;
    kframes[selectedEdges[0].src].ry = 0;
    kframes[selectedEdges[0].src].rz = 0;
    kframes[selectedEdges[0].src].tx = 0;
    kframes[selectedEdges[0].src].ty = 0;
    kframes[selectedEdges[0].src].tz = 0;
    kframes[selectedEdges[0].dst].rx = selectedEdges[0].rx;
    kframes[selectedEdges[0].dst].ry = selectedEdges[0].ry;
    kframes[selectedEdges[0].dst].rz = selectedEdges[0].rz;
    kframes[selectedEdges[0].dst].tx = selectedEdges[0].tx;
    kframes[selectedEdges[0].dst].ty = selectedEdges[0].ty;
    kframes[selectedEdges[0].dst].tz = selectedEdges[0].tz;
    for(int i=1; i<selectedEdges.size(); ++i)
    {
        int src = selectedEdges[i].src;
        int dst = selectedEdges[i].dst;
        if(index.find(src) == index.end())//若src为新顶点
        {
            kframes[src].rx = kframes[dst].rx + selectedEdges[i].rx;
            kframes[src].ry = kframes[dst].ry + selectedEdges[i].ry;
            kframes[src].rz = kframes[dst].rz + selectedEdges[i].rz;
            kframes[src].tx = kframes[dst].tx + selectedEdges[i].tx;
            kframes[src].ty = kframes[dst].ty + selectedEdges[i].ty;
            kframes[src].tz = kframes[dst].tz + selectedEdges[i].tz;
            index.insert(src);
        }else
        {
            kframes[dst].rx = kframes[src].rx - selectedEdges[i].rx;
            kframes[dst].ry = kframes[src].ry - selectedEdges[i].ry;
            kframes[dst].rz = kframes[src].rz - selectedEdges[i].rz;
            kframes[dst].tx = kframes[src].tx - selectedEdges[i].tx;
            kframes[dst].ty = kframes[src].ty - selectedEdges[i].ty;
            kframes[dst].tz = kframes[src].tz - selectedEdges[i].tz;
            index.insert(dst);
        }
    }

    kedges.clear();
    for(int i=0; i<edges.size(); ++i)
    {
        int src = edges[i].src;
        int dst = edges[i].dst;
        if(index.find(src)!=index.end() && index.find(dst)!=index.end())
            kedges.push_back(edges[i]);
    }
    //debug 由最小生成树递归生成全局位恣
    //for(int i=0; i<kframes.size(); ++i)
    //{
    //    cout<<kframes[i].rx<<" "<<kframes[i].ry<<" "<<kframes[i].rz<<" "<<kframes[i].tx<<" "<<kframes[i].ty<<" "<<kframes[i].tz<<endl;
    //}
    return 0;
}