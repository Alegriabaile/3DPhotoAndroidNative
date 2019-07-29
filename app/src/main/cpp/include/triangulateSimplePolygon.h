//
// Created by ale on 19-3-14.
//

#ifndef I3D_TRIANGULATESIMPLEPOLYGON_H
#define I3D_TRIANGULATESIMPLEPOLYGON_H
//
// Created by ale on 19-3-8.
//
#include <vector>
#include <list>
#include <opencv2/opencv.hpp>

//调试用
//#define DEBUG_TRIANGULATE_SIMPLE_POLYGON

/*输入顶点集坐标默认按普通坐标系处理：
 * y
 * ^
 * |
 * o ——> x
 * 如果是opencv顶点坐标，则需要自己先行处理好
 * （比如按顺时针方向顺序输入轮廓顶点集）
 */

//三个基础函数
//A:i-1, B:i, C:i+1
static bool isConcave(const cv::Point& A, const cv::Point& B, const cv::Point& C)
{
    //if(v1.cross(v2)>0, 逆时针旋转
    //if(v1.cross(v2)<0, 顺时针旋转
    //AB->BC逆时针则为凸点，否则为凹点
    return ((B - A).cross(C - B) < 0);//顺时针则定为凹点
    //return ((C - B).cross(B - A) < 0);//opencv 坐标系
}
//逆时针三角形ABC与凹点，若凹点在三角形里面，则凹点在AB，BC和CA左边
static bool isInside(const cv::Point& A, const cv::Point& B, const cv::Point& C, const cv::Point& concave)
{
    //if(v1.cross(v2)>0, 逆时针旋转
    if( (B-A).cross(concave-A)>0 && (C-B).cross(concave-B)>0 && (A-C).cross(concave-C)>0)
        return true;

    return false;
}

static bool isIntersected(const cv::Point& a, const cv::Point& b, const cv::Point& c, const cv::Point& d)
{
    if(std::max(a.x, b.x) < std::min(c.x, d.x))
        return false;
    if(std::max(a.y, b.y) < std::min(c.y, d.y))
        return false;

    if(std::max(c.x, d.x) < std::min(a.x, b.x))
        return false;
    if(std::max(c.y, d.y) < std::min(a.y, b.y))
        return false;
    //平行不算相交
    if((a-c).cross(d-c)*(d-c).cross(b-c)<=0)
        return false;

    if((d-a).cross(b-a)*(b-a).cross(c-a)<=0)
        return false;

    return true;
}
//A:i-1, B:i, C:i+1; B is an ear tip if all concaves are outside triangle ABC
static bool isEarTip(const cv::Point& A, const cv::Point& B, const cv::Point& C, const std::vector<bool> & concave, const std::vector<cv::Point>& vertices, const std::list<int>& indices)
{
    for(auto iter = indices.cbegin(); iter != indices.cend(); ++iter)
    {
        if(!concave[*iter])
            continue;

        auto iter_pre = std::prev(iter);
        auto iter_next = std::next(iter);

        if(iter_pre == indices.cend())
            iter_pre = std::prev(iter_pre);
        if(iter_next == indices.cend())
            iter_next = std::next(iter_next);

        //若凹点不在三角形内部(可以在三角形△ABC边上)
        if(isInside(A, B, C, vertices[*iter]))
            return false;

        //若凹点不在AC上(不与AC共线，则说明在AB/BC上)，且其相邻点组成的线段交于AC线段，则三角形ABC内部存在非mask点
        if( isIntersected(A, C, vertices[*iter_pre], vertices[*iter]) ||
            isIntersected(A, C, vertices[*iter], vertices[*iter_next]))
            return false;
    }

    return true;
}

//初始化
static void updateLists(const std::vector<cv::Point>& vertices, std::list<int>& indices, std::vector<bool>& concave, std::vector<bool>& eartips)
{
    //状态清零
    /*
    for(int i=0; i<concave.size(); ++i)
    {
        concave[i] = false;
        eartips[i] = false;
    }*/
    //寻找凹点
    for(std::list<int>::iterator iter = indices.begin(); iter!=indices.end(); ++iter)
    {
        std::list<int>::iterator iter_pre = std::prev(iter);
        std::list<int>::iterator iter_next = std::next(iter);

        if(iter_pre == indices.end())
            iter_pre = std::prev(iter_pre);
        if(iter_next == indices.end())
            iter_next = std::next(iter_next);

        cv::Point A = vertices[*iter_pre];
        cv::Point B = vertices[*iter];
        cv::Point C = vertices[*iter_next];

        //if(isConcave( A, B, C))
        //    concave[*iter] = true;
        concave[*iter] = isConcave(A, B, C);
    }
    //更新EarTip点
    for(std::list<int>::iterator iter = indices.begin(); iter!=indices.end(); ++iter)
    {
        std::list<int>::iterator iter_pre = std::prev(iter);
        std::list<int>::iterator iter_next = std::next(iter);

        if(iter_pre == indices.end())
            iter_pre = std::prev(iter_pre);
        if(iter_next == indices.end())
            iter_next = std::next(iter_next);

        cv::Point A = vertices[*iter_pre];
        cv::Point B = vertices[*iter];
        cv::Point C = vertices[*iter_next];

        //if(!concave[*iter] && isEarTip(A, B, C, concave, vertices))
        //    eartips[*iter] = true;
        if(!concave[*iter])
            eartips[*iter] = isEarTip(A, B, C, concave, vertices, indices);
    }
}
//更新相应节点的信息
static void updateConcave(const std::vector<cv::Point>& vertices, const std::list<int>& indices,
        const std::list<int>::const_iterator& iter, std::vector<bool>& concave)
{
    std::list<int>::const_iterator iter_pre = std::prev(iter);
    std::list<int>::const_iterator iter_next = std::next(iter);

    if(iter_pre == indices.end())
        iter_pre = std::prev(iter_pre);
    if(iter_next == indices.end())
        iter_next = std::next(iter_next);

    cv::Point A = vertices[*iter_pre];
    cv::Point B = vertices[*iter];
    cv::Point C = vertices[*iter_next];

    concave[*iter] = isConcave( A, B, C);
}
static void updateEarTip(const std::vector<cv::Point>& vertices, const std::list<int>& indices,
        const std::list<int>::const_iterator& iter, const std::vector<bool>& concave, std::vector<bool>& eartip)
{
    std::list<int>::const_iterator iter_pre = std::prev(iter);
    std::list<int>::const_iterator iter_next = std::next(iter);

    if(iter_pre == indices.end())
        iter_pre = std::prev(iter_pre);
    if(iter_next == indices.end())
        iter_next = std::next(iter_next);

    cv::Point A = vertices[*iter_pre];
    cv::Point B = vertices[*iter];
    cv::Point C = vertices[*iter_next];
    if(!concave[*iter])
        eartip[*iter] = isEarTip(A, B, C, concave, vertices, indices);
}

//实际调用函数
int triangulateSimplePolygon(const std::vector<cv::Point>& vertices, std::vector<cv::Point>& triangles)
{
    //std::vector<cv::Point> triangles;
    triangles.clear();

    std::list<int> indices(vertices.size());
    std::list<int>::iterator iter = indices.begin();
    for(int i=0; i<vertices.size(); ++i)
    {
        *iter = i;
        iter++;
    }

    //记录凹点与耳点
    std::vector<bool> concave(vertices.size(), false);
    std::vector<bool> eartips(vertices.size(), false);
    //初始化，O(n**2)
    updateLists(vertices, indices, concave, eartips);
    while(indices.size() >=3)
    {
#ifdef DEBUG_TRIANGULATE_SIMPLE_POLYGON
        cout<<"indices.size(): "<<indices.size()<<endl;

        cout<<"concave indices: ";
        for(int i=0; i<concave.size(); ++i)
        {
            if(concave[i])
                cout<<i<<"\t";
        }
        cout<<endl;

        cout<<"eartips indices: ";
        for(int i=0; i<eartips.size(); ++i)
        {
            if(eartips[i])
                cout<<i<<"\t";
        }
        cout<<endl;

        cout<<"triangles.size(): "<<triangles.size()<<endl;

        imshow("debug", Mat(800, 800, CV_8UC1, Scalar(0)));
        waitKey();
#endif
        for(int i=0; i<eartips.size(); ++i)
        {
            if (eartips[i])
            {
                std::list<int>::iterator iter = find(indices.begin(), indices.end(), i);
                std::list<int>::iterator iter_pre = std::prev(iter);
                std::list<int>::iterator iter_next = std::next(iter);

                if (iter_pre == indices.end())
                    iter_pre = std::prev(iter_pre);
                if (iter_next == indices.end())
                    iter_next = std::next(iter_next);

                triangles.push_back(vertices[*iter_pre]);
                triangles.push_back(vertices[*iter]);
                triangles.push_back(vertices[*iter_next]);

                eartips[*iter] = false;
                std::list<int>::iterator  temp_next = std::next(iter);
                std::list<int>::iterator temp_pre = std::prev(iter);
                indices.erase(iter);
                if(temp_next == indices.end())
                    temp_next = std::next(temp_next);
                if(temp_pre == indices.end())
                    temp_pre = std::prev(temp_pre);


                //必须先将所有Concave找出来，才能进行Eartip的判定
                //只有被clipped节点的相邻节点的状态才会发生改变，所以不需要对所有剩余节点进行状态更新
                updateConcave(vertices, indices, temp_pre, concave);
                updateConcave(vertices, indices, temp_next, concave);
                updateEarTip(vertices, indices, temp_pre, concave, eartips);
                updateEarTip(vertices, indices, temp_next, concave, eartips);

#ifdef DEBUG_TRIANGULATE_SIMPLE_POLYGON
                if(iter == indices.end())
                    cout<<"error in find eartips i in indices"<<endl;
                cout<<"iter: "<<*iter<<" i: "<<i;
                cout<<" iter_pre: "<<*iter_pre<<
                " iter_next: "<<*iter_next<<endl
                <<" temp_pre, temp_next: "<<*temp_pre<<", "<<*temp_next<<endl<<endl;
#endif
                break;
            }
        }
    }
    indices.clear();
    concave.clear();
    eartips.clear();

    return 0;
}
#endif //I3D_TRIANGULATESIMPLEPOLYGON_H
