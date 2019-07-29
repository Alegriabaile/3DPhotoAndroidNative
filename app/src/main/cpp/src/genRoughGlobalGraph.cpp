//
// Created by ale on 18-12-20.
//


#include "genRoughGlobalGraph.h"
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include<opencv2/opencv.hpp>
#include <opencv/cxeigen.hpp>
#include <opencv2/core/eigen.hpp>

//g2o的头文件
#include <g2o/types/slam3d/types_slam3d.h> //顶点类型
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

using namespace cv;
using namespace std;

int genRoughGlobalGraph(vector<i3d::Edge>& edges, vector<i3d::Frame>& kframes)
{
    g2o::LinearSolverCSparse< g2o::BlockSolver_6_3::PoseMatrixType >* linearSolver;
    linearSolver =new g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(new g2o::BlockSolver_6_3(linearSolver) );
    g2o::SparseOptimizer globalOptimizer;  // 最后用的就是这个东东
    globalOptimizer.setAlgorithm( solver );
    globalOptimizer.setVerbose(false);//false:不显示 true:显示

    std::map<int, bool> key_vertexs;

    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
    information(0,0) = information(1,1) = information(2,2) = 100;
    information(3,3) = information(4,4) = information(5,5) = 100;
    for(int i=0; i<edges.size(); i++)
    {
        if(key_vertexs.find(edges[i].src) == key_vertexs.end())
        {
            g2o::VertexSE3* v = new g2o::VertexSE3();
            v->setId(edges[i].src);
            v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
            globalOptimizer.addVertex(v);
            key_vertexs[edges[i].src] = true;
        }
        if(key_vertexs.find(edges[i].dst) == key_vertexs.end())
        {
            g2o::VertexSE3* v = new g2o::VertexSE3();
            v->setId(edges[i].dst);
            v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
            globalOptimizer.addVertex(v);
            key_vertexs[edges[i].dst] = true;
        }
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
        // 连接此边的两个顶点id
        edge->vertices() [0] = globalOptimizer.vertex( edges[i].src );
        edge->vertices() [1] = globalOptimizer.vertex( edges[i].dst );
        //static g2o::RobustKernel* robustKernel = g2o::RobustKernelFactory::instance()->construct( "Cauchy" );
        //edge->setRobustKernel( robustKernel );
        //edge->setRobustKernel( new g2o::RobustKernelHuber());
        edge->setRobustKernel( new g2o::RobustKernelCauchy());
        //cout<<"也可以将角度设大一些，表示对角度的估计更加准确"<<endl;
        edge->setInformation( information );
        //cout<<"边的估计即是pnp求解之结果"<<endl;

        cv::Mat R;
        cv::Mat rvec = (Mat_<double>(3,1) << edges[i].rx, edges[i].ry, edges[i].rz);
        cv::Rodrigues( rvec, R );
        Eigen::Matrix3d r;
        cv::cv2eigen(R, r);

        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        Eigen::AngleAxisd angle(r);
        T = angle;
        T(0,3) = edges[i].tx;
        T(1,3) = edges[i].ty;
        T(2,3) = edges[i].tz;
        edge->setMeasurement( T );

        // 将此边加入图中
        //cout<<"将此边加入图中"<<endl;
        //es.push_back(edge);
        globalOptimizer.addEdge(edge);
    }

    //cout<<"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
    //cout<<"optimizing pose graph, edges: "<<globalOptimizer.edges().size()<<endl;
    //globalOptimizer.save("result_before.g2o");
    //cout<<"globalOptimizer.initializeOptimization();"<<endl;
    globalOptimizer.initializeOptimization();
    //cout<<"指定优化步数"<<endl;
    globalOptimizer.optimize( 300 ); //可以指定优化步数

    //globalOptimizer.save( "result_after.g2o" );
    //cout<<"Optimization done."<<endl;

    for(int i=0; key_vertexs.find(i)!=key_vertexs.end(); i++)
    {
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex( i ));
        Eigen::Isometry3d pose = vertex->estimate();

        cv::Mat rvec = Mat::zeros(3, 1, CV_64FC1);
        cv::Mat R = Mat::zeros(3,3,CV_64FC1);
        Eigen::Matrix3d r(pose.rotation());//cout<<r<<endl;
        cv::eigen2cv(r, R);
        cv::Rodrigues(R, rvec);
        kframes[i].rx = rvec.at<double>(0);
        kframes[i].ry = rvec.at<double>(1);
        kframes[i].rz = rvec.at<double>(2);
        cv::Mat tvec = Mat::zeros(3, 1, CV_64FC1);
        Eigen::Vector3d t(pose.translation());
        cv::eigen2cv(t, tvec);
        kframes[i].tx = tvec.at<double>(0);
        kframes[i].ty = tvec.at<double>(1);
        kframes[i].tz = tvec.at<double>(2);
    }

    globalOptimizer.clear();

    return 0;
}