#pragma once
//相关头文件

//C/C++系统文件

//第三方库文件
#include <opencv/cv.hpp>
#include <opencv2/core.hpp>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/src/Geometry/Transform.h>
#include <pcl-1.9/pcl/pcl_base.h>
//项目内文件

class JoinPointcloud
{
private:
    /* data */
public:
    JoinPointcloud(/* args */);
    ~JoinPointcloud();
    void JoinPointcloud::CombinePointcloud(const PointCloud::Ptr,const Frame&,const Eigen::Isometry3d&,PointCloud::Ptr);
    void JoinPointcloud::ReadFrame();
    void JoinPointcloud::CvMat2Eigen(cv::Mat&,cv::Mat&,Eigen::Isometry3d* transform);
    double JoinPointcloud::NormDistance(cv::Mat&,cv::Mat&);
};