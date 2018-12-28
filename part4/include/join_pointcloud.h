#pragma once
//相关头文件

//C/C++系统文件

//第三方库文件
#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <pcl-1.9/pcl/filters/voxel_grid.h>
#include <pcl-1.9/pcl/io/pcd_io.h>
#include <pcl-1.9/pcl/point_types.h>
#include <pcl-1.9/pcl/common/transforms.h>
//项目内文件
#include "slam_transform.h"
#include "slam_estimate.h"
#include "slam_parameters.h"


class JoinPointcloud
{
private:
    /* data */
public:
    JoinPointcloud(/* args */);
    ~JoinPointcloud();
    void CombinePointcloud(const PointCloud::Ptr,const Frame&,const Eigen::Isometry3d&,PointCloud::Ptr);
    void ReadFrame(const int&,Frame*);
    void CvMat2Eigen(const cv::Mat&,const cv::Mat&,Eigen::Isometry3d* transform);
    double NormDistance(cv::Mat&,cv::Mat&);
};