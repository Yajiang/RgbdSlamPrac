#pragma once
//C++ 标准库
#include<iostream>
#include<string>

using namespace std;
//opencv 库
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//PCL 库
#include <pcl-1.9/pcl/pcl_config.h>
#include <pcl-1.9/pcl/pcl_macros.h>
#include <pcl-1.9/pcl/point_types.h>
#include <pcl-1.9/pcl/io/pcd_io.h>

//Define PCL Point
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class SlamTransform
{
    public:
    SlamTransform();
    ~SlamTransform();
    void InitCameraParam();
    void Point2dTo3d(const cv::Point3f&,cv::Point3f*);
    void Point2dTo3d(cv::Mat& rgb_data,cv::Mat& depth_data,ushort,ushort,PointT&);
    void ImageToPointCloud(const cv::Mat& rgb_data,const cv::Mat& depth_data,PointCloud::Ptr point_cloud_ptr);
    private:
    //Camera param
    double camera_factor_;
    double camera_cx_;
    double camera_cy_;
    double camera_fx_;
    double camera_fy_;
};
