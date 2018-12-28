#pragma once
//C/C++系统文件
#include<iostream>
#include<string>
//第三方库文件
using namespace std;
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
