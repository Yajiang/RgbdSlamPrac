//相关头文件
#include "join_pointcloud.h"
//C/C++系统文件
#include <string>
#include <iostream>
//第三方库文件
#include <opencv2/core.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <pcl-1.9/pcl/filters/voxel_grid.h> 
//项目内文件

using namespace std;

void JoinPointcloud::CvMat2Eigen(const cv::Mat &rotation_vector, const cv::Mat &translation_vector, Eigen::Isometry3d *transform)
{
    cv::Mat rotation_matrix;
    cv::Rodrigues(rotation_vector, rotation_matrix);
    Eigen::Matrix3d rotation_matrix_eigen;
    cv::cv2eigen(rotation_matrix, rotation_matrix_eigen);
    Eigen::AngleAxisd rotation(rotation_matrix_eigen);
    *transform = rotation;
    (*transform)(0, 3) = translation_vector.at<double>(0, 0);
    (*transform)(1, 3) = translation_vector.at<double>(1, 0);
    (*transform)(2, 3) = translation_vector.at<double>(2, 0);
    cout << (*transform).matrix() << endl;
}
void JoinPointcloud::CombinePointcloud(const PointCloud::Ptr input_cloud, const Frame &frame,const Eigen::Isometry3d& transfrom,PointCloud::Ptr output_cloud)
{
    SlamTransform slam_transform;
    PointCloud::Ptr whole_cloud(new PointCloud());

    if (input_cloud == nullptr)
    {
        slam_transform.ImageToPointCloud(frame.rgb_data, frame.depth_data, input_cloud);
        *whole_cloud = *input_cloud;
    }

    else
    {
        PointCloud::Ptr tmp_cloud(new PointCloud());
        slam_transform.ImageToPointCloud(frame.rgb_data, frame.depth_data, tmp_cloud);
        PointCloud::Ptr tmp_cloud2(new PointCloud());
        pcl::transformPointCloud(*tmp_cloud, *tmp_cloud2, transform.matrix());
        *whole_cloud = *input_cloud + *tmp_cloud2;
    }
    //点云滤波
    static pcl::VoxelGrid<PointT> voxel;
    static SlamParameters slam_parameters;
    double grid_size = atof(slam_parameters.ReadData("voxel_grid"));
    voxel.setLeafSize(grid_size,grid_size,grid_size);
    voxel.setInputCloud(whole_cloud);
    voxel.filter(*output_cloud);
}
void JoinPointcloud::ReadFrame(const int index, Frame *frame)
{
    string folder_name = "../data/rgb_png/";
    string file_name = folder_name + to_string(index);
    frame->rgb_data = cv::imread("../data/rgb1.png");

    string folder_name = "../data/depth_png/";
    string file_name = folder_name + to_string(index);
    frame->depth_data = cv::imread("../data/depth1.png", -1);
}
double JoinPointcloud::NormDistance(cv::Mat& rotation_vec,cv::Mat& translation_vec)
{
    return fabs(min(cv::norm(rotation_vec),2*M_PI-cv::norm(rotation_vec))+fabs(cv::norm(translation_vec)));
}