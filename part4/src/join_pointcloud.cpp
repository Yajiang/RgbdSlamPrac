//相关头文件
#include "join_pointcloud.h"
//C/C++系统文件
#include <string>
#include <sstream>
#include <iostream>
//第三方库文件
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <pcl-1.9/pcl/filters/voxel_grid.h>
#include <pcl-1.9/pcl/io/pcd_io.h>
#include <pcl-1.9/pcl/point_types.h>
#include <pcl-1.9/pcl/common/transforms.h>
//项目内文件

using namespace std;

JoinPointcloud::JoinPointcloud()
{
}
JoinPointcloud::~JoinPointcloud()
{
}

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
}
void JoinPointcloud::CombinePointcloud(const PointCloud::Ptr input_cloud, const Frame &frame, const Eigen::Isometry3d &transform, PointCloud::Ptr output_cloud)
{
    PointCloud::Ptr whole_cloud(new PointCloud());
    PointCloud::Ptr new_cloud(new PointCloud());
    slam_transform.ImageToPointCloud(frame.rgb_data, frame.depth_data, new_cloud);
    PointCloud::Ptr tmp_cloud(new PointCloud());
    pcl::transformPointCloud(*input_cloud, *tmp_cloud, transform.matrix());
    *whole_cloud = *tmp_cloud + *new_cloud;
    //点云滤波
    static pcl::VoxelGrid<PointT> voxel;
    double grid_size = atof(slam_parameters.ReadData("voxel_grid").c_str());
    voxel.setLeafSize(grid_size, grid_size, grid_size);
    voxel.setInputCloud(whole_cloud);
    voxel.filter(*output_cloud);
}
void JoinPointcloud::ReadFrame(const int &index, Frame *frame)
{

    string folder_name = "/home/eugene/slam/part4/data/rgb_png/";
    string file_name = folder_name + to_string(index) + ".png";
    frame->rgb_data = cv::imread(file_name.c_str());

    folder_name = "/home/eugene/slam/part4/data/depth_png/";
    file_name = folder_name + to_string(index) + ".png";
    frame->depth_data = cv::imread(file_name.c_str(), -1);
}
double JoinPointcloud::NormDistance(cv::Mat &rotation_vec, cv::Mat &translation_vec)
{
    double norm_distance = fabs(min(cv::norm(rotation_vec), 2 * M_PI - cv::norm(rotation_vec))) + fabs(cv::norm(translation_vec));
    return norm_distance;
}