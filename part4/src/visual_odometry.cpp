//相关头文件

//C/C++系统文件
#include <string>
#include <iostream>
//第三方库文件
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <pcl-1.9/pcl/filters/voxel_grid.h>
//项目内文件
#include "slam_estimate.h"
#include "slam_parameters.h"
#include "slam_transform.h"
#include "join_pointcloud.h"

int main(int argc, char const *argv[])
{
    // 提取特征并计算描述子
    SlamParameters slam_parameters;
    int start_index = atoi(slam_parameters.ReadData("start_index").c_str());
    int end_index = atoi(slam_parameters.ReadData("end_index").c_str());
    int min_inliers = atoi(slam_parameters.ReadData("min_inliers").c_str());
    double max_distance = atof(slam_parameters.ReadData("max_distance").c_str());
    int min_good_match = atoi(slam_parameters.ReadData("min_good_match").c_str());
    SlamEstimate slam_estimate;
    Frame *prev_frame;
    Frame *current_frame;
    PointCloud::Ptr input_cloud;
    PointCloud::Ptr output_cloud;
    static JoinPointcloud join_pointcloud;

    for (int i = start_index; i != end_index; i++)
    {
        if (i == start_index)
        {
            join_pointcloud.ReadFrame(i, prev_frame);
            slam_estimate.ComputeKeyPointAndDescriptor(*prev_frame);
            SlamTransform slam_transform;
            slam_transform.ImageToPointCloud(prev_frame->rgb_data, prev_frame->depth_data, input_cloud);
        }
        else
        {
            join_pointcloud.ReadData(i, current_frame);
            slam_estimate.ComputeKeyPointAndDescriptor(*current_frame);
            PnpResult pnp_result = slam_estimate.EstimateMotion(*prev_frame, *current_frame);
            if (pnp_result.inliers < min_inliers)
                continue;
            if (join_pointcloud.NormDistance(pnp_result.rotation_vector, pnp_result.translation_vector) > max_distance)
                continue;
            Eigen::Isometry3d *transform;
            join_pointcloud.CvMat2Eigen(pnp_result.rotation_vector, pnp_result.translation_vector, transform);
            join_pointcloud.CombinePointcloud(input_cloud, current_frame, *transform, output_cloud);
            prev_frame = current_frame;
        }
    }
    pcl::io::savePCDFile("../data/output_cloud", *output_cloud);
    return 0;
}