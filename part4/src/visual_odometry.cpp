//相关头文件

//C/C++系统文件
#include <string>
#include <iostream>
//第三方库文件
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <pcl-1.9/pcl/filters/voxel_grid.h>
#include <pcl-1.9/pcl/io/pcd_io.h>
#include <pcl-1.9/pcl/point_types.h>
#include <pcl-1.9/pcl/common/transforms.h>
#include <pcl-1.9/pcl/visualization/cloud_viewer.h>
//项目内文件
#include "slam_estimate.h"
#include "slam_parameters.h"
#include "slam_transform.h"
#include "join_pointcloud.h"

int main(int argc, char const *argv[])
{
    // cout<<"begin"<<endl;
    // 提取特征并计算描述子
    SlamParameters slam_parameters;
    int start_index = atoi(slam_parameters.ReadData("start_index").c_str());
    // cout<<start_index<<endl;
    int end_index = atoi(slam_parameters.ReadData("end_index").c_str());
    int min_inliers = atoi(slam_parameters.ReadData("min_inliers").c_str());
    double max_distance = atof(slam_parameters.ReadData("max_norm").c_str());
    int min_good_match = atoi(slam_parameters.ReadData("min_good_match").c_str());
    SlamEstimate slam_estimate;
    Frame *prev_frame(new Frame());
    Frame *current_frame(new Frame());
    PointCloud::Ptr input_cloud(new PointCloud());
    PointCloud::Ptr output_cloud(new PointCloud());
    static JoinPointcloud join_pointcloud;
    Eigen::Isometry3d *transform(new Eigen::Isometry3d());

    SlamTransform slam_transform;
    pcl::visualization::CloudViewer viewer("viewer");
    bool flag = false;
    for (int i = start_index; i != end_index; i++)
    {
        if (i == start_index)
        {
            join_pointcloud.ReadFrame(i, prev_frame);
            slam_estimate.ComputeKeyPointAndDescriptor(*prev_frame);
            slam_transform.ImageToPointCloud(prev_frame->rgb_data, prev_frame->depth_data, input_cloud);
        }
        else
        {
            join_pointcloud.ReadFrame(i, current_frame);
            slam_estimate.ComputeKeyPointAndDescriptor(*current_frame);
            PnpResult pnp_result = slam_estimate.EstimateMotion(*prev_frame, *current_frame);
            if (pnp_result.inliers < min_inliers)
            {
                flag = false;
                continue;
            }
            if (join_pointcloud.NormDistance(pnp_result.rotation_vector, pnp_result.translation_vector) > max_distance)
            {
                flag = false;
                continue;
            }
            join_pointcloud.CvMat2Eigen(pnp_result.rotation_vector, pnp_result.translation_vector, transform);
            join_pointcloud.CombinePointcloud(input_cloud, *current_frame, *transform, output_cloud);
            flag = true;
            swap(prev_frame, current_frame);
            swap(input_cloud, output_cloud);
            // cout << "transform" << transform->matrix() << endl;
        }
        viewer.showCloud(input_cloud);
    }
    delete prev_frame;
    delete current_frame;
    delete transform;
    pcl::io::savePCDFile("/home/eugene/slam/part4/data/output_cloud.pcd", *input_cloud);
    return 0;
}