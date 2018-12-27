//相关头文件

//C/C++系统文件
#include <string>
#include <iostream>
//第三方库文件
#include <eigen2/Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <pcl-1.9/pcl/filters/voxel_grid.h>
//项目内文件
#include "slam_estimate.h"
#include "slam_parameters.h"
#include "slam_transform.h"

int main(int argc, char const *argv[])
{
    // 提取特征并计算描述子
    
    SlamEstimate slam_estimate;
    slam_estimate.ComputeKeyPointAndDescriptor(frame1);
    slam_estimate.ComputeKeyPointAndDescriptor(frame2);
    // 求解pnp
    PnpResult pnp_result = slam_estimate.EstimateMotion(frame1,frame2);
    cout<<pnp_result.rotation_vector<<endl;
    cout<<pnp_result.translation_vector<<endl;
    Eigen::Isometry3d* transform = Eigen::Isometry3d::Identity();
    JoinPointcloud::CvMat2Eigen(pnp_result.rotation_vector,pnp_result.translation_vector,transform)
    return 0;
}
