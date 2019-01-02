#pragma once
//相关头文件

//C/C++系统文件
#include <string>
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
//项目内文件
#include "slam_transform.h"
class SlamParameters
{
  public:
    SlamParameters();
    ~SlamParameters();
    void GetCameraParameters(cv::Mat &);
    int GetScalingFactor();
    std::string GetDiscriptorMethod();
    std::string GetFeatureMethod();
    int GetMatchThreshold();
    void ReadParameters();
    void InitParameters();
    void ResetParameters();
    std::string ReadData(std::string);

  private:
    cv::FileStorage fs;
    std::string param_value_;
    std::string descriptor_extractor_;
    std::string feature_detector_;
    int good_match_threshold_;
    int scaling_factor_;
    cv::Mat camera_intrinsic_matrix_;
};