#pragma once
//相关头文件

//C/C++系统文件
#include <vector>
//第三方库文件
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
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
//项目内文件
#include "slam_parameters.h"
#include "slam_transform.h"

struct Frame
{
    ushort id;
    cv::Mat rgb_data, depth_data;
    cv::Mat descriptor;
    vector<cv::KeyPoint> key_points;
};

struct PnpResult
{
    cv::Mat rotation_vector, translation_vector;
    int inliers;
};

class SlamEstimate
{
  public:
    SlamEstimate();
    ~SlamEstimate();
    void ComputeKeyPointAndDescriptor(Frame &);
    PnpResult EstimateMotion(Frame &frame1, Frame &frame2);

  private:
    SlamParameters slam_parameters;
    SlamTransform slam_transform;
};