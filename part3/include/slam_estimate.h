#pragma once
#include <slam_parameters.h>
#include <slam_transform.h>
#include <vector>
// OpenCV 特征检测模块
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>

struct Frame
{
    cv::Mat rgb_data, depth_data;
    cv::Mat descriptor;
    vector <cv::KeyPoint> key_points;
};

struct PnpResult
{
    cv::Mat rotation_vector, translation_vector;
    int inliers;
};


class SlamEstimate
{
   SlamEstimate();
   ~SlamEstimate();
   void ComputeKeyPointAndDescriptor(Frame&);
   PnpResult EstimateMotion(Frame& frame1,Frame& frame2);
};