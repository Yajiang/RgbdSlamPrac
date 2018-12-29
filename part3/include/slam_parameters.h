#pragma once
#include <string>
#include <opencv2/opencv.hpp>

class SlamParameters
{
    public:
    SlamParameters();
    ~SlamParameters();
    void GetCameraParameters(cv::Mat&);
    int GetScalingFactor();
    std::string GetDiscriptorMethod();
    std::string GetFeatureMethod();
    int GetMatchThreshold();
    void ReadParameters();
    void InitParameters();
    void ResetParameters();
    private:
    std::string descriptor_extractor_;
    std::string feature_detector_;
    std::string param_value_;
    int good_match_threshold_;
    int scaling_factor_;
    cv::Mat camera_intrinsic_matrix_;
};