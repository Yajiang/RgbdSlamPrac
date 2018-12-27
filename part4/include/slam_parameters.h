#pragma once
//相关头文件

//C/C++系统文件
#include <string>
//第三方库文件
#include <opencv2/opencv.hpp>
//项目内文件

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
    std::string ReadData(std::string);
    private:
    std::string descriptor_extractor_;
    std::string feature_detector_;
    int good_match_threshold_;
    int scaling_factor_;
    cv::Mat camera_intrinsic_matrix_;
};