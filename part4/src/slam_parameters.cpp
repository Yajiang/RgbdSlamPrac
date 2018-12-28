//相关头文件
#include "slam_parameters.h"
//C/C++系统文件
#include <iostream>
//第三方库文件

//项目内文件

using namespace std;
SlamParameters::SlamParameters()
{
    InitParameters();
}
SlamParameters::~SlamParameters()
{
}
void SlamParameters::GetCameraParameters(cv::Mat &camera_intrinsic_matrix)
{
    camera_intrinsic_matrix = camera_intrinsic_matrix_.clone();
}
int SlamParameters::GetScalingFactor()
{
    return scaling_factor_;
}
std::string SlamParameters::GetDiscriptorMethod()
{
    std::cout << descriptor_extractor_ << std::endl;
    return descriptor_extractor_;
}
std::string SlamParameters::GetFeatureMethod()
{
    return feature_detector_;
}
int SlamParameters::GetMatchThreshold()
{
    return good_match_threshold_;
}
void SlamParameters::ReadParameters()
{
    fs["camera_intrinsic_matrix"] >> camera_intrinsic_matrix_;
    fs["FeatureDetector"] >> feature_detector_;
    fs["DescriptorExtractor"] >> descriptor_extractor_;
    fs["scaling_factor"] >> scaling_factor_;
    fs["good_match_threshold"] >> good_match_threshold_;
    fs.release();
}
void SlamParameters::InitParameters()
{
    fs.open("/home/eugene/slam/part4/params/config.yml", cv::FileStorage::WRITE);
    cout<< fs.isOpened()<<endl;
    std::string version = "";
    fs["version"] >> version;
    if (version == "")
        ResetParameters();
    ReadParameters();
}
void SlamParameters::ResetParameters()
{
    cv::Mat camera_intrinsic_matrix = (cv::Mat_<double>(3, 3) << 518, 0, 325.5, 0, 519, 253.5, 0, 0, 1);
    fs << "version"
       << "0.0";
    // fs<<"/#特征检测算法";
    fs << "FeatureDetector"
       << "GridSIFT";
    // fs<<"/#特征描述算法";
    fs << "DescriptorExtractor"
   << "SIFT";
    fs << "camera_intrinsic_matrix" << camera_intrinsic_matrix;
    fs << "scaling_factor" << 1000;
    fs << "good_match_threshold" << 4;
    fs << "start_index" << 1;
    fs << "end_index" << 10;
    fs << "voxel_grid" << 0.01;
    fs << "min_good_match" << 10;
    fs << "min_inliers" << 5;
    fs << "max_norm" << 0.3;
    fs.release();
}
string SlamParameters::ReadData(string param_name)
{
    string param_value;
    bool is_opened = fs.isOpened();
    cout << is_opened << endl;
    fs[param_name] >> param_value;
    cout << param_value << endl;
    return param_value;
}