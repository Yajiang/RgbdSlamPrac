//相关头文件
#include "slam_estimate.h"
//C/C++系统文件

//第三方库文件

//项目内文件

SlamEstimate::SlamEstimate()
{
}
SlamEstimate::~SlamEstimate()
{
}
void SlamEstimate::ComputeKeyPointAndDescriptor(Frame &frame)
{
    // 声明特征提取器与描述子提取器
    cv::Ptr<cv::FeatureDetector> feature_detector;
    cv::Ptr<cv::DescriptorExtractor> descriptor_extractor;
    cv::initModule_nonfree();
    feature_detector = cv::FeatureDetector::create(slam_parameters.GetFeatureMethod());
    descriptor_extractor = cv::DescriptorExtractor::create(slam_parameters.GetDiscriptorMethod());
    //提取关键点
    feature_detector->detect(frame.rgb_data, frame.key_points);
    //计算描述子
    descriptor_extractor->compute(frame.rgb_data, frame.key_points, frame.descriptor);
    return;
}
PnpResult SlamEstimate::EstimateMotion(Frame &frame1, Frame &frame2)
{
    //计算两帧图像之间的匹配
    vector<cv::DMatch> matches;
    cv::FlannBasedMatcher matcher;
    matcher.match(frame1.descriptor, frame2.descriptor, matches);
    double min_distance = 9999;
    for (size_t i = 0; i < matches.size(); i++)
    {
        if (matches.at(i).distance < min_distance)
        {
            min_distance = matches.at(i).distance;
        }
    }
    //滤除距离过大的匹配
    //判断依据： 最小匹配距离 × 匹配阈值
    double match_threshold = slam_parameters.GetMatchThreshold();
    vector<cv::DMatch> good_matches;
    // cout<<"match_threshold:"<<match_threshold<<endl;
    for (size_t i = 0; i < matches.size(); i++)
    {
        if (matches.at(i).distance < min_distance * match_threshold)
            good_matches.push_back(matches.at(i));
    }
    // 计算图像间的运动关系
    // 关键函数：cv::solvePnPRansac()
    // 第一个帧的三维点
    vector<cv::Point3f> space_points;
    // 第二个帧的图像点
    vector<cv::Point2f> image_points;
    cv::Point3f *point_space(new cv::Point3f());

    for (size_t i = 0; i < good_matches.size(); i++)
    {
        // query 是第一个, train 是第二个
        cv::Point2f point_2d = frame1.key_points[good_matches[i].queryIdx].pt;
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort d = frame1.depth_data.ptr<ushort>(int(point_2d.y))[int(point_2d.x)];
        if (d == 0)
            continue;
        image_points.push_back(cv::Point2f(frame2.key_points[good_matches[i].trainIdx].pt));
        // 将(u,v,d)转成(x,y,z)
        cv::Point3f point_3d(point_2d.x, point_2d.y, d);
        slam_transform.Point2dTo3d(point_3d, point_space);
        space_points.push_back(*point_space);
    }
    delete point_space;

    // 构建相机矩阵
    cv::Mat camera_matrix;
    slam_parameters.GetCameraParameters(camera_matrix);
    // 求解pnp
    cv::Mat inliers;
    PnpResult result;
    int min_inliers = atoi(slam_parameters.ReadData("min_inliers").c_str());
    if (good_matches.size() < min_inliers || space_points.size()== 0)
    {
        result.inliers = 0;
        return result;
    }
    double min_tolerance;
    min_tolerance = atof(slam_parameters.ReadData("min_tolerance").c_str());
    cv::solvePnPRansac(space_points, image_points, camera_matrix, cv::Mat(), result.rotation_vector, result.translation_vector, false, 100, min_tolerance, 100, inliers);
    result.inliers = inliers.rows;
    // 画出inliers匹配
    vector<cv::DMatch> matches_show;
    cv::Mat img_matches;
    for (size_t i = 0; i < inliers.rows; i++)
    {
        matches_show.push_back(good_matches[inliers.ptr<int>(i)[0]]);
    }
    // cv::drawMatches(frame1.rgb_data,frame1.key_points,frame2.rgb_data,frame2.key_points, matches_show, img_matches);
    // cv::imshow("inliers",img_matches);
    // cv::imwrite("../data/inliers.png", img_matches);
    // cv::waitKey(0);
    return result;
}
