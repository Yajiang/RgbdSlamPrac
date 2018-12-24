#include <slam_estimate.h>

SlamEstimate::SlamEstimate()
{

}
SlamEstimate::~SlamEstimate()
{

}
void SlamEstimate::ComputeKeyPointAndDescriptor(Frame &frame)
{
    //获取匹配参数
    SlamParameters slam_parameters;
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
    // 可视化， 显示关键点
    // cv::Mat img_show;
    // cv::drawKeypoints(frame.rgb_data,frame.keypoints,img_show,cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    // cv::imshow("keypoints", img_show);
    // cv::imwrite("../data/keypoints.png", img_show);
    // cv::waitKey(0); //暂停等待一个按键
    return;
}
PnpResult SlamEstimate::EstimateMotion(Frame &frame1, Frame &frame2)
{
    //获取匹配参数及相机内参
    SlamParameters slam_parameters;
    SlamTransform slam_transform;
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
        cv::Point3f *point_space;
        slam_transform.Point2dTo3d(point_3d, point_space);
        space_points.push_back(*point_space);
    }
    cv::Mat camera_matrix;
    // 构建相机矩阵
    slam_parameters.GetCameraParameters(camera_matrix);
    cout << "camera_matrix" << camera_matrix << endl;
    // 求解pnp
    cv::Mat inliers;
    PnpResult result;
    cv::solvePnPRansac(space_points, image_points, camera_matrix, cv::Mat(), result.rotation_vector,result.translation_vector, false, 100, 1, 100,inliers);
    result.inliers = inliers.rows;
    // 画出inliers匹配
    // vector<cv::DMatch> matches_show;
    // for (size_t i = 0; i < inliers.rows; i++)
    // {
    //     matches_show.push_back(good_matches[inliers.ptr<int>(i)[0]]);
    // }
    // cv::drawMatches(rgb_data_1, kp1, rgb_data_2, kp2, matches_show, img_matches);
    // cv::imwrite("../data/inliers.png", img_matches);
    // cv::waitKey(0);
    return result;
}
