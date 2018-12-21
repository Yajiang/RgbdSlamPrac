#include <slam_transform.h>
#include <vector>
using namespace std;

// OpenCV 特征检测模块
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>

int main(int argc, char **argv)
{
    // 声明并从data文件夹里读取两个rgb与深度图
    cv::Mat rgb_data_1 = cv::imread("../data/rgb1.png");
    cv::Mat rgb_data_2 = cv::imread("../data/rgb2.png");
    cv::Mat depth_data_1 = cv::imread("../data/depth1.png", -1);
    cv::Mat depth_data_2 = cv::imread("../data/depth2.png", -1);

    // 声明特征提取器与描述子提取器
    cv::Ptr<cv::FeatureDetector> feature_detector;
    cv::Ptr<cv::DescriptorExtractor> descriptor_extractor;

    // 构建提取器，默认两者都为sift
    // 构建sift, surf之前要初始化nonfree模块
    cv::initModule_nonfree();
    feature_detector = cv::FeatureDetector::create("GridSIFT");
    descriptor_extractor = cv::DescriptorExtractor::create("SIFT");

    vector<cv::KeyPoint> kp1, kp2;             //关键点
    feature_detector->detect(rgb_data_1, kp1); //提取关键点
    feature_detector->detect(rgb_data_2, kp2);

    cout << "Key points of two images: " << kp1.size() << ", " << kp2.size() << endl;

    // 可视化， 显示关键点
    cv::Mat img_show;
    cv::drawKeypoints(rgb_data_1, kp1, img_show, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("keypoints", img_show);
    cv::imwrite("../data/keypoints.png", img_show);
    cv::waitKey(0); //暂停等待一个按键

    // 计算描述子
    cv::Mat descriptor1, descriptor2;
    descriptor_extractor->compute(rgb_data_1, kp1, descriptor1);
    descriptor_extractor->compute(rgb_data_2, kp2, descriptor2);

    // 匹配描述子
    vector<cv::DMatch> matches;
    cv::FlannBasedMatcher matcher;
    matcher.match(descriptor1, descriptor2, matches);
    cout << "Find total " << matches.size() << " matches." << endl;

    // 可视化：显示匹配的特征
    cv::Mat img_matches;
    cv::drawMatches(rgb_data_1, kp1, rgb_data_2, kp2, matches, img_matches);
    cv::imshow("matches", img_matches);
    cv::imwrite("./data/matches.png", img_matches);
    cv::waitKey(0);

    // 筛选匹配，把距离太大的去掉
    // 这里使用的准则是去掉大于四倍最小距离的匹配
    vector<cv::DMatch> good_matches;
    double minDis = 9999;
    for (size_t i = 0; i < matches.size(); i++)
    {
        if (matches[i].distance < minDis)
            minDis = matches[i].distance;
    }

    for (size_t i = 0; i < matches.size(); i++)
    {
        if (matches[i].distance < 4 * minDis)
            good_matches.push_back(matches[i]);
    }

    // 显示 good matches
    cout << "good matches=" << good_matches.size() << endl;
    cv::drawMatches(rgb_data_1, kp1, rgb_data_2, kp2, good_matches, img_matches);
    cv::imshow("good matches", img_matches);
    cv::imwrite("../data/good_matches.png", img_matches);
    cv::waitKey(0);

    // 计算图像间的运动关系
    // 关键函数：cv::solvePnPRansac()
    // 为调用此函数准备必要的参数

    // 第一个帧的三维点
    vector<cv::Point3f> space_points;
    // 第二个帧的图像点
    vector<cv::Point2f> image_points;

    SlamTransform ST;
    for (size_t i = 0; i < good_matches.size(); i++)
    {
        // query 是第一个, train 是第二个
        cv::Point2f point_2d = kp1[good_matches[i].queryIdx].pt;
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort d = depth_data_1.ptr<ushort>(int(point_2d.y))[int(point_2d.x)];
        if (d == 0)
            continue;
        image_points.push_back(cv::Point2f(kp2[good_matches[i].trainIdx].pt));

        // 将(u,v,d)转成(x,y,z)
        cv::Point3f point_3d(point_2d.x, point_2d.y, d);
        ST.Point2dTo3d(point_3d);
        space_points.push_back(point_3d);

        
    }
    cv::Mat camera_matrix;
    ST.GetCameraParam(camera_matrix);
    cout<<"camera_matrix"<<camera_matrix<<endl;
    // 构建相机矩阵
    cv::Mat rotation_vec(3,3,CV_64F),tanslation_vec(3,1,CV_64F), inliers(3,1,CV_64F);
    // 求解pnp
    cv::solvePnPRansac(space_points, image_points, camera_matrix, cv::Mat(), rotation_vec,tanslation_vec, false, 100, 1, 100, inliers);

    cout << "inliers: " << inliers.rows << endl;
    cout << "R=" << rotation_vec << endl;
    cout << "t=" <<tanslation_vec << endl;

    // 画出inliers匹配
    vector<cv::DMatch> matches_show;
    for (size_t i = 0; i < inliers.rows; i++)
    {
        matches_show.push_back(good_matches[inliers.ptr<int>(i)[0]]);
    }
    cv::drawMatches(rgb_data_1, kp1, rgb_data_2, kp2, matches_show, img_matches);
    cv::imshow("inlier matches", img_matches);
    cv::imwrite("../data/inliers.png", img_matches);
    cv::waitKey(0);

    return 0;
}
