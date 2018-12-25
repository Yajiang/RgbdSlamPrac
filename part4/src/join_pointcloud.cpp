#include "join_point_cloud.h"

using namespace std;

JoinPointcloud::CvMat2Eigen(const cv::Mat& rotation_vector,const cv::Mat& translation_vector,Eigen::Isometry3d* transform)
{
    cv::Mat rotation_matrix;
    cv::Rodrigues(rotation_vector, rotation_matrix);
    Eigen::Matrix3d rotation_matrix_eigen;
    cv::cv2eigen(rotation_matrix, rotation_matrix_eigen);
    Eigen::AngleAxisd rotation(rotation_matrix_eigen);
    *transform = rotation;
    *transform(0,3) = translation_vector.at<double>(0,0); 
    *transform(1,3) = translation_vector.at<double>(1,0);
    *transform(2,3) = translation_vector.at<double>(2,0);
    cout<<*transform.matrix()<<endl;
}
JoinPointcloud::CombinePointcloud(Frame& frame1,Frame& frame2)
{    
    //本节要拼合data中的两对图像
    // 声明两个帧，FRAME结构请见include/slamBase.h
    //读取图像
    frame1.rgb_data = cv::imread( "../data/rgb1.png");
    frame1.depth_data = cv::imread( "../data/depth1.png", -1);
    frame2.rgb_data = cv::imread( "../data/rgb2.png");
    frame2.depth_data = cv::imread( "../data/depth2.png", -1 );
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
    // 转换点云
    SlamTransform slam_transform;
    cout<<"converting image to clouds"<<endl;
    PointCloud::Ptr cloud1(new PointCloud());
    slam_transform.ImageToPointCloud(frame1.rgb_data,frame1.depth_data,cloud1);
    PointCloud::Ptr cloud2(new PointCloud());
    slam_transform.ImageToPointCloud(frame2.rgb_data,frame2.depth_data,cloud2);
    // 合并点云
    cout<<"combining clouds"<<endl;
    PointCloud::Ptr output (new PointCloud());
    pcl::transformPointCloud(*cloud1, *output, transform.matrix());
    *output += *cloud2;
    pcl::io::savePCDFile("../data/result.pcd", *output)
    cout<<"Final result saved."<<endl;
}
JoinPointcloud::ReadFrame()
{
    
}