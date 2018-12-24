#include<iostream>
#include<slam_estimate.h>
#include<slam_parameters.h>
#include<slam_transform.h>

using namespace std;

#include <opencv2/core/eigen.hpp>

#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

// Eigen !
#include <Eigen/Core>
#include <Eigen/Geometry>
int main( int argc, char** argv )
{

    //本节要拼合data中的两对图像
    // 声明两个帧，FRAME结构请见include/slamBase.h
    Frame frame1, frame2;
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
    // 处理result
    // 将旋转向量转化为旋转矩阵
    cv::Mat rotation_matrix;
    cv::Rodrigues(pnp_result.rotation_vector, rotation_matrix);
    Eigen::Matrix3d rotation_matrix_eigen;
    cv::cv2eigen(rotation_matrix, rotation_matrix_eigen);
  
    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d transform_matrix = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd rotation(rotation_matrix_eigen);
    cout<<"translation"<<endl;
    Eigen::Translation<double,3> translation(pnp_result.translation_vector.at<double>(0,0), pnp_result.translation_vector.at<double>(0,1), pnp_result.translation_vector.at<double>(0,2));
    transform = rotation;
    transform(0,3) = pnp_result.translation_vector.at<double>(0,0); 
    transform(1,3) = pnp_result.translation_vector.at<double>(0,1);
    transform(2,3) = pnp_result.translation_vector.at<double>(0,2);

    // 转换点云
    cout<<"converting image to clouds"<<endl;
    PointCloud::Ptr cloud1 = slam_transform.ImageToPointCloud(frame1.rgb_data,frame1.depth_data);
    PointCloud::Ptr cloud2 = slam_transform.ImageToPointCloud(frame2.rgb_data,frame2.depth_data);

    // 合并点云
    cout<<"combining clouds"<<endl;
    PointCloud::Ptr output (new PointCloud());
    pcl::transformPointCloud(*cloud1, *output, transform.matrix());
    *output += *cloud2;
    pcl::io::savePCDFile("../data/result.pcd", *output);
    cout<<"Final result saved."<<endl;

    pcl::visualization::CloudViewer viewer( "viewer" );
    viewer.showCloud( output );
    while( !viewer.wasStopped() )
    {
    }
    return 0;
}