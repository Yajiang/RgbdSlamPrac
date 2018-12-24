#include <slam_transform.h>
#include <slam_parameters.h>

SlamTransform::SlamTransform()
{
    InitCameraParam();
}
SlamTransform::~SlamTransform()
{
}
void SlamTransform::InitCameraParam()
{
    SlamParameters slam_parameters;
    cv::Mat camera_intrinsic_matrix;
    slam_parameters.GetCameraParameters(camera_intrinsic_matrix);
    camera_factor_ = slam_parameters.GetScalingFactor();
    camera_fx_ = camera_intrinsic_matrix.at<double>(0,0);
    camera_fy_ = camera_intrinsic_matrix.at<double>(1,1);
    camera_cx_ = camera_intrinsic_matrix.at<double>(0,2);
    camera_cy_ = camera_intrinsic_matrix.at<double>(1,2);
}
void SlamTransform::Point2dTo3d(const cv::Point3f &point_3d,cv::Point3f* point_space)
{
    point_space->z = double(point_3d.z) / camera_factor_;
    point_space->x = (point_3d.x - camera_cx_) * point_space->z / camera_fx_;
    point_space->y = (point_3d.y - camera_cy_) * point_space->z / camera_fy_;
}

void SlamTransform::Point2dTo3d(cv::Mat &rgb_data, cv::Mat &depth_data, ushort m, ushort n, PointT &point)
{
    ushort d = depth_data.ptr<ushort>(m)[n];
    point.z = double(d) / camera_factor_;
    point.x = (n - camera_cx_) * point.z / camera_fx_;
    point.y = (m - camera_cy_) * point.z / camera_fy_;

    point.b = rgb_data.ptr<uchar>(m)[n * 3];
    point.g = rgb_data.ptr<uchar>(m)[n * 3 + 1];
    point.r = rgb_data.ptr<uchar>(m)[n * 3 + 2];
}
void SlamTransform::ImageToPointCloud(cv::Mat &rgb_data, cv::Mat &depth_data, PointCloud::Ptr point_cloud)
{
    for (ushort m = 0; m < depth_data.rows; m++)
        for (ushort n = 0; n < depth_data.cols; n++)
        {
            ushort d = depth_data.ptr<ushort>(m)[n];
            if (d == 0)
                continue;
            PointT point;
            point.z = double(d) / camera_factor_;
            point.x = (n - camera_cx_) * point.z / camera_factor_;
            point.y = (m - camera_cy_) * point.z / camera_factor_;

            point.b = rgb_data.ptr<uchar>(m)[n * 3];
            point.g = rgb_data.ptr<uchar>(m)[n * 3 + 1];
            point.r = rgb_data.ptr<uchar>(m)[n * 3 + 2];
            point_cloud->points.push_back(point);
            point_cloud->height = 1;
            point_cloud->width = point_cloud->points.size();
            point_cloud->is_dense = false;
        }
    // pcl::io::savePCDFile("../data/pointcloud.pcd", *point_cloud);
    // point_cloud->points.clear();
    // cout << "Point cloud saved." << endl;
}
