#include <slam_transform.h>

SlamTransform::SlamTransform()
{
    SetCameraParam(1000,518,519,325.5, 253.5);
}
SlamTransform::~SlamTransform()
{
}
void SlamTransform::SetCameraParam(double factor, double fx, double fy, double cx, double cy)
{
    camera_factor_ = factor;
    camera_fx_ = fx;
    camera_fy_ = fy;
    camera_cx_ = cx;
    camera_cy_ = cy;
}
// void SlamTransform::GetCameraParam(double &factor, double &fx, double &fy, double &cx, double &cy)
// {
//     factor = camera_factor_;
//     fx = camera_fx_;
//     fy = camera_fy_;
//     cx = camera_c
//     cy = camera_cy_;
// }
void SlamTransform::GetCameraParam(cv::Mat &camera_matrix)
{
    double c[3][3] = {{camera_fx_, 0, camera_cx_}, {0, camera_fy_, camera_cy_}, {0, 0, 1}};
    cv::Mat temp_matrix(3, 3, CV_64F, c);
    cout << temp_matrix << endl;
    camera_matrix = temp_matrix.clone();
}
void SlamTransform::Point2dTo3d(cv::Point3f &point)
{
    cv::Point3f temp_point(point.x,point.y,point.z);
    point.z = double(temp_point.z) / camera_factor_;
    point.x = (temp_point.x - camera_cx_) * point.z / camera_fx_;
    point.y = (temp_point.y - camera_cy_) * point.z / camera_fy_;
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
