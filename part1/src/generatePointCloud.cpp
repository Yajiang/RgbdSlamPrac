//C++ 标准库
#include<iostream>
#include<string>

using namespace std;
//opencv 库
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//PCL 库
#include <pcl-1.9/pcl/pcl_config.h>
#include <pcl-1.9/pcl/pcl_macros.h>
#include <pcl-1.9/pcl/point_types.h>
#include <pcl-1.9/pcl/io/pcd_io.h>

//Define PCL Point
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

//Camera param
const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

//Main func
int main(int argc, char const *argv[])
{
    cv::Mat rgb, depth;
    rgb = cv::imread("../data/rgb.png");
    depth = cv::imread("../data/depth.png",-1);
    PointCloud::Ptr cloud (new PointCloud);
    for (int m = 0; m < depth.rows; m++)
        for (int n = 0; n < depth.cols; n++)
        {
            ushort d = depth.ptr<ushort>(m)[n];
            if (d == 0)
                continue;
            PointT p;

            //
            p.z = double(d)/camera_factor;
            p.x = (n - camera_cx) * p.z / camera_factor;
            p.y = (m - camera_cy) * p.z / camera_factor;

            //
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3 + 1];
            p.r = rgb.ptr<uchar>(m)[n*3 + 2];

            cloud ->points.push_back(p);

        }
    cloud ->height = 1;
    cloud ->width = cloud ->points.size();
    cout <<"point cloud size = " <<cloud->points.size()<<endl;
    cloud->is_dense = false;
    pcl::io::savePCDFile("../data/pointcloud.pcd",*cloud);

    cloud->points.clear();
    cout<<"Point cloud saved."<<endl;
    return 0;
}
