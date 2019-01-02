//相关头文件

//C/C++系统文件
#include <string>
#include <iostream>
//第三方库文件
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
//PCL
#include <pcl-1.9/pcl/filters/voxel_grid.h>
#include <pcl-1.9/pcl/io/pcd_io.h>
#include <pcl-1.9/pcl/point_types.h>
#include <pcl-1.9/pcl/common/transforms.h>
#include <pcl-1.9/pcl/visualization/cloud_viewer.h>
//g2o
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
//项目内文件
#include "slam_estimate.h"
#include "slam_parameters.h"
#include "slam_transform.h"
#include "join_pointcloud.h"

int main(int argc, char const *argv[])
{
    //读取参数
    SlamParameters slam_parameters;
    int start_index = atoi(slam_parameters.ReadData("start_index").c_str());
    int end_index = atoi(slam_parameters.ReadData("end_index").c_str());
    int min_inliers = atoi(slam_parameters.ReadData("min_inliers").c_str());
    double max_distance = atof(slam_parameters.ReadData("max_norm").c_str());
    int min_good_match = atoi(slam_parameters.ReadData("min_good_match").c_str());

    SlamEstimate slam_estimate;
    SlamTransform slam_transform;
    static JoinPointcloud join_pointcloud;

    Frame *prev_frame(new Frame());
    Frame *current_frame(new Frame());
    PointCloud::Ptr input_cloud(new PointCloud());
    PointCloud::Ptr output_cloud(new PointCloud());
    Eigen::Isometry3d *transform(new Eigen::Isometry3d());
    pcl::visualization::CloudViewer viewer("viewer");
    // g2o初始化
    typedef g2o::BlockSolver_6_3 SlamBlockSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    SlamLinearSolver *linear_solver = new SlamLinearSolver();
    linear_solver->setBlockOrdering(false);
    SlamBlockSolver *block_solver = new SlamBlockSolver(std::unique_ptr<SlamLinearSolver>(linear_solver));
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<SlamBlockSolver>(block_solver));

    g2o::SparseOptimizer global_optimizer;
    global_optimizer.setAlgorithm(solver);
    global_optimizer.setVerbose(false);

    int prev_index;
    int current_index;

    for (int i = start_index; i != end_index; i++)
    {
        if (i == start_index)
        {
            prev_index = i;
            g2o::VertexSE3 *vertex = new g2o::VertexSE3();
            vertex->setId(prev_index);
            vertex->setEstimate(Eigen::Isometry3d::Identity());
            vertex->setFixed(true);
            global_optimizer.addVertex(vertex);

            join_pointcloud.ReadFrame(i, prev_frame);
            slam_estimate.ComputeKeyPointAndDescriptor(*prev_frame);
            // slam_transform.ImageToPointCloud(prev_frame->rgb_data, prev_frame->depth_data, input_cloud);
        }
        else
        {
            current_index = i;
            join_pointcloud.ReadFrame(i, current_frame);
            slam_estimate.ComputeKeyPointAndDescriptor(*current_frame);
            PnpResult pnp_result = slam_estimate.EstimateMotion(*prev_frame, *current_frame);
            if (pnp_result.inliers < min_inliers)
            {
                continue;
            }
            if (join_pointcloud.NormDistance(pnp_result.rotation_vector, pnp_result.translation_vector) > max_distance)
            {
                continue;
            }
            g2o::VertexSE3 *vertex = new g2o::VertexSE3();
            vertex->setId(current_index);
            vertex->setEstimate(Eigen::Isometry3d::Identity());
            global_optimizer.addVertex(vertex);

            g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
            edge->vertices()[0] = global_optimizer.vertex(prev_index);
            edge->vertices()[1] = global_optimizer.vertex(current_index);

            join_pointcloud.CvMat2Eigen(pnp_result.rotation_vector, pnp_result.translation_vector, transform);

            Eigen::Matrix<double, 6, 6> information_matrix = Eigen::Matrix<double, 6, 6>::Identity();
            information_matrix(0, 0) = information_matrix(1, 1) = information_matrix(2, 2) = 10;
            information_matrix(3, 3) = information_matrix(3, 3) = information_matrix(3, 3) = 10;
            edge->setInformation(information_matrix);
            edge->setMeasurement(*transform);
            global_optimizer.addEdge(edge);

            // join_pointcloud.CombinePointcloud(input_cloud, *current_frame, *transform, output_cloud);
            swap(prev_frame, current_frame);
            prev_index = current_index;
            // swap(input_cloud, output_cloud);
            // cout << "transform" << transform->matrix() << endl;
        }

        // viewer.showCloud(input_cloud);
    }
    cout << "optimizing pose graph, vertices" << global_optimizer.vertices().size() << endl;
    global_optimizer.save("/home/eugene/slam/part5/data/result_before.g2o");
    global_optimizer.initializeOptimization();
    global_optimizer.optimize(100);
    global_optimizer.save("/home/eugene/slam/part5/data/result_after.g2o");
    cout << "Opotimization done." << endl;
    global_optimizer.clear();
    delete prev_frame;
    delete current_frame;
    delete transform;
    // pcl::io::savePCDFile("/home/eugene/slam/part4/data/output_cloud.pcd", *input_cloud);
    return 0;
}