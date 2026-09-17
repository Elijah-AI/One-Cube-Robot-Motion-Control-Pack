#include <iostream>
#include <opencv2/opencv.hpp>

#include <sys/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/pca.h>

using namespace std;
using namespace cv;

int main()
{
    std::string cloud_path = "/home/cat/workspace/software/111/vision_sorter1128_zt/third_party/sort_detection_npu/data/1116test/Depth_2025_12_11_16_32_46_851.ply";

    // 创建一个点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 读取彩色点云
    if (pcl::io::loadPLYFile(cloud_path, *Cloud) == -1)
    {
        std::cout << "path=" << cloud_path << std::endl;
        PCL_ERROR("Couldn't read file colored_point_cloud.ply \n");
    }

    // 2. 定义变换矩阵 (你的矩阵)
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    // transform << -0.982618093491, 0.043478425592, 0.180475100875, 0.0,
    //              -0.040235821158, -0.998956918716, 0.021590923890, 0.0,
    //              0.181225597858,  0.023970639333,  0.983342528343, 0.0,
    //              0.0,             0.0,             0.0,            1.0;

    transform << 0.975356, 0.197892, -0.0975652, 0.761739,
        0.203906, -0.639572, 0.741195, -1.09914,
        0.0842767, -0.742823, -0.664162, 0.323307,
        0, 0, 0, 1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*Cloud, *transformed_cloud, transform);

    Eigen::Vector3f point, transformed_point;
    point[0] = -0.042052957;
    point[1] = -0.153420153;
    point[2] = 1.113580241;

    pcl::io::savePLYFile("scara_transformed_cloud.ply", *transformed_cloud);

    // 应用变换
    // Eigen::Vector3f transformed_point = pcl::transformPoint(point, transform);
    transformed_point[0] = (transform(0, 0) * point[0] + transform(0, 1) * point[1] + transform(0, 2) * point[2] + transform(0, 3)) * 1e3;
    transformed_point[1] = (transform(1, 0) * point[0] + transform(1, 1) * point[1] + transform(1, 2) * point[2] + transform(1, 3)) * 1e3;
    transformed_point[2] = (transform(2, 0) * point[0] + transform(2, 1) * point[1] + transform(2, 2) * point[2] + transform(2, 3)) * 1e3;

    // 输出结果
    cout << "原始点: (" << point[0] << ", " << point[1] << ", " << point[2] << ")" << endl;
    cout << "变换后: (" << transformed_point[0] << ", "
         << transformed_point[1] << ", "
         << transformed_point[2] << ")" << endl;

    return 0;
}