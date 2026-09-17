#include <pcl/filters/extract_indices.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <iostream>
#include<memory>
#include <opencv2/opencv.hpp>


#include <pcl/segmentation/extract_clusters.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr removeDistantPointsByClustering(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    // 创建KD树用于聚类
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // 执行欧几里得聚类
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.01); // 2厘米
    ec.setMinClusterSize(10);     // 最小聚类点数
    ec.setMaxClusterSize(25000);  // 最大聚类点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // 找到最大的聚类（主点云）
    size_t max_cluster_size = 0;
    int main_cluster_index = -1;

    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {
        if (cluster_indices[i].indices.size() > max_cluster_size)
        {
            max_cluster_size = cluster_indices[i].indices.size();
            main_cluster_index = i;
        }
    }

    // 只保留主聚类
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (main_cluster_index >= 0)
    {
        for (const auto &idx : cluster_indices[main_cluster_index].indices)
        {
            filtered_cloud->points.push_back(cloud->points[idx]);
        }
    }

    return filtered_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr extractPlanePoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const pcl::PointIndices::Ptr& inliers) 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false); // false表示提取内点，true表示提取外点
    extract.filter(*plane_cloud);
    return plane_cloud;
}


#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>


double computeAngleWithZAxis(const pcl::ModelCoefficients::Ptr& coefficients)
{
    if (coefficients->values.size() < 3)
        return 0.0;
    
    // 平面法向量 (A, B, C)
    Eigen::Vector3f plane_normal(coefficients->values[0], 
                                coefficients->values[1], 
                                coefficients->values[2]);
    plane_normal.normalize();
    
    // Z轴单位向量
    Eigen::Vector3f z_axis(0.0f, 0.0f, 1.0f);
    
    // 计算夹角（点积公式）
    double dot_product = plane_normal.dot(z_axis);
    double angle_rad = std::acos(std::abs(dot_product)); // 取绝对值得到锐角
    
    return angle_rad;
}


bool detectAndExtractTopPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud, Eigen::Vector4f& plane_center) {
// 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.005);  // 调整此阈值以适应您的数据

    // 创建提取对象
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>(*filtered_cloud));
    std::vector<Eigen::Vector4f> plane_centers;
    int i = 0, nr_points = filtered_cloud->size();


    // 循环提取所有平面
    while (cloud_f->size() > 0.1 * nr_points)  // 当剩余点云大于原始点云的10%时继续
    {
        // 分割最大的平面组件
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        
        seg.setInputCloud(cloud_f);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // 提取平面内点
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud(cloud_f);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*plane_cloud);

        // std::string name = std::to_string(i) + "cloud.ply";
        // pcl::io::savePLYFile(name, *plane_cloud);

        // 计算平面中心点
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*plane_cloud, centroid);

        double angle_deg = computeAngleWithZAxis(coefficients) * 180.0 / M_PI ;
        centroid[3] = angle_deg;
        // std::cout << "  - Angle with Z-axis: " << angle_deg << std::endl;

        // std::cout << "Plane " << i+1 << " center: (" 
        //           << centroid[0] << ", " 
        //           << centroid[1] << ", " 
        //           << centroid[2] << ", " 
        //           << centroid[3] << ")" << std::endl;

        plane_centers.push_back(centroid);

        // 从点云中移除已处理的平面点
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*cloud_temp);
        cloud_f.swap(cloud_temp);
        i++;
    }

    if(!plane_centers.empty()){
        std::sort(plane_centers.begin(), plane_centers.end(), 
        [](const Eigen::Vector4f& a, const Eigen::Vector4f& b) {
            return a[2] > b[2];  // 按Z坐标降序
        });

        plane_center = plane_centers[0];

        if(plane_centers.size() >1){
            for (size_t j = 1; j < plane_centers.size(); ++j) {
                auto centroid = plane_centers[j];
                //最上层平面角度比下一层平面角度大15度，则舍弃
                if(plane_center[3] - centroid[3] > 15){
                    plane_center = centroid;
                    //
                    std::cout << " 角度问题 center: ("  << centroid[0] << ", " 
                            << centroid[1] << ", " 
                            << centroid[2] << ", " 
                            << centroid[3] << ")" << std::endl;
                    std::cout << " 角度问题 center: ("  << plane_center[0] << ", " 
                                        << plane_center[1] << ", " 
                                        << plane_center[2] << ", " 
                                        << plane_center[3] << ")" << std::endl;
                }

            }
        }
        return true;

    }else{
        return false;
    }


}




int main()
{

    // std::string img_path = "/home/zima/WORKSPACE/mazirong/02/vision_sorter/third_party/sort_detection/data/scara/Color_2025_08_13_19_13_02_752.bmp";
    std::string cloud_path = "/home/zima/WORKSPACE/datasets/dreame/自动供包/点云/问题1/边缘未取到/01/111croped_cloud.ply";


    // std::cout << "path=" << img_path << std::endl;
    // cv::Mat frame = cv::imread(img_path);

    // 创建一个点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr croped_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 读取彩色点云
    if (pcl::io::loadPLYFile(cloud_path, *croped_cloud) == -1) {
        std::cout << "path=" << cloud_path << std::endl;
        PCL_ERROR("Couldn't read file colored_point_cloud.ply \n");
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    filtered_cloud = removeDistantPointsByClustering(croped_cloud);

    Eigen::Vector4f plane_center;

    bool ret = detectAndExtractTopPlane(filtered_cloud, plane_center);

    std::cout << " center: ("  << plane_center[0] << ", " 
                                << plane_center[1] << ", " 
                                << plane_center[2] << ", " 
                                << plane_center[3] << ")" << std::endl;


}
