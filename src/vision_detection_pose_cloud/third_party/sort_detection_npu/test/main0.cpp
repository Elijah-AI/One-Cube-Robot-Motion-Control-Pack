#include <iostream>
#include <opencv2/opencv.hpp>
#include "object_region_detection.h"

#include <sys/time.h>
#include <glog/logging.h>
#include <gflags/gflags.h>
 
using namespace std;
using namespace cv;

pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleOrganizedCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
    // 获取原始尺寸
    int original_width = 1280;
    int original_height = 800;

    // 计算目标尺寸（向下取整）
    int new_width = original_width / 2;
    int new_height = original_height / 2;

    // 创建输出点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    output_cloud->width = new_width;
    output_cloud->height = new_height;
    output_cloud->is_dense = input_cloud->is_dense; // 注意：下采样后可能不再 dense
    output_cloud->points.resize(new_width * new_height);
    LOG(ERROR) << "points.size(): " << input_cloud->points.size();

    // 遍历新点云的每个位置
    for (int j = 0; j < new_height; j++)
    { // 行 (y方向)
        for (int i = 0; i < new_width; i++)
        { // 列 (x方向)
            int original_index = (2 * j) * original_width + (2 * i);

            // 目标点云中的索引：顺序存放
            output_cloud->points[j * new_width + i] = input_cloud->points[original_index];
        }
    }
    LOG(ERROR) << "points.size(): " << input_cloud->points.size();
    return output_cloud;
}

 
int main(int argc, char** argv) 
{
    std::unique_ptr<ObjectRegionDetection> ObjectRegionDetector = std::make_unique<ObjectRegionDetection>();

    google::InitGoogleLogging("");


    std::string folder_path = "/home/neardi/ros2_ws/src/vision_detection_pose_cloud/third_party/sort_detection_npu/data/yq/01/red_block_cam0_106_606903000.bmp";
    std::string output_path = "/home/neardi/ros2_ws/src/vision_detection_pose_cloud/third_party/sort_detection_npu/data/yq_result/01";

    std::vector<cv::String> paths;
    cv::glob(folder_path, paths, false);
    

    for (const auto& path : paths) {
        std::cout << "path=" << path << std::endl;
        cv::Mat frame = cv::imread(path);
        if (frame.empty()) {
            std::cout << "错误: 无法读取图像，图像为空!" << std::endl;
            continue;; // 或者处理错误
        }
        std::string::size_type iPos = path.find_last_of('/') + 1;
        std::string img_name = path.substr(iPos, path.length() - iPos); 
        std::cout << "img_name=" << img_name << std::endl;

        std::string cloud_path = path.substr(0, path.length() - 4) + ".ply";

        // 创建一个点云指针
        pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 读取彩色点云
        if (pcl::io::loadPLYFile(cloud_path, *Cloud) == -1)
        {
            std::cout << "path=" << cloud_path << std::endl;
            PCL_ERROR("Couldn't read file colored_point_cloud.ply \n");
        }

        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_resize(new pcl::PointCloud<pcl::PointXYZ>);
        // cloud_resize = downsampleOrganizedCloud(Cloud);
        // resize(frame, frame, cv::Size(640, 400));


 
        LocalizationInfo localization_info;

        int localization_success = ObjectRegionDetector->DetectRegion(frame, Cloud, &localization_info);
        LOG(ERROR) << "localization_success: " << localization_success;
 
        // imshow("result", frame);
        // waitKey(0);
        std::string write_path = output_path + img_name.substr(0, img_name.length() - 4) + ".bmp";
        std::cout << "write_path=" << write_path << std::endl;
        cv::imwrite(write_path, frame);
    }

 
    return 0;
}