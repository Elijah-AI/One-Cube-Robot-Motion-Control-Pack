#include <iostream>
#include <opencv2/opencv.hpp>


#include <sys/time.h>
#include "sort_dectect.h" // 包含 sort_dectect.h 头文件
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std; 

DEFINE_string(test_case_dir,"/home/cat/workspace/software/111/vision_sorter828/third_party/sort_detection_npu/data/scara/01","test case path");
DEFINE_string(test_save_dir,"/media/quicktron/D0BA51FDBA51E090/PROJECT/BarCode/Testing/field_cases/3test/","test save path");

static double getCurTimeUS() {
    int64_t cur_time = 0;
    struct timespec spec;

    clock_gettime(CLOCK_MONOTONIC, &spec);
    cur_time = (int64_t)spec.tv_sec * 1000000 + spec.tv_nsec / 1000;
    return cur_time;
}

// 在点云中查找与目标点(x, y, z)最近的点，返回其索引
int findNearestPointIndex(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    float target_x, 
    float target_y, 
    float target_z)
{
    // 检查点云是否为空
    if (cloud->empty()) {
        std::cerr << "Error: Point cloud is empty!" << std::endl;
        return -1;
    }

    // 创建KdTree对象
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // 定义目标点
    pcl::PointXYZ target_point;
    target_point.x = target_x;
    target_point.y = target_y;
    target_point.z = target_z;

    // 存储最近邻搜索结果：索引和距离平方
    std::vector<int> nearest_indices(1);  // 存储最近点的索引
    std::vector<float> nearest_distances(1);  // 存储最近点的距离平方

    // 执行最近邻搜索（搜索1个最近点）
    int result = kdtree.nearestKSearch(target_point, 1, nearest_indices, nearest_distances);

    if (result > 0) {
        // 输出结果（距离平方开根号即为实际距离）
        std::cout << "找到最近点：" << std::endl;
        std::cout << "索引：" << nearest_indices[0] << std::endl;
        std::cout << "坐标：(" 
                  << cloud->points[nearest_indices[0]].x << ", " 
                  << cloud->points[nearest_indices[0]].y << ", " 
                  << cloud->points[nearest_indices[0]].z << ")" << std::endl;
        std::cout << "距离：" << sqrt(nearest_distances[0]) << std::endl;
        return nearest_indices[0];  // 返回最近点的索引
    } else {
        std::cerr << "未找到最近点！" << std::endl;
        return -1;
    }
}



int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(""); 
    FLAGS_logtostderr = 1;
    FLAGS_minloglevel = 1; 

    std::vector<cv::String> file_name_all;
    std::string input_dir = FLAGS_test_case_dir;

    cv::glob(input_dir, file_name_all, true);
    LOG(WARNING)<<"test case dir: "<<input_dir;
    LOG(WARNING)<<"test save dir: "<<FLAGS_test_save_dir;


    for (size_t i = 0u; i < file_name_all.size(); i++) {
        std::string file_name = file_name_all[i];
        if(file_name.substr(file_name.length() - 4, file_name.length()) != ".bmp"){
            continue;
        }

        std::string::size_type iPos = file_name.find_last_of('/') + 1;
        std::string img_name = file_name.substr(iPos, file_name.length() - iPos); 
        cv::Mat frame = cv::imread(file_name);
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        // cv::resize(frame, frame, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);

        LOG(WARNING)<< "image doing: " << img_name;
        std::string cloud_name = file_name.substr(0, file_name.length() - 4) + ".ply";
        LOG(WARNING)<< "cloud_name doing: " << cloud_name;
        
        // 创建一个点云指针
        pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // 读取彩色点云
        if (pcl::io::loadPLYFile(cloud_name, *Cloud) == -1)
        {
            std::cout << "path=" << cloud_name << std::endl;
            PCL_ERROR("Couldn't read file colored_point_cloud.ply \n");
            continue;
        }

        DecodeInfo decode_info;
        ParaConfig para;
        while(1){
            cv::Mat frame_copy = frame.clone();
            double start = getCurTimeUS();
            slotDetection(frame_copy, Cloud, &para, &decode_info);
            LOG(WARNING)<< "time: " << (getCurTimeUS() - start) * 1e-3;
        }



        //   // 创建一个新的彩色点云
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // // 设置彩色点云的大小
        // coloredCloud->width = Cloud->points.size() +1;
        // coloredCloud->height = 1;
        // coloredCloud->is_dense = Cloud->is_dense;
        // coloredCloud->points.resize(Cloud->points.size() +1);
        // LOG(ERROR) << "points.size(): " << coloredCloud->points.size();
        // // 将点云和颜色结合
        // for (size_t i = 0; i < Cloud->points.size() -1 ; ++i)
        // {
        //     const pcl::PointXYZ &point = Cloud->points[i];
        //     // 将点和颜色赋值到新的点云中
        //     coloredCloud->points[i].x = point.x;
        //     coloredCloud->points[i].y = point.y;
        //     coloredCloud->points[i].z = point.z;
        //     coloredCloud->points[i].r = 0; // 红色通道
        //     coloredCloud->points[i].g = 255; // 绿色通道
        //     coloredCloud->points[i].b = 0; // 蓝色通道
        // }
        // int idx = coloredCloud->points.size()-1;
        // coloredCloud->points[idx].x = decode_info.result_before_calib[0];
        // coloredCloud->points[idx].y = decode_info.result_before_calib[1];
        // coloredCloud->points[idx].z = decode_info.result_before_calib[2];
        // coloredCloud->points[idx].r = 255; // 红色通道
        // coloredCloud->points[idx].g = 0; // 绿色通道
        // coloredCloud->points[idx].b = 0; // 蓝色通道
        // // 保存为 PCD 文件
        // pcl::io::savePLYFile("colored_point_cloud.ply", *coloredCloud);

        int nearest_index = findNearestPointIndex(Cloud, decode_info.result_before_calib[0][0], decode_info.result_before_calib[0][1], decode_info.result_before_calib[0][2]);

        cv::Point center(
            nearest_index % frame.cols,
            nearest_index / frame.cols);
        cv::circle(frame, center, 5, cv::Scalar(255, 0, 0), -1);

        LOG(ERROR) << frame.cols << ", " <<  frame.rows;


        // cv::imshow("result", frame);
        cv::imwrite("result.png", frame);
        // cv::waitKey(0);

    }


    return 0;
}