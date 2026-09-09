#include <iostream>
#include <opencv2/opencv.hpp>
 
#include "yolov8_seg_onnx.h"
#include "yolov8_seg_utils.h"
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

 
using namespace std;
using namespace cv;

bool is_debug_ = true;


void rotatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, Eigen::Matrix4f transform){
    pcl::transformPointCloud(*inputCloud, *inputCloud, transform);
};


//只检测白色
bool findBoundingBoxes(const cv::Mat binaryImage, cv::Rect& bbox) {

    cv::bitwise_not(binaryImage, binaryImage);
    // 3. 查找轮廓（只检测外部轮廓）
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binaryImage, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 3. 找到面积最大的轮廓
    double max_area = 0;
    int max_idx = -1;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_idx = i;
        }
    }

    if (max_idx != -1) {
        bbox = cv::boundingRect(contours[max_idx]); // 轴对齐矩形

        if(is_debug_){
            // 在原始图像上绘制有效的轮廓
            cv::Mat outputImage = cv::Mat::zeros(binaryImage.size(), CV_8UC3);
            cv::rectangle(outputImage, bbox, cv::Scalar(0, 255, 0), 2);
            cv::imshow("outputImage", outputImage);
            cv::waitKey(0);
        }
        return true;
    }else{
        return false;
    }

}


bool filterOrderedPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
                            cv::Rect &bbox, float minDepth, float maxDepth) {

    const int width = 640;  // 图像宽度
    const int height = 400; // 图像高度
    cv::Mat depthImage = cv::Mat::zeros(height, width, CV_8UC1);
    
    for (size_t i = 0; i < inputCloud->points.size(); ++i) {
        const pcl::PointXYZ& point = inputCloud->points[i];

        int brightness = 0;

        if(point.z >= maxDepth || point.z <= minDepth){
            brightness = 255;
        }

        depthImage.at<uchar>(i / depthImage.cols, i % depthImage.cols) = static_cast<uchar>(brightness);
    }
    if(is_debug_){
        cv::imshow("depthImage Color Image", depthImage);
        cv::waitKey(); // 等待
    }

    return findBoundingBoxes(depthImage, bbox);

}


void extractMaskedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud, 
    const cv::Mat& mask, const cv::Rect& roi, const cv::Mat img){

    for (int y = 0; y <  roi.height ; ++y) {
        for (int x = 0; x <  roi.width ; ++x) {
            if (mask.at<uchar>(y , x) > 0) {
                // img.at<cv::Vec3b>(y + roi.y, x + roi.x) = cv::Vec3b(0, 255, 0);
                int idx = (y + roi.y) * img.cols + (x  + roi.x);
                outputCloud->points.push_back(input_cloud->points[idx]);
            }
        }
    }

    // pcl::io::savePLYFile("croped_cloud.ply", *output);

    outputCloud->width = outputCloud->points.size();
    outputCloud->height = 1;
}

#include <pcl/filters/extract_indices.h>

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



 
int main()
{
    std::string model_path = "/home/zima/WORKSPACE/mazirong/02/vision_sorter/third_party/sort_detection/models/epoch90.onnx";

    bool use_gpu = false;
    std::unique_ptr<Yolov8SegOnnx> yolov8_seg = std::make_unique<Yolov8SegOnnx>(model_path, use_gpu);
 
    vector<Scalar> color;
    srand(time(0));
 
    for (int i=0; i<80; i++){
        int b = rand() % 256;
        int g = rand() % 256;
        int r = rand() % 256;
        color.push_back(Scalar(b,g,r));
    }

    std::string img_path = "/home/zima/WORKSPACE/mazirong/02/vision_sorter/third_party/sort_detection/data/scara/Color_2025_08_13_19_13_02_752.bmp";
    std::string cloud_path = "/home/zima/WORKSPACE/mazirong/02/vision_sorter/third_party/sort_detection/data/scara/Depth_2025_08_13_19_13_01_017.ply";


    std::cout << "path=" << img_path << std::endl;
    cv::Mat frame = cv::imread(img_path);

    // 创建一个点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 读取彩色点云
    if (pcl::io::loadPLYFile(cloud_path, *Cloud) == -1) {
        std::cout << "path=" << cloud_path << std::endl;
        PCL_ERROR("Couldn't read file colored_point_cloud.ply \n");
    }


        // 2. 定义变换矩阵 (你的矩阵)
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform << -0.982618093491, 0.043478425592, 0.180475100875, 0.0,
                 -0.040235821158, -0.998956918716, 0.021590923890, 0.0,
                 0.181225597858,  0.023970639333,  0.983342528343, 0.0,
                 0.0,             0.0,             0.0,            1.0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*Cloud, *transformed_cloud, transform);
    // pcl::io::savePLYFile("transform_cloud.ply", *transformed_cloud);

    cv::Rect bbox;
    int stats = filterOrderedPointCloud(transformed_cloud, bbox, 0.6, 1.0);

    if(stats){
        if(is_debug_){
        // cv::rectangle(frame, bbox, cv::Scalar(0, 255, 0), 2);
        cv::imshow("outputImage", frame);
        cv::waitKey();
        }
    }

        // 扩展 bbox 向四周扩展 10 像素
    bbox.x = std::max(0, bbox.x - 10); // 确保不超出左边界
    bbox.y = std::max(0, bbox.y - 10); // 确保不超出上边界
    bbox.width = std::min(frame.cols - bbox.x, bbox.width + 20); // 确保不超出右边界
    bbox.height = std::min(frame.rows - bbox.y, bbox.height + 20); // 确保不超出下边界

    cv::Mat cropped = frame(bbox);  // 关键行：截取矩形区域
    cv::imshow("cropped", cropped);
    cv::waitKey();
    // 显示/保存结果
    cv::imshow("Cropped Image", cropped);
    cv::imwrite("cropped.png", cropped);
    std::cout << "Mat size: " << cropped.rows << " x " << cropped.cols << std::endl;


    vector<OutputSeg> output;
    output.clear();
    bool find = yolov8_seg->OnnxDetect(cropped, output);
    std::cout << "path===============" << std::endl;
    

    std::sort(output.begin(), output.end());

    for (auto& seg : output) {
        seg.box = cv::Rect(seg.box.x + bbox.x, seg.box.y + bbox.y, seg.box.width, seg.box.height);
        std::cout << "seg.cof:" <<  seg.confidence << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr croped_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    extractMaskedCloud(transformed_cloud, croped_cloud, output[1].boxMask, output[1].box, frame );

    pcl::io::savePLYFile("111croped_cloud.ply", *croped_cloud);



        // 2. RANSAC拟合平面
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setInputCloud(croped_cloud);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.005);
    seg.segment(*inliers, *coefficients);

    // 3. 计算中心点
    if (!inliers->indices.empty()) {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*croped_cloud, inliers->indices, centroid);

        auto plane_cloud = extractPlanePoints(croped_cloud, inliers);
        pcl::io::savePLYFile("22222croped_cloud.ply", *plane_cloud);

        std::cout << "平面方程: " 
                  << coefficients->values[0] << "x + "
                  << coefficients->values[1] << "y + "
                  << coefficients->values[2] << "z + "
                  << coefficients->values[3] << " = 0\n"
                  << "中心点坐标: ("
                  << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ")" 
                  << std::endl;



    
            // 1. 使用 PCA 计算主方向
            pcl::PCA<pcl::PointXYZ> pca;
            pca.setInputCloud(plane_cloud);
            Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();

            // 2. 定义坐标系
            Eigen::Vector3f origin = centroid.head<3>();
            Eigen::Vector3f x_axis = eigen_vectors.col(0); // 长边方向（x轴）
            Eigen::Vector3f y_axis = eigen_vectors.col(1); // 短边方向（y轴）
            Eigen::Vector3f z_axis = x_axis.cross(y_axis); // z轴

            // 3. 检查法向量方向
            Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
            if (z_axis.dot(plane_normal) < 0) {
                z_axis = -z_axis;
            }

            // 4. 输出坐标系信息
            std::cout << "局部坐标系：" << std::endl;
            std::cout << "原点: (" << origin[0] << ", " << origin[1] << ", " << origin[2] << ")" << std::endl;
            std::cout << "x轴: (" << x_axis[0] << ", " << x_axis[1] << ", " << x_axis[2] << ")" << std::endl;
            std::cout << "y轴: (" << y_axis[0] << ", " << y_axis[1] << ", " << y_axis[2] << ")" << std::endl;
            std::cout << "z轴: (" << z_axis[0] << ", " << z_axis[1] << ", " << z_axis[2] << ")" << std::endl;

            // 5. 变换点云到新坐标系（可选）
            Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
            transform.block<3, 1>(0, 0) = x_axis;
            transform.block<3, 1>(0, 1) = y_axis;
            transform.block<3, 1>(0, 2) = z_axis;
            transform.block<3, 1>(0, 3) = origin;

            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*plane_cloud, *transformed_cloud, transform.inverse());


    } else {
        std::cerr << "拟合失败！" << std::endl;
    }





    if (find){
        DrawPred(frame, output, yolov8_seg->_className, color);
    }else{
        cout << "not find!\n";        
    }

    imshow("result", frame);
    waitKey(0);


 
    return 0;
}