#include "object_region_detection.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h> // 添加这行
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/common/eigen.h>
#include <pcl/common/pca.h>
#include <iostream>
#include <vector>

#include <glog/logging.h>
#include <gflags/gflags.h>

ObjectRegionDetection::ObjectRegionDetection()
{
    // std::string model_path = "/home/neardi/workspace/project/sort_detection_npu/models/best_3576.rknn";
    std::string model_path = "/home/neardi/ros2_ws/src/vision_detection_pose_cloud/third_party/sort_detection_npu/models/box1.rknn";

    memset(&rknn_app_ctx_, 0, sizeof(rknn_app_context_t));

    init_post_process();

    int ret = init_yolov8_seg_model(model_path.c_str(), &rknn_app_ctx_);
    if (ret != 0)
    {
        LOG(ERROR) << "init_yolov8_seg_model fail!";
    }

    is_debug_ = true;
    ply_save_dir_ = "/home/neardi/ros2_ws/src/vision_detection_pose_cloud/third_party/sort_detection_npu/data/";
    srand(time(0));

    color.push_back(cv::Scalar(255, 0, 0));
    color.push_back(cv::Scalar(0, 255, 0));

    for (int i = 0; i < 80; i++)
    {
        int b = rand() % 256;
        int g = rand() % 256;
        int r = rand() % 256;
        color.push_back(cv::Scalar(b, g, r));
    }
    is_resize_ = false;
    max_plane_angle_ = 45;
    last_best_centroid = Eigen::Vector4f::Constant(1000000000.0f);
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
}

ObjectRegionDetection::~ObjectRegionDetection()
{
}

bool convert_mat_to_image_buffer(const cv::Mat &mat, image_buffer_t &img_buf)
{
    // 1. 检查输入Mat的有效性
    if (mat.empty())
    {
        return false;
    }

    // 2. 设置基础信息
    img_buf.width = mat.cols;
    img_buf.height = mat.rows;
    img_buf.width_stride = mat.step; // OpenCV的step就是width_stride
    img_buf.height_stride = mat.rows;
    img_buf.format = IMAGE_FORMAT_RGB888;
    img_buf.fd = -1; // 不使用DMABUF

    // 3. 计算数据大小并分配内存
    img_buf.size = mat.rows * mat.step; // 或者 mat.total() * mat.elemSize()
    img_buf.virt_addr = new unsigned char[img_buf.size];

    if (!img_buf.virt_addr)
    {
        return false; // 内存分配失败
    }

    // 4. 拷贝数据
    memcpy(img_buf.virt_addr, mat.data, img_buf.size);

    return true;
}





void ObjectRegionDetection::extractMaskedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                                               const cv::Mat &mask, const cv::Rect &roi, const cv::Mat img)
{
    output_cloud->clear();
    output_cloud->width = roi.width;
    output_cloud->height = roi.height;
    output_cloud->is_dense = false;

    LOG(ERROR) << "roi.width: " << roi.width << ", roi.height: " << roi.height;

    if (!is_resize_)
    {
        for (int y = 0; y < roi.height; ++y)
        {
            for (int x = 0; x < roi.width; ++x)
            {
                if (mask.at<uchar>(y, x) > 0)
                {
                    int idx = (y + roi.y) * img.cols + (x + roi.x);

                    output_cloud->points.push_back(input_cloud->points[idx]);
                }
                else
                {
                    pcl::PointXYZ nan_point;
                    nan_point.x = 0;
                    nan_point.y = 0;
                    nan_point.z = 0;
                    output_cloud->points.push_back(nan_point);
                }
            }
        }
    }
    else
    {
        for (int y = 0; y < roi.height; y += 2)
        {
            for (int x = 0; x < roi.width; x += 2)
            {
                if (mask.at<uchar>(y, x) > 0)
                {
                    int idx = (y + roi.y) * img.cols + (x + roi.x);

                    output_cloud->points.push_back(input_cloud->points[idx]);
                }
                else
                {
                    pcl::PointXYZ nan_point;
                    nan_point.x = 0;
                    nan_point.y = 0;
                    nan_point.z = 0;
                    output_cloud->points.push_back(nan_point);
                }
            }
        }
    }
}





double computeAngleWithZAxis(const pcl::ModelCoefficients &coefficients)
{
    if (coefficients.values.size() < 3)
        return 0.0;

    // 平面法向量 (A, B, C)
    Eigen::Vector3f plane_normal(coefficients.values[0],
                                 coefficients.values[1],
                                 coefficients.values[2]);
    plane_normal.normalize();

    // Z轴单位向量
    Eigen::Vector3f z_axis(0.0f, 0.0f, 1.0f);

    // 计算夹角（点积公式）
    double dot_product = plane_normal.dot(z_axis);
    double angle_rad = std::acos(std::abs(dot_product)); // 取绝对值得到锐角

    return angle_rad;
}



template <typename PointT>
std::vector<pcl::PointXYZ> computePlaneCentroids(
    const typename pcl::PointCloud<PointT>::Ptr &cloud,
    const std::vector<pcl::PointIndices> &inlier_indices)
{
    std::vector<pcl::PointXYZ> centroids;

    for (const auto &indices : inlier_indices)
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, indices.indices, centroid);
        centroids.emplace_back(centroid[0], centroid[1], centroid[2]);
    }

    return centroids;
}

#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr removeInvalidPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double &area,
    bool remove_nan = true,
    bool remove_zero = true,
    float zero_threshold = 1e-6f)
{
    if (!cloud || cloud->empty())
    {
        LOG(ERROR) << "Input cloud is empty or null!";
        return nullptr;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filtered_cloud->reserve(cloud->size());

    int nan_count = 0;
    int zero_count = 0;

    for (const auto &point : *cloud)
    {
        bool is_valid = true;

        // 检查NaN值
        if (remove_nan && !pcl::isFinite(point))
        {
            is_valid = false;
            nan_count++;
        }

        // 检查零点
        if (remove_zero && is_valid)
        {
            if (std::abs(point.x) < zero_threshold &&
                std::abs(point.y) < zero_threshold &&
                std::abs(point.z) < zero_threshold)
            {
                is_valid = false;
                zero_count++;
            }
        }

        if (is_valid)
        {
            pcl::PointXYZ p2 = point; // 创建新点
            p2.z = 0;
            filtered_cloud->push_back(point);
        }
    }
    // 凸包计算面积
    //  filtered_cloud->width = filtered_cloud->size();
    //  filtered_cloud->height = 1;
    //  filtered_cloud->is_dense = true;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
    // std::vector<pcl::Vertices> polygons;

    // pcl::ConvexHull<pcl::PointXYZ> chull;
    // chull.setInputCloud(filtered_cloud);
    // chull.setComputeAreaVolume(true);
    // chull.reconstruct(*hull, polygons);

    // area = chull.getTotalArea();

    // LOG(ERROR) << ", 清理无效点: " << cloud->size() << " -> " << filtered_cloud->size()
    //         << " 点, 移除 " << nan_count << " 个NaN点, " << zero_count << " 个零点";

    return filtered_cloud;
}

bool ObjectRegionDetection::multiPlaneSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int idx, Eigen::Vector4f &plane_center)
{
    // if (!cloud->isOrganized()) {
    //     LOG(ERROR) << "点云不是有序的！无法使用 OrganizedMultiPlaneSegmentation";
    //     LOG(ERROR) << "点云尺寸: " << cloud->width << "x" << cloud->height;
    //     LOG(ERROR) << "点云总数: " << cloud->size();
    //     return false;
    // }

    double density = 0;
    pcl::PointXYZ min_pt, max_pt;
    double area;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = removeInvalidPoints(cloud, area);
    pcl::getMinMax3D(*filtered_cloud, min_pt, max_pt);

    float z_min = min_pt.z;
    float z_max = max_pt.z;
    // float area = min_pt.x * min_pt.y;
    // LOG(ERROR) << "z_min: " << z_min << ", z_max: " << z_max ;

    // 法线估计
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    // 使用积分图像法线估计（有序点云专用）
    //     pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    //     // ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    //     ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    //     ne.setMaxDepthChangeFactor(0.02f);
    //     ne.setNormalSmoothingSize(10.0f);
    //     ne.setInputCloud(cloud);
    //     ne.compute(*normals);

    // 1. 设置法线估计参数
    float normal_estimation_radius = 0.03; // 法线估计的搜索半径
    // 2. 设置多平面分割参数
    int min_inliers = 300;                 // 最小内点数
    float max_curvature = 0.1;             // 最大曲率阈值
    float angular_threshold_degrees = 5.0; // 角度阈值（度）
    float distance_threshold = 0.02;       // 点到平面距离阈值

    int cloud_size = filtered_cloud->size();
    if (cloud_size > 6000)
    {
        min_inliers = 1000;
        max_curvature = 0.015;
    }
    else if (cloud_size > 5000)
    {
        min_inliers = 700;
        max_curvature = 0.02;
    }
    else if (cloud_size > 4000)
    {
        min_inliers = 500;
        max_curvature = 0.03;
    }
    else if (cloud_size > 3000)
    {
        min_inliers = 400;
        max_curvature = 0.05;
    }
    else if (cloud_size > 2000)
    {
        min_inliers = 300;
        max_curvature = 0.06;
        distance_threshold = 0.03;
    }
    else if (cloud_size > 1000)
    {
        min_inliers = 300;
        max_curvature = 0.08;
    }
    else if (cloud_size > 500)
    {
        min_inliers = 150;
        max_curvature = 0.1;
        distance_threshold = 0.03;
    }
    else
    {
        min_inliers = 100;
        max_curvature = 0.1;
        distance_threshold = 0.02;
    }

    LOG(ERROR) << "idx:" << idx << ", cloud_size: " << cloud_size << ", z_max: " << z_max << ", min_inliers: " << min_inliers << ", max_curvature: " << max_curvature << ", max_curvature: " << distance_threshold;

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(normal_estimation_radius); // 搜索半径
    ne.compute(*normals);
    // 3. 创建多平面分割对象并设置参数
    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers(min_inliers);
    mps.setMaximumCurvature(max_curvature); // 曲率值
    mps.setAngularThreshold(0.017453 * angular_threshold_degrees);
    mps.setDistanceThreshold(distance_threshold); // 点到平面距离
    // 4. 提供输入并执行分割
    mps.setInputCloud(cloud);
    mps.setInputNormals(normals);
    // 创建容器来接收结果
    std::vector<pcl::PlanarRegion<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZ>>> regions;
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;
    pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;

    // 执行分割和提炼
    mps.segmentAndRefine(regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

    LOG(ERROR) << "Number of planes found: " << regions.size();
    if (regions.size() == 0)
    {
        return false;
    }

    // 找到点数最多的面如果点数最多的面角度正常
    int max_idx = -1;
    // for (size_t i = 0; i < inlier_indices.size(); ++i)
    // {
    //     double angle_deg = computeAngleWithZAxis(model_coefficients[i]) * 180.0 / M_PI;
    //     if (angle_deg > max_plane_angle_)
    //     {
    //         LOG(ERROR) << "angle_deg > max_plane_angle_ continue!";
    //         continue;
    //     }
    //     else
    //     {
    //         if (max_idx == -1 || inlier_indices[i].indices.size() > inlier_indices[max_idx].indices.size())
    //         {
    //             max_idx = i;
    //         }
    //     }
    // }

    // 先找点数最多的面在判断
    for (size_t i = 0; i < inlier_indices.size(); ++i)
    {

        if (max_idx == -1 || inlier_indices[i].indices.size() > inlier_indices[max_idx].indices.size())
        {
            max_idx = i;
        }

        LOG(ERROR) << "idx:" << idx << ", indices size: " << inlier_indices[i].indices.size();
    }

    if (inlier_indices[max_idx].indices.size() < 500)
    {
        double angle_deg = computeAngleWithZAxis(model_coefficients[max_idx]) * 180.0 / M_PI;
        if (angle_deg > max_plane_angle_)
        {
            LOG(ERROR) << "angle_deg > max_plane_angle_ continue! : " << angle_deg;
            return false;
        }
    }

    if (max_idx != -1)
    {
        // 计算平面中心点
        pcl::compute3DCentroid(*cloud, inlier_indices[max_idx].indices, plane_center);
        plane_center[3] = static_cast<float>(inlier_indices[max_idx].indices.size());
        LOG(ERROR) << "center: (" << plane_center[0] << ", "
                   << plane_center[1] << ", "
                   << plane_center[2] << ", "
                   << plane_center[3] << ")";
        return true;
    }
    else
    {
        return false;
    }

    if (is_debug_)
    {
        // 计算平面中心点
        std::vector<pcl::PointXYZ> plane_centroids = computePlaneCentroids<pcl::PointXYZ>(cloud, inlier_indices);

        for (size_t i = 0; i < plane_centroids.size(); ++i)
        {
            std::cout << "Plane " << i + 1 << " centroid: "
                      << plane_centroids[i].x << ", "
                      << plane_centroids[i].y << ", "
                      << plane_centroids[i].z << std::endl;

            double angle_deg = computeAngleWithZAxis(model_coefficients[i]) * 180.0 / M_PI;

            LOG(ERROR) << "Plane " << i + 1 << " center: ("
                       << plane_centroids[i].x << ", "
                       << plane_centroids[i].y << ", "
                       << plane_centroids[i].z << ", "
                       << angle_deg << ")";

            pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            for (const auto &index : inlier_indices[i].indices)
            {
                plane_cloud->points.push_back(cloud->points[index]);
            }

            plane_cloud->width = plane_cloud->points.size();
            plane_cloud->height = 1;
            plane_cloud->is_dense = false;

            std::string name1 = ply_save_dir_ + std::to_string(idx) + std::to_string(i) + "p.ply";
            pcl::io::savePLYFile(name1, *plane_cloud);
        }
    }
}

void DrawPred(cv::Mat &img, std::vector<OutputSeg> result, std::vector<std::string> classNames, std::vector<cv::Scalar> color)
{
    cv::Mat mask = img.clone();
    for (int i = 0; i < result.size(); i++)
    {
        // LOG(ERROR) << "I: " << i;
        int left, top;
        left = result[i].box.x;
        top = result[i].box.y;
        int color_num = i;
        cv::rectangle(img, result[i].box, color[i], 2, 8);
        if (result[i].boxMask.rows && result[i].boxMask.cols > 0)
            mask(result[i].box).setTo(color[i], result[i].boxMask);
        char buffer[20];
        std::snprintf(buffer, sizeof(buffer), "%.2f", result[i].confidence);
        std::string label = std::to_string(i) + ":" + buffer;
        int baseLine;
        cv::Size labelSize = getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        top = std::max(top, labelSize.height);
        // cv::rectangle(img, cv::Point(left, top - int(1.5 * labelSize.height)), cv::Point(left + int(1.5 * labelSize.width), top + baseLine), cv::Scalar(0, 255, 0), cv::FILLED);
        // putText(img, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.7, color[result[0].id], 2);
    }
    addWeighted(img, 0.5, mask, 0.5, 0, img); // add mask to src
    //    imshow("1", img);
    // imwrite("out.bmp", img);
    //    waitKey();
    // destroyAllWindows();
}

// 在点云中查找与目标点(x, y, z)最近的点，返回其索引
int findNearestPointIndex(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    float target_x,
    float target_y,
    float target_z)
{
    // 检查点云是否为空
    if (cloud->empty())
    {
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
    std::vector<int> nearest_indices(1);     // 存储最近点的索引
    std::vector<float> nearest_distances(1); // 存储最近点的距离平方

    // 执行最近邻搜索（搜索1个最近点）
    int result = kdtree.nearestKSearch(target_point, 1, nearest_indices, nearest_distances);

    if (result > 0)
    {
        // 输出结果（距离平方开根号即为实际距离）
        std::cout << "找到最近点：" << std::endl;
        std::cout << "索引：" << nearest_indices[0] << std::endl;
        std::cout << "坐标：("
                  << cloud->points[nearest_indices[0]].x << ", "
                  << cloud->points[nearest_indices[0]].y << ", "
                  << cloud->points[nearest_indices[0]].z << ")" << std::endl;
        std::cout << "距离：" << sqrt(nearest_distances[0]) << std::endl;
        return nearest_indices[0]; // 返回最近点的索引
    }
    else
    {
        std::cerr << "未找到最近点！" << std::endl;
        return -1;
    }
}

#include <cmath>

double calculateDistanceHypot(const Eigen::Vector4f &p1, const Eigen::Vector4f &p2)
{
    double dx = p2[0] - p1[0];
    double dy = p2[1] - p1[1];
    double dz = p2[2] - p1[2];
    return std::hypot(dx, dy, dz);
}

// -1: roi 提取失败
// -2: 分割检测失败
// -3: 拟合平面失败
// -4: 空箱
bool ObjectRegionDetection::fitPlaneAndEstimatePose6D(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    Eigen::Vector3f &position,
    Eigen::Vector3f &euler_rpy,
    pcl::ModelCoefficients::Ptr &plane_coefficients,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &inlier_cloud)
{
    position.setZero();
    euler_rpy.setZero();

    if (!cloud || cloud->empty())
    {
        LOG(ERROR) << "fitPlaneAndEstimatePose6D: input cloud empty";
        return false;
    }

    // ========================================================================
    // 第一步：数据预处理（过滤无效点）
    // ========================================================================
    // 移除 NaN 和 (0,0,0) 无效点，同时保持点云有序性（通过不删除，只跳过）
    // 但 RANSAC 不需要有序，可以直接过滤掉无效点
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filtered_cloud->reserve(cloud->size());

    for (const auto &pt : cloud->points)
    {
        // 跳过 (0,0,0) 和 NaN
        if (pt.x == 0.0f && pt.y == 0.0f && pt.z == 0.0f)
            continue;
        if (!pcl::isFinite(pt))
            continue;
        filtered_cloud->push_back(pt);
    }

    LOG(ERROR) << "Original cloud size: " << cloud->size() 
               << ", filtered: " << filtered_cloud->size();

    if (filtered_cloud->size() < 100)
    {
        LOG(ERROR) << "Too few valid points after filtering";
        return false;
    }

    // ========================================================================
    // 第二步：RANSAC 多平面拟合（循环剔除，找点数最多的平面）
    // ========================================================================
    const int MAX_PLANES_TO_TRY = 5;      // 最多尝试提取 5 个平面
    const int MIN_INLIERS = 200;          // 最小内点数
    const float DISTANCE_THRESHOLD = 0.0015f;  // 距离阈值 1.5cm
    const int RANSAC_ITERATIONS = 200;

    // 记录最佳平面（内点数最多）
    pcl::PointIndices::Ptr best_inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr best_coefficients(new pcl::ModelCoefficients);
    int best_inlier_count = 0;

    // 剩余点云（每次移除已找到的平面）
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *remaining_cloud = *filtered_cloud;

    LOG(INFO) << "Starting RANSAC multi-plane fitting, max planes: " << MAX_PLANES_TO_TRY;

    for (int plane_idx = 0; plane_idx < MAX_PLANES_TO_TRY; ++plane_idx)
    {
        if (remaining_cloud->size() < MIN_INLIERS)
        {
            LOG(INFO) << "Remaining cloud too small, stopping at plane #" << plane_idx;
            break;
        }

        // RANSAC 拟合平面
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(RANSAC_ITERATIONS);
        seg.setDistanceThreshold(DISTANCE_THRESHOLD);
        seg.setInputCloud(remaining_cloud);
        seg.segment(*inliers, *coefficients);

        // 如果内点太少，停止尝试
        if (inliers->indices.size() < MIN_INLIERS)
        {
            LOG(INFO) << "Plane #" << plane_idx 
                      << " inliers: " << inliers->indices.size() 
                      << " < " << MIN_INLIERS << ", stopping";
            break;
        }

        LOG(INFO) << "Plane #" << plane_idx 
                  << " inliers: " << inliers->indices.size() 
                  << " / " << remaining_cloud->size();

        // 更新最佳平面（内点数最多）
        if (inliers->indices.size() > best_inlier_count)
        {
            best_inlier_count = static_cast<int>(inliers->indices.size());
            best_inliers = inliers;
            best_coefficients = coefficients;
        }

        // 从剩余点云中移除当前平面的内点，以便寻找下一个平面
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(remaining_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);  // 保留非内点
        extract.filter(*remaining_cloud);
    }

    // 检查是否找到有效平面
    if (best_inlier_count < MIN_INLIERS)
    {
        LOG(ERROR) << "No valid plane found, best_inlier_count=" << best_inlier_count;
        return false;
    }

    LOG(INFO) << "Selected best plane with " << best_inlier_count << " inliers";

    // ========================================================================
    // 第三步：使用最佳平面结果提取内点云（使用原始 filtered_cloud 的索引）
    // ========================================================================
    // 注意：best_inliers 中的索引是基于 remaining_cloud 的，
    // 但 remaining_cloud 经过多次剔除后，索引已经不对应 filtered_cloud 了！
    // 所以需要直接从 filtered_cloud 中重新拟合一次，获得对应 filtered_cloud 的索引

    // 重新对 filtered_cloud 进行 RANSAC，使用最佳系数作为种子
    // 更稳健的方法：直接用最佳系数计算 distance，筛选内点
    inlier_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    plane_coefficients = best_coefficients;

    // 手动提取内点（用最佳平面系数）
    float a = best_coefficients->values[0];
    float b = best_coefficients->values[1];
    float c = best_coefficients->values[2];
    float d = best_coefficients->values[3];
    float norm = std::sqrt(a*a + b*b + c*c);
    if (norm > 0)
    {
        a /= norm; b /= norm; c /= norm; d /= norm;
    }

    inlier_cloud->reserve(filtered_cloud->size());
    for (const auto &pt : filtered_cloud->points)
    {
        float dist = std::abs(a * pt.x + b * pt.y + c * pt.z + d);
        if (dist < DISTANCE_THRESHOLD)
        {
            inlier_cloud->push_back(pt);
        }
    }

    LOG(INFO) << "Extracted inlier cloud size: " << inlier_cloud->size();

    if (inlier_cloud->size() < MIN_INLIERS)
    {
        LOG(ERROR) << "Inlier cloud too small: " << inlier_cloud->size();
        return false;
    }

    // ========== 保存调试文件 ==========
    if (is_debug_)
    {
        std::string inlier_filename = ply_save_dir_ + std::to_string(inlier_cloud->size()) + "_inlier_cloud.ply";
        pcl::io::savePLYFile(inlier_filename, *inlier_cloud);
        LOG(INFO) << "Saved inlier cloud to: " << inlier_filename;
    }

    // ========================================================================
    // 第四步：姿态估计（保持不变）
    // ========================================================================
    // ========================================================================
    // 第三步：PCA 主成分分析 + 6D 位姿构造
    // 目标：在拟合出的平面上建立目标坐标系（X右 Y前 Z下），输出位置+ZYX欧拉角
    // ========================================================================

    // PCA（主成分分析）：对内点云做特征分解，得到表面点云的主方差方向
    // 用途：eigenvectors.col(0) 是方差最大的方向，作为候选 X 轴
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(inlier_cloud);
    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();

    // Z 轴：直接用拟合平面的法向量 (A,B,C) 作为 Z 轴
    // 平面方程 Ax+By+Cz+D=0，法向量即 (A,B,C)
    Eigen::Vector3f z_axis(best_coefficients->values[0],
                           best_coefficients->values[1],
                           best_coefficients->values[2]);
    z_axis.normalize();               // 单位化
    if (z_axis.z() < 0) z_axis = -z_axis;   // 让 Z 轴朝相机正方向（Z 分量>0）

    // X 轴：取 PCA 第一主成分（方差最大方向）作为 X 轴候选
    Eigen::Vector3f x_axis = eigen_vectors.col(0);
    x_axis.normalize();

    // Y 轴：由 Z×X 叉乘补出（保证右手系）
    Eigen::Vector3f y_axis = z_axis.cross(x_axis);
    y_axis.normalize();
    // 重新正交化 X 轴：X = Y×Z（消除 X 与 Z 不严格垂直的误差）
    x_axis = y_axis.cross(z_axis);
    x_axis.normalize();

    // 约定式定向：通过绕 X 轴旋转 180° 将坐标系翻转为 "X朝右 Y朝前 Z朝下"
    // 矩阵作用： diag(1,-1,-1) @ R 会把 Y 和 Z 轴取反，X 轴不变
    Eigen::Matrix3f R_transform;
    R_transform << 1, 0, 0,
                   0, -1, 0,
                   0, 0, -1;
    // 拼装当前旋转矩阵（列向量为三轴）
    Eigen::Matrix3f R_current;
    R_current << x_axis, y_axis, z_axis;
    // 应用翻转约定
    Eigen::Matrix3f R_new = R_transform * R_current;

    // 取出翻转后的三轴向量
    Eigen::Vector3f x_new = R_new.col(0);
    Eigen::Vector3f y_new = R_new.col(1);
    Eigen::Vector3f z_new = R_new.col(2);

    // 符号约束：锁定旋向（避免 180° 歧义）
    // x_new.x()>0 表示 X 轴朝 +X，取反让其朝 -X（与目标朝向一致）
    if (x_new.x() > 0) x_new = -x_new;
    if (y_new.y() > 0) y_new = -y_new;
    if (z_new.z() > 0) z_new = -z_new;

    // 重新拼装最终旋转矩阵
    Eigen::Matrix3f R;
    R << x_new, y_new, z_new;

    // SVD 正交化：上面多次叉乘+符号调整可能破坏正交性，用 SVD 投影到最近旋转矩阵
    // R = U·V^T 是 R 的最近正交旋转矩阵（Procrustes 问题解）
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    R = svd.matrixU() * svd.matrixV().transpose();
    // 若投影后变成反射矩阵（行列式<0），整体取反修正为合法旋转
    if (R.determinant() < 0) R = -R;

    // 位置：取内点点云的 3D 质心作为目标位置
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*inlier_cloud, centroid);
    position = centroid.head<3>();

    // ZYX 内禀欧拉角分解（等价于 XYZ 外禀）：R = Rz(yaw)·Ry(pitch)·Rx(roll)
    // pitch 从 R(2,0) 反解（asin 限幅到 [-1,1]）
    float pitch = std::asin(-std::max(-1.0f, std::min(1.0f, R(2, 0))));
    float roll, yaw;
    if (std::abs(std::cos(pitch)) > 1e-6f)
    {
        // 非奇异情形：可独立求出 roll 和 yaw
        roll = std::atan2(R(2, 1), R(2, 2));
        yaw = std::atan2(R(1, 0), R(0, 0));
    }
    else
    {
        // 奇异情形（pitch=±90°，万向锁）：yaw 设为 0，roll 从剩余元素反解
        roll = std::atan2(-R(1, 2), R(1, 1));
        yaw = 0.0f;
    }
    euler_rpy << roll, pitch, yaw;

    LOG(ERROR) << "pose6d: pos=(" << position.x() << ", " << position.y() << ", " << position.z()
              << "), rpy_deg=(" << roll * 180.0f / M_PI << ", " << pitch * 180.0f / M_PI << ", "
              << yaw * 180.0f / M_PI << ")";

    return true;
}

int ObjectRegionDetection::DetectRegion(cv::Mat &img_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr &transformed_cloud, LocalizationInfo *localization_region)
{
    
    // 1、固定尺寸
    // 2、固定尺寸内寻找
    // 3、固定尺寸点云下采样寻找加速


    // cv::imwrite("cropped.png", cropped);
    localization_region->output.clear();
    std::vector<OutputSeg> output;
    image_buffer_t src_image;
    cv::Mat resize_cropped = img_in.clone();
    
    cv::cvtColor(resize_cropped, resize_cropped, cv::COLOR_BGR2RGB);
    convert_mat_to_image_buffer(resize_cropped, src_image);
    object_detect_result_list od_results;

    int ret = inference_yolov8_seg_model(&rknn_app_ctx_, &src_image, &od_results);
    if (ret != 0)
    {
        printf("init_yolov8_seg_model fail! ret=%d\n", ret);
        return -2;
    }
    LOG(ERROR) << "od_results.count: " << od_results.count;
    if (od_results.count == 0)
    {
        localization_region->is_box_empty = true;
        return -4;
    }

    if (is_debug_ && od_results.count >= 1)
    {
        for (int i = 0; i < od_results.count; i++)
        {
            int width = src_image.width;
            int height = src_image.height;
            cv::Mat seg_mat(height, width, CV_8UC1, od_results.results_seg[i].seg_mask);
            std::string name = std::to_string(i) + ".png";
            cv::imwrite(name, seg_mat);
            printf("idx%d\n", i);
        }
    }

    // 推理结果筛选处理
    if (od_results.count >= 1)
    {
        for (int i = 0; i < od_results.count; i++)
        {
            object_detect_result *det_result = &(od_results.results[i]);
            if (det_result->box.right <= 0 ||
                det_result->box.bottom <= 0 ||
                det_result->box.left >= src_image.width ||
                det_result->box.top >= src_image.height)
            {
                LOG(ERROR) << "边界超出! ";
                continue;
            }

            int x1 = std::max(0, std::min(det_result->box.left, src_image.width - 1));
            int y1 = std::max(0, std::min(det_result->box.top, src_image.height - 1));
            int x2 = std::max(0, std::min(det_result->box.right, src_image.width));
            int y2 = std::max(0, std::min(det_result->box.bottom, src_image.height));
            cv::Rect bbox(x1, y1, x2 - x1, y2 - y1);
            // LOG(ERROR) << "src_image: " << src_image.height << ", " << src_image.width;
            // LOG(ERROR) << "bbox: " << det_result->box.left << ", " << det_result->box.top << ", " << det_result->box.right << ", " << det_result->box.bottom;
            // LOG(ERROR) << "bbox id: " << det_result->cls_id <<  "," << bbox.x << ", " << bbox.y << ", " << bbox.height << ", " << bbox.width;

            cv::Mat seg_mat(src_image.height, src_image.width, CV_8UC1, od_results.results_seg[i].seg_mask);
            cv::Mat boxMask = seg_mat(bbox);

            OutputSeg r;
            r.id = det_result->cls_id;
            r.confidence = det_result->prop;

            if (r.confidence < 0.85)
            {
                LOG(ERROR) << "置信度太低 跳过!" << r.confidence;
                continue;
            }
            // LOG(ERROR) << "置信度!" << r.confidence;

            {
                r.box = bbox;
                r.boxMask = boxMask;
            }

            output.push_back(r);
        }
    }
    if (src_image.virt_addr != NULL)
    {
        free(src_image.virt_addr);
    }
    LOG(ERROR) << "置信度!" ;

    bool find = true;
    if (find)
    {
        // 提取最大置信度物品
        std::sort(output.begin(), output.end());
        if (output.size() > 0)
        {
            for (const auto &seg : output)
            {
                    localization_region->output.push_back(seg);
            }
        }
        if (localization_region->output.empty())
        {
            LOG(ERROR) << "未检测到物品!";
            return -1;
        }
        
        for (auto &seg : localization_region->output)
        {
            seg.box = cv::Rect(seg.box.x , seg.box.y, seg.box.width, seg.box.height);
            LOG(ERROR) << "id: " << seg.id << "seg.cof:" << seg.confidence;
        }

        int best_idx = -1;
        Eigen::Vector4f best_centroid = Eigen::Vector4f::Zero();
        std::vector<std::pair<Eigen::Vector4f, int>> centroid_idx_pairs;

        // #pragma omp parallel for
        for (size_t i = 0; i < localization_region->output.size(); ++i)
        {
            auto &region = localization_region->output[i];

            pcl::PointCloud<pcl::PointXYZ>::Ptr croped_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            extractMaskedCloud(transformed_cloud, croped_cloud, region.boxMask, region.box, img_in);

            LOG(ERROR) << "transformed_cloud.size(): " << transformed_cloud->size() << ", croped_cloud.size(): " << croped_cloud->size();
            if (is_debug_)
            {
                std::string name = ply_save_dir_ + std::to_string(i) + "cloud.ply";
                pcl::io::savePLYFile(name, *croped_cloud);
            }

            //拟合*croped_cloud平面返回最多点数的平面，进行pca主成分分析提取出6d姿态
            Eigen::Vector3f pose_position, pose_rpy;
            pcl::ModelCoefficients::Ptr plane_coef;
            pcl::PointCloud<pcl::PointXYZ>::Ptr plane_inliers(new pcl::PointCloud<pcl::PointXYZ>);
            if (fitPlaneAndEstimatePose6D(croped_cloud, pose_position, pose_rpy, plane_coef, plane_inliers))
            {
                region.pose_position = pose_position;
                region.pose_rpy = pose_rpy;
            }
            LOG(ERROR) << "pose_position: " << pose_position.x() << ", " << pose_position.y() << ", " << pose_position.z()
                << "), rpy_deg=(" << region.pose_rpy.x() * 180.0f / M_PI << ", " << region.pose_rpy.y() * 180.0f / M_PI << ", "
                << region.pose_rpy.z() * 180.0f / M_PI << ")" << "id:" << region.id <<"confidence:"<< region.confidence;

        }
     

        std::vector<std::string> className = {
            "box1"};

        DrawPred(img_in, localization_region->output, className, color);
        cv::rectangle(img_in, localization_region->output[best_idx].box, cv::Scalar(0, 0, 255), 2, 8);
        // cv::rectangle(img_in, crop_bbox, cv::Scalar(0, 0, 255), 2, 8);
        // 计算矩形框的中心点
        cv::Point center(
            localization_region->output[best_idx].box.x + localization_region->output[best_idx].box.width / 2,
            localization_region->output[best_idx].box.y + localization_region->output[best_idx].box.height / 2);
        cv::circle(img_in, center, 5, cv::Scalar(0, 0, 255), -1);


        std::string img_in_path = ply_save_dir_ + "img.bmp";
        cv::imwrite(img_in_path, img_in);
        LOG(ERROR) << "Saved img_in to: " << img_in_path;

        // cv::imshow("img_in", img_in);
        // cv::waitKey(0);
    }

    return 1;
}

int ObjectRegionDetection::DetectRegion(cv::Mat &img_in)
{
    std::vector<OutputSeg> output;
    image_buffer_t src_image;
    // cv::cvtColor(img_in, img_in, cv::COLOR_BGR2RGB);
    convert_mat_to_image_buffer(img_in, src_image);
    object_detect_result_list od_results;

    int ret = inference_yolov8_seg_model(&rknn_app_ctx_, &src_image, &od_results);
    if (ret != 0)
    {
        printf("init_yolov8_seg_model fail! ret=%d\n", ret);
        return -2;
    }else{
        LOG(ERROR) << "od_results.count: " << od_results.count;
    }

    for (int i = 0; i < od_results.count; i++)
    {
        object_detect_result *det_result = &(od_results.results[i]);
        // if (det_result->box.right <= 0 ||
        //     det_result->box.bottom <= 0 ||
        //     det_result->box.left >= src_image.width ||
        //     det_result->box.top >= src_image.height)
        // {
        //     LOG(ERROR) << "边界超出! ";
        //     continue;
        // }

        int x1 = std::max(0, std::min(det_result->box.left, src_image.width - 1));
        int y1 = std::max(0, std::min(det_result->box.top, src_image.height - 1));
        int x2 = std::max(0, std::min(det_result->box.right, src_image.width));
        int y2 = std::max(0, std::min(det_result->box.bottom, src_image.height));
        cv::Rect bbox(x1, y1, x2 - x1, y2 - y1);
        LOG(ERROR) << "src_image: " << src_image.height << ", " << src_image.width;
        LOG(ERROR) << "bbox: " << det_result->box.left << ", " << det_result->box.top << ", " << det_result->box.right << ", " << det_result->box.bottom;
        LOG(ERROR) << "bbox id: " << det_result->cls_id <<  "," << bbox.x << ", " << bbox.y << ", " << bbox.height << ", " << bbox.width;
        LOG(ERROR) << "prop: " << det_result->prop;
        OutputSeg r;
        r.id = det_result->cls_id;
        r.confidence = det_result->prop;
        r.box = bbox;
        cv::Mat seg_mat(src_image.height, src_image.width, CV_8UC1, od_results.results_seg[i].seg_mask);
        r.boxMask = seg_mat(bbox).clone();
        output.push_back(r);
    }

    std::vector<std::string> className = {
        "box1"};
    DrawPred(img_in, output, className, color);
    return 1;
}
