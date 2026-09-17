
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <opencv2/opencv.hpp>
#include "yolov8_seg.h"
#include "image_utils.h"
#include "file_utils.h"
#include "image_drawing.h"

struct OutputSeg
{
    int id;           // 结果类别id
    float confidence; // 结果置信度
    cv::Rect box;     // 矩形框
    cv::Mat boxMask;  // 矩形框内mask，节省内存空间和加快速度
    Eigen::Vector3f pose_position; // 平面拟合后的6D位姿位置
    Eigen::Vector3f pose_rpy;      // ZYX内禀欧拉角(roll,pitch,yaw)
    bool operator<(const OutputSeg &other) const
    {
        return confidence > other.confidence; // 注意是>实现降序
    }
};

struct LocalizationInfo
{
    LocalizationInfo()
    {
        is_box_empty = false;
        num_object_detected = 0;
        min_depth = 0;
        max_depth = 0;
        min_limit = 0;
        max_limit = 0;
        output.clear();
        output_raw.clear();
    }

    int num_object_detected;

    std::vector<OutputSeg> output;
    std::vector<Eigen::Vector4f> output_raw; // x,y,z,r
    bool is_box_empty;

    float min_depth;
    float max_depth;
    float min_limit;
    float max_limit;

    cv::Rect crop_bbox;

    // 限制识别到的物品大小

    // 限制处理的数量
};

class ObjectRegionDetection
{
public:
    ObjectRegionDetection();
    ~ObjectRegionDetection();

    int DetectRegion(cv::Mat &img_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr &transformed_cloud, LocalizationInfo *localization_region);
    int DetectRegion(cv::Mat &img_in);

protected:
    bool findBoundingBoxes(const cv::Mat binaryImage, cv::Rect &bbox);

    bool filterOrderedPointCloud(cv::Mat &img_in, pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
                                 cv::Rect &bbox, float minDepth, float maxDepth);

    void extractMaskedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                            const cv::Mat &mask, const cv::Rect &roi, const cv::Mat img);

    pcl::PointCloud<pcl::PointXYZ>::Ptr applyPassThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                               float min_limit = std::numeric_limits<float>::lowest(),
                                                               float max_limit = std::numeric_limits<float>::max(),
                                                               const std::string &field_name = "z",
                                                               bool negative = false);

    pcl::PointCloud<pcl::PointXYZ>::Ptr removeDistantPointsByClustering(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr extractPlanePoints(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const pcl::PointIndices::Ptr &inliers);

    bool multiPlaneSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                int idx, Eigen::Vector4f &plane_center);

    bool fitPlaneAndEstimatePose6D(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                   Eigen::Vector3f &position,
                                   Eigen::Vector3f &euler_rpy,
                                   pcl::ModelCoefficients::Ptr &plane_coefficients,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr &inlier_cloud);

    bool is_debug_;
    std::string ply_save_dir_;   // PLY 调试文件保存目录
    std::vector<cv::Scalar> color;
    bool is_resize_;
    double max_plane_angle_;

    rknn_app_context_t rknn_app_ctx_;

    Eigen::Vector4f last_best_centroid;
    std::mutex pairs_mutex;
};