
#include <mutex>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/kdtree/kdtree_flann.h>
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
    bool pose_ok = false;           // 6D位姿是否检测成功
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

    int DetectRegion(cv::Mat &img_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr &transformed_cloud, LocalizationInfo *localization_region, const std::string &target_obj_name = "");
    int DetectRegion(cv::Mat &img_in);

    // 设置纸箱顶面尺寸先验（米）
    void setBoxSize(float len, float wid)
    {
        box_len_ = std::max(len, wid);
        box_wid_ = std::min(len, wid);
    }

    // 设置点云配准目标模型点云路径（默认 py/register.py 同款 data/model/model_bs_m.ply）
    void setRegistrationModel(const std::string &path)
    {
        reg_model_path_ = path;
        reg_model_ready_ = false; // 路径变更后下次调用重新初始化
    }

    // 设置 rack 点云配准目标模型点云路径
    void setRackRegistrationModel(const std::string &path)
    {
        rack_model_path_ = path;
        rack_model_ready_ = false;
    }

protected:
    bool findBoundingBoxes(const cv::Mat binaryImage, cv::Rect &bbox);

    bool filterOrderedPointCloud(cv::Mat &img_in, pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
                                 cv::Rect &bbox, float minDepth, float maxDepth);

    void extractMaskedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                            const cv::Mat &mask, const cv::Rect &roi, const cv::Mat img);

    // 点云配准 6D 位姿估计（毫秒级方案）:
    // PCA 主轴粗对齐(料盘≈平面矩形, 4 个保定向候选, 替代 FPFH+RANSAC 全局配准)
    // + OMP 并行多尺度 point-to-plane ICP 精配准(目标树预建, 自研循环);
    // 位姿 = ICP 变换(源->目标模型)的逆矩阵, 即模型原点在相机/机器人系下的位姿
    bool registerAndEstimatePose6D(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                   Eigen::Vector3f &position,
                                   Eigen::Vector3f &euler_rpy);

    // 目标模型点云惰性初始化（加载+统计去离群+下采样+法线+PCA 主轴, 仅首次调用执行）
    bool initRegistrationModel();

    // 配准质量: 变换后源点云中与目标 KD 树近邻距离 < max_dist 的点占比; rmse 为内点均方根距离
    float registrationFitness(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                              const Eigen::Matrix4f &transform, float max_dist, float *rmse = nullptr);

    // 自研 OMP point-to-plane ICP: 目标 KD 树只读查询多线程并行,
    // 粗三级阈值 {10,2,0.5}x体素 + 第 4 级 0.5x体素细尺度终修, 增量足够小提前收敛
    bool icpPointToPlaneOMP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                            const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_fine,
                            const Eigen::Matrix4f &init_T,
                            Eigen::Matrix4f &out_T);

    // ---- Rack 点云配准 6D 位姿估计（仿照 box 版本，使用 rack_bs_m.ply 模板，方便后续针对性优化）----
    bool registerAndEstimatePose6DRack(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                       Eigen::Vector3f &position,
                                       Eigen::Vector3f &euler_rpy);
    bool initRackRegistrationModel();
    float rackRegistrationFitness(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                                  const Eigen::Matrix4f &transform, float max_dist, float *rmse = nullptr);
    bool icpPointToPlaneOMPRack(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                                const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_fine,
                                const Eigen::Matrix4f &init_T,
                                Eigen::Matrix4f &out_T);

    bool is_debug_;
    std::string ply_save_dir_;   // PLY 调试文件保存根目录
    std::string debug_session_dir_; // 本次 DetectRegion 的调试会话目录（is_debug_ 开启时每帧新建）
    std::vector<cv::Scalar> color;
    bool is_resize_;
    float box_len_;              // 纸箱顶面长边尺寸（米，先验）
    float box_wid_;              // 纸箱顶面短边尺寸（米，先验）

    // ---------------- 点云配准 6D 位姿估计 ----------------
    std::string reg_model_path_;                                  // 目标模型点云路径
    float reg_voxel_ = 0.f;                                       // 体素大小 = 目标对角线 * 0.02
    bool reg_model_ready_ = false;
    std::mutex reg_model_mutex_;
    pcl::PointCloud<pcl::PointNormal>::Ptr reg_target_pn_;        // 目标下采样点云+法线（ICP 目标, 与 reg_target_down_ 同序）
    pcl::PointCloud<pcl::PointXYZ>::Ptr reg_target_down_;         // 目标下采样点云（KD 树 / PCA / fitness）
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr reg_kdtree_;             // 目标下采样 KD 树（只读 NN 查询, 线程安全）
    pcl::PointCloud<pcl::PointNormal>::Ptr reg_fine_pn_;          // 细尺度精修目标(0.5×体素)点云+法线（ICP 第 4 级终修用）
    pcl::PointCloud<pcl::PointXYZ>::Ptr reg_fine_cloud_;          // 细尺度精修目标点云
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr reg_fine_kdtree_;        // 细尺度精修目标 KD 树
    float reg_fine_voxel_ = 0.f;                                  // 细尺度体素 = reg_voxel_ × 0.5
    Eigen::Matrix3f reg_model_axes_{Eigen::Matrix3f::Identity()}; // 模型主轴(列: 长边u/短边v/法线n, 右手系, n 朝上)
    Eigen::Vector3f reg_model_centroid_{Eigen::Vector3f::Zero()}; // 模型下采样质心
    Eigen::Vector2f reg_model_extents_{Eigen::Vector2f::Ones()};  // 模型沿长/短边方向跨度(源尺寸预检基准)

    // ---------------- Rack 点云配准 6D 位姿估计 ----------------
    std::string rack_model_path_;                                     // rack 目标模型点云路径
    float rack_voxel_ = 0.f;
    bool rack_model_ready_ = false;
    std::mutex rack_model_mutex_;
    pcl::PointCloud<pcl::PointNormal>::Ptr rack_target_pn_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr rack_target_down_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr rack_kdtree_;
    pcl::PointCloud<pcl::PointNormal>::Ptr rack_fine_pn_;         // 细尺度精修目标(0.5×体素)点云+法线
    pcl::PointCloud<pcl::PointXYZ>::Ptr rack_fine_cloud_;         // 细尺度精修目标点云
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr rack_fine_kdtree_;       // 细尺度精修目标 KD 树
    float rack_fine_voxel_ = 0.f;                                  // 细尺度体素 = rack_voxel_ × 0.5
    Eigen::Matrix3f rack_model_axes_{Eigen::Matrix3f::Identity()};
    Eigen::Vector3f rack_model_centroid_{Eigen::Vector3f::Zero()};
    Eigen::Vector2f rack_model_extents_{Eigen::Vector2f::Ones()};

    rknn_app_context_t rknn_app_ctx_;

    Eigen::Vector4f last_best_centroid;
    std::mutex pairs_mutex;
};