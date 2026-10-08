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
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/eigen.h>
#include <pcl/common/pca.h>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <omp.h>
#include <chrono>
#include <iostream>
#include <vector>
#include <algorithm>
#include <utility>
#include <atomic>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <cmath>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>

#include <glog/logging.h>
#include <gflags/gflags.h>

ObjectRegionDetection::ObjectRegionDetection()
{
    // std::string model_path = "/home/neardi/workspace/project/sort_detection_npu/models/best_3576.rknn";
    std::string model_path = SORT_DETECTION_MODEL_DIR "/tray924.rknn";

    memset(&rknn_app_ctx_, 0, sizeof(rknn_app_context_t));

    init_post_process();

    int ret = init_yolov8_seg_model(model_path.c_str(), &rknn_app_ctx_);
    if (ret != 0)
    {
        LOG(ERROR) << "init_yolov8_seg_model fail!";
    }

    is_debug_ = true;
    ply_save_dir_ = SORT_DETECTION_DEBUG_DIR;
    if (!ply_save_dir_.empty() && ply_save_dir_.back() != '/')
    {
        ply_save_dir_ += '/';
    }
    debug_session_dir_ = ply_save_dir_; // 默认指向根目录；DetectRegion 开始时按帧新建会话目录
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
    box_len_ = 0.38f;
    box_wid_ = 0.29f;
    // 点云配准目标模型（对标 py/register.py 的 data/model/model_bs_m.ply，可用 setRegistrationModel 更改）
    reg_model_path_ = SORT_DETECTION_MODEL_DIR "/model_bs_m.ply";
    // rack 配准目标模型（rack_bs_m.ply）
    rack_model_path_ = SORT_DETECTION_MODEL_DIR "/rack_bs_m.ply";
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

static bool saveDebugPLYXYZ(const std::string &path, const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    std::ofstream f(path);
    if (!f.is_open())
    {
        LOG(ERROR) << "saveDebugPLYXYZ: cannot open " << path;
        return false;
    }
    f << "ply\nformat ascii 1.0\n"
      << "element vertex " << cloud.size() << "\n"
      << "property float x\nproperty float y\nproperty float z\n"
      << "end_header\n";
    f << std::fixed << std::setprecision(6);
    for (const auto &p : cloud)
        f << p.x << " " << p.y << " " << p.z << "\n";
    return true;
}

// Open3D 导出的 PLY 为 binary_little_endian + double 坐标, 而 PCL 1.12 的 PLYReader
// 只支持 float 坐标——double 属性匹配不上字段, 点会被静默读成全 (0,0,0)（KD 树全同点
// 退化, 后续 SOR/法线估计会"卡死"数分钟）。此读取器支持标量属性的 binary LE PLY
// (float/double/整型 xyz), 作为 PCL 读取失败或退化时的回退。仅解析 vertex 元素。
static bool loadPLYBinaryXYZ(const std::string &path, pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open())
        return false;

    auto type_size = [](const std::string &t) -> int {
        if (t == "char" || t == "int8" || t == "uchar" || t == "uint8")
            return 1;
        if (t == "short" || t == "int16" || t == "ushort" || t == "uint16")
            return 2;
        if (t == "int" || t == "int32" || t == "uint" || t == "uint32" || t == "float" || t == "float32")
            return 4;
        if (t == "double" || t == "float64")
            return 8;
        return -1;
    };

    bool binary_le = false, header_ok = false, in_vertex = false;
    long vertex_count = -1;
    std::vector<int> sizes;
    int x_i = -1, y_i = -1, z_i = -1;

    std::string line;
    while (std::getline(f, line))
    {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        std::istringstream iss(line);
        std::vector<std::string> tk;
        std::string w;
        while (iss >> w)
            tk.push_back(w);
        if (tk.empty())
            continue;
        if (tk[0] == "format")
        {
            binary_le = (tk.size() > 1 && tk[1] == "binary_little_endian");
        }
        else if (tk[0] == "element")
        {
            in_vertex = (tk.size() > 2 && tk[1] == "vertex");
            if (in_vertex)
                vertex_count = atol(tk[2].c_str());
        }
        else if (tk[0] == "property" && in_vertex)
        {
            if (tk.size() < 3 || tk[1] == "list")
                return false; // list 属性不支持
            int sz = type_size(tk[1]);
            if (sz < 0)
                return false;
            if (tk[2] == "x")
                x_i = static_cast<int>(sizes.size());
            else if (tk[2] == "y")
                y_i = static_cast<int>(sizes.size());
            else if (tk[2] == "z")
                z_i = static_cast<int>(sizes.size());
            sizes.push_back(sz);
        }
        else if (tk[0] == "end_header")
        {
            header_ok = true;
            break;
        }
    }
    if (!header_ok || !binary_le || vertex_count <= 0 || x_i < 0 || y_i < 0 || z_i < 0)
        return false;

    // 顶点记录布局: 各属性紧密排列无填充, 记录 x/y/z 的字节偏移
    int rec_size = 0, off_x = 0, off_y = 0, off_z = 0;
    for (size_t i = 0; i < sizes.size(); ++i)
    {
        if (static_cast<int>(i) == x_i)
            off_x = rec_size;
        else if (static_cast<int>(i) == y_i)
            off_y = rec_size;
        else if (static_cast<int>(i) == z_i)
            off_z = rec_size;
        rec_size += sizes[i];
    }

    cloud.clear();
    cloud.reserve(vertex_count);
    std::vector<char> buf(static_cast<size_t>(vertex_count) * rec_size);
    f.read(buf.data(), buf.size());
    if (f.gcount() != static_cast<std::streamsize>(buf.size()))
        return false;

    // 主机为小端(RK3588/x86), 与 binary_little_endian 一致, 直接 memcpy
    auto read_scalar = [](const char *base, int size) -> double {
        if (size == 4)
        {
            float v;
            memcpy(&v, base, 4);
            return v;
        }
        if (size == 8)
        {
            double v;
            memcpy(&v, base, 8);
            return v;
        }
        if (size == 2)
        {
            short v;
            memcpy(&v, base, 2);
            return v;
        }
        if (size == 1)
            return static_cast<double>(*reinterpret_cast<const unsigned char *>(base));
        int v;
        memcpy(&v, base, 4);
        return v;
    };
    for (long i = 0; i < vertex_count; ++i)
    {
        const char *rec = buf.data() + static_cast<size_t>(i) * rec_size;
        pcl::PointXYZ p;
        p.x = static_cast<float>(read_scalar(rec + off_x, sizes[x_i]));
        p.y = static_cast<float>(read_scalar(rec + off_y, sizes[y_i]));
        p.z = static_cast<float>(read_scalar(rec + off_z, sizes[z_i]));
        cloud.push_back(p);
    }
    return true;
}

// ================= 点云配准 6D 位姿估计（毫秒级方案） =================
// 流程(精配准部分与 py/register.py 的多尺度 point-to-plane ICP 等价, 全局粗配准
// 换成针对平面矩形的 PCA 主轴对齐——FPFH+RANSAC 在 RK3588 上秒级, 无法实时):
//   1. 目标模型惰性初始化(只做一次): 加载(支持 double PLY 回退) + 统计去离群
//      + 体素(对角线*0.02)下采样 + 法线 + PCA 主轴 + KD 树
//   2. 源点云: 清理 (0,0,0) 填充点 -> 统计去离群 -> 体素下采样
//   3. PCA 粗对齐: 源/模型主轴配对生成 4 个保定向候选(绕法线/长边/短边 180°),
//      每候选一次 fitness 预评分, 取前二
//   4. OMP 并行多尺度 point-to-plane ICP 精配准(阈值 10/2/0.5 倍体素, 由粗到细)
//   5. 位姿 = ICP 变换(源->目标模型)的逆矩阵, 即模型原点在相机/机器人系下的 6D 位姿
//      (RPY 为 ZYX 顺序: R = Rz(yaw) * Ry(pitch) * Rx(roll), 弧度)
bool ObjectRegionDetection::initRegistrationModel()
{
    std::lock_guard<std::mutex> lock(reg_model_mutex_);
    if (reg_model_ready_)
        return true;

    reg_target_pn_.reset(new pcl::PointCloud<pcl::PointNormal>);
    reg_target_down_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    reg_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);

    // 目标点云加载: 自定义解析器优先(直接支持 double 坐标 binary PLY, 25 万点 ~15ms);
    // 失败(如 ASCII PLY)再回退 PCL 读取器——若顺序相反, PCL 会先把 double PLY 完整
    // 读成全 0 点(耗时约 0.5s)再被丢弃, 白白浪费一次性初始化时间
    const auto t0 = std::chrono::steady_clock::now();
    auto elapsed_s = [&]() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    };
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    std::string load_via = "custom";
    Eigen::Vector4f minp = Eigen::Vector4f::Zero(), maxp = Eigen::Vector4f::Zero();
    if (!loadPLYBinaryXYZ(reg_model_path_, *target) || target->empty())
    {
        target->clear();
        if (pcl::io::loadPLYFile(reg_model_path_, *target) < 0 || target->empty())
        {
            LOG(ERROR) << "register: load model fail: " << reg_model_path_;
            return false;
        }
        load_via = "pcl";
    }
    else
    {
        pcl::getMinMax3D(*target, minp, maxp);
    }
    if ((maxp - minp).head<3>().norm() < 1e-6f && load_via == "custom")
    {
        // 自定义读取成功但点云退化(异常文件), 再试 PCL
        target->clear();
        if (pcl::io::loadPLYFile(reg_model_path_, *target) >= 0 && !target->empty())
            load_via = "pcl";
    }
    LOG(ERROR) << "register: model init [1/4] loaded(" << load_via << "), pts=" << target->size()
               << ", " << elapsed_s() << "s";
    if (target->size() > 20)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(target);
        sor.setMeanK(20);
        sor.setStddevMulThresh(2.0);
        sor.filter(*filtered);
        if (!filtered->empty())
            target = filtered;
    }
    LOG(ERROR) << "register: model init [2/4] outlier removed, pts=" << target->size() << ", " << elapsed_s() << "s";

    // 体素大小 = 目标包围盒对角线 * 0.02（保证目标下采样后仍有足够点数做特征匹配）
    pcl::getMinMax3D(*target, minp, maxp); // SOR 后重算（minp/maxp 在加载段已声明）
    reg_voxel_ = (maxp - minp).head<3>().norm() * 0.02f;
    if (reg_voxel_ < 1e-4f)
    {
        LOG(ERROR) << "register: model degenerate, voxel=" << reg_voxel_;
        return false;
    }

    // 下采样 + 特征(与 Python preprocess 一致: 法线/FPFH 均在下采样点云上计算)。
    // 实测 25.7 万点的模型: 全云 SOR ~1.6s, 下采样 ~0.02s, 法线+FPFH ~0.1s, 一次性初始化 <2s。
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(target);
    vg.setLeafSize(reg_voxel_, reg_voxel_, reg_voxel_);
    vg.filter(*reg_target_down_);
    if (reg_target_down_->empty())
    {
        LOG(ERROR) << "register: model downsample empty";
        return false;
    }
    LOG(ERROR) << "register: model init [3/4] downsampled, pts=" << reg_target_down_->size()
               << ", voxel=" << reg_voxel_ << ", " << elapsed_s() << "s";

    // 法线(半径 2 倍体素)——ICP point-to-plane 只需目标侧法线
    pcl::PointCloud<pcl::Normal>::Ptr normals_down(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(reg_target_down_);
    ne.setRadiusSearch(reg_voxel_ * 2.0f);
    ne.compute(*normals_down);
    // ICP 目标 = 下采样点云 + 法线（与 reg_target_down_ 逐点同序, 自研 ICP 按下标取法线）
    pcl::concatenateFields(*reg_target_down_, *normals_down, *reg_target_pn_);

    // 细尺度精修目标(0.5×体素): ICP 第 4 级终修用, 一次性构建(基于 SOR 后全分辨率 target)
    reg_fine_voxel_ = reg_voxel_ * 0.5f;
    reg_fine_pn_.reset(new pcl::PointCloud<pcl::PointNormal>);
    reg_fine_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    reg_fine_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    {
        pcl::VoxelGrid<pcl::PointXYZ> vgf;
        vgf.setInputCloud(target);
        vgf.setLeafSize(reg_fine_voxel_, reg_fine_voxel_, reg_fine_voxel_);
        vgf.filter(*reg_fine_cloud_);
        pcl::PointCloud<pcl::Normal>::Ptr normals_fine(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nef;
        nef.setInputCloud(reg_fine_cloud_);
        nef.setRadiusSearch(reg_fine_voxel_ * 2.0f);
        nef.compute(*normals_fine);
        pcl::concatenateFields(*reg_fine_cloud_, *normals_fine, *reg_fine_pn_);
        // 剔除法线 NaN 的点(半径搜索失败的点), 保持 fine_cloud_/fine_pn_ 逐点同序
        pcl::PointCloud<pcl::PointXYZ> kept_xyz;
        pcl::PointCloud<pcl::PointNormal> kept_pn;
        kept_xyz.reserve(reg_fine_pn_->size());
        kept_pn.reserve(reg_fine_pn_->size());
        for (size_t i = 0; i < reg_fine_pn_->size(); ++i)
        {
            const pcl::PointNormal &pn = (*reg_fine_pn_)[i];
            if (std::isfinite(pn.normal_x) && std::isfinite(pn.normal_y) && std::isfinite(pn.normal_z))
            {
                kept_pn.push_back(pn);
                kept_xyz.push_back((*reg_fine_cloud_)[i]);
            }
        }
        *reg_fine_cloud_ = kept_xyz;
        *reg_fine_pn_ = kept_pn;
        if (!reg_fine_cloud_->empty())
            reg_fine_kdtree_->setInputCloud(reg_fine_cloud_);
    }
    LOG(ERROR) << "register: model init fine target, pts=" << reg_fine_cloud_->size()
               << ", voxel=" << reg_fine_voxel_ << ", " << elapsed_s() << "s";

    // 模型 PCA 主轴（粗对齐候选的模型侧基准, O(n) 无 KNN）:
    // u=最大方差方向(长边), n=最小方差方向(法线, 朝上 +z), v=n×u 保证 (u,v,n) 右手系
    {
        Eigen::Vector4f c4;
        pcl::compute3DCentroid(*reg_target_down_, c4);
        Eigen::Matrix3f cov;
        pcl::computeCovarianceMatrix(*reg_target_down_, c4, cov);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
        Eigen::Vector3f n_m = es.eigenvectors().col(0); // 特征值升序, col(0)=最小
        Eigen::Vector3f u_m = es.eigenvectors().col(2); // col(2)=最大
        if (n_m.z() < 0)
            n_m = -n_m; // 模型以相机俯视采集, 顶面法线在模型系下朝 +z
        reg_model_axes_.col(0) = u_m;
        reg_model_axes_.col(1) = n_m.cross(u_m);
        reg_model_axes_.col(2) = n_m;
        reg_model_centroid_ = c4.head<3>();
        // 模型沿长/短边方向跨度（源点云尺寸一致性预检的基准）
        {
            float a_min = 1e9f, a_max = -1e9f, b_min = 1e9f, b_max = -1e9f;
            for (const auto &p : reg_target_down_->points)
            {
                const Eigen::Vector3f d = p.getVector3fMap() - reg_model_centroid_;
                const float a = d.dot(u_m);
                const float b = d.dot(reg_model_axes_.col(1));
                a_min = std::min(a_min, a);
                a_max = std::max(a_max, a);
                b_min = std::min(b_min, b);
                b_max = std::max(b_max, b);
            }
            const float la = a_max - a_min, lb = b_max - b_min;
            reg_model_extents_ = (la >= lb) ? Eigen::Vector2f(la, lb) : Eigen::Vector2f(lb, la);
            LOG(ERROR) << "register: model extents (long, short) = (" << reg_model_extents_.x()
                       << ", " << reg_model_extents_.y() << ") m";
        }
    }

    reg_kdtree_->setInputCloud(reg_target_down_);
    reg_model_ready_ = true;
    LOG(ERROR) << "register: model init [4/4] icp target ready, " << elapsed_s() << "s";
    return true;
}

float ObjectRegionDetection::registrationFitness(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                                                 const Eigen::Matrix4f &transform, float max_dist, float *rmse)
{
    if (!reg_kdtree_ || !reg_kdtree_->getInputCloud() || source->empty())
    {
        if (rmse)
            *rmse = 0.f;
        return 0.f;
    }
    pcl::PointCloud<pcl::PointXYZ> tmp;
    pcl::transformPointCloud(*source, tmp, transform);
    const float max_sq = max_dist * max_dist;
    const int n_total = static_cast<int>(tmp.size());
    int inliers = 0, n = 0;
    double sum_sq = 0.0;
    // KD 树只读 NN 查询线程安全, OMP 并行归约（num_threads 强制, 防 RKNN 改全局 OMP 线程数）
    const int nthreads = std::min(omp_get_num_procs(), 8);
#pragma omp parallel num_threads(nthreads)
    {
        pcl::Indices idx(1);
        std::vector<float> dist(1);
        int l_in = 0, l_n = 0;
        double l_sq = 0.0;
#pragma omp for nowait
        for (int i = 0; i < n_total; ++i)
        {
            const auto &p = tmp[i];
            if (!pcl::isFinite(p))
                continue;
            ++l_n;
            if (reg_kdtree_->nearestKSearch(p, 1, idx, dist) > 0 && dist[0] <= max_sq)
            {
                ++l_in;
                l_sq += dist[0]; // FLANN 返回的是平方距离
            }
        }
#pragma omp critical
        {
            inliers += l_in;
            n += l_n;
            sum_sq += l_sq;
        }
    }
    if (rmse)
        *rmse = (inliers > 0) ? std::sqrt(sum_sq / inliers) : 0.f;
    return (n > 0) ? static_cast<float>(inliers) / static_cast<float>(n) : 0.f;
}

// 自研 OMP point-to-plane ICP（替代 pcl::IterativeClosestPoint: PCL 的对应点搜索单线程,
// 且 1.12 无 OMP 版; KD 树只读 NN 查询线程安全, 用 OpenMP 并行加速核心瓶颈）。
// 线性化增量 [w; t]: 残差 r = n·(p' + w×p' + t - q), 法方程 A=[(p'×n)ᵀ nᵀ], b=n·(q-p')
// 三级对应阈值 {10,2,0.5}×体素 × {15,8,8} 次迭代 + 第 4 级细尺度终修, 增量足够小提前收敛
bool ObjectRegionDetection::icpPointToPlaneOMP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                                               const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_fine,
                                               const Eigen::Matrix4f &init_T,
                                               Eigen::Matrix4f &out_T)
{
    const int n_src = static_cast<int>(source->size());
    const auto &tgt = *reg_target_down_;
    const auto &tgt_pn = *reg_target_pn_;
    Eigen::Matrix4f T = init_T;
    const float scales[3] = {10.0f, 2.0f, 0.5f};
    const int iters[3] = {15, 8, 8};
    std::vector<int> nn(n_src, -1); // 每源点的目标最近邻下标(-1=超出阈值)

    for (int stage = 0; stage < 3; ++stage)
    {
        const float max_d = reg_voxel_ * scales[stage];
        const float max_d2 = max_d * max_d;
        for (int it = 0; it < iters[stage]; ++it)
        {
            const Eigen::Matrix3f R = T.topLeftCorner<3, 3>();
            const Eigen::Vector3f t = T.block<3, 1>(0, 3);
            // 1) 并行最近邻查询（源点先按当前 T 变换）。
            //    idx/dist 在 parallel 块内、for 外声明——每线程仅构造一次,
            //    避免循环内堆分配(多线程下 malloc 锁竞争显著)。
            //    num_threads 子句强制线程数——优先级高于 omp_set_num_threads:
            //    RKNN 运行时初始化时会把进程 OMP 线程数改为 1, 若不强制,
            //    这里的并行会静默退化为单线程(ICP 从几十 ms 拖到秒级)
            const int nthreads = std::min(omp_get_num_procs(), 8);
#pragma omp parallel num_threads(nthreads)
            {
                pcl::Indices idx(1);
                std::vector<float> dist(1);
#pragma omp for
                for (int i = 0; i < n_src; ++i)
                {
                    Eigen::Vector3f q = R * (*source)[i].getVector3fMap() + t;
                    pcl::PointXYZ qp;
                    qp.getVector3fMap() = q;
                    nn[i] = (reg_kdtree_->nearestKSearch(qp, 1, idx, dist) > 0 && dist[0] <= max_d2) ? idx[0] : -1;
                }
            }
            // 2) 法方程累加（2106 点串行累加仅几十 µs, 不必并行）
            Eigen::Matrix<float, 6, 6> ATA = Eigen::Matrix<float, 6, 6>::Zero();
            Eigen::Matrix<float, 6, 1> ATb = Eigen::Matrix<float, 6, 1>::Zero();
            int pairs = 0;
            for (int i = 0; i < n_src; ++i)
            {
                const int j = nn[i];
                if (j < 0)
                    continue;
                const Eigen::Vector3f ps = R * (*source)[i].getVector3fMap() + t;
                const Eigen::Vector3f qt = tgt[j].getVector3fMap();
                const Eigen::Vector3f nt = tgt_pn[j].getNormalVector3fMap();
                Eigen::Matrix<float, 6, 1> a;
                a << ps.cross(nt), nt;
                ATA += a * a.transpose();
                ATb += a * (nt.dot(qt - ps));
                ++pairs;
            }
            if (pairs < 30) // 对应点过少, 位姿不可解
                return false;
            // 3) 解增量并左乘更新: T <- ΔT * T
            Eigen::Matrix<float, 6, 1> x = ATA.ldlt().solve(ATb);
            if (!x.allFinite())
                return false;
            // Z 轴约束(仅粗级): 只允许绕 Z 轴旋转(yaw), 禁止 roll/pitch。
            // 粗级防大角度翻面收敛到错误局部极小(tray 偶发匹配错的根因);
            // pitch/roll 残差由第 4 级终修(解除约束)修正。
            x[0] = 0.f; // wx = 0 (no roll)
            x[1] = 0.f; // wy = 0 (no pitch)
            const Eigen::Vector3f w = x.head<3>();
            const Eigen::Vector3f dt = x.tail<3>();
            Eigen::Matrix4f dT = Eigen::Matrix4f::Identity();
            const float wn = w.norm();
            if (wn > 1e-12f)
                dT.topLeftCorner<3, 3>() = Eigen::AngleAxisf(wn, w / wn).toRotationMatrix();
            dT.block<3, 1>(0, 3) = dt;
            T = dT * T;
            // 收敛判据按级放宽(1e-6=1µm 对毫米级噪声数据过苛, 会永远跑满迭代):
            // 粗级 0.5mm / 中 0.1mm / 细 0.05mm 增量即视为本级收敛
            const float conv = (stage == 0) ? 5e-4f : (stage == 1 ? 1e-4f : 5e-5f);
            if (wn < conv && dt.norm() < conv)
                break; // 已收敛, 跳出本级
        }
    }

    // ---- 第 4 级: 细尺度终修(0.5×体素源/目标, 阈值 1×细体素, 解除 Z 约束) ----
    // 粗三级 Z 约束冻结了 PCA 粗对齐引入的 pitch/roll 残差; 终修解除约束,
    // 允许小量 pitch/roll 修正(小阈值不会触发大角度翻面)。
    // 细目标未就绪/对应点过少/解非有限时保留粗级结果, 不判失败
    if (source_fine && !source_fine->empty() && reg_fine_kdtree_ && reg_fine_kdtree_->getInputCloud())
    {
        const int n_fn = static_cast<int>(source_fine->size());
        const auto &ftgt = *reg_fine_cloud_;
        const auto &ftgt_pn = *reg_fine_pn_;
        std::vector<int> fnn(n_fn, -1);
        const float max_d = reg_fine_voxel_;
        const float max_d2 = max_d * max_d;
        for (int it = 0; it < 10; ++it)
        {
            const Eigen::Matrix3f R = T.topLeftCorner<3, 3>();
            const Eigen::Vector3f t = T.block<3, 1>(0, 3);
            const int nthreads = std::min(omp_get_num_procs(), 8);
#pragma omp parallel num_threads(nthreads)
            {
                pcl::Indices idx(1);
                std::vector<float> dist(1);
#pragma omp for
                for (int i = 0; i < n_fn; ++i)
                {
                    Eigen::Vector3f q = R * (*source_fine)[i].getVector3fMap() + t;
                    pcl::PointXYZ qp;
                    qp.getVector3fMap() = q;
                    fnn[i] = (reg_fine_kdtree_->nearestKSearch(qp, 1, idx, dist) > 0 && dist[0] <= max_d2) ? idx[0] : -1;
                }
            }
            Eigen::Matrix<float, 6, 6> ATA = Eigen::Matrix<float, 6, 6>::Zero();
            Eigen::Matrix<float, 6, 1> ATb = Eigen::Matrix<float, 6, 1>::Zero();
            int pairs = 0;
            for (int i = 0; i < n_fn; ++i)
            {
                const int j = fnn[i];
                if (j < 0)
                    continue;
                const Eigen::Vector3f ps = R * (*source_fine)[i].getVector3fMap() + t;
                const Eigen::Vector3f qt = ftgt[j].getVector3fMap();
                const Eigen::Vector3f nt = ftgt_pn[j].getNormalVector3fMap();
                Eigen::Matrix<float, 6, 1> a;
                a << ps.cross(nt), nt;
                ATA += a * a.transpose();
                ATb += a * (nt.dot(qt - ps));
                ++pairs;
            }
            if (pairs < 30)
                break; // 细级对应点过少, 保留粗级结果
            Eigen::Matrix<float, 6, 1> x = ATA.ldlt().solve(ATb);
            if (!x.allFinite())
                break;
            const Eigen::Vector3f w = x.head<3>();
            const Eigen::Vector3f dt = x.tail<3>();
            Eigen::Matrix4f dT = Eigen::Matrix4f::Identity();
            const float wn = w.norm();
            if (wn > 1e-12f)
                dT.topLeftCorner<3, 3>() = Eigen::AngleAxisf(wn, w / wn).toRotationMatrix();
            dT.block<3, 1>(0, 3) = dt;
            T = dT * T;
            if (wn < 5e-5f && dt.norm() < 5e-5f)
                break;
        }
    }
    out_T = T;
    return T.allFinite();
}

bool ObjectRegionDetection::registerAndEstimatePose6D(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                      Eigen::Vector3f &position,
                                                      Eigen::Vector3f &euler_rpy)
{
    position.setZero();
    euler_rpy.setZero();
    if (!initRegistrationModel())
        return false;

    const int min_pts = 300;
    const int max_down_pts = 8000; // 对应 downsample_source 的 FPS 抽稀上限（PCL 1.12 无 FPS, 用随机均匀采样替代）
    const float voxel = reg_voxel_;
    const auto t_begin = std::chrono::steady_clock::now();

    // ---- 1. 源点云清理: 去掉 (0,0,0) 填充点与 NaN（extractMaskedCloud 的 mask 外填充） ----
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &p : *cloud)
        if (pcl::isFinite(p) && (std::abs(p.x) > 1e-6f || std::abs(p.y) > 1e-6f || std::abs(p.z) > 1e-6f))
            source->push_back(p);
    if (static_cast<int>(source->size()) < min_pts)
    {
        LOG(ERROR) << "register: source pts too few: " << source->size();
        return false;
    }

    // ---- 2. 统计离群剔除 (nb_neighbors=20, std_ratio=2.0) ----
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(source);
        sor.setMeanK(20);
        sor.setStddevMulThresh(2.0);
        sor.filter(*filtered);
        if (static_cast<int>(filtered->size()) >= min_pts)
            source = filtered;
    }
    LOG(ERROR) << "register: source outlier removed, pts=" << source->size()
               << " (omp max_threads=" << omp_get_max_threads() << ", procs=" << omp_get_num_procs() << ")";

    // ---- 3. 体素下采样（超上限时随机抽稀），后续配准均在下采样点云上进行 ----
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_down(new pcl::PointCloud<pcl::PointXYZ>);
    {
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(source);
        vg.setLeafSize(voxel, voxel, voxel);
        vg.filter(*source_down);
        if (static_cast<int>(source_down->size()) > max_down_pts)
        {
            pcl::RandomSample<pcl::PointXYZ> rs;
            rs.setInputCloud(source_down);
            rs.setSample(max_down_pts);
            pcl::PointCloud<pcl::PointXYZ> thinned;
            rs.filter(thinned);
            *source_down = thinned;
        }
    }
    if (static_cast<int>(source_down->size()) < 100)
    {
        LOG(ERROR) << "register: source downsample too few: " << source_down->size();
        return false;
    }
    LOG(ERROR) << "register: source downsampled, pts=" << source_down->size();

    // ---- 3b. 细尺度源点云(0.5×体素), 供 ICP 第 4 级终修 ----
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_fine(new pcl::PointCloud<pcl::PointXYZ>);
    {
        pcl::VoxelGrid<pcl::PointXYZ> vgf;
        vgf.setInputCloud(source);
        vgf.setLeafSize(voxel * 0.5f, voxel * 0.5f, voxel * 0.5f);
        vgf.filter(*source_fine);
        if (static_cast<int>(source_fine->size()) > 20000) // 上限防止终修 NN 查询耗时失控
        {
            pcl::RandomSample<pcl::PointXYZ> rs;
            rs.setInputCloud(source_fine);
            rs.setSample(20000);
            pcl::PointCloud<pcl::PointXYZ> thinned;
            rs.filter(thinned);
            *source_fine = thinned;
        }
    }
    LOG(ERROR) << "register: source fine pts=" << source_fine->size();

    // ---- 4. PCA 粗对齐候选（毫秒级, 替代 FPFH+RANSAC 全局配准） ----
    // 料盘≈平面矩形: 源/模型的 PCA 主轴天然对应, 特征值排序消除轴交换歧义,
    // 仅剩 4 个保定向候选(绕法线 180° × 绕长/短边 180°)。每个候选一次 fitness
    // 预评分(O(n) NN), 取前二进入 ICP 精配准。若料盘顶面完全无特征, 180° 候选
    // 几何等价、评分并列——这是对称物体的固有歧义, 与全局配准算法无关。
    Eigen::Matrix3f Us;
    Eigen::Vector3f c_s;
    {
        Eigen::Vector4f c4;
        pcl::compute3DCentroid(*source_down, c4);
        c_s = c4.head<3>();
        Eigen::Matrix3f cov;
        pcl::computeCovarianceMatrix(*source_down, c4, cov);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
        Eigen::Vector3f n_s = es.eigenvectors().col(0); // 最小特征值 -> 法线
        Eigen::Vector3f u_s = es.eigenvectors().col(2); // 最大特征值 -> 长边方向
        if (n_s.z() < 0)
            n_s = -n_s; // 相机俯视, 顶面法线朝上(+z); 近竖直面时符号不稳, 由候选评分兜底
        Us.col(0) = u_s;
        Us.col(1) = n_s.cross(u_s); // (u,v,n) 右手系
        Us.col(2) = n_s;
    }

    // ---- 4b. 源尺寸一致性预检 ----
    // 模型是平面矩形: point-to-plane ICP 沿平面滑动(2 平移+1 旋转)不受约束,
    // 任何远端小平面块都能"贴"上模型平面得到高 fitness(单向度量的固有缺陷)。
    // 先比较源/模型沿长/短边方向的跨度: 明显偏小=残缺/远处小块, 偏大=含背景,
    // 都直接拒绝, 不进入候选评分与 ICP。
    {
        float a_min = 1e9f, a_max = -1e9f, b_min = 1e9f, b_max = -1e9f;
        for (const auto &p : source_down->points)
        {
            const Eigen::Vector3f d = p.getVector3fMap() - c_s;
            const float a = d.dot(Us.col(0));
            const float b = d.dot(Us.col(1));
            a_min = std::min(a_min, a);
            a_max = std::max(a_max, a);
            b_min = std::min(b_min, b);
            b_max = std::max(b_max, b);
        }
        const float la = a_max - a_min, lb = b_max - b_min;
        const Eigen::Vector2f src_ext = (la >= lb) ? Eigen::Vector2f(la, lb) : Eigen::Vector2f(lb, la);
        const float r_long = src_ext.x() / reg_model_extents_.x();
        const float r_short = src_ext.y() / reg_model_extents_.y();
        LOG(ERROR) << "register: extent ratio (long, short) = (" << r_long << ", " << r_short << ")";
        // 下限容忍部分视野(70% 截断), 上限容忍少量边缘噪声
        if (r_long < 0.6f || r_short < 0.6f || r_long > 1.15f || r_short > 1.15f)
        {
            LOG(ERROR) << "register: source extent mismatch vs model, reject";
            return false;
        }
    }

    // R_k = Um * flip_k * Usᵀ, flip_k 为模型系三轴符号翻转(行列式保持 +1),
    // 平移把源质心对到模型质心(部分视野的质心偏移由 ICP 第一级大阈值吸收)
    // 法向约束: 模型与源点云法向均已朝上(Z>0, 见上文 n_s.z<0 翻转与 initRegistrationModel),
    // flips[2]/flips[3] 会翻转法向(绕长/短边 180°, 翻面), 必然匹配差, 跳过;
    // 仅保留 flips[0]/flips[1] (绕法线 180°, 法线仍朝上), 从候选层面杜绝翻面,
    // ICP 粗级 Z 约束(禁 roll/pitch)只需防止增量翻面, 法线 z 分量保持不变。
    Eigen::Matrix4f cands[2];
    float scores[2];
    const Eigen::Matrix3f &Um = reg_model_axes_;
    const Eigen::Matrix3f flips[2] = {
        Eigen::Matrix3f::Identity(),
        (Eigen::Matrix3f() << -1, 0, 0, 0, -1, 0, 0, 0, 1).finished()}; // 绕法线 180°
    for (int k = 0; k < 2; ++k)
    {
        Eigen::Matrix4f &T = cands[k];
        T.setIdentity();
        T.topLeftCorner<3, 3>() = Um * flips[k] * Us.transpose();
        T.block<3, 1>(0, 3) = reg_model_centroid_ - T.topLeftCorner<3, 3>() * c_s;
        scores[k] = registrationFitness(source_down, T, voxel * 1.5f);
    }
    int order[2] = {0, 1};
    std::sort(order, order + 2, [&](int a, int b) { return scores[a] > scores[b]; });
    LOG(ERROR) << "register: pca candidates fitness (z-constrained): " << scores[order[0]] << ", " << scores[order[1]];

    // ---- 5. ICP 精配准: 前二候选全评估, 综合分选优 ----
    // 不再"达标即止": 0.31 的平庸局部极小值会打败本可到 0.8 的正确候选。
    // 综合分 = fitness - rmse/阈值: fitness 只数内点个数是粗度量,
    // 同 fitness 下 RMSE 小者更贴合, 防边缘错配的高分候选胜出
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    float final_fitness = 0.f, final_rmse = 0.f;
    float best_score = -1e9f;
    for (int k = 0; k < 2; ++k)
    {
        Eigen::Matrix4f T_k;
        if (!icpPointToPlaneOMP(source_down, source_fine, cands[order[k]], T_k))
            continue;
        float rmse_k = 0.f;
        float fit_k = registrationFitness(source_down, T_k, voxel * 1.5f, &rmse_k);
        const float score_k = fit_k - rmse_k / (voxel * 1.5f);
        LOG(ERROR) << "register: icp cand#" << order[k] << ", fitness=" << fit_k
                   << ", rmse=" << rmse_k << ", score=" << score_k;
        if (score_k > best_score)
        {
            best_score = score_k;
            final_fitness = fit_k;
            final_rmse = rmse_k;
            T = T_k;
        }
    }
    if (final_fitness < 0.3f) // 1.5 倍体素阈值内不足 30% 点对齐, 视为配准失败
    {
        LOG(ERROR) << "register: icp fail, fitness=" << final_fitness;
        return false;
    }

    // ---- 5b. 反向覆盖度校验（模型->源, 补 fitness 单向度量的盲区） ----
    // fitness 只度量"源点贴近模型表面", 平面小块贴上大平面也能高分;
    // 覆盖度反过来问"模型面有多少被源点解释": 真实料盘观测(即使部分视野)
    // 应解释模型面的大半, 而残缺小块只会覆盖模型一角。
    float coverage = 0.f;
    {
        pcl::PointCloud<pcl::PointXYZ> aligned;
        pcl::transformPointCloud(*source_down, aligned, T);
        const float cov_d2 = (voxel * 1.5f) * (voxel * 1.5f);
        const int n_m = static_cast<int>(reg_target_down_->size());
        const int n_s = static_cast<int>(aligned.size());
        const auto &mp = reg_target_down_->points;
        const auto &sp = aligned.points;
        int covered = 0;
        const int nthreads = std::min(omp_get_num_procs(), 8);
#pragma omp parallel for reduction(+ : covered) num_threads(nthreads)
        for (int j = 0; j < n_m; ++j)
        {
            float best = cov_d2 + 1.f; // 略大于阈值, 仅命中才计
            for (int i = 0; i < n_s; ++i)
            {
                const float dx = mp[j].x - sp[i].x;
                const float dy = mp[j].y - sp[i].y;
                const float dz = mp[j].z - sp[i].z;
                const float d2 = dx * dx + dy * dy + dz * dz;
                if (d2 < best)
                    best = d2;
            }
            if (best <= cov_d2)
                ++covered;
        }
        coverage = static_cast<float>(covered) / static_cast<float>(n_m);
    }
    LOG(ERROR) << "register: coverage=" << coverage;
    if (coverage < 0.3f)
    {
        LOG(ERROR) << "register: coverage too low, reject (small patch on plane?)";
        return false;
    }

    // ---- 6. 6D 位姿 = ICP 变换(源->目标)的逆（目标->源）, 与 py/register.py 的 print_pose_6d(T_inv) 一致 ----
    Eigen::Matrix4f T_inv = T.inverse();
    position = T_inv.block<3, 1>(0, 3);
    // 旋转再正交化: ICP 多次增量左乘在 float 下累积正交性误差,
    // SVD 投影回 SO(3) (R=U·Vᵀ) 后再取欧拉角, 避免姿态角偏差
    {
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(T_inv.block<3, 3>(0, 0),
                                             Eigen::ComputeFullU | Eigen::ComputeFullV);
        T_inv.block<3, 3>(0, 0) = svd.matrixU() * svd.matrixV().transpose();
    }
    Eigen::Matrix3f R = T_inv.block<3, 3>(0, 0); // ZYX 欧拉角: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    float roll = std::atan2(R(2, 1), R(2, 2));
    float pitch = std::asin(std::min(1.f, std::max(-1.f, -R(2, 0))));
    float yaw = std::atan2(R(1, 0), R(0, 0));
    euler_rpy = Eigen::Vector3f(roll, pitch, yaw);

    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_begin).count();
    LOG(ERROR) << "register ok: fitness=" << final_fitness << ", coverage=" << coverage
               << ", rmse=" << final_rmse
               << ", pos=(" << position.x() << ", " << position.y() << ", " << position.z() << ")"
               << ", rpy_deg=(" << roll * 180.f / M_PI << ", " << pitch * 180.f / M_PI
               << ", " << yaw * 180.f / M_PI << "), time=" << elapsed << "s";

    if (is_debug_)
    {
        static std::atomic<int> reg_dbg_seq{0};
        pcl::PointCloud<pcl::PointXYZ> aligned;
        pcl::transformPointCloud(*source_down, aligned, T);
        std::string name = debug_session_dir_ + "reg_" + std::to_string(reg_dbg_seq.fetch_add(1)) + ".ply";
        saveDebugPLYXYZ(name, aligned);
        LOG(ERROR) << "register debug: " << name;
    }
    return true;
}

// ================= Rack 点云配准 6D 位姿估计 =================
// 仿照 box 版本，使用 rack_bs_m.ply 模板，算法内部暂与 box 一致，
// 后续可针对 rack 几何特征做针对性优化（如非平面结构、不同 PCA 策略等）。
bool ObjectRegionDetection::initRackRegistrationModel()
{
    std::lock_guard<std::mutex> lock(rack_model_mutex_);
    if (rack_model_ready_)
        return true;

    rack_target_pn_.reset(new pcl::PointCloud<pcl::PointNormal>);
    rack_target_down_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    rack_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);

    const auto t0 = std::chrono::steady_clock::now();
    auto elapsed_s = [&]() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    };
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    std::string load_via = "custom";
    Eigen::Vector4f minp = Eigen::Vector4f::Zero(), maxp = Eigen::Vector4f::Zero();
    if (!loadPLYBinaryXYZ(rack_model_path_, *target) || target->empty())
    {
        target->clear();
        if (pcl::io::loadPLYFile(rack_model_path_, *target) < 0 || target->empty())
        {
            LOG(ERROR) << "rack register: load model fail: " << rack_model_path_;
            return false;
        }
        load_via = "pcl";
    }
    else
    {
        pcl::getMinMax3D(*target, minp, maxp);
    }
    if ((maxp - minp).head<3>().norm() < 1e-6f && load_via == "custom")
    {
        target->clear();
        if (pcl::io::loadPLYFile(rack_model_path_, *target) >= 0 && !target->empty())
            load_via = "pcl";
    }
    LOG(ERROR) << "rack register: model init [1/4] loaded(" << load_via << "), pts=" << target->size()
               << ", " << elapsed_s() << "s";
    if (target->size() > 20)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(target);
        sor.setMeanK(20);
        sor.setStddevMulThresh(2.0);
        sor.filter(*filtered);
        if (!filtered->empty())
            target = filtered;
    }
    LOG(ERROR) << "rack register: model init [2/4] outlier removed, pts=" << target->size() << ", " << elapsed_s() << "s";

    pcl::getMinMax3D(*target, minp, maxp);
    rack_voxel_ = (maxp - minp).head<3>().norm() * 0.02f;
    if (rack_voxel_ < 1e-4f)
    {
        LOG(ERROR) << "rack register: model degenerate, voxel=" << rack_voxel_;
        return false;
    }

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(target);
    vg.setLeafSize(rack_voxel_, rack_voxel_, rack_voxel_);
    vg.filter(*rack_target_down_);
    if (rack_target_down_->empty())
    {
        LOG(ERROR) << "rack register: model downsample empty";
        return false;
    }
    LOG(ERROR) << "rack register: model init [3/4] downsampled, pts=" << rack_target_down_->size()
               << ", voxel=" << rack_voxel_ << ", " << elapsed_s() << "s";

    pcl::PointCloud<pcl::Normal>::Ptr normals_down(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(rack_target_down_);
    ne.setRadiusSearch(rack_voxel_ * 2.0f);
    ne.compute(*normals_down);
    pcl::concatenateFields(*rack_target_down_, *normals_down, *rack_target_pn_);

    // 细尺度精修目标(0.5×体素): ICP 第 4 级终修用
    rack_fine_voxel_ = rack_voxel_ * 0.5f;
    rack_fine_pn_.reset(new pcl::PointCloud<pcl::PointNormal>);
    rack_fine_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    rack_fine_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    {
        pcl::VoxelGrid<pcl::PointXYZ> vgf;
        vgf.setInputCloud(target);
        vgf.setLeafSize(rack_fine_voxel_, rack_fine_voxel_, rack_fine_voxel_);
        vgf.filter(*rack_fine_cloud_);
        pcl::PointCloud<pcl::Normal>::Ptr normals_fine(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nef;
        nef.setInputCloud(rack_fine_cloud_);
        nef.setRadiusSearch(rack_fine_voxel_ * 2.0f);
        nef.compute(*normals_fine);
        pcl::concatenateFields(*rack_fine_cloud_, *normals_fine, *rack_fine_pn_);
        pcl::PointCloud<pcl::PointXYZ> kept_xyz;
        pcl::PointCloud<pcl::PointNormal> kept_pn;
        kept_xyz.reserve(rack_fine_pn_->size());
        kept_pn.reserve(rack_fine_pn_->size());
        for (size_t i = 0; i < rack_fine_pn_->size(); ++i)
        {
            const pcl::PointNormal &pn = (*rack_fine_pn_)[i];
            if (std::isfinite(pn.normal_x) && std::isfinite(pn.normal_y) && std::isfinite(pn.normal_z))
            {
                kept_pn.push_back(pn);
                kept_xyz.push_back((*rack_fine_cloud_)[i]);
            }
        }
        *rack_fine_cloud_ = kept_xyz;
        *rack_fine_pn_ = kept_pn;
        if (!rack_fine_cloud_->empty())
            rack_fine_kdtree_->setInputCloud(rack_fine_cloud_);
    }
    LOG(ERROR) << "rack register: model init fine target, pts=" << rack_fine_cloud_->size()
               << ", voxel=" << rack_fine_voxel_ << ", " << elapsed_s() << "s";

    {
        Eigen::Vector4f c4;
        pcl::compute3DCentroid(*rack_target_down_, c4);
        Eigen::Matrix3f cov;
        pcl::computeCovarianceMatrix(*rack_target_down_, c4, cov);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
        Eigen::Vector3f n_m = es.eigenvectors().col(0);
        Eigen::Vector3f u_m = es.eigenvectors().col(2);
        if (n_m.z() < 0)
            n_m = -n_m;
        rack_model_axes_.col(0) = u_m;
        rack_model_axes_.col(1) = n_m.cross(u_m);
        rack_model_axes_.col(2) = n_m;
        rack_model_centroid_ = c4.head<3>();
        {
            float a_min = 1e9f, a_max = -1e9f, b_min = 1e9f, b_max = -1e9f;
            for (const auto &p : rack_target_down_->points)
            {
                const Eigen::Vector3f d = p.getVector3fMap() - rack_model_centroid_;
                const float a = d.dot(u_m);
                const float b = d.dot(rack_model_axes_.col(1));
                a_min = std::min(a_min, a);
                a_max = std::max(a_max, a);
                b_min = std::min(b_min, b);
                b_max = std::max(b_max, b);
            }
            const float la = a_max - a_min, lb = b_max - b_min;
            rack_model_extents_ = (la >= lb) ? Eigen::Vector2f(la, lb) : Eigen::Vector2f(lb, la);
            LOG(ERROR) << "rack register: model extents (long, short) = (" << rack_model_extents_.x()
                       << ", " << rack_model_extents_.y() << ") m";
        }
    }

    rack_kdtree_->setInputCloud(rack_target_down_);
    rack_model_ready_ = true;
    LOG(ERROR) << "rack register: model init [4/4] icp target ready, " << elapsed_s() << "s";
    return true;
}

float ObjectRegionDetection::rackRegistrationFitness(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                                                     const Eigen::Matrix4f &transform, float max_dist, float *rmse)
{
    if (!rack_kdtree_ || !rack_kdtree_->getInputCloud() || source->empty())
    {
        if (rmse)
            *rmse = 0.f;
        return 0.f;
    }
    pcl::PointCloud<pcl::PointXYZ> tmp;
    pcl::transformPointCloud(*source, tmp, transform);
    const float max_sq = max_dist * max_dist;
    const int n_total = static_cast<int>(tmp.size());
    int inliers = 0, n = 0;
    double sum_sq = 0.0;
    const int nthreads = std::min(omp_get_num_procs(), 8);
#pragma omp parallel num_threads(nthreads)
    {
        pcl::Indices idx(1);
        std::vector<float> dist(1);
        int l_in = 0, l_n = 0;
        double l_sq = 0.0;
#pragma omp for nowait
        for (int i = 0; i < n_total; ++i)
        {
            const auto &p = tmp[i];
            if (!pcl::isFinite(p))
                continue;
            ++l_n;
            if (rack_kdtree_->nearestKSearch(p, 1, idx, dist) > 0 && dist[0] <= max_sq)
            {
                ++l_in;
                l_sq += dist[0];
            }
        }
#pragma omp critical
        {
            inliers += l_in;
            n += l_n;
            sum_sq += l_sq;
        }
    }
    if (rmse)
        *rmse = (inliers > 0) ? std::sqrt(sum_sq / inliers) : 0.f;
    return (n > 0) ? static_cast<float>(inliers) / static_cast<float>(n) : 0.f;
}

bool ObjectRegionDetection::icpPointToPlaneOMPRack(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_fine,
                                                   const Eigen::Matrix4f &init_T,
                                                   Eigen::Matrix4f &out_T)
{
    const int n_src = static_cast<int>(source->size());
    const auto &tgt = *rack_target_down_;
    const auto &tgt_pn = *rack_target_pn_;
    Eigen::Matrix4f T = init_T;
    const float scales[3] = {10.0f, 2.0f, 0.5f};
    const int iters[3] = {15, 8, 8};
    std::vector<int> nn(n_src, -1);

    for (int stage = 0; stage < 3; ++stage)
    {
        const float max_d = rack_voxel_ * scales[stage];
        const float max_d2 = max_d * max_d;
        for (int it = 0; it < iters[stage]; ++it)
        {
            const Eigen::Matrix3f R = T.topLeftCorner<3, 3>();
            const Eigen::Vector3f t = T.block<3, 1>(0, 3);
            const int nthreads = std::min(omp_get_num_procs(), 8);
#pragma omp parallel num_threads(nthreads)
            {
                pcl::Indices idx(1);
                std::vector<float> dist(1);
#pragma omp for
                for (int i = 0; i < n_src; ++i)
                {
                    Eigen::Vector3f q = R * (*source)[i].getVector3fMap() + t;
                    pcl::PointXYZ qp;
                    qp.getVector3fMap() = q;
                    nn[i] = (rack_kdtree_->nearestKSearch(qp, 1, idx, dist) > 0 && dist[0] <= max_d2) ? idx[0] : -1;
                }
            }
            Eigen::Matrix<float, 6, 6> ATA = Eigen::Matrix<float, 6, 6>::Zero();
            Eigen::Matrix<float, 6, 1> ATb = Eigen::Matrix<float, 6, 1>::Zero();
            int pairs = 0;
            for (int i = 0; i < n_src; ++i)
            {
                const int j = nn[i];
                if (j < 0)
                    continue;
                const Eigen::Vector3f ps = R * (*source)[i].getVector3fMap() + t;
                const Eigen::Vector3f qt = tgt[j].getVector3fMap();
                const Eigen::Vector3f nt = tgt_pn[j].getNormalVector3fMap();
                Eigen::Matrix<float, 6, 1> a;
                a << ps.cross(nt), nt;
                ATA += a * a.transpose();
                ATb += a * (nt.dot(qt - ps));
                ++pairs;
            }
            if (pairs < 30)
                return false;
            Eigen::Matrix<float, 6, 1> x = ATA.ldlt().solve(ATb);
            if (!x.allFinite())
                return false;
            // Z轴约束(仅粗级): 只允许绕Z轴旋转(yaw), 禁止roll/pitch
            // 粗级防大角度翻面; pitch/roll 残差由第 4 级终修(解除约束)修正
            x[0] = 0.f;  // wx = 0 (no roll)
            x[1] = 0.f;  // wy = 0 (no pitch)
            const Eigen::Vector3f w = x.head<3>();
            const Eigen::Vector3f dt = x.tail<3>();
            Eigen::Matrix4f dT = Eigen::Matrix4f::Identity();
            const float wn = w.norm();
            if (wn > 1e-12f)
                dT.topLeftCorner<3, 3>() = Eigen::AngleAxisf(wn, w / wn).toRotationMatrix();
            dT.block<3, 1>(0, 3) = dt;
            T = dT * T;
            const float conv = (stage == 0) ? 5e-4f : (stage == 1 ? 1e-4f : 5e-5f);
            if (wn < conv && dt.norm() < conv)
                break;
        }
    }

    // ---- 第 4 级: 细尺度终修(0.5×体素源/目标, 阈值 1×细体素, 解除Z约束) ----
    // 粗三级 Z 约束冻结了 PCA 粗对齐引入的 pitch/roll 残差; 终修解除约束,
    // 允许小量 pitch/roll 修正(小阈值不会触发大角度翻面)。细目标未就绪/
    // 对应点过少/解非有限时保留粗级结果, 不判失败
    if (source_fine && !source_fine->empty() && rack_fine_kdtree_ && rack_fine_kdtree_->getInputCloud())
    {
        const int n_fn = static_cast<int>(source_fine->size());
        const auto &ftgt = *rack_fine_cloud_;
        const auto &ftgt_pn = *rack_fine_pn_;
        std::vector<int> fnn(n_fn, -1);
        const float max_d = rack_fine_voxel_;
        const float max_d2 = max_d * max_d;
        for (int it = 0; it < 10; ++it)
        {
            const Eigen::Matrix3f R = T.topLeftCorner<3, 3>();
            const Eigen::Vector3f t = T.block<3, 1>(0, 3);
            const int nthreads = std::min(omp_get_num_procs(), 8);
#pragma omp parallel num_threads(nthreads)
            {
                pcl::Indices idx(1);
                std::vector<float> dist(1);
#pragma omp for
                for (int i = 0; i < n_fn; ++i)
                {
                    Eigen::Vector3f q = R * (*source_fine)[i].getVector3fMap() + t;
                    pcl::PointXYZ qp;
                    qp.getVector3fMap() = q;
                    fnn[i] = (rack_fine_kdtree_->nearestKSearch(qp, 1, idx, dist) > 0 && dist[0] <= max_d2) ? idx[0] : -1;
                }
            }
            Eigen::Matrix<float, 6, 6> ATA = Eigen::Matrix<float, 6, 6>::Zero();
            Eigen::Matrix<float, 6, 1> ATb = Eigen::Matrix<float, 6, 1>::Zero();
            int pairs = 0;
            for (int i = 0; i < n_fn; ++i)
            {
                const int j = fnn[i];
                if (j < 0)
                    continue;
                const Eigen::Vector3f ps = R * (*source_fine)[i].getVector3fMap() + t;
                const Eigen::Vector3f qt = ftgt[j].getVector3fMap();
                const Eigen::Vector3f nt = ftgt_pn[j].getNormalVector3fMap();
                Eigen::Matrix<float, 6, 1> a;
                a << ps.cross(nt), nt;
                ATA += a * a.transpose();
                ATb += a * (nt.dot(qt - ps));
                ++pairs;
            }
            if (pairs < 30)
                break;
            Eigen::Matrix<float, 6, 1> x = ATA.ldlt().solve(ATb);
            if (!x.allFinite())
                break;
            // 终修级解除 Z 约束: 允许全 6-DOF 微调, 修正 pitch/roll 残差
            const Eigen::Vector3f w = x.head<3>();
            const Eigen::Vector3f dt = x.tail<3>();
            Eigen::Matrix4f dT = Eigen::Matrix4f::Identity();
            const float wn = w.norm();
            if (wn > 1e-12f)
                dT.topLeftCorner<3, 3>() = Eigen::AngleAxisf(wn, w / wn).toRotationMatrix();
            dT.block<3, 1>(0, 3) = dt;
            T = dT * T;
            if (wn < 5e-5f && dt.norm() < 5e-5f)
                break;
        }
    }
    out_T = T;
    return T.allFinite();
}

bool ObjectRegionDetection::registerAndEstimatePose6DRack(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                          Eigen::Vector3f &position,
                                                          Eigen::Vector3f &euler_rpy)
{
    position.setZero();
    euler_rpy.setZero();
    if (!initRackRegistrationModel())
        return false;

    const int min_pts = 300;
    const int max_down_pts = 8000;
    const float voxel = rack_voxel_;
    const auto t_begin = std::chrono::steady_clock::now();

    // ---- 1. 源点云清理 ----
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &p : *cloud)
        if (pcl::isFinite(p) && (std::abs(p.x) > 1e-6f || std::abs(p.y) > 1e-6f || std::abs(p.z) > 1e-6f))
            source->push_back(p);
    if (static_cast<int>(source->size()) < min_pts)
    {
        LOG(ERROR) << "rack register: source pts too few: " << source->size();
        return false;
    }

    // ---- 2. 统计离群剔除 ----
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(source);
        sor.setMeanK(20);
        sor.setStddevMulThresh(2.0);
        sor.filter(*filtered);
        if (static_cast<int>(filtered->size()) >= min_pts)
            source = filtered;
    }
    LOG(ERROR) << "rack register: source outlier removed, pts=" << source->size();

    // ---- 3. 体素下采样 ----
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_down(new pcl::PointCloud<pcl::PointXYZ>);
    {
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(source);
        vg.setLeafSize(voxel, voxel, voxel);
        vg.filter(*source_down);
        if (static_cast<int>(source_down->size()) > max_down_pts)
        {
            pcl::RandomSample<pcl::PointXYZ> rs;
            rs.setInputCloud(source_down);
            rs.setSample(max_down_pts);
            pcl::PointCloud<pcl::PointXYZ> thinned;
            rs.filter(thinned);
            *source_down = thinned;
        }
    }
    if (static_cast<int>(source_down->size()) < 100)
    {
        LOG(ERROR) << "rack register: source downsample too few: " << source_down->size();
        return false;
    }
    LOG(ERROR) << "rack register: source downsampled, pts=" << source_down->size();

    // ---- 3a. 细尺度源点云(0.5×体素), 供 ICP 第 4 级终修 ----
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_fine(new pcl::PointCloud<pcl::PointXYZ>);
    {
        pcl::VoxelGrid<pcl::PointXYZ> vgf;
        vgf.setInputCloud(source);
        vgf.setLeafSize(voxel * 0.5f, voxel * 0.5f, voxel * 0.5f);
        vgf.filter(*source_fine);
        if (static_cast<int>(source_fine->size()) > 20000)
        {
            pcl::RandomSample<pcl::PointXYZ> rs;
            rs.setInputCloud(source_fine);
            rs.setSample(20000);
            pcl::PointCloud<pcl::PointXYZ> thinned;
            rs.filter(thinned);
            *source_fine = thinned;
        }
    }
    LOG(ERROR) << "rack register: source fine pts=" << source_fine->size();

    // ---- 3b. 最大簇筛选（去除背景碎片） ----
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(source_down);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(voxel * 2.0f);
        ec.setMinClusterSize(50);
        ec.setMaxClusterSize(source_down->size() + 1);
        ec.setSearchMethod(tree);
        ec.setInputCloud(source_down);
        ec.extract(cluster_indices);
        if (cluster_indices.empty())
        {
            LOG(ERROR) << "rack register: no cluster found, skip";
            return false;
        }
        int best_idx = 0;
        size_t best_size = 0;
        for (size_t i = 0; i < cluster_indices.size(); ++i)
        {
            if (cluster_indices[i].indices.size() > best_size)
            {
                best_size = cluster_indices[i].indices.size();
                best_idx = i;
            }
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr largest(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(source_down);
        extract.setIndices(pcl::IndicesPtr(new std::vector<int>(cluster_indices[best_idx].indices.begin(),
                                                                cluster_indices[best_idx].indices.end())));
        extract.filter(*largest);
        LOG(ERROR) << "rack register: clusters=" << cluster_indices.size()
                   << ", largest=" << best_size << "/" << source_down->size();
        source_down = largest;
    }

    // ---- 4. PCA 粗对齐候选 ----
    Eigen::Matrix3f Us;
    Eigen::Vector3f c_s;
    {
        Eigen::Vector4f c4;
        pcl::compute3DCentroid(*source_down, c4);
        c_s = c4.head<3>();
        Eigen::Matrix3f cov;
        pcl::computeCovarianceMatrix(*source_down, c4, cov);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
        Eigen::Vector3f n_s = es.eigenvectors().col(0);
        Eigen::Vector3f u_s = es.eigenvectors().col(2);
        if (n_s.z() < 0)
            n_s = -n_s;
        Us.col(0) = u_s;
        Us.col(1) = n_s.cross(u_s);
        Us.col(2) = n_s;
    }

    // ---- 4b. 源尺寸一致性预检 ----
    {
        float a_min = 1e9f, a_max = -1e9f, b_min = 1e9f, b_max = -1e9f;
        for (const auto &p : source_down->points)
        {
            const Eigen::Vector3f d = p.getVector3fMap() - c_s;
            const float a = d.dot(Us.col(0));
            const float b = d.dot(Us.col(1));
            a_min = std::min(a_min, a);
            a_max = std::max(a_max, a);
            b_min = std::min(b_min, b);
            b_max = std::max(b_max, b);
        }
        const float la = a_max - a_min, lb = b_max - b_min;
        const Eigen::Vector2f src_ext = (la >= lb) ? Eigen::Vector2f(la, lb) : Eigen::Vector2f(lb, la);
        const float r_long = src_ext.x() / rack_model_extents_.x();
        const float r_short = src_ext.y() / rack_model_extents_.y();
        LOG(ERROR) << "rack register: extent ratio (long, short) = (" << r_long << ", " << r_short << ")";
        // rack 的 mask 不精确（常为全填充矩形），源点云含背景导致尺寸偏大
        // 放宽到 [0.3, 4.0]，后续 mask 优化后可收紧
        if (r_long < 0.3f || r_short < 0.3f || r_long > 4.0f || r_short > 4.0f)
        {
            LOG(ERROR) << "rack register: source extent mismatch vs model, reject";
            return false;
        }
    }

    // ---- 5. ICP 精配准 ----
    Eigen::Matrix4f cands[4];
    float scores[4];
    const Eigen::Matrix3f &Um = rack_model_axes_;
    const Eigen::Matrix3f flips[4] = {
        Eigen::Matrix3f::Identity(),
        (Eigen::Matrix3f() << -1, 0, 0, 0, -1, 0, 0, 0, 1).finished(),
        (Eigen::Matrix3f() << 1, 0, 0, 0, -1, 0, 0, 0, -1).finished(),
        (Eigen::Matrix3f() << -1, 0, 0, 0, 1, 0, 0, 0, -1).finished()};
    // 法向量约束: 模型与源点云法向均已朝上(Z>0)
    // flips[2] 和 flips[3] 会翻转法向(翻面), 必然匹配差, 跳过
    for (int k = 0; k < 2; ++k)
    {
        Eigen::Matrix4f &T = cands[k];
        T.setIdentity();
        T.topLeftCorner<3, 3>() = Um * flips[k] * Us.transpose();
        T.block<3, 1>(0, 3) = rack_model_centroid_ - T.topLeftCorner<3, 3>() * c_s;
        scores[k] = rackRegistrationFitness(source_down, T, voxel * 1.5f);
    }
    int order[2] = {0, 1};
    std::sort(order, order + 2, [&](int a, int b) { return scores[a] > scores[b]; });
    LOG(ERROR) << "rack register: pca candidates fitness (z-constrained): " << scores[order[0]] << ", " << scores[order[1]];

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    float final_fitness = 0.f, final_rmse = 0.f;
    float best_score = -1e9f;
    for (int k = 0; k < 2; ++k)
    {
        Eigen::Matrix4f T_k;
        if (!icpPointToPlaneOMPRack(source_down, source_fine, cands[order[k]], T_k))
            continue;
        float rmse_k = 0.f;
        float fit_k = rackRegistrationFitness(source_down, T_k, voxel * 1.5f, &rmse_k);
        const float score_k = fit_k - rmse_k / (voxel * 1.5f);
        LOG(ERROR) << "rack register: icp cand#" << order[k] << ", fitness=" << fit_k
                   << ", rmse=" << rmse_k << ", score=" << score_k;
        if (score_k > best_score)
        {
            best_score = score_k;
            final_fitness = fit_k;
            final_rmse = rmse_k;
            T = T_k;
        }
    }
    if (final_fitness < 0.3f)
    {
        LOG(ERROR) << "rack register: icp fail, fitness=" << final_fitness;
        return false;
    }

    // ---- 5b. 反向覆盖度校验 ----
    float coverage = 0.f;
    {
        pcl::PointCloud<pcl::PointXYZ> aligned;
        pcl::transformPointCloud(*source_down, aligned, T);
        const float cov_d2 = (voxel * 1.5f) * (voxel * 1.5f);
        const int n_m = static_cast<int>(rack_target_down_->size());
        const int n_s = static_cast<int>(aligned.size());
        const auto &mp = rack_target_down_->points;
        const auto &sp = aligned.points;
        int covered = 0;
        const int nthreads = std::min(omp_get_num_procs(), 8);
#pragma omp parallel for reduction(+ : covered) num_threads(nthreads)
        for (int j = 0; j < n_m; ++j)
        {
            float best = cov_d2 + 1.f;
            for (int i = 0; i < n_s; ++i)
            {
                const float dx = mp[j].x - sp[i].x;
                const float dy = mp[j].y - sp[i].y;
                const float dz = mp[j].z - sp[i].z;
                const float d2 = dx * dx + dy * dy + dz * dz;
                if (d2 < best)
                    best = d2;
            }
            if (best <= cov_d2)
                ++covered;
        }
        coverage = static_cast<float>(covered) / static_cast<float>(n_m);
    }
    LOG(ERROR) << "rack register: coverage=" << coverage;
    if (coverage < 0.3f)
    {
        LOG(ERROR) << "rack register: coverage too low, reject";
        return false;
    }

    // ---- 6. 6D 位姿 ----
    Eigen::Matrix4f T_inv = T.inverse();
    position = T_inv.block<3, 1>(0, 3);
    // 旋转再正交化: SVD 投影回 SO(3), 消除 float 增量累积的非正交误差
    {
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(T_inv.block<3, 3>(0, 0),
                                             Eigen::ComputeFullU | Eigen::ComputeFullV);
        T_inv.block<3, 3>(0, 0) = svd.matrixU() * svd.matrixV().transpose();
    }
    Eigen::Matrix3f R = T_inv.block<3, 3>(0, 0);
    float roll = std::atan2(R(2, 1), R(2, 2));
    float pitch = std::asin(std::min(1.f, std::max(-1.f, -R(2, 0))));
    float yaw = std::atan2(R(1, 0), R(0, 0));
    euler_rpy = Eigen::Vector3f(roll, pitch, yaw);

    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_begin).count();
    LOG(ERROR) << "rack register ok: fitness=" << final_fitness << ", coverage=" << coverage
               << ", rmse=" << final_rmse
               << ", pos=(" << position.x() << ", " << position.y() << ", " << position.z() << ")"
               << ", rpy_deg=(" << roll * 180.f / M_PI << ", " << pitch * 180.f / M_PI
               << ", " << yaw * 180.f / M_PI << "), time=" << elapsed << "s";

    if (is_debug_)
    {
        static std::atomic<int> rack_dbg_seq{0};
        pcl::PointCloud<pcl::PointXYZ> aligned;
        pcl::transformPointCloud(*source_down, aligned, T);
        std::string name = debug_session_dir_ + "rack_reg_" + std::to_string(rack_dbg_seq.fetch_add(1)) + ".ply";
        saveDebugPLYXYZ(name, aligned);
        LOG(ERROR) << "rack register debug: " << name;
    }
    return true;
}
// -2: 分割检测失败
// -3: 拟合平面失败
// -4: 空箱
int ObjectRegionDetection::DetectRegion(cv::Mat &img_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr &transformed_cloud, LocalizationInfo *localization_region, const std::string &target_obj_name)
{
    // 1、固定尺寸
    // 2、固定尺寸内寻找
    // 3、固定尺寸点云下采样寻找加速

    // is_debug_ 开启时每帧新建会话目录，本次调用的所有调试文件集中存放
    if (is_debug_)
    {
        static std::atomic<int> session_seq{0};
        std::time_t tt = std::time(nullptr);
        std::tm tmv{};
        localtime_r(&tt, &tmv);
        char ts[24];
        std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tmv);
        debug_session_dir_ = ply_save_dir_  + ts  + "_" + target_obj_name + "/";
        mkdir(ply_save_dir_.c_str(), 0755);       // 确保根目录存在（已存在则忽略）
        mkdir(debug_session_dir_.c_str(), 0755);  // 新建会话目录
        LOG(ERROR) << "debug session dir: " << debug_session_dir_;
    }

    // cv::imwrite("cropped.png", cropped);
    localization_region->output.clear();
    std::vector<OutputSeg> output;
    image_buffer_t src_image;
    cv::Mat resize_cropped = img_in.clone();
    
    cv::cvtColor(resize_cropped, resize_cropped, cv::COLOR_BGR2RGB);
    convert_mat_to_image_buffer(resize_cropped, src_image);
    object_detect_result_list od_results;

    LOG(ERROR) << "od_results.count: ";

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
            std::string name = debug_session_dir_ + std::to_string(i) + ".png";
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

            if (r.confidence < 0.5)
            {
                LOG(ERROR) << "置信度太低 跳过!" << r.confidence << " cls_id:" << det_result->cls_id;
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

        if (is_debug_)
        {
            // 二进制 PLY: 102 万点 ASCII 约 30MB+逐点 sprintf, 实测 ~2.5s/帧; 二进制仅 12MB, ~0.2s
            std::string name = debug_session_dir_ + "_yuan.ply";
            pcl::io::savePLYFileBinary(name, *transformed_cloud);
        }


        // #pragma omp parallel for
        for (size_t i = 0; i < localization_region->output.size(); ++i)
        {
            auto &region = localization_region->output[i];

            pcl::PointCloud<pcl::PointXYZ>::Ptr croped_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            extractMaskedCloud(transformed_cloud, croped_cloud, region.boxMask, region.box, img_in);

            LOG(ERROR) << "transformed_cloud.size(): " << transformed_cloud->size() << ", croped_cloud.size(): " << croped_cloud->size();
            if (is_debug_)
            {
                std::string name = debug_session_dir_ + std::to_string(i) + "cloud.ply";
                pcl::io::savePLYFileBinary(name, *croped_cloud);
            }

            // 根据 target_obj_name 筛选目标：只处理匹配的类别
            // 模型输出类别: 0=box1, 1=rack。target_obj_name 必须指定，否则报错
            if (target_obj_name.empty())
            {
                LOG(ERROR) << "target_obj_name 为空，必须指定目标名称 (box/rack)";
                return -1;
            }
            bool is_rack = (target_obj_name == "rack");
            if (!is_rack && target_obj_name != "tray")
            {
                LOG(ERROR) << "target_obj_name 不支持: " << target_obj_name << " (仅支持 tray/rack)";
                return -1;
            }
            if (is_rack && region.id != 1)
            {
                LOG(ERROR) << "skip id=" << region.id << ", target_obj_name=rack";
                continue;
            }
            if (!is_rack && region.id != 0)
            {
                LOG(ERROR) << "skip id=" << region.id << ", target_obj_name=" << target_obj_name;
                continue;
            }
            // 点云配准估计 6D 位姿
            Eigen::Vector3f pose_position, pose_rpy;
            if (is_rack)
            {
                region.pose_ok = registerAndEstimatePose6DRack(croped_cloud, pose_position, pose_rpy);
            }
            else
            {
                region.pose_ok = registerAndEstimatePose6D(croped_cloud, pose_position, pose_rpy);
            }
            if (region.pose_ok)
            {
                region.pose_position = pose_position;
                region.pose_rpy = pose_rpy;
            }
            else
            {
                LOG(ERROR) << (is_rack ? "rack" : "box") << " registerAndEstimatePose6D failed, id:" << region.id;
            }
            LOG(ERROR) << "pose_position: " << pose_position.x() << ", " << pose_position.y() << ", " << pose_position.z()
                << "), rpy_deg=(" << region.pose_rpy.x() * 180.0f / M_PI << ", " << region.pose_rpy.y() * 180.0f / M_PI << ", "
                << region.pose_rpy.z() * 180.0f / M_PI << ")" << "id:" << region.id <<"confidence:"<< region.confidence;

        }
     

        std::vector<std::string> className = {
            "box1", "rack"};

        DrawPred(img_in, localization_region->output, className, color);
        cv::rectangle(img_in, localization_region->output[best_idx].box, cv::Scalar(0, 0, 255), 2, 8);
        // cv::rectangle(img_in, crop_bbox, cv::Scalar(0, 0, 255), 2, 8);
        // 计算矩形框的中心点
        cv::Point center(
            localization_region->output[best_idx].box.x + localization_region->output[best_idx].box.width / 2,
            localization_region->output[best_idx].box.y + localization_region->output[best_idx].box.height / 2);
        cv::circle(img_in, center, 5, cv::Scalar(0, 0, 255), -1);


        if (is_debug_)
        {
            std::string img_in_path = debug_session_dir_ + "img.bmp";
            cv::imwrite(img_in_path, img_in);
            LOG(ERROR) << "Saved img_in to: " << img_in_path;
        }

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
