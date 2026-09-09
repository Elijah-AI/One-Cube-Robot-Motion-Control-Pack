#include "sort_dectect.h" // 包含 sort_dectect.h 头文件
#include "sort_dectect.h" // 包含 sort_dectect.h 头文件
#include <opencv2/opencv.hpp>

#include <sys/time.h>
#include "object_region_detection.h"
#include <glog/logging.h>
#include <gflags/gflags.h>

Result6D::Result6D() : id(0), confidence(0), pose_xyz(Eigen::Vector3f::Zero()), pose_rpy(Eigen::Vector3f::Zero()) {}

ParaConfig::ParaConfig() : tx_param(0), ty_param(0), tz_param(0), theta_x_param(0),
                           theta_y_param(0), theta_z_param(0), min_depth(0), max_depth(0),
                           min_limit(0), max_limit(0)
{
}

ParaConfig::ParaConfig(const ParaConfig &para_config)
{
    tx_param = para_config.tx_param;
    ty_param = para_config.ty_param;
    tz_param = para_config.tz_param;
    theta_x_param = para_config.theta_x_param;
    theta_y_param = para_config.theta_y_param;
    theta_z_param = para_config.theta_z_param;
    min_depth = para_config.min_depth;
    max_depth = para_config.max_depth;
    min_limit = para_config.min_limit;
    max_limit = para_config.max_limit;
}

DecodeInfo::DecodeInfo() : relia(0), loc_ok(0) {}

DecodeInfo::DecodeInfo(const DecodeInfo &decode_info)
{
    relia = decode_info.relia;
    loc_ok = decode_info.loc_ok;
    output_result = decode_info.output_result;
}

namespace sortDection
{

    std::unique_ptr<ObjectRegionDetection> ObjectRegionDetector = std::make_unique<ObjectRegionDetection>();
}

void slotDetection(cv::Mat &img_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, struct ParaConfig *para_config_ptr,
                   struct DecodeInfo *decode_info_ptr)
{

    struct DecodeInfo &decode_info = *decode_info_ptr;
    struct ParaConfig &para_config = *para_config_ptr;


    Eigen::Matrix4f transform;
    transform <<    0.999957,  0.00911789, -0.00138879,    0.745326,
                    0.00904671,    -0.99896 , -0.0446955 ,  0.0602985,
                    -0.00179487 ,  0.0446811,      -0.999  ,  0.740087,
                            0 ,          0   ,        0   ,        1;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::transformPointCloud(*cloud_in, *transformed_cloud, transform);
    // pcl::io::savePLYFile("/home/cat/workspace/software/111/vision_sorter/build/runtransformed_cloud.ply", *transformed_cloud);

    LocalizationInfo localization_info;

    localization_info.min_depth = 1;
    localization_info.max_depth = 1.1;
    localization_info.min_limit = 0.65; // 好似无用
    localization_info.max_limit = 1.2;  // 好似无用

    int localization_success = sortDection::ObjectRegionDetector->DetectRegion(img_in, cloud_in, &localization_info);
    LOG(ERROR) << "localization_success: " << localization_success;

    decode_info.loc_ok = localization_success;

    if (localization_success != 1)
    {
        std::cout << "localization faild!" << std::endl;
        return;
    }

    if (1)
    {
        // 手眼标定外参（与 ros2_sender_27.py 中 HAND_EYE_MATRIX 一致）
        // T_gripper<-camera = [R | t; 0 0 0 1]，含旋转 R 和平移 t
        Eigen::Matrix4f T_hand_eye;
        T_hand_eye << 0.01571449f, -0.30814851f,  0.9512087f,  0.10762206f,
                      0.00216534f, -0.95131259f, -0.3082185f,  0.12872836f,
                      0.99987466f,  0.00690311f, -0.01428272f, -0.01844746f,
                      0.0f,         0.0f,          0.0f,         1.0f;

        // 遍历检测到的每个目标，将相机系位姿转换到末端（gripper）坐标系
        for (const auto &seg : localization_info.output)
        {
            // 1. 相机系下的旋转矩阵（由 ZYX 内禀欧拉角恢复）
            //    R_cam = Rz(yaw) * Ry(pitch) * Rx(roll)
            float roll  = seg.pose_rpy.x();
            float pitch = seg.pose_rpy.y();
            float yaw   = seg.pose_rpy.z();

            Eigen::Matrix3f Rx, Ry, Rz;
            Rx << 1, 0, 0,
                  0, std::cos(roll), -std::sin(roll),
                  0, std::sin(roll),  std::cos(roll);
            Ry << std::cos(pitch), 0, std::sin(pitch),
                  0, 1, 0,
                  -std::sin(pitch), 0, std::cos(pitch);
            Rz << std::cos(yaw), -std::sin(yaw), 0,
                  std::sin(yaw),  std::cos(yaw), 0,
                  0, 0, 1;
            Eigen::Matrix3f R_cam = Rz * Ry * Rx;

            // 2. 构造相机系下的 4x4 齐次位姿 T_cam
            Eigen::Matrix4f T_cam = Eigen::Matrix4f::Identity();
            T_cam.block<3, 3>(0, 0) = R_cam;
            T_cam.block<3, 1>(0, 3) = seg.pose_position;

            // 3. 左乘手眼矩阵：T_gripper = T_gripper<-camera * T_cam
            Eigen::Matrix4f T_gripper = T_hand_eye * T_cam;

            // 4. 从 T_gripper 提取位置
            Eigen::Vector3f pos_gripper = T_gripper.block<3, 1>(0, 3);

            // 5. 从 T_gripper 的旋转部分提取 ZYX 内禀欧拉角 (roll, pitch, yaw)
            //    R = Rz(yaw)*Ry(pitch)*Rx(roll) 反解公式
            Eigen::Matrix3f R_gripper = T_gripper.block<3, 3>(0, 0);
            float pitch_g = std::asin(-std::max(-1.0f, std::min(1.0f, R_gripper(2, 0))));
            float roll_g, yaw_g;
            if (std::abs(std::cos(pitch_g)) > 1e-6f)
            {
                roll_g = std::atan2(R_gripper(2, 1), R_gripper(2, 2));
                yaw_g  = std::atan2(R_gripper(1, 0), R_gripper(0, 0));
            }
            else
            {
                roll_g = std::atan2(-R_gripper(1, 2), R_gripper(1, 1));
                yaw_g  = 0.0f;
            }

            // 6. 填充 Result6D 并存入输出列表
            Result6D result;
            result.id = seg.id;
            result.confidence = seg.confidence;
            result.pose_xyz = pos_gripper;
            result.pose_rpy = Eigen::Vector3f(roll_g, pitch_g, yaw_g);
            decode_info.output_result.push_back(result);

            LOG(ERROR) << "Result6D gripper pose: pos=("
                       << pos_gripper.x() << ", " << pos_gripper.y() << ", " << pos_gripper.z()
                       << "), rpy_deg=("
                       << roll_g * 180.0f / M_PI << ", " << pitch_g * 180.0f / M_PI << ", "
                       << yaw_g * 180.0f / M_PI << ")";
        }
    }
}
