#include "sort_dectect.h" // 包含 sort_dectect.h 头文件
#include <opencv2/opencv.hpp>

#include <sys/time.h>
#include "object_region_detection.h"
#include <glog/logging.h>
#include <gflags/gflags.h>

Result6D::Result6D() : id(0), confidence(0), pose_xyz(Eigen::Vector3f::Zero()), pose_rpy(Eigen::Vector3f::Zero()), pose_ok(false) {}

ParaConfig::ParaConfig() : tx_param(0), ty_param(0), tz_param(0), theta_x_param(0),
                           theta_y_param(0), theta_z_param(0), min_depth(0), max_depth(0),
                           min_limit(0), max_limit(0), box_len(0.38f), box_wid(0.29f)
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
    box_len = para_config.box_len;
    box_wid = para_config.box_wid;
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
                   struct DecodeInfo *decode_info_ptr, const std::string &target_obj_name)
{

    struct DecodeInfo &decode_info = *decode_info_ptr;
    struct ParaConfig &para_config = *para_config_ptr;

    LocalizationInfo localization_info;

    localization_info.min_depth = 1;
    localization_info.max_depth = 1.1;
    localization_info.min_limit = 0.65; // 好似无用
    localization_info.max_limit = 1.2;  // 好似无用

    // 箱尺寸先验：影响边外推（中心=近边+尺寸/2）与尺寸校验
    // if (para_config.box_len > 0.f && para_config.box_wid > 0.f){
    //     sortDection::ObjectRegionDetector->setBoxSize(para_config.box_len, para_config.box_wid);
    // }
    sortDection::ObjectRegionDetector->setBoxSize(0.38f, 0.29f); // 箱尺寸先验：影响边外推（中心=近边+尺寸/2）与尺寸校验

    int localization_success = sortDection::ObjectRegionDetector->DetectRegion(img_in, cloud_in, &localization_info, target_obj_name);
    LOG(ERROR) << "localization_success: " << localization_success;

    decode_info.loc_ok = localization_success;

    if (localization_success != 1)
    {
        std::cout << "localization faild!" << std::endl;
        return;
    }

    // 点云已在机器人基坐标系下，位姿直接透传输出，无需外参变换
    for (const auto &seg : localization_info.output)
    {
        if (!seg.pose_ok)
        {
            LOG(ERROR) << "skip result id=" << seg.id << " pose_ok=false, not sent";
            continue;
        }
        Result6D result;
        result.id = seg.id;
        result.confidence = seg.confidence;
        result.pose_xyz = seg.pose_position;
        result.pose_rpy = seg.pose_rpy;
        result.pose_ok = seg.pose_ok;
        decode_info.output_result.push_back(result);

        LOG(ERROR) << "Result6D pose: pos=("
                   << seg.pose_position.x() << ", " << seg.pose_position.y() << ", " << seg.pose_position.z()
                   << "), rpy_deg=("
                   << seg.pose_rpy.x() * 180.0f / M_PI << ", " << seg.pose_rpy.y() * 180.0f / M_PI << ", "
                   << seg.pose_rpy.z() * 180.0f / M_PI << ")";
    }
}
