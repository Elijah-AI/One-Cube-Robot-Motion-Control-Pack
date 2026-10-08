#ifndef SORT_DECTECT_H_
#define SORT_DECTECT_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <sys/time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



struct Result6D {
    Result6D();   // 显式声明默认构造函数（定义在 sort_dectect.cpp）
    int id;           // 结果类别id
    float confidence; // 结果置信度
    Eigen::Vector3f pose_xyz; // 平面拟合后的6D位姿位置
    Eigen::Vector3f pose_rpy;      // ZYX内禀欧拉角(roll,pitch,yaw)
    bool pose_ok = false;           // 6D位姿是否检测成功
};


struct DecodeInfo {
    DecodeInfo();
    DecodeInfo(const DecodeInfo& decode_info);
    int relia;
    int loc_ok;
    std::vector<Result6D> output_result;
};


struct ParaConfig {
    ParaConfig();
    ParaConfig(const ParaConfig& para_config);   

  	double tx_param;
	double ty_param;
	double tz_param;
	double theta_x_param;
	double theta_y_param;
	double theta_z_param;

    float min_depth;
    float max_depth;
    float min_limit;
    float max_limit;

    float box_len;   // 纸箱顶面长边尺寸（米），影响边外推与尺寸校验
    float box_wid;   // 纸箱顶面短边尺寸（米）
};


void slotDetection(cv::Mat& img_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,  struct ParaConfig* para_config_ptr,
                  struct DecodeInfo* decode_info_ptr, const std::string& target_obj_name = "");

#endif // SORT_DECTECT_H_

