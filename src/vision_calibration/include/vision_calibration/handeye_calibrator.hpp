#ifndef VISION_CALIBRATION__HANDEYE_CALIBRATOR_HPP_
#define VISION_CALIBRATION__HANDEYE_CALIBRATOR_HPP_

#include <string>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/core/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"

namespace vision_calibration
{

// 手眼标定参数
struct HandEyeParams
{
  // RANSAC
  double ransac_threshold{0.002};   // 内点误差阈值 (m, 作用于综合误差)
  int ransac_sample_size{8};        // 每次随机采样数
  int ransac_n_iter{5000};          // RANSAC 迭代次数
  int ransac_seed{42};              // 随机种子

  // 综合误差旋转权重 (m/rad): combined = sqrt(t_err^2 + w^2 * r_err^2)
  // 旋转误差 theta(rad) 折算为 w*theta 米的等效平移误差
  double rotation_weight{0.2};

  // LM 非线性精修最大迭代次数 (0=关闭精修)
  int refine_iterations{30};

  // 头部位姿不再走参数：CHECK_STATUS 时从 /Control/HeadPose
  // (MotorStatus.end_effector_pose[0]) 读取，保存于 camera_intrinsics.yaml
  // 的 head_position / head_orientation_xyzw 字段，calibrate() 加载。
};

/**
 * 手眼标定器（纯算法，与 ROS 通信/action 解耦）
 *
 * 输入: 数据目录（含 camera_intrinsics.yaml / image_XX.png / pose_XX.yaml）
 * 输出: JSON 结果字符串 + data_dir/calibration_result.yaml
 *
 * 流程:
 *   1. 读取相机内参 (camera_intrinsics.yaml)
 *   2. 遍历 image_XX.png: 角点检测 + solvePnP
 *   3. 读取 pose_XX.yaml: 末端位姿
 *   4. 全集标定 (清洗前基准)
 *   5. RANSAC 标定 (自动剔除异常位姿)
 *   6. 计算相机在头部坐标系下的位姿 T_cam2head
 *   7. 输出 JSON + 保存 calibration_result.yaml
 */
class HandEyeCalibrator
{
public:
  HandEyeCalibrator(const HandEyeParams & params, const rclcpp::Logger & logger);

  // 执行完整标定流程
  //   data_dir: 数据目录（内参/图像/位姿均从此读取，结果也写回此目录）
  //   mode: "TO_HAND" (眼在手外) / "IN_HAND" (眼在手中)
  bool calibrate(const std::string & data_dir, const std::string & mode,
                 std::string & result_json, std::string & err_msg);

private:
  HandEyeParams params_;
  rclcpp::Logger logger_;
};

}  // namespace vision_calibration

#endif  // VISION_CALIBRATION__HANDEYE_CALIBRATOR_HPP_
