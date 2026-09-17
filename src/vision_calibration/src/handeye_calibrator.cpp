#include "vision_calibration/handeye_calibrator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iomanip>
#include <numeric>
#include <random>
#include <set>
#include <sstream>

#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

namespace fs = std::filesystem;

namespace vision_calibration
{

// ============================================================================
// 内部辅助函数（匿名命名空间，文件内可见）
// ============================================================================
namespace {

// 读取 pose_XX.yaml (tcp_translation + tcp_quaternion_xyzw)
bool readPose(const std::string & path, cv::Vec3d & t, cv::Quatd & q)
{
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) { return false; }
  cv::Vec3d trans;
  cv::Vec4d quat_xyzw;
  fs["tcp_translation"] >> trans;
  fs["tcp_quaternion_xyzw"] >> quat_xyzw;
  fs.release();
  t = trans;
  // cv::Quatd 构造顺序为 (w,x,y,z), 输入是 (x,y,z,w)
  q = cv::Quatd(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]);
  return true;
}

// 由旋转矩阵 R (3x3) 和平移 t (3x1) 构造 4x4 齐次变换矩阵
cv::Mat makeHomogeneous(const cv::Mat & R, const cv::Mat & t)
{
  cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
  cv::Mat R64;
  R.reshape(1, 3).convertTo(R64, CV_64F);
  cv::Mat t64;
  t.reshape(1, 3).convertTo(t64, CV_64F);
  R64.copyTo(T(cv::Rect(0, 0, 3, 3)));
  t64.copyTo(T(cv::Rect(3, 0, 1, 3)));
  return T;
}

// 用指定索引子集做手眼标定
//   TO_HAND (眼在手外): 末端位姿取逆后传入 calibrateHandEye
//   IN_HAND (眼在手中): 末端位姿直接传入 calibrateHandEye
void calibrateSubset(
  const std::string & mode,
  const std::vector<int> & idx,
  const std::vector<cv::Mat> & R_g2b,
  const std::vector<cv::Mat> & t_g2b,
  const std::vector<cv::Mat> & R_t2c,
  const std::vector<cv::Mat> & t_t2c,
  cv::Mat & R_result, cv::Mat & t_result)
{
  std::vector<cv::Mat> R_tool, t_tool, R_cam_sub, t_cam_sub;
  for (int i : idx) {
    if (mode == "TO_HAND") {
      cv::Mat Rg = R_g2b[i];
      cv::Mat tg = t_g2b[i];
      R_tool.push_back(Rg.t());             // R^{-1} = R^T
      t_tool.push_back(-Rg.t() * tg);       // t^{-1} = -R^T * t
    } else {
      R_tool.push_back(R_g2b[i]);
      t_tool.push_back(t_g2b[i]);
    }
    R_cam_sub.push_back(R_t2c[i]);
    t_cam_sub.push_back(t_t2c[i]);
  }
  cv::calibrateHandEye(R_tool, t_tool, R_cam_sub, t_cam_sub,
                       R_result, t_result, cv::CALIB_HAND_EYE_TSAI);
}

// 3x1 向量的反对称矩阵 [p]x
cv::Mat skew(const cv::Mat & p3)
{
  const double x = p3.at<double>(0);
  const double y = p3.at<double>(1);
  const double z = p3.at<double>(2);
  return (cv::Mat_<double>(3, 3) << 0.0, -z, y,
                                      z, 0.0, -x,
                                      -y, x, 0.0);
}

// se(3) 指数映射: 扭量 xi=(omega, v) (6x1) -> 4x4 齐次矩阵
cv::Mat se3Exp(const cv::Mat & xi)
{
  const cv::Mat w = xi.rowRange(0, 3).clone();
  const cv::Mat v = xi.rowRange(3, 6).clone();
  const double th = cv::norm(w);
  cv::Mat R;
  cv::Rodrigues(w, R);
  cv::Mat t;
  if (th < 1e-12) {
    t = v.clone();
  } else {
    const cv::Mat W = skew(w);
    const cv::Mat V = cv::Mat::eye(3, 3, CV_64F) +
      ((1.0 - std::cos(th)) / (th * th)) * W +
      ((th - std::sin(th)) / (th * th * th)) * (W * W);
    t = V * v;
  }
  cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
  R.copyTo(T(cv::Rect(0, 0, 3, 3)));
  t.copyTo(T(cv::Rect(3, 0, 1, 3)));
  return T;
}

// SE(3) 伴随矩阵 Ad_T (6x6, 扭量顺序 (omega, v))
// 满足: T * exp(xi^) * T^{-1} = exp((Ad_T xi)^)
cv::Mat adjoint(const cv::Mat & T)
{
  const cv::Mat R = T(cv::Rect(0, 0, 3, 3));
  const cv::Mat p = T(cv::Rect(3, 0, 1, 3));
  cv::Mat Ad = cv::Mat::zeros(6, 6, CV_64F);
  R.copyTo(Ad(cv::Rect(0, 0, 3, 3)));          // 左上: R
  R.copyTo(Ad(cv::Rect(3, 3, 3, 3)));          // 右下: R
  cv::Mat(skew(p) * R).copyTo(Ad(cv::Rect(0, 3, 3, 3)));  // 左下: [p]x R
  return Ad;
}

// 位姿集合的平均位姿 (平移算术平均 + 四元数符号对齐平均)
cv::Mat meanPose(const std::vector<cv::Mat> & Ys)
{
  cv::Mat t_mean = cv::Mat::zeros(3, 1, CV_64F);
  for (const auto & Y : Ys) {
    t_mean += Y(cv::Rect(3, 0, 1, 3));
  }
  t_mean /= static_cast<double>(Ys.size());

  double qw = 0.0, qx = 0.0, qy = 0.0, qz = 0.0;
  double qw0 = 0.0, qx0 = 0.0, qy0 = 0.0, qz0 = 0.0;
  {
    const cv::Quatd q0 = cv::Quatd::createFromRotMat(
      cv::Matx33d(Ys[0](cv::Rect(0, 0, 3, 3))));
    qw0 = q0.w; qx0 = q0.x; qy0 = q0.y; qz0 = q0.z;
  }
  for (const auto & Y : Ys) {
    const cv::Quatd qi = cv::Quatd::createFromRotMat(
      cv::Matx33d(Y(cv::Rect(0, 0, 3, 3))));
    // 符号对齐 (q 与 -q 表示同一旋转)
    const double dot = qi.w * qw0 + qi.x * qx0 + qi.y * qy0 + qi.z * qz0;
    const double s = dot < 0.0 ? -1.0 : 1.0;
    qw += s * qi.w; qx += s * qi.x; qy += s * qi.y; qz += s * qi.z;
  }
  double n = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (n < 1e-15) {
    qw = qw0; qx = qx0; qy = qy0; qz = qz0;
    n = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  }
  const cv::Quatd qm(qw / n, qx / n, qy / n, qz / n);   // (w,x,y,z)
  const cv::Matx33d Rm = qm.toRotMat3x3();

  cv::Mat Z = cv::Mat::eye(4, 4, CV_64F);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Z.at<double>(i, j) = Rm(i, j);
    }
  }
  t_mean.copyTo(Z(cv::Rect(3, 0, 1, 3)));
  return Z;
}

// 单帧一致性误差: 平移误差 + 旋转误差
struct FrameErr
{
  double t_err{0.0};       // 平移到均值位姿的距离 (m)
  double r_err{0.0};       // 旋转到均值位姿的夹角 (rad)
  double combined{0.0};    // 综合误差 sqrt(t^2 + w^2 * r^2) (m)
};

// 计算一致性误差: 标定板在参考坐标系下位姿 Y(i) 的一致性
//   TO_HAND: Y(i) = inv(T_g2b(i)) * T_cam2base * T_c2t(i)  (标定板在末端坐标系下)
//   IN_HAND: Y(i) = T_g2b(i) * T_cam2gripper * T_c2t(i)   (标定板在基坐标系下)
//   每帧误差 = 相对平均位姿的平移偏差 + 旋转偏差(加权折算为米)
std::vector<FrameErr> computeConsistencyError(
  const std::string & mode,
  const cv::Mat & R_result, const cv::Mat & t_result,
  const std::vector<cv::Mat> & R_g2b, const std::vector<cv::Mat> & t_g2b,
  const std::vector<cv::Mat> & R_t2c, const std::vector<cv::Mat> & t_t2c,
  const std::vector<int> & idx,
  double rot_weight)
{
  const cv::Mat T_result = makeHomogeneous(R_result, t_result);
  std::vector<cv::Mat> Ys;
  for (int i : idx) {
    const cv::Mat T_g2b = makeHomogeneous(R_g2b[i], t_g2b[i]);
    const cv::Mat T_c2t = makeHomogeneous(R_t2c[i], t_t2c[i]);
    const cv::Mat Y = (mode == "TO_HAND")
      ? (cv::Mat)(T_g2b.inv() * T_result * T_c2t)
      : (cv::Mat)(T_g2b * T_result * T_c2t);
    Ys.push_back(Y);
  }
  const cv::Mat Zbar = meanPose(Ys);
  const cv::Mat RZ = Zbar(cv::Rect(0, 0, 3, 3));
  const cv::Mat tZ = Zbar(cv::Rect(3, 0, 1, 3));

  std::vector<FrameErr> out;
  out.reserve(Ys.size());
  for (const auto & Y : Ys) {
    cv::Mat rvec_e;
    cv::Rodrigues(Y(cv::Rect(0, 0, 3, 3)) * RZ.t(), rvec_e);
    const cv::Mat dt = Y(cv::Rect(3, 0, 1, 3)) - tZ;
    FrameErr fe;
    fe.t_err = cv::norm(dt);
    fe.r_err = cv::norm(rvec_e);
    fe.combined = std::sqrt(fe.t_err * fe.t_err +
                           rot_weight * rot_weight * fe.r_err * fe.r_err);
    out.push_back(fe);
  }
  return out;
}

// RANSAC 手眼标定: 随机抽样标定, 选"内点数最多+全集误差最小"的子集, 自动剔除异常位姿
//   评分使用综合误差 (平移 + 加权旋转)
void calibrateRansac(
  const std::string & mode,
  const std::vector<cv::Mat> & R_g2b,
  const std::vector<cv::Mat> & t_g2b,
  const std::vector<cv::Mat> & R_t2c,
  const std::vector<cv::Mat> & t_t2c,
  double threshold, int n_iter, int sample_size,
  unsigned int seed, double rot_weight,
  cv::Mat & R_result, cv::Mat & t_result,
  std::vector<int> & used_idx, std::vector<int> & outlier_idx)
{
  const int n = static_cast<int>(R_g2b.size());
  std::vector<int> all_idx(n);
  std::iota(all_idx.begin(), all_idx.end(), 0);

  // 初始基准: 全集标定
  calibrateSubset(mode, all_idx, R_g2b, t_g2b, R_t2c, t_t2c, R_result, t_result);
  std::vector<FrameErr> best_e = computeConsistencyError(
    mode, R_result, t_result, R_g2b, t_g2b, R_t2c, t_t2c, all_idx, rot_weight);
  int best_inliers = 0;
  double best_mean = 0.0;
  for (const auto & f : best_e) {
    if (f.combined < threshold) {++best_inliers;}
    best_mean += f.combined;
  }
  best_mean /= n;

  std::mt19937 gen(seed);
  for (int iter = 0; iter < n_iter; ++iter) {
    // 无放回随机抽样 sample_size 个索引
    std::vector<int> sub;
    std::sample(all_idx.begin(), all_idx.end(), std::back_inserter(sub),
                sample_size, gen);
    std::sort(sub.begin(), sub.end());

    cv::Mat R_, t_;
    calibrateSubset(mode, sub, R_g2b, t_g2b, R_t2c, t_t2c, R_, t_);
    std::vector<FrameErr> e = computeConsistencyError(
      mode, R_, t_, R_g2b, t_g2b, R_t2c, t_t2c, all_idx, rot_weight);
    int inliers = 0;
    double mean = 0.0;
    for (const auto & f : e) {
      if (f.combined < threshold) {++inliers;}
      mean += f.combined;
    }
    mean /= n;
    // 评分: 内点数优先, 内点数相同时取平均误差更小者
    if (inliers > best_inliers ||
        (inliers == best_inliers && mean < best_mean)) {
      best_inliers = inliers;
      best_mean = mean;
      R_result = R_.clone();
      t_result = t_.clone();
      best_e = e;
    }
  }

  // 全集上误差超阈值者判定为异常
  outlier_idx.clear();
  for (int i = 0; i < n; ++i) {
    if (best_e[i].combined >= threshold) { outlier_idx.push_back(i); }
  }

  // 用内点集(全集减异常)重新标定
  std::vector<int> inlier_idx;
  {
    std::set<int> outlier_set(outlier_idx.begin(), outlier_idx.end());
    for (int i = 0; i < n; ++i) {
      if (outlier_set.find(i) == outlier_set.end()) {
        inlier_idx.push_back(i);
      }
    }
  }
  if (static_cast<int>(inlier_idx.size()) >= sample_size) {
    calibrateSubset(mode, inlier_idx, R_g2b, t_g2b, R_t2c, t_t2c, R_result, t_result);
  }
  used_idx = inlier_idx;
}

// LM/Gauss-Newton 非线性精修: 在指定帧集上优化 X, 最小化 Y(i) 一致性残差
//   目标: min_X Σ_i ( ‖t(Y_i) - t(Z̄)‖² + w²·‖log(R(Y_i)R(Z̄)ᵀ)‖² )
//   参数化: X ← exp(ξ^)·X, 一阶雅可比 J_i = [[wI,0],[-[t_i]×,I]]·Ad_{A_i}
void refineHandEye(
  const std::string & mode,
  const std::vector<int> & idx,
  const std::vector<cv::Mat> & R_g2b,
  const std::vector<cv::Mat> & t_g2b,
  const std::vector<cv::Mat> & R_t2c,
  const std::vector<cv::Mat> & t_t2c,
  double rot_weight, int max_iter,
  cv::Mat & R_result, cv::Mat & t_result)
{
  if (idx.size() < 3 || max_iter <= 0) { return; }

  // 预计算: Y_i = A_i · X · B_i
  //   TO_HAND:  A_i = inv(T_g2b_i)
  //   IN_HAND:  A_i = T_g2b_i
  std::vector<cv::Mat> A, B;
  for (int i : idx) {
    const cv::Mat Tg = makeHomogeneous(R_g2b[i], t_g2b[i]);
    const cv::Mat Tc = makeHomogeneous(R_t2c[i], t_t2c[i]);
    A.push_back(mode == "TO_HAND" ? Tg.inv() : Tg);
    B.push_back(Tc);
  }
  const int m = static_cast<int>(A.size());
  cv::Mat X = makeHomogeneous(R_result, t_result);

  const cv::Mat I3 = cv::Mat::eye(3, 3, CV_64F);

  for (int iter = 0; iter < max_iter; ++iter) {
    // 当前 Y_i 与平均位姿 Z̄
    std::vector<cv::Mat> Y(m);
    for (int k = 0; k < m; ++k) {
      Y[k] = A[k] * X * B[k];
    }
    const cv::Mat Zbar = meanPose(Y);

    // Gauss-Newton 正规方程
    cv::Mat H = cv::Mat::zeros(6, 6, CV_64F);
    cv::Mat g = cv::Mat::zeros(6, 1, CV_64F);
    for (int k = 0; k < m; ++k) {
      const cv::Mat tY = Y[k](cv::Rect(3, 0, 1, 3));
      const cv::Mat RZ = Zbar(cv::Rect(0, 0, 3, 3));
      const cv::Mat tZ = Zbar(cv::Rect(3, 0, 1, 3));

      // 残差 e = [w·e_r ; e_t]  (e_r = log(R_Y R_Z^T), e_t = t_Y - t_Z)
      cv::Mat rvec_e;
      cv::Rodrigues(Y[k](cv::Rect(0, 0, 3, 3)) * RZ.t(), rvec_e);
      cv::Mat e(6, 1, CV_64F);
      for (int d = 0; d < 3; ++d) {
        e.at<double>(d) = rot_weight * rvec_e.at<double>(d);
        e.at<double>(3 + d) = tY.at<double>(d) - tZ.at<double>(d);
      }

      // 一阶雅可比: δe_r = η_ω, δe_t = η_v - [t_Y]× η_ω, η = Ad_{A_i} ξ
      cv::Mat J0 = cv::Mat::zeros(6, 6, CV_64F);
      cv::Mat(rot_weight * I3).copyTo(J0(cv::Rect(0, 0, 3, 3)));
      cv::Mat(-skew(tY)).copyTo(J0(cv::Rect(0, 3, 3, 3)));
      I3.copyTo(J0(cv::Rect(3, 3, 3, 3)));
      const cv::Mat J = J0 * adjoint(A[k]);

      H += J.t() * J;
      g += J.t() * e;
    }

    // 阻尼 Gauss-Newton 求解
    cv::Mat Hd = H.clone();
    for (int d = 0; d < 6; ++d) {
      Hd.at<double>(d, d) += 1e-10 + 1e-8 * std::abs(H.at<double>(d, d));
    }
    cv::Mat dxi;
    if (!cv::solve(Hd, -g, dxi)) { break; }

    // 更新 X ← exp(ξ^)·X
    X = se3Exp(dxi) * X;

    if (cv::norm(dxi) < 1e-12) { break; }   // 收敛
  }

  // 写回结果
  R_result = X(cv::Rect(0, 0, 3, 3)).clone();
  t_result = X(cv::Rect(3, 0, 1, 3)).clone();
}

}  // anonymous namespace

// ============================================================================
// 构造函数
// ============================================================================
HandEyeCalibrator::HandEyeCalibrator(const HandEyeParams & params, const rclcpp::Logger & logger)
: params_(params), logger_(logger)
{
}

// ============================================================================
// 完整标定流程
// ============================================================================
bool
HandEyeCalibrator::calibrate(
  const std::string & data_dir, const std::string & mode,
  std::string & result_json, std::string & err_msg)
{
  // 枚举实际存在的 image_*.png（编号可能不连续，按编号排序）
  std::vector<int> img_ids;
  for (auto & p : fs::directory_iterator(data_dir)) {
    std::string name = p.path().filename().string();
    if (name.rfind("image_", 0) == 0 && p.path().extension() == ".png") {
      try {
        img_ids.push_back(std::stoi(name.substr(6)));
      } catch (...) {
        // 非数字编号，忽略
      }
    }
  }
  std::sort(img_ids.begin(), img_ids.end());
  const int n = static_cast<int>(img_ids.size());
  if (n < 3) {
    err_msg = "采集帧数不足 (当前 " + std::to_string(n) + ", 需 >=3)";
    return false;
  }

  // === 1. 读取相机内参 ===
  const std::string intrinsics_path =
    (fs::path(data_dir) / "camera_intrinsics.yaml").string();
  cv::FileStorage fs(intrinsics_path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    err_msg = "无法打开内参文件: " + intrinsics_path;
    return false;
  }
  cv::Mat camera_matrix, dist_coeffs;
  int board_cols = 0, board_rows = 0;
  double square_size = 0.0;
  fs["camera_matrix"] >> camera_matrix;
  fs["dist_coeffs"]  >> dist_coeffs;
  fs["board_cols"]   >> board_cols;
  fs["board_rows"]   >> board_rows;
  fs["square_size"]  >> square_size;
  // 头部位姿（CHECK_STATUS 时从 /Control/HeadPose 采集保存）
  std::vector<double> head_position, head_orientation_xyzw;
  cv::FileNode hp_node = fs["head_position"];
  cv::FileNode hq_node = fs["head_orientation_xyzw"];
  if (!hp_node.empty() && !hq_node.empty()) {
    hp_node >> head_position;
    hq_node >> head_orientation_xyzw;
  }
  fs.release();
  if (board_cols < 2 || board_rows < 2 || square_size <= 0) {
    err_msg = "内参文件棋盘格参数无效";
    return false;
  }
  if (head_position.size() != 3 || head_orientation_xyzw.size() != 4) {
    err_msg = "内参文件缺少头部位姿 (head_position/head_orientation_xyzw)，"
              "请重新执行 CHECK_STATUS（需 /Control/HeadPose 在线）";
    return false;
  }
  cv::Size board_size(board_cols, board_rows);
  RCLCPP_INFO(logger_,
    "内参已加载: 棋盘 %dx%d, 格子 %.4fm, 帧数=%d", board_cols, board_rows, square_size, n);

  // === 2. 构造标定板世界坐标点 (Z=0) ===
  std::vector<cv::Point3f> objp;
  for (int i = 0; i < board_size.height; ++i) {
    for (int j = 0; j < board_size.width; ++j) {
      objp.emplace_back(static_cast<float>(j * square_size),
                        static_cast<float>(i * square_size),
                        0.0f);
    }
  }

  // === 3. 遍历图片: 角点检测 + solvePnP + 读取末端位姿 ===
  std::vector<cv::Mat> R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list;
  std::vector<int> valid_ids;                 // 成功处理的帧号（与上述列表一一对应）
  int skipped = 0;                            // 跳过帧计数
  cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 100, 1e-5);

  for (int idx : img_ids) {
    char buf[64];
    // 读取图像
    std::snprintf(buf, sizeof(buf), "/image_%02d.png", idx);
    const std::string img_path = data_dir + buf;
    cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);
    if (img.empty()) {
      RCLCPP_WARN(logger_, "跳过帧 %d: 读取图片失败", idx);
      ++skipped;
      continue;
    }
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    // 角点检测 + 亚像素细化
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(gray, board_size, corners,
              cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FILTER_QUADS);
    if (!found) {
      RCLCPP_WARN(logger_, "跳过帧 %d: 未找到角点 (%s)", idx, img_path.c_str());
      ++skipped;
      continue;
    }
    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);

    // solvePnP: 求标定板在相机坐标系下的位姿
    cv::Mat rvec, tvec;
    cv::solvePnP(objp, corners, camera_matrix, dist_coeffs, rvec, tvec);
    cv::Mat R_t2c;
    cv::Rodrigues(rvec, R_t2c);

    // 读取对应机械臂末端位姿
    std::snprintf(buf, sizeof(buf), "/pose_%02d.yaml", idx);
    cv::Vec3d t_pose;
    cv::Quatd q_pose;
    if (!readPose(data_dir + buf, t_pose, q_pose)) {
      RCLCPP_WARN(logger_, "跳过帧 %d: 读取位姿失败 (%s)", idx, (data_dir + buf).c_str());
      ++skipped;
      continue;
    }

    cv::Matx33d R_g2b = q_pose.toRotMat3x3();
    R_t2c_list.push_back(R_t2c);
    t_t2c_list.push_back(tvec);
    R_g2b_list.push_back(cv::Mat(R_g2b));
    t_g2b_list.push_back(cv::Mat(t_pose));
    valid_ids.push_back(idx);
  }

  const int N = static_cast<int>(R_t2c_list.size());
  if (skipped > 0) {
    RCLCPP_WARN(logger_, "共跳过 %d 帧 (角点检测失败/文件缺失)", skipped);
  }
  if (N < 3) {
    err_msg = "有效帧数不足 (有效 " + std::to_string(N) +
              ", 跳过 " + std::to_string(skipped) + ", 需 >=3)";
    return false;
  }
  RCLCPP_INFO(logger_, "成功处理 %d 组数据 (跳过 %d 帧), RANSAC: 阈值=%.4fm 采样=%d 迭代=%d",
    N, skipped, params_.ransac_threshold, params_.ransac_sample_size, params_.ransac_n_iter);

  std::vector<int> all_idx(N);
  std::iota(all_idx.begin(), all_idx.end(), 0);

  // === 4. 全集标定 (清洗前基准) ===
  cv::Mat R_all, t_all;
  calibrateSubset(mode, all_idx, R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list, R_all, t_all);
  std::vector<FrameErr> err_all = computeConsistencyError(
    mode, R_all, t_all, R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list,
    all_idx, params_.rotation_weight);
  double mean_all = 0.0;
  for (const auto & f : err_all) { mean_all += f.combined; }
  mean_all /= err_all.size();
  RCLCPP_INFO(logger_, "清洗前平均误差: %.6f m (综合, 含加权旋转)", mean_all);

  // === 5. RANSAC 标定 ===
  cv::Mat R_result, t_result;
  std::vector<int> used_idx, outlier_idx;
  calibrateRansac(mode, R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list,
                  params_.ransac_threshold, params_.ransac_n_iter,
                  params_.ransac_sample_size, params_.ransac_seed,
                  params_.rotation_weight,
                  R_result, t_result, used_idx, outlier_idx);

  // === 5.5 LM 非线性精修 (在内点集上优化一致性) ===
  double refine_before = 0.0;
  double refine_after = 0.0;
  if (used_idx.size() >= 3 && params_.refine_iterations > 0) {
    // 精修前: 内点集平均综合误差
    {
      auto e_in = computeConsistencyError(
        mode, R_result, t_result, R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list,
        used_idx, params_.rotation_weight);
      for (const auto & f : e_in) { refine_before += f.combined; }
      refine_before /= e_in.size();
    }
    refineHandEye(mode, used_idx, R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list,
                  params_.rotation_weight, params_.refine_iterations,
                  R_result, t_result);
    // 精修后: 内点集平均综合误差
    {
      auto e_in = computeConsistencyError(
        mode, R_result, t_result, R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list,
        used_idx, params_.rotation_weight);
      for (const auto & f : e_in) { refine_after += f.combined; }
      refine_after /= e_in.size();
    }
    RCLCPP_INFO(logger_, "LM 精修: 内点平均综合误差 %.6f -> %.6f m",
      refine_before, refine_after);
  }

  // 清洗后误差 (全集, 综合误差)
  std::vector<FrameErr> errors = computeConsistencyError(
    mode, R_result, t_result, R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list,
    all_idx, params_.rotation_weight);
  double mean_err = 0.0;
  for (const auto & f : errors) { mean_err += f.combined; }
  mean_err /= errors.size();

  // 四元数
  cv::Matx33d Rmat(R_result);
  cv::Quatd q = cv::Quatd::createFromRotMat(Rmat);

  RCLCPP_INFO(logger_,
    "标定完成: 使用 %zu 组, 剔除 %zu 组, 清洗后平均误差 %.6f m",
    used_idx.size(), outlier_idx.size(), mean_err);

  // 打印使用的组（真实帧号）
  {
    std::ostringstream oss;
    oss << "使用位姿组: [";
    for (size_t i = 0; i < used_idx.size(); ++i) {
      oss << valid_ids[used_idx[i]];
      if (i + 1 < used_idx.size()) {oss << ", ";}
    }
    oss << "]";
    RCLCPP_INFO(logger_, "%s", oss.str().c_str());
  }
  // 打印剔除的组（真实帧号）
  {
    std::ostringstream oss;
    oss << "剔除异常组: [";
    for (size_t i = 0; i < outlier_idx.size(); ++i) {
      oss << valid_ids[outlier_idx[i]];
      if (i + 1 < outlier_idx.size()) {oss << ", ";}
    }
    oss << "]";
    RCLCPP_INFO(logger_, "%s", oss.str().c_str());
  }
  // 逐组误差（真实帧号: 综合误差 = sqrt(t² + w²·r²)）
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "每组误差(/m):" << std::endl;
    for (size_t i = 0; i < errors.size(); ++i) {
      const bool is_outlier =
        std::find(outlier_idx.begin(), outlier_idx.end(), static_cast<int>(i)) != outlier_idx.end();
      oss << "  帧 " << valid_ids[i] << ": " << errors[i].combined
          << " (t=" << errors[i].t_err << ", r=" << errors[i].r_err << " rad)"
          << (is_outlier ? "  <-- 剔除" : "") << std::endl;
    }
    oss << "平均综合误差: " << mean_err;
    RCLCPP_INFO(logger_, "%s", oss.str().c_str());
  }

  RCLCPP_INFO(logger_, "平移: [%.6f, %.6f, %.6f]",
    t_result.at<double>(0), t_result.at<double>(1), t_result.at<double>(2));
  RCLCPP_INFO(logger_, "四元数(xyzw): [%.6f, %.6f, %.6f, %.6f]",
    q.x, q.y, q.z, q.w);

  // === 6. 计算相机在头部坐标系下的位姿 ===
  // T_cam2head = T_head2base.inv() * T_cam2base
  cv::Mat R_cam2head, t_cam2head;
  cv::Quatd q_cam2head;
  {
    // 头部位姿 (xyzw -> cv::Quatd 构造顺序 w,x,y,z)
    cv::Quatd q_head(head_orientation_xyzw[3],
                     head_orientation_xyzw[0],
                     head_orientation_xyzw[1],
                     head_orientation_xyzw[2]);
    cv::Matx33d R_head = q_head.toRotMat3x3();
    cv::Vec3d t_head(head_position[0], head_position[1],
                     head_position[2]);

    // 构造 T_head2base (4x4)
    cv::Mat T_head2base = cv::Mat::eye(4, 4, CV_64F);
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        T_head2base.at<double>(i, j) = R_head(i, j);
      }
    }
    for (int i = 0; i < 3; ++i) {
      T_head2base.at<double>(i, 3) = t_head[i];
    }

    // T_cam2base (标定结果)
    cv::Mat T_cam2base_mat = makeHomogeneous(R_result, t_result);

    // T_cam2head = T_head2base.inv() * T_cam2base
    cv::Mat T_cam2head_mat = T_head2base.inv() * T_cam2base_mat;

    // 分解为 R, t, 四元数
    R_cam2head = T_cam2head_mat(cv::Rect(0, 0, 3, 3)).clone();
    t_cam2head = T_cam2head_mat(cv::Rect(3, 0, 1, 3)).clone();
    cv::Matx33d Rmat_res(R_cam2head);
    q_cam2head = cv::Quatd::createFromRotMat(Rmat_res);

    RCLCPP_INFO(logger_, "相机->头部 平移: [%.6f, %.6f, %.6f]",
      t_cam2head.at<double>(0), t_cam2head.at<double>(1), t_cam2head.at<double>(2));
    RCLCPP_INFO(logger_, "相机->头部 四元数(xyzw): [%.6f, %.6f, %.6f, %.6f]",
      q_cam2head.x, q_cam2head.y, q_cam2head.z, q_cam2head.w);
  }

  // === 7. 保存标定结果到 calibration_result.yaml ===
  const std::string result_path =
    (fs::path(data_dir) / "calibration_result.yaml").string();
  {
    cv::FileStorage fs_out(result_path, cv::FileStorage::WRITE);
    if (fs_out.isOpened()) {
      fs_out << "mode" << mode;
      fs_out << "rotation_matrix" << R_result;
      fs_out << "translation_vector" << t_result;
      fs_out << "quaternion_xyzw" << cv::Vec4d(q.x, q.y, q.z, q.w);
      fs_out << "mean_error_before_ransac" << mean_all;
      fs_out << "mean_error_after_ransac" << mean_err;
      fs_out << "mean_error_after_refine" << refine_after;
      fs_out << "rotation_weight" << params_.rotation_weight;
      fs_out << "used_count" << static_cast<int>(used_idx.size());
      fs_out << "outlier_count" << static_cast<int>(outlier_idx.size());
      // 相机在头部坐标系下的位姿
      fs_out << "cam2head_rotation_matrix" << R_cam2head;
      fs_out << "cam2head_translation_vector" << t_cam2head;
      fs_out << "cam2head_quaternion_xyzw" <<
        cv::Vec4d(q_cam2head.x, q_cam2head.y, q_cam2head.z, q_cam2head.w);
      // 头部在基坐标系下的位姿（参考输入，来自内参文件）
      fs_out << "head_position" <<
        cv::Vec3d(head_position[0], head_position[1], head_position[2]);
      fs_out << "head_orientation_xyzw" <<
        cv::Vec4d(head_orientation_xyzw[0], head_orientation_xyzw[1],
                  head_orientation_xyzw[2], head_orientation_xyzw[3]);
      fs_out.release();
      RCLCPP_INFO(logger_, "标定结果已保存: %s", result_path.c_str());
    } else {
      RCLCPP_WARN(logger_, "无法保存标定结果文件: %s", result_path.c_str());
    }
  }

  // === 8. 构造 JSON 结果 ===
  std::ostringstream j;
  j << std::setprecision(16);
  j << "{";
  j << "\"mode\":\"" << mode << "\"";
  j << ",\"captures\":" << N;
  // 旋转矩阵
  j << ",\"rotation_matrix\":[";
  for (int r = 0; r < 3; ++r) {
    j << "[";
    for (int c = 0; c < 3; ++c) {
      j << R_result.at<double>(r, c);
      if (c < 2) {j << ",";}
    }
    j << "]";
    if (r < 2) {j << ",";}
  }
  j << "]";
  // 平移向量
  j << ",\"translation_vector\":[";
  j << t_result.at<double>(0) << "," << t_result.at<double>(1) << "," << t_result.at<double>(2);
  j << "]";
  // 四元数 xyzw
  j << ",\"quaternion_xyzw\":[" << q.x << "," << q.y << "," << q.z << "," << q.w << "]";
  // 相机在头部坐标系下的位姿
  j << ",\"cam2head_rotation_matrix\":[";
  for (int r = 0; r < 3; ++r) {
    j << "[";
    for (int c = 0; c < 3; ++c) {
      j << R_cam2head.at<double>(r, c);
      if (c < 2) {j << ",";}
    }
    j << "]";
    if (r < 2) {j << ",";}
  }
  j << "]";
  j << ",\"cam2head_translation_vector\":[";
  j << t_cam2head.at<double>(0) << "," << t_cam2head.at<double>(1) << "," << t_cam2head.at<double>(2);
  j << "]";
  j << ",\"cam2head_quaternion_xyzw\":[" << q_cam2head.x << "," << q_cam2head.y
    << "," << q_cam2head.z << "," << q_cam2head.w << "]";
  // 使用/剔除帧号（映射回真实 image 编号）
  j << ",\"used_frame_ids\":[";
  for (size_t i = 0; i < used_idx.size(); ++i) {
    j << valid_ids[used_idx[i]];
    if (i + 1 < used_idx.size()) {j << ",";}
  }
  j << "]";
  j << ",\"outlier_frame_ids\":[";
  for (size_t i = 0; i < outlier_idx.size(); ++i) {
    j << valid_ids[outlier_idx[i]];
    if (i + 1 < outlier_idx.size()) {j << ",";}
  }
  j << "]";
  // 跳过的帧（角点检测失败/文件缺失）
  j << ",\"skipped_count\":" << skipped;
  // 误差
  j << ",\"errors\":[";
  for (size_t i = 0; i < errors.size(); ++i) {
    j << errors[i].combined;
    if (i + 1 < errors.size()) {j << ",";}
  }
  j << "]";
  j << ",\"translation_errors\":[";
  for (size_t i = 0; i < errors.size(); ++i) {
    j << errors[i].t_err;
    if (i + 1 < errors.size()) {j << ",";}
  }
  j << "]";
  j << ",\"rotation_errors\":[";
  for (size_t i = 0; i < errors.size(); ++i) {
    j << errors[i].r_err;
    if (i + 1 < errors.size()) {j << ",";}
  }
  j << "]";
  j << ",\"mean_error\":" << mean_err;
  j << ",\"mean_error_before_ransac\":" << mean_all;
  j << ",\"mean_error_after_refine\":" << refine_after;
  j << ",\"rotation_weight\":" << params_.rotation_weight;
  j << "}";

  result_json = j.str();
  err_msg.clear();
  return true;
}

}  // namespace vision_calibration
