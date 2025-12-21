/*
 * plain_slam_ros2
 * 
 * Copyright (c) 2025 Naoki Akai
 * All rights reserved.
 *
 * This software is provided free of charge for academic and personal use only.
 * Commercial use is strictly prohibited without prior written permission from the author.
 *
 * Conditions:
 *   - Non-commercial use only
 *   - Attribution required in any academic or derivative work
 *   - Redistribution permitted with this license header intact
 *
 * Disclaimer:
 *   This software is provided "as is" without warranty of any kind.
 *   The author is not liable for any damages arising from its use.
 *
 * For commercial licensing inquiries, please contact:
 *   Naoki Akai
 *   Email: n.akai.goo[at]gmail.com   ([at] -> @)
 *   Subject: [plain_slam_ros2] Commercial License Inquiry
 */

#include <plain_slam/lio_3d_interface.hpp>

namespace pslam {

LIO3DInterface::LIO3DInterface() {
  localization_mode_ = false;

  imu_measures_ = boost::circular_buffer<IMUMeasure>(500);
  acc_scale_ = 1.0f;

  gravity_estimation_enabled_ = true;

  scan_cloud_clip_range_ = 1.0f;
  scan_cloud_filter_size_ = 0.25f;

  min_active_points_rate_ = 0.9f;

  use_loose_coupling_ = false;
  num_max_iteration_ = 5;
  num_max_matching_points_ = 50000;
  max_correspondence_dist_ = 1.0f;
  optimization_convergence_th_ = 0.1f;

  is_map_initialized_ = false;
  last_map_updated_stamp_ = 0.0;

  imu_state_cov_ = StateCov::Identity();
  imu_odom_state_cov_ = imu_state_cov_;

  const Eigen::Vector3f til = Eigen::Vector3f(-0.006253f, 0.011775f, 0.028535f);
  Eigen::Matrix3f Ril;
  Ril << -1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 1.0f;
  imu_state_.Til = Sophus::SE3f(Ril, til);

  // const Eigen::Vector3f til = Eigen::Vector3f(-0.013, -0.01862, 0.06125);
  // Eigen::Matrix3f Ril;
  // Ril << 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f;
  // imu_state_.Til = Sophus::SE3f(Ril, til);
}

LIO3DInterface::~LIO3DInterface() {

}

void LIO3DInterface::ReadLIOParams() {
  const std::string yaml_file = param_files_dir_ + "/lio_3d_params.yaml";
  const YAML::Node config = YAML::LoadFile(yaml_file);

  const auto initial_trans_vec = config["initial_pose"]["translation"];
  const auto initial_rot_vec = config["initial_pose"]["rotation"];
  const Eigen::Vector3f initial_trans(
    initial_trans_vec[0].as<float>(),
    initial_trans_vec[1].as<float>(),
    initial_trans_vec[2].as<float>());
  const float roll = initial_rot_vec[0].as<float>();
  const float pitch = initial_rot_vec[1].as<float>();
  const float yaw = initial_rot_vec[2].as<float>();
  const Eigen::Matrix3f initial_rot_mat = 
    (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
     Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
     Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())).toRotationMatrix();
  imu_state_.T = Sophus::SE3f(initial_rot_mat, initial_trans);

  const auto T_imu_lidar_trans_vec = config["extrinsics"]["imu_to_lidar"]["translation"];
  const auto T_imu_lidar_rot_vec = config["extrinsics"]["imu_to_lidar"]["rotation"];
  const Eigen::Vector3f T_imu_lidar_trans(
    T_imu_lidar_trans_vec[0].as<float>(),
    T_imu_lidar_trans_vec[1].as<float>(),
    T_imu_lidar_trans_vec[2].as<float>());
  const float til_roll = T_imu_lidar_rot_vec[0].as<float>();
  const float til_pitch = T_imu_lidar_rot_vec[1].as<float>();
  const float til_yaw = T_imu_lidar_rot_vec[2].as<float>();
  const Eigen::Matrix3f T_imu_lidar_rot_mat = 
    (Eigen::AngleAxisf(til_yaw, Eigen::Vector3f::UnitZ()) *
     Eigen::AngleAxisf(til_pitch, Eigen::Vector3f::UnitY()) *
     Eigen::AngleAxisf(til_roll, Eigen::Vector3f::UnitX())).toRotationMatrix();
  imu_state_.Til = Sophus::SE3f(T_imu_lidar_rot_mat, T_imu_lidar_trans);

  acc_scale_ = config["imu_params"]["acc_scale"].as<float>();

  scan_cloud_clip_range_ = config["scan_cloud_preprocess"]["clip_range"].as<float>();
  scan_cloud_filter_size_ = config["scan_cloud_preprocess"]["filter_size"].as<float>();

  const float keyframe_dist_th = config["keyframe"]["distance_th"].as<float>();
  const float keyframe_angle_th = config["keyframe"]["angle_th"].as<float>();
  min_active_points_rate_ = config["keyframe"]["min_active_points_rate"].as<float>();
  kf_detector_.SetDistanceThreshold(keyframe_dist_th);
  kf_detector_.SetAngleThreshold(keyframe_angle_th * M_PI / 180.0f);

  const size_t num_keyframes = config["normal_map"]["num_keyframes"].as<size_t>();
  const float filter_size = config["normal_map"]["filter_size"].as<float>();
  const size_t num_normal_points = config["normal_map"]["num_normal_points"].as<size_t>();
  const float normal_eigen_val_thresh = config["normal_map"]["normal_eigen_val_thresh"].as<float>();
  normal_map_.SetMaxKeyframeSize(num_keyframes);
  normal_map_.SetFilterSize(filter_size);
  normal_map_.SetNumNormalPoints(num_normal_points);
  normal_map_.SetNormalEigenValThreshhold(normal_eigen_val_thresh);

  use_loose_coupling_ = config["estimator"]["use_loose_coupling"].as<bool>();
  num_max_iteration_ = config["estimator"]["num_max_iteration"].as<size_t>();
  num_max_matching_points_ = config["estimator"]["num_max_matching_points"].as<size_t>();
  max_correspondence_dist_ = config["estimator"]["max_correspondence_dist"].as<float>();
  optimization_convergence_th_ = config["estimator"]["convergence_th"].as<float>();
  const float huber_delta = config["estimator"]["huber_delta"].as<float>();
  hg_observer_.SetHuberDelta(huber_delta);
  joint_optimizer_.SetHuberDelta(huber_delta);
}

void LIO3DInterface::SetScanCloud(
  const PointCloud3f& scan_cloud,
  const std::vector<float>& scan_intensities,
  const std::vector<double>& scan_stamps) {
  if (gravity_estimation_enabled_) {
    // Skip scan processing: waiting for gravity direction to be estimated
    return;
  }

  // Clip close scan points and transform to the IMU frame
  const size_t raw_cloud_size = scan_cloud.size();
  scan_cloud_.clear();
  scan_intensities_.clear();
  scan_stamps_.clear();
  scan_cloud_.reserve(raw_cloud_size);
  scan_intensities_.reserve(raw_cloud_size);
  scan_stamps_.reserve(raw_cloud_size);
  for (size_t i = 0; i < raw_cloud_size; ++i) {
    const Point3f& p = scan_cloud[i];
    if (p.norm() >= scan_cloud_clip_range_) {
      scan_cloud_.push_back(imu_state_.Til * p); // Convert to the IMU frame
      scan_intensities_.push_back(scan_intensities[i]);
      scan_stamps_.push_back(scan_stamps[i]);
    }
  }

  if (!is_map_initialized_) {
    aligned_scan_cloud_.resize(scan_cloud_.size());
    for (size_t i = 0; i < scan_cloud_.size(); ++i) {
      aligned_scan_cloud_[i] = imu_state_.T * scan_cloud_[i];
    }
    kf_detector_.AddKeyframe(imu_state_.T);
    normal_map_.AddKeyframe(imu_state_.T, aligned_scan_cloud_);
    is_map_initialized_ = true;
    is_map_updated_ = true;
    return;
  }

  // Do not use scan_stamps_ because it loses some scans when the scan is clipped.
  const double start_stamp = scan_stamps.front();
  const double end_stamp = scan_stamps.back();
  std::vector<IMUMeasure> relevant_imu_measures;

  // Retrieve relevant IMU measurements
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    size_t erase_idx = 0;
    for (size_t i = 0; i < imu_measures_.size(); ++i) {
      const auto& imu = imu_measures_[i];
      const double stamp = imu.stamp;
      if (stamp < start_stamp) {
        erase_idx++;
      } else if (stamp <= end_stamp) {
        relevant_imu_measures.push_back(imu);
        erase_idx++;
      } else {
        break;
      }
    }

    if (erase_idx > 0) {
      imu_measures_.erase_begin(erase_idx);
    }
  }

  if (relevant_imu_measures.empty()) {
    std::cerr << "[WARN] No matching IMU data for scan timestamp range ["
      << start_stamp << ", " << end_stamp << "]" << std::endl;
    // printf("%.10lf --- %.10lf\n", imu_measures_.front().stamp, imu_measures_.back().stamp);
    // return;
  }

  const State prev_state = imu_state_;

  if (relevant_imu_measures.size() > 0) {
    preintegrator_.Preintegration(relevant_imu_measures, imu_state_, imu_state_cov_);
    preintegrator_.DeskewScanCloud(imu_state_, relevant_imu_measures, scan_stamps_, scan_cloud_);
  }

  const State pred_state = imu_state_;
  const float preint_time = preintegrator_.GetPreintegrationTime();

  const VoxelGridFilter vgf(scan_cloud_filter_size_);
  const PointCloud3f filtered_scan_cloud = vgf.filter(scan_cloud_);
  // PointCloud3f filtered_scan_cloud;
  // std::vector<float> filtered_scan_intensities;
  // vgf.filter(scan_cloud_, scan_intensities_,
  //   filtered_scan_cloud, filtered_scan_intensities);

  float active_points_rate;

  if (use_loose_coupling_) {
    // Reset the covariance to prevent overflow.
    imu_state_cov_ = StateCov::Identity();
    if (!hg_observer_.Estimate(pred_state, scan_cloud_,
      num_max_iteration_, num_max_matching_points_, max_correspondence_dist_,
      optimization_convergence_th_, preint_time, imu_state_, normal_map_)) {
      std::cerr << "[WARN] Optimization has not converged" << std::endl;
      // return;
    }
    active_points_rate = hg_observer_.GetActivePointsRate();
  } else {
    const StateCov pred_state_cov = imu_state_cov_;
    if (!joint_optimizer_.Estimate(prev_state, pred_state, pred_state_cov,
      filtered_scan_cloud, num_max_iteration_, num_max_matching_points_,
      max_correspondence_dist_, optimization_convergence_th_, preint_time,
      relevant_imu_measures, imu_state_, imu_state_cov_, normal_map_)) {
      std::cerr << "[WARN] Optimization has not converged" << std::endl;
      // return;
    }
    active_points_rate = joint_optimizer_.GetActivePointsRate();
  }

  // PrintState(imu_state_);
  WriteLiDARPose(scan_stamps.front(), imu_state_);
  // std::cout << imu_state_cov_.block<6, 6>(0, 0) << std::endl;

  // Compensate for the odometry state
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_odom_state_ = imu_state_;
    imu_odom_state_cov_ = imu_state_cov_;
  }

  // Transform the scan to the odometry frame
  aligned_scan_cloud_.resize(scan_cloud_.size());
  for (size_t i = 0; i < scan_cloud_.size(); ++i) {
    aligned_scan_cloud_[i] = imu_state_.T * scan_cloud_[i];
  }

  // Perform LiDAR-IMU calibration
  // Finalize the LIO process after calibration
  // const float delta_time = preintegrator_.GetPreintegrationTime();
  // const Sophus::SE3f imu_rel_T = preintegrator_.GetDeltaT();
  // const Sophus::SE3f Tol = imu_state_.T * imu_state_.Til;
  // li_calibrator_.SetData(delta_time, imu_rel_T, Tol);
  // li_calibrator_.Calibrate(imu_state_.Til);

  // Update the local map if the new state is classified as a keyframe
  if (localization_mode_) {
    return;
  } else if (kf_detector_.IsKeyframe(imu_state_.T) ||
    active_points_rate < min_active_points_rate_) {
    const double dt = start_stamp - last_map_updated_stamp_;
    if (dt > 1.0) {
      kf_detector_.UpdateKeyframe(imu_state_.T);
      normal_map_.AddKeyframe(imu_state_.T, aligned_scan_cloud_);
      is_map_updated_ = true;
      last_map_updated_stamp_ = start_stamp;
      // std::cout << "Local map is updated." << std::endl;
    }
  }
}

void LIO3DInterface::SetIMUMeasure(const IMUMeasure& measure) {
  IMUMeasure m = measure;
  m.acc *= acc_scale_;

  if (gravity_estimation_enabled_) {
    if (g_estimator_.HasEstimated()) {
      const Sophus::SO3f R = imu_state_.T.so3() * g_estimator_.GetRotationMatrix();
      const Eigen::Vector3f t = imu_state_.T.translation();
      imu_state_.T = Sophus::SE3f(R, t);
      gravity_estimation_enabled_ = false;
    } else {
      g_estimator_.EstimateGravityDirec(m);
      return;
    }
  }

  std::lock_guard<std::mutex> lock(imu_mutex_);
  imu_measures_.push_back(m);
  preintegrator_.Preintegration(m, imu_odom_state_, imu_odom_state_cov_);
}

bool LIO3DInterface::ReadMapCloud(const std::string map_cloud_file) {
  PointCloud3f cloud;
  std::vector<float> intensities;
  if (!ReadPointCloud(map_cloud_file, cloud, intensities)) {
    std::cerr << "[WARNING] Failed to read: " << map_cloud_file << std::endl;
    return false;
  }
  normal_map_.AddKeyframe(imu_state_.T, cloud);
  is_map_initialized_ = true;
  is_map_updated_ = true;
  return true;
}

bool LIO3DInterface::ReadMapCloudPCD(const std::string& map_cloud_dir) {
  if (!std::filesystem::exists(map_cloud_dir)
      || !std::filesystem::is_directory(map_cloud_dir)) {
    std::cerr << "[ERROR] Invalid directory: " << map_cloud_dir << std::endl;
    return false;
  }

  PointCloud3f map_cloud;
  // std::vector<float> map_intensities;
  size_t total_points = 0;

  for (const auto& entry : std::filesystem::directory_iterator(map_cloud_dir)) {
    if (entry.is_regular_file()) {
      const std::string path = entry.path().string();
      if (entry.path().extension() == ".pcd") {
        PointCloud3f cloud;
        std::vector<float> intensities;
        if (ReadPCD(path, cloud, intensities)) {
          total_points += cloud.size();
          map_cloud.insert(map_cloud.end(), cloud.begin(), cloud.end());
          // map_intensities.insert(map_intensities_.end(),
          //   intensities.begin(), intensities.end());
          std::cout << "[INFO] Loaded " << path << " ("
            << cloud.size() << " points)" << std::endl;
        } else {
          std::cerr << "[WARNING] Failed to read: " << path << std::endl;
          return false;
        }
      }
    }
  }

  std::cout << "[INFO] Total points loaded: " << total_points << std::endl;
  normal_map_.AddKeyframe(imu_state_.T, map_cloud);
  is_map_initialized_ = true;
  is_map_updated_ = true;
  return true;
}

} // namespace pslam
