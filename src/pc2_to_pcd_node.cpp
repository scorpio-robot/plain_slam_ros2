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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <plain_slam/types.hpp>
#include <plain_slam/io.hpp>
#include <plain_slam/voxel_grid_filter.hpp>

#include <plain_slam_ros2/ros_utils.hpp>

namespace plain_slam {

class PC2ToPCDNode : public rclcpp::Node {
 public:
  explicit PC2ToPCDNode(const rclcpp::NodeOptions & options)
  : Node("pc2_to_pcd_node", options) {
    this->declare_parameter<std::string>("pointcloud_topic", "/livox/lidar");
    std::string pointcloud_topic;
    this->get_parameter("pointcloud_topic", pointcloud_topic);
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&PC2ToPCDNode::PointcloudCallback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("output_pcd_file", "/tmp/output.pcd");
    this->get_parameter("output_pcd_file", output_pcd_file_);

    this->declare_parameter<float>("filter_size", 0.1f);
    this->get_parameter("filter_size", filter_size_);

    cloud_.clear();
    intensities_.clear();

    RCLCPP_INFO(this->get_logger(), "PointCloud2 to PCD file node initialized");
  }

  ~PC2ToPCDNode() {
    if (cloud_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No pointcloud data received. Nothing to save.");
      return;
    }

    if (pslam::WriteBinaryPCD(output_pcd_file_, cloud_, intensities_) == true) {
      RCLCPP_INFO(this->get_logger(), "Saved pointcloud to %s",
        output_pcd_file_.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save pointcloud to %s",
        output_pcd_file_.c_str());
    }
  }

 private:
  void PointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pslam::PointCloud3f scan_cloud;
    std::vector<float> scan_intensities;
    std::vector<double> scan_stamps;
    ParseLivoxCloud(msg, scan_cloud, scan_intensities, scan_stamps);

    const pslam::VoxelGridFilter vgf(filter_size_);
    pslam::PointCloud3f filtered_cloud;
    std::vector<float> filtered_intensities;
    vgf.filter(scan_cloud, scan_intensities,
      filtered_cloud, filtered_intensities);

    cloud_.insert(cloud_.end(),
      filtered_cloud.begin(), filtered_cloud.end());
    intensities_.insert(intensities_.end(),
      filtered_intensities.begin(), filtered_intensities.end());
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  std::string output_pcd_file_;
  float filter_size_;

  pslam::PointCloud3f cloud_;
  std::vector<float> intensities_;
};

}  // namespace plain_slam

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(plain_slam::PC2ToPCDNode)
