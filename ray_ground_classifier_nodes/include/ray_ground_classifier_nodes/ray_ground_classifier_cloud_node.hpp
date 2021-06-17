// Copyright 2017-2020 the Autoware Foundation, Arm Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
/// \file
/// \brief This file defines a ROS 2 node that contains a VLP16 driver and a ground filter and
///        publishes three different point cloud topics

#ifndef RAY_GROUND_CLASSIFIER_NODES__RAY_GROUND_CLASSIFIER_CLOUD_NODE_HPP_
#define RAY_GROUND_CLASSIFIER_NODES__RAY_GROUND_CLASSIFIER_CLOUD_NODE_HPP_

#include <ray_ground_classifier/types.hpp>
#include <ray_ground_classifier/point_cloud_utils.hpp>
#include <ray_ground_classifier_nodes/visibility_control.hpp>
#include <ray_ground_classifier/ray_aggregator.hpp>
#include <ray_ground_classifier/ray_ground_classifier.hpp>
// #include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <ros/ros.h>
#include <ros/console.h>

using autoware::common::types::bool8_t;
using autoware::common::types::char8_t;

namespace autoware
{
namespace perception
{
namespace filters
{
namespace ray_ground_classifier_nodes
{

using sensor_msgs::PointCloud2;

/// \brief A node that takes in unstructured point clouds and partitions them into ground and
///        nonground points
class RayGroundClassifierCloudNode
{
public:
  /// \brief default constructor, starts node
  /// \param[in] options an rclcpp::NodeOptions object to configure the node
  /// \throw std::runtime_error if configuration fails
  explicit RayGroundClassifierCloudNode(ros::NodeHandle& nh);

private:
  /// \brief Resets state of ray aggregator and messages
  RAY_GROUND_CLASSIFIER_NODES_LOCAL void reset();
  // Algorithmic core
  ray_ground_classifier::RayGroundClassifier m_classifier;
  ray_ground_classifier::RayAggregator m_aggregator;
  // preallocated message
  PointCloud2 m_ground_msg;
  PointCloud2 m_nonground_msg;
  const std::size_t m_pcl_size;
  const std::string m_frame_id;
  // Basic stateful stuff, will get refactored after we have a proper state machine implementation
  bool8_t m_has_failed;
  // publishers and subscribers
  const std::chrono::nanoseconds m_timeout;
  ros::Subscriber m_raw_sub_ptr; //PointCloud2
  std::shared_ptr<ros::Publisher> m_ground_pub_ptr; //PointCloud2
  std::shared_ptr<ros::Publisher> m_nonground_pub_ptr; //PointCloud2
  /// \brief Read samples from the subscription
  void callback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg);
  uint32_t m_ground_pc_idx;
  uint32_t m_nonground_pc_idx;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};  // class RayGroundFilterDriverNode
}  // namespace ray_ground_classifier_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware
#endif  // RAY_GROUND_CLASSIFIER_NODES__RAY_GROUND_CLASSIFIER_CLOUD_NODE_HPP_
