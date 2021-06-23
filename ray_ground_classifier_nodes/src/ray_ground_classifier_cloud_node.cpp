// Copyright 2018-2020 the Autoware Foundation, Arm Limited
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
#include <ray_ground_classifier/types.hpp>
#include <ray_ground_classifier/point_cloud_utils.hpp>
#include <ray_ground_classifier_nodes/ray_ground_classifier_cloud_node.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_components/register_node_macro.hpp>
#include <stdlib.h>

#include <ros/console.h>

#include <limits>
#include <string>

namespace autoware
{
namespace perception
{
namespace filters
{
namespace ray_ground_classifier_nodes
{
////////////////////////////////////////////////////////////////////////////////
using autoware::common::types::PointXYZIF;
using autoware::common::types::float32_t;
using autoware::perception::filters::ray_ground_classifier::PointPtrBlock;

using std::placeholders::_1;

using autoware::common::lidar_utils::has_intensity_and_throw_if_no_xyz;
using autoware::common::lidar_utils::init_pcl_msg;
using autoware::common::lidar_utils::add_point_to_cloud_raw;

RayGroundClassifierCloudNode::RayGroundClassifierCloudNode(ros::NodeHandle& nh)
: //Node("ray_ground_classifier", node_options),
  // m_classifier(ray_ground_classifier::Config{
  //         static_cast<float32_t>(declare_parameter("classifier.sensor_height_m").get<float32_t>()),
  //         static_cast<float32_t>(declare_parameter(
  //           "classifier.max_local_slope_deg").get<float32_t>()),
  //         static_cast<float32_t>(declare_parameter(
  //           "classifier.max_global_slope_deg").get<float32_t>()),
  //         static_cast<float32_t>(declare_parameter(
  //           "classifier.nonground_retro_thresh_deg").get<float32_t>()),
  //         static_cast<float32_t>(declare_parameter(
  //           "classifier.min_height_thresh_m").get<float32_t>()),
  //         static_cast<float32_t>(declare_parameter(
  //           "classifier.max_global_height_thresh_m").get<float32_t>()),
  //         static_cast<float32_t>(declare_parameter(
  //           "classifier.max_last_local_ground_thresh_m").get<float32_t>()),
  //         static_cast<float32_t>(declare_parameter(
  //           "classifier.max_provisional_ground_distance_m").get<float32_t>())
  //       }),
  m_classifier(ray_ground_classifier::Config{
          0.0, //0.58, //sensor_height_m How high the pointcloud frame center off the ground
          10.0, // 15.0,// max_local_slope_deg Maximum permissible slope for two ground points within reclass_threshold
          5.7, // max_global_slope_deg Maximum permissible slope from base footprint of sensor
          70.0, // nonground_retro_thresh_deg How steep consecutive points need to be to retroactively annotate a point as nonground
          0.05, // min_height_thresh_m Local height threshold can be no less than this
          1.45, // max_global_height_thresh_m height threshold can be no more than this for both global and local cone
          1.6, // max_last_local_ground_thresh_m Saturation threshold for locality wrt last ground point (for classifying as ground from nonground)
          5.0// max_provisional_ground_distance_m Max radial distance until provisional ground is not influenced by next points
        }),
  // m_aggregator(ray_ground_classifier::RayAggregator::Config{
  //         static_cast<float32_t>(declare_parameter(
  //           "aggregator.min_ray_angle_rad").get<float32_t>()),
  //         static_cast<float32_t>(declare_parameter(
  //           "aggregator.max_ray_angle_rad").get<float32_t>()),
  //         static_cast<float32_t>(declare_parameter("aggregator.ray_width_rad").get<float32_t>()),
  //         static_cast<std::size_t>(
  //           declare_parameter("aggregator.max_ray_points").get<std::size_t>())
  //       }),
  m_aggregator(ray_ground_classifier::RayAggregator::Config{
         -3.14159, // min_ray_angle_rad
        3.14159, // max_ray_angle_rad
            0.001, // ray_width_rad
           256 // min_ray_points
        }),

  m_pcl_size(28800), //pcl_size
  m_frame_id("base_link"),
  m_has_failed(false),
  m_timeout(110), //cloud_timeout_ms
  m_ground_pc_idx{0},
  m_nonground_pc_idx{0},
  tf_listener_{tf_buffer_}
{
  // initialize messages
  init_pcl_msg(m_ground_msg, m_frame_id.c_str(), m_pcl_size);
  init_pcl_msg(m_nonground_msg, m_frame_id.c_str(), m_pcl_size);

 m_raw_sub_ptr = nh.subscribe("points_in", 1, &RayGroundClassifierCloudNode::callback, this);

  // m_raw_sub_ptr(create_subscription<PointCloud2>(
  //     "points_in",
  //     rclcpp::QoS(10), std::bind(&RayGroundClassifierCloudNode::callback, this, _1))),
  // m_ground_pub_ptr(create_publisher<PointCloud2>(
  //     "points_ground", rclcpp::QoS(10))),
  // m_nonground_pub_ptr(create_publisher<PointCloud2>(
  //     "points_nonground", rclcpp::QoS(10))),

  m_ground_pub_ptr = std::make_shared<ros::Publisher>();
  *m_ground_pub_ptr = nh.advertise<sensor_msgs::PointCloud2>("points_ground", 10);
  m_nonground_pub_ptr = std::make_shared<ros::Publisher>();
  *m_nonground_pub_ptr = nh.advertise<sensor_msgs::PointCloud2>("points_nonground", 10);

}

////////////////////////////////////////////////////////////////////////////////
void
RayGroundClassifierCloudNode::callback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr input(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::copyPointCloud(*msg, *input);

  PointXYZIF pt_tmp;
  pt_tmp.id = static_cast<uint16_t>(PointXYZIF::END_OF_SCAN_ID);
  const ray_ground_classifier::PointXYZIFR eos_pt{&pt_tmp};

  // transform to m_frame_id
  if (input->header.frame_id != m_frame_id)
  {
    try
    {
      constexpr double transform_wait_time {0.2};
      geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
          m_frame_id, input->header.frame_id , pcl_conversions::fromPCL(input->header).stamp, ros::Duration{transform_wait_time});

      // Eigen::Matrix4f eigen_transform;
      // pcl_ros::transformAsMatrix(transform.transform, eigen_transform);
      // pcl_ros::transformPointCloud(eigen_transform, *msg, *msg);
      pcl_ros::transformPointCloud<pcl::PointXYZI>(*input, *input, transform.transform);
      input->header.frame_id = m_frame_id;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_ERROR_STREAM(ex.what());
      return;
    }
  }

  try {
    // Reset messages and aggregator to ensure they are in a good state
    reset();
    // Verify header
    if (input->header.frame_id != m_ground_msg.header.frame_id) {
      throw std::runtime_error(
              "RayGroundClassifierCloudNode: raw topic from unexpected "
              "frame (expected '" + m_ground_msg.header.frame_id +
              "', got '" + msg->header.frame_id + "')");
    }
    // Verify the consistency of PointCloud msg
    // const auto data_length = input->width * input->height * input->point_step;
    // if ((input->data.size() != input->row_step) || (data_length != input->row_step)) {
    //   throw std::runtime_error("RayGroundClassifierCloudNode: Malformed PointCloud2");
    // }
    // Verify the point cloud format and assign correct point_step
    // if (!has_intensity_and_throw_if_no_xyz(msg)) {
    //   ROS_WARN_STREAM(
    //     "RayGroundClassifierNode Warning: PointCloud doesn't have intensity field");
    // }
    // Harvest timestamp
    m_nonground_msg.header.stamp = pcl_conversions::fromPCL(input->header.stamp);
    m_ground_msg.header.stamp = pcl_conversions::fromPCL(input->header.stamp);
    // Add all points to aggregator
    // Iterate through the data, but skip intensity in case the point cloud does not have it.
    // For example:
    //
    // point_step = 4
    // x y z i a b c x y z i a b c
    // ^------       ^------
    size_t num_ready = 0;
    bool8_t has_encountered_unknown_exception = false;
    bool8_t abort = false;

    // std::cout << "pointcloud size: " << (msg->data.size()/msg->point_step) << std::endl;
    std::cout << "pointcloud size: " << input->points.size() << std::endl;
    // std::size_t point_step = msg->point_step;
    for (std::size_t idx = 0U; idx < input->points.size(); idx += 1) {
      if (abort) {
        continue;
      }
      try {
        // PointXYZIF * pt;
        // TODO(c.ho) Fix below deviation after #2131 is in
        //lint -e{925, 9110} Need to convert pointers and use bit for external API NOLINT
        pcl::PointXYZI pt_msg = input->points[idx];
        PointXYZIF* pt(new PointXYZIF());
        pt->x = pt_msg.x;
        pt->y = pt_msg.y;
        pt->z = pt_msg.z;
        pt->intensity = pt_msg.intensity;
        // don't bother inserting the points almost (0,0).
        // Too many of those makes the bin 0 overflow
        if ((std::isfinite(pt->x) &&std::isfinite(pt->y)) && ((fabsf(pt->x) > std::numeric_limits<decltype(pt->x)>::epsilon()) ||
          (fabsf(pt->y) > std::numeric_limits<decltype(pt->y)>::epsilon())))
        {
          if (!m_aggregator.insert(pt)) {
            m_aggregator.end_of_scan();
          }
        } else {
          uint32_t local_nonground_pc_idx;
          local_nonground_pc_idx = m_nonground_pc_idx++;
          if (!add_point_to_cloud_raw(m_nonground_msg, *pt, local_nonground_pc_idx)) {
            throw std::runtime_error(
                    "RayGroundClassifierNode: Overran nonground msg point capacity");
          }
        }
      } catch (const std::runtime_error & e) {
        m_has_failed = true;
        ROS_INFO_STREAM(e.what());
        abort = true;
      } catch (const std::exception & e) {
        m_has_failed = true;
        ROS_INFO_STREAM(e.what());
        abort = true;
      } catch (...) {
        ROS_INFO_STREAM(
          "RayGroundClassifierCloudNode has encountered an unknown failure");
        abort = true;
        has_encountered_unknown_exception = true;
      }
    }

    // if abort, we skip all remaining the parallel work to be able to return/throw
    if (!abort) {
      m_aggregator.end_of_scan();
      num_ready = m_aggregator.get_ready_ray_count();

      // Partition each ray
      for (size_t i = 0; i < num_ready; i++) {
        if (abort) {
          continue;
        }
        // Note: if an exception occurs in this loop, the aggregator can get into a bad state
        // (e.g. overrun capacity)
        PointPtrBlock ground_blk;
        PointPtrBlock nonground_blk;
        try {
          auto ray = m_aggregator.get_next_ray();
          // partition: should never fail, guaranteed to have capacity via other checks
          m_classifier.partition(ray, ground_blk, nonground_blk);

          // Add ray to point clouds
          for (auto & ground_point : ground_blk) {
            uint32_t local_ground_pc_idx;

            local_ground_pc_idx = m_ground_pc_idx++;
            if (!add_point_to_cloud_raw(
                m_ground_msg, *ground_point,
                local_ground_pc_idx))
            {
              throw std::runtime_error(
                      "RayGroundClassifierNode: Overran ground msg point capacity");
            }
          }
          for (auto & nonground_point : nonground_blk) {
            uint32_t local_nonground_pc_idx;
            local_nonground_pc_idx = m_nonground_pc_idx++;
            if (!add_point_to_cloud_raw(
                m_nonground_msg, *nonground_point,
                local_nonground_pc_idx))
            {
              throw std::runtime_error(
                      "RayGroundClassifierNode: Overran nonground msg point capacity");
            }
          }
        } catch (const std::runtime_error & e) {
          m_has_failed = true;
          ROS_INFO_STREAM(e.what());
          abort = true;
        } catch (const std::exception & e) {
          m_has_failed = true;
          ROS_INFO_STREAM(e.what());
          abort = true;
        } catch (...) {
          ROS_INFO_STREAM(
            "RayGroundClassifierCloudNode has encountered an unknown failure");
          abort = true;
          has_encountered_unknown_exception = true;
        }
      }
    }

    if (has_encountered_unknown_exception) {
      m_has_failed = true;
      ROS_INFO_STREAM(
        "RayGroundClassifierCloudNode has encountered an unknown failure");
      throw;
    }

    if (abort) {
      return;
    }

    // Resize the clouds down to their actual sizes.
    autoware::common::lidar_utils::resize_pcl_msg(m_ground_msg, m_ground_pc_idx);
    autoware::common::lidar_utils::resize_pcl_msg(m_nonground_msg, m_nonground_pc_idx);
    // publish: nonground first for the possible microseconds of latency
    m_nonground_pub_ptr->publish(m_nonground_msg);
    m_ground_pub_ptr->publish(m_ground_msg);
  } catch (const std::runtime_error & e) {
    m_has_failed = true;
    ROS_INFO_STREAM(e.what());
  } catch (const std::exception & e) {
    m_has_failed = true;
    ROS_INFO_STREAM(e.what());
  } catch (...) {
    ROS_INFO_STREAM(
      "RayGroundClassifierCloudNode has encountered an unknown failure");
    throw;
  }
}
////////////////////////////////////////////////////////////////////////////////
void RayGroundClassifierCloudNode::reset()
{
  // reset aggregator: Needed in case an error is thrown during partitioning of cloud
  //                   which would lead to filled rays and overflow during next callback
  m_aggregator.reset();
  // reset messages
  autoware::common::lidar_utils::reset_pcl_msg(m_ground_msg, m_pcl_size, m_ground_pc_idx);
  autoware::common::lidar_utils::reset_pcl_msg(m_nonground_msg, m_pcl_size, m_nonground_pc_idx);
}
}  // namespace ray_ground_classifier_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

