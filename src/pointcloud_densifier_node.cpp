// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/*
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "pointcloud_densifier/pointcloud_densifier_node.hpp"
#include "pointcloud_densifier/occupancy_grid.hpp"  // new include

namespace pointcloud_densifier
{

PointCloudDensifierNode::PointCloudDensifierNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_densifier_node", options),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  using std::placeholders::_1;
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&PointCloudDensifierNode::onPointCloud, this, _1));

  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", 10);

  // Declare and get parameters (note the new grid_resolution parameter)
  this->declare_parameter<int>("num_previous_frames", 1);
  this->declare_parameter<double>("x_min", 80.0);
  this->declare_parameter<double>("x_max", 200.0);
  this->declare_parameter<double>("y_min", -20.0);
  this->declare_parameter<double>("y_max", 20.0);
  this->declare_parameter<double>("grid_resolution", 0.30);

  num_previous_frames_ = this->get_parameter("num_previous_frames").as_int();
  x_min_ = this->get_parameter("x_min").as_double();
  x_max_ = this->get_parameter("x_max").as_double();
  y_min_ = this->get_parameter("y_min").as_double();
  y_max_ = this->get_parameter("y_max").as_double();
  grid_resolution_ = this->get_parameter("grid_resolution").as_double();
}

void PointCloudDensifierNode::onPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{


  // Measure time
  auto start_time = std::chrono::high_resolution_clock::now();


  // Convert ROS message to PCL point cloud.
  pcl::PointCloud<PointXYZIRC>::Ptr raw_pointcloud_ptr(new pcl::PointCloud<PointXYZIRC>);
  pcl::fromROSMsg(*input_msg, *raw_pointcloud_ptr);
  RCLCPP_INFO(get_logger(), "Received point cloud with %ld points", raw_pointcloud_ptr->size());

  // -----------------------------
  // FILTER CURRENT POINT CLOUD (ROI)
  pcl::PointCloud<PointXYZIRC>::Ptr far_front_pointcloud_ptr(new pcl::PointCloud<PointXYZIRC>);
  for (const auto & point : raw_pointcloud_ptr->points) {
    if (point.x > x_min_ && point.x < x_max_ && point.y > y_min_ && point.y < y_max_) {
      far_front_pointcloud_ptr->push_back(point);
    }
  }
  
  // Build occupancy grid from the current filtered cloud.
  pointcloud_densifier::OccupancyGrid occupancy_grid(x_min_, x_max_, y_min_, y_max_, grid_resolution_);
  occupancy_grid.updateOccupancy(*far_front_pointcloud_ptr);

  // Start with current (static) points.
  pcl::PointCloud<PointXYZIRC>::Ptr combined_pointcloud_ptr(new pcl::PointCloud<PointXYZIRC>);
  *combined_pointcloud_ptr = *raw_pointcloud_ptr;

  // -----------------------------
  // TRANSFORM AND MERGE PREVIOUS CLOUDS (using occupancy grid filtering)
  for (const auto & previous_pointcloud : previous_pointclouds_) {
    if (!previous_pointcloud.cloud || previous_pointcloud.cloud->empty()) {
      continue;
    }
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      tf2::TimePoint time_point = tf2::TimePoint(
        std::chrono::nanoseconds(input_msg->header.stamp.nanosec) +
        std::chrono::seconds(input_msg->header.stamp.sec));
      tf2::TimePoint prev_time_point = tf2::TimePoint(
        std::chrono::nanoseconds(previous_pointcloud.header.stamp.nanosec) +
        std::chrono::seconds(previous_pointcloud.header.stamp.sec));
      transform_stamped = tf_buffer_->lookupTransform(
        input_msg->header.frame_id,
        time_point,
        previous_pointcloud.header.frame_id,
        prev_time_point,
        "map"  // fixed frame
      );
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "%s", ex.what());
      continue;
    }
    
    RCLCPP_INFO(get_logger(), "Transformed point cloud from %s to %s",
      previous_pointcloud.header.frame_id.c_str(), input_msg->header.frame_id.c_str());

    // Validate and obtain the transformation.
    Eigen::Isometry3d transform_eigen;
    try {
      transform_eigen = tf2::transformToEigen(transform_stamped);
      if (!transform_eigen.matrix().allFinite()) {
        RCLCPP_WARN(get_logger(), "Invalid transform matrix, skipping point cloud");
        continue;
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Transform error: %s", ex.what());
      continue;
    }
    
    // Transform previous point cloud.
    pcl::PointCloud<PointXYZIRC> transformed_previous_cloud;
    try {
      transformPointCloudXYZIRC(*previous_pointcloud.cloud, transformed_previous_cloud,
                                 transform_eigen.matrix());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Point cloud transformation failed: %s", ex.what());
      continue;
    }
    
    // Add previous points only if they fall into occupied grid cells.
    for (const auto & point : transformed_previous_cloud.points) {
      if (occupancy_grid.isOccupied(point.x, point.y)) {
        combined_pointcloud_ptr->push_back(point);
      }
    }
  }

  // -----------------------------
  // Store the current filtered point cloud for future use.
  StoredPointCloud current_pointcloud;
  current_pointcloud.cloud = far_front_pointcloud_ptr;
  current_pointcloud.header = input_msg->header;
  if (previous_pointclouds_.size() >= static_cast<size_t>(num_previous_frames_)) {
    previous_pointclouds_.pop_front();
  }
  previous_pointclouds_.push_back(current_pointcloud);

  // Convert merged PCL cloud to ROS message.
  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*combined_pointcloud_ptr, output_msg);
  output_msg.header = input_msg->header;

  // Publish the combined point cloud.
  pointcloud_pub_->publish(output_msg);
  RCLCPP_INFO(get_logger(), "Published point cloud with %ld points", combined_pointcloud_ptr->size());

  // Measure time
  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_time = end_time - start_time;
  RCLCPP_INFO(get_logger(), "Processing time: %f [s]", elapsed_time.count());
}

void PointCloudDensifierNode::transformPointCloudXYZIRC(
  const pcl::PointCloud<PointXYZIRC>& cloud_in,
  pcl::PointCloud<PointXYZIRC>& cloud_out,
  const Eigen::Matrix4d& transform
) {
  cloud_out.points.resize(cloud_in.points.size());
  for (size_t i = 0; i < cloud_in.points.size(); ++i) {
    const auto& point_in = cloud_in.points[i];
    auto& point_out = cloud_out.points[i];
    
    Eigen::Vector4d pt(point_in.x, point_in.y, point_in.z, 1.0);
    pt = transform * pt;
    
    point_out.x = pt[0];
    point_out.y = pt[1];
    point_out.z = pt[2];
    point_out.intensity = point_in.intensity;
    point_out.return_type = point_in.return_type;
    point_out.channel = point_in.channel;
  }
}

}  

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_densifier::PointCloudDensifierNode)
