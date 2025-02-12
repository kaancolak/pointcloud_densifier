#ifndef POINTCLOUD_DENSIFIER__POINTCLOUD_DENSIFIER_NODE_HPP_
#define POINTCLOUD_DENSIFIER__POINTCLOUD_DENSIFIER_NODE_HPP_

#include <deque>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>
#include "autoware_point_types/types.hpp"
#include "pointcloud_densifier/occupancy_grid.hpp"

namespace pointcloud_densifier
{

using autoware_point_types::PointXYZIRC;  

struct StoredPointCloud {
  pcl::PointCloud<PointXYZIRC>::Ptr cloud;
  std_msgs::msg::Header header;
};

class PointCloudDensifierNode : public rclcpp::Node
{
public:
  explicit PointCloudDensifierNode(const rclcpp::NodeOptions & options);

private:
  void onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);

  void transformPointCloudXYZIRC(const pcl::PointCloud<PointXYZIRC>& cloud_in,
  pcl::PointCloud<PointXYZIRC>& cloud_out,const Eigen::Matrix4d& transform );

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::deque<StoredPointCloud> previous_pointclouds_;
  int num_previous_frames_;
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double grid_resolution_;  // new grid resolution parameter
  
};

}  // namespace pointcloud_densifier

#endif  // POINTCLOUD_DENSIFIER__POINTCLOUD_DENSIFIER_NODE_HPP_