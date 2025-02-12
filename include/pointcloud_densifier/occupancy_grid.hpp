#ifndef POINTCLOUD_DENSIFIER__OCCUPANCY_GRID_HPP_
#define POINTCLOUD_DENSIFIER__OCCUPANCY_GRID_HPP_

#include <vector>
#include <pcl/point_cloud.h>
#include "autoware_point_types/types.hpp"

namespace pointcloud_densifier
{
using autoware_point_types::PointXYZIRC;

class OccupancyGrid
{
public:
  OccupancyGrid(double x_min, double x_max, double y_min, double y_max, double resolution);
  void updateOccupancy(const pcl::PointCloud<PointXYZIRC>& cloud);
  bool isOccupied(double x, double y) const;
  
private:
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double resolution_;
  size_t cols_;
  size_t rows_;
  std::vector<bool> grid_;
  size_t index(size_t row, size_t col) const;
};

} // namespace pointcloud_densifier

#endif  // POINTCLOUD_DENSIFIER__OCCUPANCY_GRID_HPP_
