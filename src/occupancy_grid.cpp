#include "pointcloud_densifier/occupancy_grid.hpp"
#include <cmath>
#include <algorithm>

namespace pointcloud_densifier
{

OccupancyGrid::OccupancyGrid(double x_min, double x_max, double y_min, double y_max, double resolution)
: x_min_(x_min), x_max_(x_max), y_min_(y_min), y_max_(y_max), resolution_(resolution)
{
  cols_ = static_cast<size_t>(std::ceil((x_max_ - x_min_) / resolution_));
  rows_ = static_cast<size_t>(std::ceil((y_max_ - y_min_) / resolution_));
  grid_.resize(rows_ * cols_, false);
}

void OccupancyGrid::updateOccupancy(const pcl::PointCloud<PointXYZIRC>& cloud)
{
  std::fill(grid_.begin(), grid_.end(), false);
  for (const auto & point : cloud.points) {
    if (point.x < x_min_ || point.x >= x_max_ ||
        point.y < y_min_ || point.y >= y_max_) {
      continue;
    }
    size_t col = static_cast<size_t>((point.x - x_min_) / resolution_);
    size_t row = static_cast<size_t>((point.y - y_min_) / resolution_);
    grid_[ index(row, col) ] = true;
  }
}

bool OccupancyGrid::isOccupied(double x, double y) const
{
  if (x < x_min_ || x >= x_max_ || y < y_min_ || y >= y_max_) return false;
  size_t col = static_cast<size_t>((x - x_min_) / resolution_);
  size_t row = static_cast<size_t>((y - y_min_) / resolution_);
  return grid_[ index(row, col) ];
}

size_t OccupancyGrid::index(size_t row, size_t col) const
{
  return row * cols_ + col;
}

}  // namespace pointcloud_densifier
