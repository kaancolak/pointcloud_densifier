# Pointcloud Densifier

Enhances sparse point cloud data by leveraging information from previous LiDAR frames, creating a denser representation of the long range points.

## Overview

The pointcloud_densifier package combines multiple frames of LiDAR data to create a denser point cloud output. 

## Input/Output

| Name     | Type                    | Description                                |
|----------|------------------------|--------------------------------------------|
| `input`  | `sensor_msgs/PointCloud2` | Input point cloud from LiDAR              |
| `output` | `sensor_msgs/PointCloud2` | Densified point cloud  |

## Parameters

### Core Parameters

| Name                  | Type   | Default Value | Description                                      |
|----------------------|--------|---------------|--------------------------------------------------|
| `num_previous_frames`| int    | 2             | Number of previous frames to combine              |
| `x_min`              | float  | 80.0          | Minimum x coordinate of ROI in meters            |
| `x_max`              | float  | 200.0         | Maximum x coordinate of ROI in meters            |
| `y_min`              | float  | -20.0         | Minimum y coordinate of ROI in meters            |
| `y_max`              | float  | 20.0          | Maximum y coordinate of ROI in meters            |