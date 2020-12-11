# Lidar Obstacle Detection

Implement Lidar obstacle detection algorithm on real Lidar sensor data.

- Down sampled a point cloud using a voxel grid filter
- Implenemted planar segmentation using RANSAC algorithm
- Performed Euclidean clustering algorithm with KD-Tree to cluster obstacle

## Files

```environment.cpp``` : main cpp file to start the program.
```processPointClouds.cpp/.h``` : Contain all the functions for processing Lidar sensor data.
```kdtree.h``` : KD-Tree structure implementation.
```sensors/data/pcd/*``` : Lidar sensor data(.pcd)

## Functions

```ProcessPointClouds::FilterCloud()``` : Reduce the number of points using voxelized grid approach.
```ProcessPointClouds::RansacPlane()``` : Segment the road plane from point cloud using RANSAC algorithm and return a std::pair of point cloud containing plane and obstacles respectively.
```ProcessPointClouds::euclideanClustering``` : Associate groups of points by how close together they are. Also use a KD-Tree data structure to speed up the look time from O(n) to O(log(n)).
