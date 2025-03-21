# C++ Project: 2D Lidar odometry in the Real-World 

## Introduction

To build 2D Odometry map 

single pointcloud ![ScreenCapture_2024-07-01-15-37-39](https://github.com/prasanna1511/2D-Lidar-Odometry/assets/53254596/4deb6fae-0451-40b2-b16b-c516c7d5a993)

## Steps

- find correspondences
- compute TRansformations for the correspondences(rotation,translation)
- apply the transformations to the target scan
- extract the transformed poins
- accumulate the transformed points, by applying icp for consecutive scans

### 2D Lidar Odometry with Iterative Closest Point (ICP) Algorithm

Building 2D Lidar odometry Map using ICP algorithm to compute relative pose of a robot in a 2D plane by aligning consecutive Lidar scans through an unknown correspondence ICP
---

### Prerequisites
- **C++17 or higher**: Required for modern C++ features.
- **Eigen**: Linear algebra library for matrix operations.
- **Open3D**: For visualization of 2D point clouds.
- **CMake**: For building the project.

### Installation
1. Clone the repository and navigate to its root directory.
2. Build the project using CMake:
   ```bash
   cmake -Bbuild
   cmake --build build
   ```
3. Run the main program with the following command:
   ```bash
   .build/main data/
   ```
---
first scan of 2D lidar dat will look as 

![ScreenCapture_2024-07-01-15-37-39](https://github.com/prasanna1511/2D-Lidar-Odometry/assets/53254596/4deb6fae-0451-40b2-b16b-c516c7d5a993)
## Methodology

### Point Cloud Preprocessing

1. **Downsampling**: 
2. **Grid Mapping**:

### Iterative Closest Point (ICP) Algorithm

The ICP algorithm aligns two consecutive scans to estimate the robot's movement and build its trajectory. This is implemented in the following steps:

1. **Unknown Correspondence ICP** 
2. **Nearest Neighbor Search** 
3. **Transformation Calculation**
   - **Covariance Matrix Calculation** 
   - **SVD Decomposition** 
4. **Error Calculation** 
5. **Iteration** 

### Point Cloud Transformation

1. **Applying Transformation**: transformation (rotation and translation) is applied to the source point cloud, aligning it to the target point cloud.
2. **Concatenation**: After each alignment, the newly transformed point cloud is concatenated with the cumulative point cloud to build map
---

###  Output:
1. **Initial Point Cloud**
2. **Accumulated Map**
   ![2d map](https://github.com/prasanna1511/2D-Lidar-Odometry/blob/main/FinalProject/Screenshot%20from%202024-10-31%2011-52-16.png)
---
## Contributors

- tizianoGuadagnino
- saurabh1002

---
## Author

- Prasanna.Bijja

