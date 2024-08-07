# Modern C++ 2024 Final Project: 2D Lidar odometry in the Real-World 



This repository contains code for a Lidar Odometry project, focusing to calculate motion by indentation and pose by aggregation from the given laser scan data.

## Table of Contents

- [Lidar Odometry Project](#lidar-odometry-project)

  - [Introduction](#introduction)
  - [Setup](#setup)
  - [ICP-Registartion](#icp)




## Introduction

This project aims to 2D Odometry map 

## Setup

To run this project, you will need:

- C++ compiler (supporting C++17)
- Eigen library for linear algebra operations
- Open3D library for 3D visualization


### CMake

Ensure CMake is properly configured to build the project. Example `CMakeLists.txt` files are provided for `dataloader` and `viewer`.

## Data Loading

### `LaserScanDataset` Class

The `LaserScanDataset` class loads `.bin` files containing 2D point cloud data. Each file is processed to extract `x` and `y` coordinates using the `ReadLaserScan` function.

## Data Visualization

```
./build/main data
```
Intial 2D lidar dat will look as ![ScreenCapture_2024-07-01-15-37-39](https://github.com/prasanna1511/2D-Lidar-Odometry/assets/53254596/4deb6fae-0451-40b2-b16b-c516c7d5a993)


## Next Steps

- find correspondences
- compute TRansformations for the correspondences(rotation,translation)
- apply the transformations to the target scan
- extract the transformed poins
- accumulate the transformed points, by applying icp for consecutive scans

## Author

- Prasanna.Bijja
- GitHub: [Your GitHub Profile](https://github.com/prasanna1511)
