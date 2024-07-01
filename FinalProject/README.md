### Modern C++ 2024 Final Project: 2D Lidar odometry in the Real-World 



# Lidar Odometry Project

This repository contains code for a Lidar Odometry project, focusing to calculate motion by indentation and pose by aggregation from the given laser scan data.

## Table of Contents

- [Lidar Odometry Project](#lidar-odometry-project)
  - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
  - [Setup](#setup)
    - [CMake](#cmake)
  - [Data Loading](#data-loading)
    - [`LaserScanDataset` Class](#laserscandataset-class)
  - [Data Visualization](#data-visualization)
  - [Next Steps](#next-steps)
  - [Author](#author)
  - [COntributors](#contributors)


## Introduction

This project aims to implement 2D Lidar Odometry using C++. Lidar data is stored in binary files (.bin).

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
Intial 2D lidar dat will look as (ScreenCapture_2024-07-01-15-47-21.png and ScreenCapture_2024-07-01-15-37-39.png)


## Next Steps

- To calculate motion by indentation and pose by aggregation

## Author

- Prasanna.Bijja
- GitHub: [Your GitHub Profile](https://github.com/prasanna1511)


## COntributors

- tizianoGuadagnino
- saurabh1002

---