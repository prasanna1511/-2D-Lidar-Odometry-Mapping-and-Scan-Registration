# 2D Lidar-Odometry Mapping and Scan Registration

This C++ project implements a robust 2D mapping pipeline using Lidar and odometry data. It leverages the Iterative Closest Point (ICP) algorithm for scan registration and accumulates aligned point clouds into a global map. The result is a high-quality 2D map suitable for robotics and navigation applications.

## Features

- Reads and preprocesses 2D Lidar scan data
- Registers consecutive scans using the ICP algorithm
- Accumulates registered scans into a global map
- Outputs both the initial scan and the constructed 2D map
- Modular CMake-based build system

## Installation

1. Clone the repository and navigate to its root directory.
2. Build the project using CMake:
   ```bash
   cmake -Bbuild
   cmake --build build
   ```
3. Run the main program with the following command:
   ```bash
   ./build/main data/
   ```

## Methodology

### Point Cloud Preprocessing
Lidar scan data is read and filtered to remove outliers and noise before registration.

### Iterative Closest Point (ICP) Algorithm
The ICP algorithm aligns each new scan to the previously accumulated map, estimating the relative transformation.

### Point Cloud Transformation and Accumulation
Aligned scans are transformed and merged to incrementally build a global 2D map.

## Output

1. **Single Point Cloud**

    ![First scan](https://github.com/prasanna1511/2D-Lidar-Odometry/assets/53254596/4deb6fae-0451-40b2-b16b-c516c7d5a993)
2. **Accumulated Map**

    ![2d map](https://github.com/prasanna1511/2D-Lidar-Odometry/blob/main/FinalProject/Screenshot%20from%202024-10-31%2011-52-16.png)

## Author

- Prasanna Bijja

## Contributors

- tizianoGuadagnino
- saurabh1002

---
