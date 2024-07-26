#include "dataloader/dataloader.hpp"
#include "viewer/viewer.hpp"
#include <iostream>

#include <open3d/Open3D.h>
#include "icp/icp.hpp"
#include <Eigen/Core>
#include <Eigen/LU>

int main(int argc, char const *argv[]) {
    if (argc != 2) {
        std::cerr << "[ERROR] Please provide the path to a directory containing point cloud files" << std::endl;
        return 1;
    }

    const std::string data_root_dir(argv[1]);

std::cout << "Hello, World!" << std::endl;

dataset::LaserScanDataset laser_scan_dataset(data_root_dir);
std::size_t num_scans = laser_scan_dataset.size();
std::cout << "Number of laser scans: " << num_scans << std::endl;


    if (num_scans > 0) {
        auto pcd = laser_scan_dataset[0];//viewing pcd
    //     for (int i = 0; i < 10; i++) {
    //     std::cout << "Point " << i << ": " << pcd[i].transpose() << std::endl;
    // }
        viewCloud(pcd);
    }
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();

    for (size_t i = 1; i < num_scans; i++) {
        auto source_points = laser_scan_dataset[i - 1];
        auto target_points = laser_scan_dataset[i];

        // std::cout << "Source points size: " << source_points.size() << std::endl;
        // std::cout << "Target points size: " << target_points.size() << std::endl;

        if (source_points.size() != target_points.size()) {
            // std::cerr << "Source and target points mismatch. Skipping this pair." << std::endl;
            continue; // Skip this pair of scans
        }

        // std::cout << "Source points sample: " << std::endl;
        // for (size_t j = 0; j < std::min(source_points.size(), size_t(10)); ++j) {
        //     std::cout << source_points[j].transpose() << std::endl;
        // }
        
        // std::cout << "Target points sample: " << std::endl;
        // for (size_t j = 0; j < std::min(target_points.size(), size_t(10)); ++j) {
        //     std::cout << target_points[j].transpose() << std::endl;
        // }

        for (size_t j = 0; j < source_points.size(); j++) {
            Eigen::Vector2d p = source_points[j];
            Eigen::Vector2d q = target_points[j];
            Eigen::Vector2d error = errorij(p, q, T);
            // std::cout << "Error: " << error.transpose() << std::endl;

            Eigen::Matrix<double, 2, 3> J = Jacobian(p, q, T);
            // std::cout << "Jacobian: " << J << std::endl;

            Eigen::Matrix<double, 3, 3> H;
            Eigen::Vector3d b;
            linear_system({{p, q}}, T, H, b);
            // std::cout << "H: " << H << std::endl;
            // std::cout << "b: " << b.transpose() << std::endl;

            Eigen::Vector3d delta = delta_x(H, b);
            // std::cout << "Delta: " << delta.transpose() << std::endl;


            T = apply_transformation(delta, T);
            // std::cout << "Transformation: " << T << std::endl;
            
        }
    }
    
return 0;

}


