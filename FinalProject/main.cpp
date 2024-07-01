

#include "dataloader/dataloader.hpp"
#include "viewer/viewer.hpp"
#include <iostream>

#include <open3d/Open3D.h>


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
        viewCloud(pcd);
    }


return 0;


}


