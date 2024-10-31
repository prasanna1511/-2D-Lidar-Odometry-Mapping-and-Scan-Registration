#include <iostream>
#include <vector>
#include "dataloader/dataloader.hpp"
#include "viewer/viewer.hpp"
#include "icp/icp.hpp"
#include <chrono>
#include <iomanip>
#include <open3d/Open3D.h>

int main() {
    // Load the dataset
    std::string dataRoot = "data/";
    dataset::LaserScanDataset laserData(dataRoot);

    int numScans = laserData.size();
    int scansToProcess = numScans - 1;
    double pixelSize = 0.08;
    dataset::LaserScanDataset::PointCloud src = laserData[0];
    dataset::LaserScanDataset::PointCloud target;

        
    int progress = 0;
    for (int iter = 1; iter <= scansToProcess; iter++) {
        target = laserData[iter];
        icp::icpUnknownCorrespondence(src, target, pixelSize);
        src = icp::concatenatePointClouds(src, target);
        src = icp::downsamplePointCloud(src, 0.13, 1);
        target.clear();
    }
    viewCloud(src);

    return 0;
}

