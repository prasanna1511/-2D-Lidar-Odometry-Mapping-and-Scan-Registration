#include "icp/icp.hpp"
#include <algorithm>
#include <iostream>
#include <open3d/Open3D.h>
#include <open3d/core/ShapeUtil.h>
#include <utility>
#include <vector>
#include "icp/icp.hpp"


std::vector<Eigen::Matrix3d> Transformations(const std::vector<Eigen::Vector2d> &pointcloud){
    std::vector<Eigen::Matrix3d> transformations;
    transformations.reserve(pointcloud.size()-1);

for (size_t i=1; i<pointcloud.size(); i++){
    double dx = pointcloud[i].x() - pointcloud[i-1].x();
    double dy = pointcloud[i].y() - pointcloud[i-1].y();
    double theta = std::atan2(dy, dx);

    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    T.block<2,2>(0,0) << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);
    T.block<2,1>(0,2) << dx, dy;

    transformations.push_back(T);

}
return transformations;
}
