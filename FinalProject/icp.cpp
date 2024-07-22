#include "icp/icp.hpp"
#include <unordered_map>
#include <cmath>
#include <fstream>
#include <iostream>


Eigen::Vector2d centroid(const std::vector<Eigen::Vector2d> &points){
    Eigen::Vector2d centroid;
    for (const auto &point : points) {
        centroid += point;
    }
    return centroid / points.size();

}

Eigen::Vector2d covariance(const std::vector<Eigen::Vector2d> &source_points, 
                            const Eigen::Vector2d &source_mean, 
                           const std::vector<Eigen::Vector2d> &target_points, 
                           const Eigen::Vector2d &target_mean){
    Eigen::Vector2d covariance;
    for (std::size_t i = 0; i < source_points.size(); ++i) {
        covariance += (source_points[i] - source_mean).cwiseProduct(target_points[i] - target_mean);
        covariance = covariance / source_points.size();
                           }
    return covariance;
}


Eigen::Matrix3d save_transformation(const Eigen::Matrix3d &old_transform, 
                                    const Eigen::Matrix3d &new_transform){
    return new_transform * old_transform;
                                    }


double error(const std::vector<Eigen::Vector2d> &source_points, 
            const std::vector<Eigen::Vector2d> &target_points, 
            const Eigen::Matrix3d &transform){
    double error = 0.0;
    for (std::size_t i = 0; i < source_points.size(); ++i) {
        Eigen::Vector3d source_point = transform * Eigen::Vector3d(source_points[i].x(), source_points[i].y(), 1.0);
        error += (source_point.head<2>() - target_points[i]).squaredNorm();
            }
    return error;
}



