#include "icp/icp.hpp"
#include <unordered_map>
#include <cmath>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>


namespace icp {


using PointCloud = std::vector<Eigen::Vector2d>;

std::vector<Eigen::Vector2d> DownSample(const PointCloud &pcd, double voxel_size) {
    std::unordered_map<int, Eigen::Vector2d> downsampled_vox;
    for (const auto &point : pcd) {
        int x = std::round(point[0] / voxel_size);
        int y = std::round(point[1] / voxel_size);
        int key = x * 73856093 + y* 19349663;
        downsampled_vox[key] = point;
    }
    std::vector<Eigen::Vector2d> downsampled_pcd;
    for (const auto &pair : downsampled_vox) {
        downsampled_pcd.push_back(pair.second);
    }
    return downsampled_pcd;
}


int nearestNeigbour(const Eigen::Vector2d &point, const PointCloud &ref_pcd) {
    double min_dist = std::numeric_limits<double>::max();
    int near_idx = -1;
    for (int i = 0; i < ref_pcd.size(); i++) {
        double dist = (point - ref_pcd[i]).norm();
        if (dist < min_dist) {
            min_dist = dist;
            near_idx = i;
        }
    }
    return near_idx;
}

Eigen::Vector2d Mean(const PointCloud &pcd) {
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (const auto &point : pcd) {
        mean += point;
    }
    return mean / pcd.size();
}

std::vector<Eigen::Vector2d> substractMean(const PointCloud &pcd, const Eigen::Vector2d &mean){
    std::vector<Eigen::Vector2d> centered_pcd;
    for (const auto &point : pcd) {
    centered_pcd.push_back(point - mean);
}
    return centered_pcd;

}

Eigen::Matrix2d Covariance(const std::vector<Eigen::Vector2d>& p_prime, const std::vector<Eigen::Vector2d>& q_prime) {
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (int i = 0; i < p_prime.size(); i++) {
        covariance += p_prime[i] * q_prime[i].transpose();
    }
    return covariance / p_prime.size();
}

Eigen::Matrix2d R(const Eigen::Matrix2d &covariance) {
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();
    Eigen::Matrix2d R = V * U.transpose();
    return R;
}

Eigen::Vector2d t(const Eigen::Vector2d &mean_pcd, const Eigen::Vector2d &mean_ref_pcd, const Eigen::Matrix2d &R) {
    Eigen::Vector2d t = mean_ref_pcd - R * mean_pcd;
    return t;
}

Eigen::Matrix3d ApplyTransformation(const Eigen::Matrix2d &R, const Eigen::Vector2d &t) {
    Eigen::Matrix3d transformation = Eigen::Matrix3d::Identity();
    transformation.topLeftCorner<2, 2>() = R;
    transformation.topRightCorner<2, 1>() = t;
    return transformation;
}




} // namespace icp
