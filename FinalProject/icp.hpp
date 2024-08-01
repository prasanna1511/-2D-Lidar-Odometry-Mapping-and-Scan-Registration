#pragma once

#include <Eigen/Core>
#include <unordered_map>
#include <vector>
#include <utility>
#include <functional>
#include <Eigen/Dense>


namespace icp {
using PointCloud = std::vector<Eigen::Vector2d>;

struct ICPResult{
    std::vector<Eigen::Matrix3d> transformations;
    std::vector<Eigen::Vector2d> final_pcd;
};

std::vector<Eigen::Vector2d> DownSample(const PointCloud &pcd, double voxel_size);

int nearestNeigbour(const Eigen::Vector2d &point, const PointCloud &ref_pcd);

Eigen::Vector2d Mean(const PointCloud &pcd);

std::vector<Eigen::Vector2d> substractMean(const PointCloud &pcd, const Eigen::Vector2d &mean);

Eigen::Matrix2d Covariance(const std::vector<Eigen::Matrix<double, 2, 1>>& p_prime, const std::vector<Eigen::Matrix<double, 2, 1>>& q_prime);

Eigen::Matrix2d R(const Eigen::Matrix2d &covariance);

Eigen::Vector2d t(const Eigen::Vector2d &mean_pcd, const Eigen::Vector2d &mean_ref_pcd, const Eigen::Matrix2d &R);

Eigen::Matrix3d ApplyTransformation(const Eigen::Matrix2d &R, const Eigen::Vector2d &t);

ICPResult icp(const std::vector<Eigen::Vector2d> &pcds, double voxel_size, int max_iter, double tolerance);


}// namespace icp
