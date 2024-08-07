#pragma once
#include <unordered_map>
#include <vector>
#include <Eigen/Dense>


namespace icp {
struct Pixel {
    int x, y;
    Pixel(int x_, int y_) : x(x_), y(y_) {}
    bool operator==(const Pixel &other) const {
        return x == other.x && y == other.y;
    }
};

struct PixelHash {
    std::size_t operator()(const Pixel &p) const {
        return std::hash<int>()(p.x) ^ std::hash<int>()(p.y);
    }
};

using PointCloud = std::vector<Eigen::Vector2d>;

// Function declarations
std::unordered_map<Pixel, std::vector<Eigen::Vector2d>, PixelHash> GridMap(const std::vector<Eigen::Vector2d> &coords, double pixel_size);
std::vector<Eigen::Vector2d> downsample(const std::vector<Eigen::Vector2d> &coords, double voxel_size);
std::tuple<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> nearestNeighbor(const std::vector<Eigen::Vector2d> &source, const std::vector<Eigen::Vector2d> &target);
Eigen::Vector2d centroid(const std::vector<Eigen::Vector2d> &coords);
Eigen::Matrix2d Covariance(const PointCloud &source, const PointCloud &target);
Eigen::Matrix2d R(const Eigen::Matrix2d &cov);
Eigen::Matrix3d icp_known_correspondence(const std::vector<Eigen::Vector2d> &s_correspondences, const std::vector<Eigen::Vector2d> &t_correspondences);
Eigen::Matrix3d icp_unknown_correspondence(const std::vector<Eigen::Vector2d> &src_, const std::vector<Eigen::Vector2d> &target, const double &pixel_size);
std::vector<Eigen::Vector2d> apply_transformation(const Eigen::Matrix3d &transformation, const std::vector<Eigen::Vector2d> &src);
std::vector<Eigen::Vector2d> concat_pointclouds(std::vector<Eigen::Vector2d> &first, const std::vector<Eigen::Vector2d> &second);
std::vector<Eigen::Vector2d> downsample(const std::vector<Eigen::Vector2d> &vec, const double &pixel_size, const int &n_points);
double Error(const std::vector<Eigen::Vector2d> &source, const std::vector<Eigen::Vector2d> &target, const Eigen::Matrix3d &T);
Eigen::MatrixXd Jacobian(const Eigen::Vector2d &p, const Eigen::Vector2d &q, const Eigen::Matrix3d &T);

} // namespace icp
