#pragma once

#include <Eigen/Core>
#include <unordered_map>
#include <vector>
#include <utility>
#include <functional>

namespace std {
    template <>
    struct hash<Eigen::Vector2d> {
        std::size_t operator()(const Eigen::Vector2d &vec) const noexcept {
            std::size_t h1 = std::hash<double>{}(vec.x());
            std::size_t h2 = std::hash<double>{}(vec.y());
            return h1 ^ (h2 << 1); // Combine the two hash values
        }
    };
}

Eigen::Vector2d centroid(const std::vector<Eigen::Vector2d> &points);

Eigen::Vector2d covariance(const std::vector<Eigen::Vector2d> &source_points, 
                            const Eigen::Vector2d &source_mean, 
                           const std::vector<Eigen::Vector2d> &target_points, 
                           const Eigen::Vector2d &target_mean);

Eigen::Matrix3d save_transformation(const Eigen::Matrix3d &old_transform, 
                                    const Eigen::Matrix3d &new_transform);

double error(const std::vector<Eigen::Vector2d> &source_points, 
            const std::vector<Eigen::Vector2d> &target_points, 
            const Eigen::Matrix3d &transform);

std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> neighbour_pixels(const Eigen::Vector2d &pixel, double range);

std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> GetAdjacentPixels(const Eigen::Vector2d &pixel, double range);

std::vector<Eigen::Vector2d> pixel_points(const std::unordered_map<Eigen::Vector2d, std::vector<Eigen::Vector2d>> &target_grid, const Eigen::Vector2d &pixel);

Eigen::Matrix3d icp(const std::vector<Eigen::Vector2d> &source_points, 
                    const std::vector<Eigen::Vector2d> &target_points, 
                    const Eigen::Matrix3d &initial_guess, 
                    const std::size_t max_iterations, 
                    const double tolerance);

std::unordered_map<Eigen::Vector2d, std::vector<Eigen::Vector2d>> grid(const std::vector<Eigen::Vector2d> &points);

std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> k_near(const Eigen::Vector2d &pixel, double range);


std::vector<Eigen::Vector2d> apply_transformation(const std::vector<Eigen::Vector2d> &points, 
                                                const Eigen::Matrix3d &transform);

std::vector<Eigen::Vector2d> concat_points(const std::vector<Eigen::Vector2d> &source_points, 
                                                const std::vector<Eigen::Vector2d> &target_points);

std::vector<Eigen::Vector2d> downsample(const std::vector<Eigen::Vector2d> &points, 
                                                const std::size_t factor);

std::vector<Eigen::Vector2d> downsample_mean(const std::vector<Eigen::Vector2d> &points,
                                                 const std::size_t factor);


