#pragma once

#include <Eigen/Core>
#include <unordered_map>
#include <vector>
#include <utility>


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
