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


std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> neighbour_pixels(const Eigen::Vector2d &pixel, double range){
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> neighbours;
    for (double x = pixel.x() - range; x <= pixel.x() + range; ++x) {
        for (double y = pixel.y() - range; y <= pixel.y() + range; ++y) {
            neighbours.emplace_back(Eigen::Vector2d(x, y), Eigen::Vector2d(x, y));
        }
    }
    return neighbours;
}


std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> GetAdjacentPixels(const Eigen::Vector2d &pixel, double range){
    return neighbour_pixels(pixel, range);
}

std::vector<Eigen::Vector2d> pixel_points(const std::unordered_map<Eigen::Vector2d, std::vector<Eigen::Vector2d>> &target_grid, const Eigen::Vector2d &pixel){
    auto it = target_grid.find(pixel);
    if (it != target_grid.end()) {
        return it->second;
    }
    return {};
}


Eigen::Matrix3d icp(const std::vector<Eigen::Vector2d> &source_points, 
                    const std::vector<Eigen::Vector2d> &target_points, 
                    const Eigen::Matrix3d &initial_guess, 
                    const std::size_t max_iterations, 
                    const double tolerance){
    Eigen::Matrix3d transform = initial_guess;
    for (std::size_t i = 0; i < max_iterations; ++i) {
        Eigen::Vector2d source_mean = centroid(source_points);
        Eigen::Vector2d target_mean = centroid(target_points);
        Eigen::Vector2d cov = covariance(source_points, source_mean, target_points, target_mean);
        Eigen::Matrix3d new_transform;
        new_transform << cov.x(), -cov.y(), target_mean.x(),
                         cov.y(), cov.x(), target_mean.y(),
                         0.0, 0.0, 1.0;
        transform = save_transformation(transform, new_transform);
        if (error(source_points, target_points, transform) < tolerance) {
            break;
        }
    }
    return transform;
}


std::unordered_map<Eigen::Vector2d, std::vector<Eigen::Vector2d>> grid(const std::vector<Eigen::Vector2d> &points){
    std::unordered_map<Eigen::Vector2d, std::vector<Eigen::Vector2d>> target_grid;
    for (const auto &point : points) {
        Eigen::Vector2d pixel = point;
        target_grid[pixel].push_back(point);
    }
    return target_grid;
}

std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> k_near(const Eigen::Vector2d &pixel, double range){
    return neighbour_pixels(pixel, range);
}



std::vector<Eigen::Vector2d> apply_transformation(const std::vector<Eigen::Vector2d> &points, 
                                                const Eigen::Matrix3d &transform){
    std::vector<Eigen::Vector2d> transformed_points;
    for (const auto &point : points) {
        Eigen::Vector3d transformed_point = transform * Eigen::Vector3d(point.x(), point.y(), 1.0);
        transformed_points.emplace_back(transformed_point.x(), transformed_point.y());
        }
    return transformed_points;                                            
        }


std::vector<Eigen::Vector2d> concat_points(const std::vector<Eigen::Vector2d> &source_points, 
                                                const std::vector<Eigen::Vector2d> &target_points){
    std::vector<Eigen::Vector2d> points = source_points;
    points.insert(points.end(), target_points.begin(), target_points.end());
    return points;

                                                }

                                                                                 
std::vector<Eigen::Vector2d> downsample(const std::vector<Eigen::Vector2d> &points,
                                                const std::size_t factor){
    std::vector<Eigen::Vector2d> downsampled_points;
    for (std::size_t i = 0; i < points.size(); i += factor) {
        downsampled_points.push_back(points[i]);
    }
    return downsampled_points;
                                                }


std::vector<Eigen::Vector2d> downsample_mean(const std::vector<Eigen::Vector2d> &points,
                                                 const std::size_t factor){
    std::vector<Eigen::Vector2d> downsampled_points;
    for (std::size_t i = 0; i < points.size(); i += factor) {
        Eigen::Vector2d mean;
        for (std::size_t j = 0; j < factor; ++j) {
            mean += points[i + j];
        }
        downsampled_points.push_back(mean / factor);
    }
    return downsampled_points;
                                                }
