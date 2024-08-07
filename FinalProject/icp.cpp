#include "icp.hpp"
#include <cmath>
#include <limits>
#include <iostream>

namespace icp {

std::unordered_map<Pixel, std::vector<Eigen::Vector2d>, PixelHash> GridMap(const std::vector<Eigen::Vector2d> &coords, double pixel_size) {
    std::unordered_map<Pixel, std::vector<Eigen::Vector2d>, PixelHash> grid_map;
    for (const auto &coord : coords) {
        Pixel p(static_cast<int>(coord.x() / pixel_size), static_cast<int>(coord.y() / pixel_size));
        grid_map[p].push_back(coord);
    }
    return grid_map;
}

std::vector<Eigen::Vector2d> downsample(const std::vector<Eigen::Vector2d> &coords, double voxel_size) {
    std::unordered_map<Pixel, Eigen::Vector2d, PixelHash> downSample_map;
    for (const auto &coord : coords) {
        int x = std::floor(coord.x() / voxel_size);
        int y = std::floor(coord.y() / voxel_size);
        Pixel p(x, y);
        downSample_map[p] = coord;
    }
    std::vector<Eigen::Vector2d> downsampled_coords;
    for (const auto &entry : downSample_map) {
        downsampled_coords.push_back(entry.second);
    }
    return downsampled_coords;
}

std::tuple<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> nearestNeighbor(const std::vector<Eigen::Vector2d> &source, const std::vector<Eigen::Vector2d> &target) {
    std::vector<Eigen::Vector2d> nearneigh_source;
    std::vector<Eigen::Vector2d> nearneigh_target;
    for (const auto &coord : source) {
        double min_dist = std::numeric_limits<double>::max();
        Eigen::Vector2d nearest_point;
        for (const auto &target_coord : target) {
            double dist = (coord - target_coord).norm();
            if (dist < min_dist) {
                min_dist = dist;
                nearest_point = target_coord;
            }
        }
        nearneigh_source.push_back(coord);
        nearneigh_target.push_back(nearest_point);
    }
    return std::make_tuple(nearneigh_source, nearneigh_target);
}

Eigen::Vector2d centroid(const std::vector<Eigen::Vector2d> &coords) {
    Eigen::Vector2d sum = Eigen::Vector2d::Zero();
    for (const auto &coord : coords) {
        sum += coord;
    }
    return sum / coords.size();
}

Eigen::Matrix2d Covariance(const PointCloud &source, const PointCloud &target) {
    Eigen::Vector2d mean_source = centroid(source);
    Eigen::Vector2d mean_target = centroid(target);
    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (size_t i = 0; i < source.size(); i++) {
        cov += (source[i] - mean_source) * (target[i] - mean_target).transpose();
    }
    return cov / source.size();
}

Eigen::Matrix2d R(const Eigen::Matrix2d &cov) {
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();
    Eigen::Matrix2d R = V * U.transpose();
    if (R.determinant() < 0) {
        V.col(1) *= -1;
        R = V * U.transpose();
    }
    return R;
}

Eigen::Matrix3d icp_known_correspondence(const std::vector<Eigen::Vector2d> &s_correspondences, const std::vector<Eigen::Vector2d> &t_correspondences) {
    Eigen::Vector2d centroid_source = centroid(s_correspondences);
    Eigen::Vector2d centroid_target = centroid(t_correspondences);

    Eigen::Matrix2d cov = Covariance(s_correspondences, t_correspondences);
    Eigen::Matrix2d R_mat = R(cov);
    Eigen::Vector2d t_vec = centroid_target - R_mat * centroid_source;

    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    T.block<2,2>(0,0) = R_mat;
    T.block<2,1>(0,2) = t_vec;

    return T;
}

Eigen::Matrix3d icp_unknown_correspondence(const std::vector<Eigen::Vector2d> &src_, const std::vector<Eigen::Vector2d> &target, const double &pixel_size) {
    int max_iterations = 50;
    int iter = 0;
    double old_err = INFINITY;

    std::vector<Eigen::Vector2d> src = src_;
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d t = Eigen::Matrix3d::Identity();

    std::unordered_map<Pixel, std::vector<Eigen::Vector2d>, PixelHash> target_grid = GridMap(target, pixel_size);

    while (true) {
        iter++;
        auto [s_correspondences, t_correspondences] = nearestNeighbor(src, target);
        t = icp_known_correspondence(s_correspondences, t_correspondences);

        if (iter == 1) {
            T = t;
        } else {
            T = t * T;
        }

        src = apply_transformation(t, src);

        double err = Error(src, target, t);
        if (iter == max_iterations || err == old_err) {
            break;
        }

        old_err = err;

        // Clear the correspondences
        s_correspondences.clear();
        t_correspondences.clear();
    }

    // Apply final transformation to the entire target scan and extract x, y points
    std::vector<Eigen::Vector2d> transformed_points = apply_transformation(T, target);

    // Clear transformed points after appending to cumulative pointcloud
    transformed_points.clear();

    return T;
}

std::vector<Eigen::Vector2d> apply_transformation(const Eigen::Matrix3d &transformation, const std::vector<Eigen::Vector2d> &src) {
    std::vector<Eigen::Vector2d> transformed_points;
    Eigen::Matrix2d R = transformation.block<2, 2>(0, 0);
    Eigen::Vector2d t = transformation.block<2, 1>(0, 2);

    for (const auto &point : src) {
        transformed_points.emplace_back(R * point + t);
    }

    return transformed_points;
}

std::vector<Eigen::Vector2d> concat_pointclouds(std::vector<Eigen::Vector2d> &first, const std::vector<Eigen::Vector2d> &second) {
    std::vector<Eigen::Vector2d> result = first;
    result.insert(result.end(), second.begin(), second.end());
    return result;
}

double Error(const std::vector<Eigen::Vector2d> &source, const std::vector<Eigen::Vector2d> &target, const Eigen::Matrix3d &T) {
    double error = 0;
    Eigen::Matrix2d R = T.block<2,2>(0,0);
    Eigen::Vector2d t = T.block<2,1>(0,2);
    for (size_t i = 0; i < source.size(); i++) {
        error += (target[i] - R * source[i] - t).norm();
    }
    return error / source.size();
}

Eigen::MatrixXd Jacobian(const Eigen::Vector2d &p, const Eigen::Vector2d &q, const Eigen::Matrix3d &T) {
    Eigen::Vector2d rp = (T.block<2,2>(0,0) * p + T.block<2,1>(0,2));

    Eigen::MatrixXd J(2, 3);
    J(0, 0) = 1;
    J(0, 1) = 0;
    J(0, 2) = -rp.y();
    J(1, 0) = 0;
    J(1, 1) = 1;
    J(1, 2) = rp.x();

    return J;
}

} // namespace icp
