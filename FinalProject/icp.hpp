#pragma once

#include <Eigen/Core>
#include <unordered_map>
#include <vector>
#include <utility>
#include <functional>
#include <Eigen/Dense>


Eigen::Vector2d errorij(const Eigen::Vector2d &p, const Eigen::Vector2d &q, const Eigen::Matrix3d &T);


Eigen::Matrix<double, 2, 3> Jacobian(const Eigen::Vector2d &p, const Eigen::Vector2d &q, const Eigen::Matrix3d &T);


void linear_system(const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &correspondences,
                    const Eigen::Matrix3d &T,
                    Eigen::Matrix<double, 3, 3> &H,
                    Eigen::Vector3d &b);

Eigen::Vector3d delta_x(const Eigen::Matrix<double,3, 3> &H, const Eigen::Vector3d &b);

Eigen::Matrix3d apply_transformation(const Eigen::Vector3d &delta_x, const Eigen::Matrix3d &T);




