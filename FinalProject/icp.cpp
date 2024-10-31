#include <algorithm>
#include <cmath>
#include <limits>
#include "icp.hpp"
#include <open3d/Open3D.h>
#include <open3d/core/ShapeUtil.h>
#include <utility>


namespace {

    Eigen::Vector2d centroid(const std::vector<Eigen::Vector2d> &coords) {
        Eigen::Vector2d sum = Eigen::Vector2d::Zero();
        for (const auto &point : coords) {
            sum += point;
        }
        return sum /coords.size();
    }

    Eigen::Matrix2d computeCovariance(const std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target) {
        Eigen ::Vector2d srcMean = centroid(src);
        Eigen::Vector2d targetMean = centroid(target);
        Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
        for (size_t i = 0; i < src.size(); i++) {
            cov += (target[i] - targetMean) * (src[i] - srcMean).transpose();
        }
        return cov;
    }

    std::vector<Eigen::Vector2d> applyTransformation(const Eigen::Matrix3d &transformation, std::vector<Eigen::Vector2d> &coords) {
        Eigen::Matrix2d rotation = transformation.block<2, 2>(0, 0);
        Eigen::Vector2d translation = transformation.block<2, 1>(0, 2);

        std::transform(coords.cbegin(), coords.cend(), coords.begin(), [&rotation, &translation](const auto &p) {
            return rotation * p + translation;
        });
        return coords;
    }

    double calculateError(const std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target) {
        double error = 0.0;
        for (size_t i = 0; i < src.size(); i++) {
            error += (target[i] - src[i]).squaredNorm();
        }
        return error;
    }

    std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> createGridMap(const std::vector<Eigen::Vector2d> &coords, const double &pixelSize) {
        std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> grid;
        for (const Eigen::Vector2d &point : coords) {
            const Pixel p(point, pixelSize);
            grid[p].push_back(point);
        }
        return grid;
    }

    std::vector<Pixel> findNeighbourPixels(const Pixel &p, const int range = 1) {
        std::vector<Pixel> neighbourPixels;
        neighbourPixels.reserve(9);
        for (int x = p.i - range; x <= p.i + range; x++) {
            for (int y = p.j - range; y <= p.j + range; y++) {
                neighbourPixels.emplace_back(x, y);
            }
        }
        return neighbourPixels;
    }

    std::vector<Eigen::Vector2d> getPixelPoints(const std::vector<Pixel> &pixels, const std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> &targetGrid) {
        std::vector<Eigen::Vector2d> points;
        for (const auto &pixel : pixels) {
            const auto it = targetGrid.find(pixel);
            if (it != targetGrid.end()) {
                points.insert(points.end(), it->second.begin(), it->second.end());
            }
        }
        return points;
    }

    std::tuple<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> findNearestNeighbours(const std::vector<Eigen::Vector2d> &src, const std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> &targetGrid, const double &pixelSize) {
        std::vector<Eigen::Vector2d> srcCorrespondences, targetCorrespondences;
        srcCorrespondences.reserve(src.size());
        targetCorrespondences.reserve(src.size());

        for (const Eigen::Vector2d &point : src) {
            const Pixel p(point, pixelSize);
            const std::vector<Pixel> &neighbours = findNeighbourPixels(p, 1);
            const std::vector<Eigen::Vector2d> &potentialPoints = getPixelPoints(neighbours, targetGrid);

            if (potentialPoints.empty()) continue;

            double minDist = std::numeric_limits<double>::infinity();
            Eigen::Vector2d closestPoint = Eigen::Vector2d::Zero();

            for (const Eigen::Vector2d &candidate : potentialPoints) {
                double dist = (point - candidate).norm();
                if (dist < minDist) {
                    minDist = dist;
                    closestPoint = candidate;
                }
            }
            srcCorrespondences.push_back(point);
            targetCorrespondences.push_back(closestPoint);
        }
        return std::make_tuple(srcCorrespondences, targetCorrespondences);
    }

    
    Eigen::Matrix3d computeIcpKnownCorrespondence(std::vector<Eigen::Vector2d> src, const std::vector<Eigen::Vector2d> &target) {

    const Eigen::Vector2d &srcMean = centroid(src);
    const Eigen::Vector2d &targetMean = centroid(target);
    Eigen::Matrix2d covariance = computeCovariance(src, target);

    Eigen::JacobiSVD<Eigen::Matrix2d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d rotation = svd.matrixU() * svd.matrixV().transpose();
    Eigen::Vector2d translation = targetMean - rotation * srcMean;

    Eigen::Matrix3d transformation = Eigen::Matrix3d::Identity();
    transformation.block<2, 2>(0, 0) = rotation;
    transformation.block<2, 1>(0, 2) = translation;

   

    return transformation;
}

} // namespace

namespace icp {


    void icpUnknownCorrespondence(std::vector<Eigen::Vector2d> &src,
                                              const std::vector<Eigen::Vector2d> &target,
                                              const double &pixelSize) {
        int maxIterations = 15;  
        double oldError = std::numeric_limits<double>::infinity(); 
        Eigen::Matrix3d totalTransformation = Eigen::Matrix3d::Identity(); 

        std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> targetGrid = createGridMap(target, pixelSize);

        for (int iter = 0; iter < maxIterations; iter++) {
            auto [srcCorrespondences, targetCorrespondences] = findNearestNeighbours(src, targetGrid, pixelSize);
            Eigen::Matrix3d transformation = computeIcpKnownCorrespondence(srcCorrespondences, targetCorrespondences);

            src = applyTransformation(transformation, src);

            totalTransformation = transformation * totalTransformation;

            double error = calculateError(src, target);
           
            if (std::abs(oldError - error) < 1e-6) {
                break;
            }
            oldError = error;
        }
    }

    std::vector<Eigen::Vector2d> concatenatePointClouds(std::vector<Eigen::Vector2d> &leftCloud, const std::vector<Eigen::Vector2d> &rightCloud) {
        leftCloud.reserve(leftCloud.size() + rightCloud.size());
        leftCloud.insert(leftCloud.end(), rightCloud.begin(), rightCloud.end());
        return leftCloud;
    }

    std::vector<Eigen::Vector2d> downsamplePointCloud(const std::vector<Eigen::Vector2d> &coords, const double &pixelSize, const int &maxPoints) {
        std::vector<Eigen::Vector2d> filteredPoints;
        std::unordered_map<Pixel, int> pixelMap;

        for (const Eigen::Vector2d &point : coords) {
            const Pixel p(point, pixelSize);
            if (pixelMap[p] < maxPoints) {
                pixelMap[p]++;
                filteredPoints.push_back(point);
            }
        }
        return filteredPoints;
    }

} // namespace icp
