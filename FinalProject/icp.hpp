#pragma once
#include <unordered_map>
#include <vector>
#include <Eigen/Dense>


// Pixel struct for discretizing points based on a given pixel size
struct Pixel {
    Pixel(int x, int y) : i(x), j(y) {}

    Pixel(const Eigen::Vector2d &point, const double &pixelSize)
        : i(static_cast<int>(point[0] / pixelSize)),
          j(static_cast<int>(point[1] / pixelSize)) {}

    bool operator==(const Pixel &px) const {
        return i == px.i && j == px.j;
    }

    int i;
    int j;
};

namespace std {
    template <>
    struct hash<Pixel> {
        size_t operator()(const Pixel &px) const {
            return ((1 << 20) - 1) & (px.i * 73856093 ^ px.j * 19349663);
        }
        Eigen::Vector2d centroid(const std::vector<Eigen::Vector2d> &coords);

        Eigen::Matrix2d computeCovariance(const std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target); 

        std::vector<Eigen::Vector2d> applyTransformation(const Eigen::Matrix3d &transformation, std::vector<Eigen::Vector2d> &coords);

        double calculateError(const std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target);

        std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> createGridMap(const std::vector<Eigen::Vector2d> &coords, const double &pixelSize);

        std::vector<Pixel> findNeighbourPixels(const Pixel &p, const int range = 1);

        std::vector<Eigen::Vector2d> getPixelPoints(const std::vector<Pixel> &pixels, const std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> &targetGrid);

        std::tuple<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> findNearestNeighbours(const std::vector<Eigen::Vector2d> &src, const std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> &targetGrid, const double &pixelSize);
        
        Eigen::Matrix3d computeIcpKnownCorrespondence(std::vector<Eigen::Vector2d> src, const std::vector<Eigen::Vector2d> &target);
  
    };
} // namespace std

namespace icp {

 
    void icpUnknownCorrespondence(std::vector<Eigen::Vector2d> &src,
                                              const std::vector<Eigen::Vector2d> &target,
                                              const double &pixelSize
                                              );

    std::vector<Eigen::Vector2d> extractTrajectoryPoints(const std::vector<Eigen::Matrix3d> &trajectory);

    std::vector<Eigen::Vector2d> concatenatePointClouds(std::vector<Eigen::Vector2d> &leftCloud,
                                                        const std::vector<Eigen::Vector2d> &rightCloud);

    std::vector<Eigen::Vector2d> downsamplePointCloud(const std::vector<Eigen::Vector2d> &coords,
                                                      const double &pixelSize, const int &maxPoints);

} // namespace icp
