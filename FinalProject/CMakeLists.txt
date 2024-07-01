cmake_minimum_required(VERSION 3.16)
PROJECT(lidar_odometry)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

find_package(Eigen3 REQUIRED)
find_package(Open3D REQUIRED HINTS ${PROJECT_SOURCE_DIR}/Open3D/lib/cmake)

add_subdirectory(dataloader)
add_subdirectory(viewer)


add_executable(main main.cpp)
target_link_libraries(main PRIVATE dataloader viewer Eigen3::Eigen Open3D::Open3D)