add_library(icp STATIC icp.cpp)

target_sources(icp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/icp.cpp)
target_compile_features(icp PUBLIC cxx_std_17)
target_include_directories(icp PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>)
target_link_libraries(icp PUBLIC Eigen3::Eigen)
target_link_libraries(icp PUBLIC Open3D::Open3D)