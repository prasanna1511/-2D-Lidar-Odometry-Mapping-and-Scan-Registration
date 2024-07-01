add_library(viewer STATIC viewer.cpp)

target_include_directories(viewer PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>)
target_link_libraries(viewer PUBLIC Open3D::Open3D)
