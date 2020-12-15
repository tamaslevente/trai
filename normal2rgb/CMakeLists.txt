cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(normal2rgb)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


include_directories(${PCL_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (normal2rgb src/normal2rgb.cpp)
add_executable (combine_depth_ir src/combine_depth_ir.cpp)
add_executable (voxel_filter src/voxel_filter.cpp)
add_executable (pcd2xyz src/pcd2xyz.cpp)
add_executable (compute_normals src/compute_normals.cpp)
add_executable (cloud2image src/cloud2image.cpp)
add_executable (jpg2png src/jpg2png.cpp)


target_link_libraries (normal2rgb ${PCL_LIBRARIES})
target_link_libraries( combine_depth_ir ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries (voxel_filter ${PCL_LIBRARIES})
target_link_libraries (pcd2xyz ${PCL_LIBRARIES})
target_link_libraries (compute_normals ${PCL_LIBRARIES})
target_link_libraries( cloud2image ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries( jpg2png ${OpenCV_LIBS})



