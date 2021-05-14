CMAKE_MINIMUM_REQUIRED(VERSION 3.4)

PROJECT(ppfplane)
SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/" ${CMAKE_MODULE_PATH})

# ROOTs
SET(EIGEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/eigen")
SET(CERES_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/build-ceres-solver/")
SET(PANGOLIN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/build-Pangolin/src")

########## INCLUDED LIBRARIES ##########

# Eigen
SET(EIGEN_INCLUDE_DIR_HINTS ${EIGEN_ROOT})
FIND_PACKAGE(Eigen3 3.3.4 REQUIRED)
#INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${EIGEN_ROOT})
MESSAGE(STATUS "Found Eigen headers in: ${EIGEN3_INCLUDE_DIR}")

# Ceres Solver
FIND_PACKAGE(Ceres REQUIRED HINTS "${CERES_ROOT}")
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
MESSAGE(STATUS "Found Ceres headers in: ${CERES_INCLUDE_DIRS}")

# Pangolin viewer
SET(Pangolin_DIR ${PANGOLIN_ROOT})
FIND_PACKAGE(Pangolin REQUIRED)
INCLUDE_DIRECTORIES(${Pangolin_INCLUDE_DIR})
MESSAGE(STATUS "Found Pangolin headers in: ${Pangolin_INCLUDE_DIR}")

# Sophus
INCLUDE_DIRECTORIES( 3rdParty/Sophus/ )

# CLI, command line parser
INCLUDE_DIRECTORIES( 3rdParty/CLI11/include/ )

# nanoflann
INCLUDE_DIRECTORIES( 3rdParty/nanoflann/ )

# tinyply
INCLUDE_DIRECTORIES( 3rdParty/tinyply/ )
# ADD_LIBRARY( tinyply /home/alex-pop/Desktop/Doctorat/Side_projects/Volume_Box_2/catkin_ws/src/orthogonal-planes/3rdParty/tinyply/tinyply/tinyply.cpp )
ADD_LIBRARY( tinyply 3rdParty/tinyply/tinyply/tinyply.cpp )
TARGET_COMPILE_OPTIONS( tinyply PRIVATE -std=c++11 )


FIND_PACKAGE(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/voxel_filter_node.cfg
  cfg/passthrough_filter_node.cfg
  cfg/plane_segmentation_node.cfg
  cfg/compute_volume_node.cfg
  cfg/compute_surface_normals_node.cfg
  cfg/line_detect_node.cfg
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pcl_tutorial
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)

########## CUSTOM LIBRARIES ##########

INCLUDE_DIRECTORIES( include/ )

## visualize OpenCV point cloud
SET( LIB_NAME vis_lib )
ADD_LIBRARY( ${LIB_NAME} /home/alex-pop/Desktop/Doctorat/Side_projects/Volume_Box_2/catkin_ws/src/orthogonal-planes/include/visualize/pcshow.cpp
                         /home/alex-pop/Desktop/Doctorat/Side_projects/Volume_Box_2/catkin_ws/src/orthogonal-planes/include/visualize/pcshow_lines.cpp
                         /home/alex-pop/Desktop/Doctorat/Side_projects/Volume_Box_2/catkin_ws/src/orthogonal-planes/include/visualize/pcshow_corners.cpp
                          )
TARGET_LINK_LIBRARIES( ${LIB_NAME} ${Pangolin_LIBRARIES})
TARGET_COMPILE_OPTIONS( ${LIB_NAME} PRIVATE -std=c++11 )

## graph
SET( LIB_NAME graph_lib )
ADD_LIBRARY( ${LIB_NAME} /home/alex-pop/Desktop/Doctorat/Side_projects/Volume_Box_2/catkin_ws/src/orthogonal-planes/include/graph/PlaneGraph.cpp
                         /home/alex-pop/Desktop/Doctorat/Side_projects/Volume_Box_2/catkin_ws/src/orthogonal-planes/include/graph/ParallelPlaneGraph_filtering.cpp
                         /home/alex-pop/Desktop/Doctorat/Side_projects/Volume_Box_2/catkin_ws/src/orthogonal-planes/include/graph/ParallelPlaneGraph_clustering.cpp
                         /home/alex-pop/Desktop/Doctorat/Side_projects/Volume_Box_2/catkin_ws/src/orthogonal-planes/include/graph/ParallelPlaneGraph_optimization.cpp
                          )
TARGET_LINK_LIBRARIES( ${LIB_NAME} ${CERES_LIBRARIES})
TARGET_COMPILE_OPTIONS( ${LIB_NAME} PRIVATE -std=c++11 )

## corners
SET( LIB_NAME corner_lib )
ADD_LIBRARY( ${LIB_NAME} /home/alex-pop/Desktop/Doctorat/Side_projects/Volume_Box_2/catkin_ws/src/orthogonal-planes/include/corners/Corner.cpp
                          )
TARGET_LINK_LIBRARIES( ${LIB_NAME} ${CERES_LIBRARIES})
TARGET_COMPILE_OPTIONS( ${LIB_NAME} PRIVATE -std=c++11 )

## PPF
SET( LIB_NAME ppf_lib )
ADD_LIBRARY( ${LIB_NAME} /home/alex-pop/Desktop/Doctorat/Side_projects/Volume_Box_2/catkin_ws/src/orthogonal-planes/include/PPF/PairDetector.cpp
                          )
TARGET_COMPILE_OPTIONS( ${LIB_NAME} PRIVATE -std=c++11 )


add_executable(voxel_filter_node src/voxel_filter_node.cpp)
add_executable(passthrough_filter_node src/passthrough_filter_node.cpp)
add_executable(plane_segmentation_node src/plane_segmentation_node.cpp)
add_executable(trianglemesh_node src/trianglemesh_node.cpp)
add_executable(compute_volume_node src/compute_volume_node.cpp)
add_executable(compute_surface_normals_node src/compute_surface_normals_node.cpp)
add_executable(Lines_node src/Lines_node.cpp)
add_executable(line_detect_node ply_detect_refine/src/main.cpp)


target_link_libraries(voxel_filter_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(passthrough_filter_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(plane_segmentation_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(trianglemesh_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(compute_volume_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(compute_surface_normals_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(Lines_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

target_link_libraries(line_detect_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${PLY_PPDetectRefine} tinyply
vis_lib # links to Pangolin_LIBRARIES
graph_lib # links to CERES_LIBRARIES
ppf_lib)


add_dependencies(voxel_filter_node ${PROJECT_NAME}_gencfg)
add_dependencies(passthrough_filter_node ${PROJECT_NAME}_gencfg)
add_dependencies(plane_segmentation_node ${PROJECT_NAME}_gencfg)
add_dependencies(trianglemesh_node ${PROJECT_NAME}_gencfg)
add_dependencies(compute_volume_node ${PROJECT_NAME}_gencfg)
add_dependencies(compute_surface_normals_node ${PROJECT_NAME}_gencfg)
add_dependencies(Lines_node ${PROJECT_NAME}_gencfg)
add_dependencies(line_detect_node ${PROJECT_NAME}_gencfg)




########## DIRECTORIES ##########

ADD_SUBDIRECTORY(ply_detect_refine)
ADD_SUBDIRECTORY(ply_corners)
